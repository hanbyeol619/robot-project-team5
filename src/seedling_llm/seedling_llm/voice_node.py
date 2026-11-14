import io
import os
import threading
import wave

import numpy as np
import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
from dotenv import load_dotenv
from openwakeword.model import Model
from ament_index_python.packages import get_package_share_directory

RATE = 16000
CHUNK = 1280  # 80ms, openwakeword 권장 크기


def resource_path(name):
    return os.path.join(get_package_share_directory("seedling_llm"), "resource", name)


def load_api_key():
    if os.getenv("OPENAI_API_KEY"):
        return os.getenv("OPENAI_API_KEY")
    load_dotenv(resource_path(".env"))
    return os.getenv("OPENAI_API_KEY")


class VoiceNode(Node):
    def __init__(self):
        super().__init__("voice_node")

        # 파라미터
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("record_seconds", 4.0)
        self.declare_parameter("device_index", -1)   # -1이면 기본 마이크
        self.threshold = float(self.get_parameter("threshold").value)
        self.record_seconds = float(self.get_parameter("record_seconds").value)
        device_index = int(self.get_parameter("device_index").value)

        # 웨이크워드 모델
        self.ww = Model(
            wakeword_models=[resource_path("hello_kernel.onnx")],
            melspec_model_path=resource_path("melspectrogram.onnx"),
            embedding_model_path=resource_path("embedding_model.onnx"),
            inference_framework="onnx",
        )
        self.ww_name = list(self.ww.models.keys())[0]

        # STT
        api_key = load_api_key()
        if not api_key:
            raise RuntimeError("OPENAI_API_KEY를 찾을 수 없습니다")
        self.client = OpenAI(api_key=api_key, timeout=15.0)

        # 퍼블리셔
        self.text_pub = self.create_publisher(String, "/voice/text", 10)
        self.event_pub = self.create_publisher(String, "/voice/event", 10)

        # 마이크
        self.pa = pyaudio.PyAudio()
        self.stream = self.pa.open(
            format=pyaudio.paInt16, channels=1, rate=RATE, input=True,
            frames_per_buffer=CHUNK,
            input_device_index=None if device_index < 0 else device_index,
        )

        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f"Voice node started (wakeword={self.ww_name}, threshold={self.threshold})"
        )

    # -------------------------------------------------
    def read_chunk(self):
        return self.stream.read(CHUNK, exception_on_overflow=False)

    def loop(self):
        n, best, loud = 0, 0.0, 0.0
        while self.running and rclpy.ok():
            audio = np.frombuffer(self.read_chunk(), dtype=np.int16)
            score = self.ww.predict(audio)[self.ww_name]
            # 2초마다 마이크 크기와 최고 점수 표시 (웨이크워드가 안 될 때 원인 찾기용)
            n += 1
            best = max(best, score)
            loud = max(loud, float(np.abs(audio.astype(np.int32)).max()) / 32768 * 100)
            if n >= 25:
                hint = ' ← 너무 작아요' if loud < 3 else (' ← 너무 커서 찢어져요' if loud > 98 else '')
                self.get_logger().info(
                    f'[마이크] 크기 {loud:.0f}%{hint} | HELLO KERNEL 점수 최고 {best:.2f} (기준 {self.threshold})')
                n, best, loud = 0, 0.0, 0.0
            if score < self.threshold:
                continue

            self.get_logger().info(f"웨이크워드 감지 (score={score:.2f})")
            self.event_pub.publish(String(data="wakeup"))

            frames = self.record()
            self.event_pub.publish(String(data="recorded"))

            text = self.transcribe(frames)
            if text:
                self.text_pub.publish(String(data=text))
                self.get_logger().info(f"STT: {text}")
            else:
                self.event_pub.publish(String(data="stt_failed"))

            # STT 하는 동안 쌓인 소리 버리고 웨이크워드 상태 초기화
            self.flush_stream()
            self.ww.reset()
            self.get_logger().info("다시 대기 중...")

    def record(self):
        self.get_logger().info(f"녹음 시작 ({self.record_seconds:.0f}초)")
        n = int(RATE / CHUNK * self.record_seconds)
        return [self.read_chunk() for _ in range(n)]

    def flush_stream(self):
        try:
            avail = self.stream.get_read_available()
            if avail > 0:
                self.stream.read(avail, exception_on_overflow=False)
        except Exception:
            pass

    def transcribe(self, frames):
        buf = io.BytesIO()
        with wave.open(buf, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(RATE)
            wf.writeframes(b"".join(frames))
        buf.seek(0)
        buf.name = "speech.wav"
        try:
            result = self.client.audio.transcriptions.create(
                model="whisper-1", file=buf, language="ko"
            )
            return result.text.strip()
        except Exception as e:
            self.get_logger().error(f"STT 오류: {e}")
            return ""

    def destroy_node(self):
        self.running = False
        try:
            self.stream.stop_stream()
            self.stream.close()
            self.pa.terminate()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()