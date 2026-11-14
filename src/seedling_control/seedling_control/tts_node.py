#!/usr/bin/env python3
"""TTS 노드: /tts/text 문장을 OpenAI TTS 로 읽어 스피커로 재생.

- API 키: 환경변수 OPENAI_API_KEY → 없으면 seedling_llm 패키지의 resource/.env
- 말하는 동안 /tts/state 에 'speaking', 끝나면 'idle' 발행
  (voice_node 가 이 값을 보고 웨이크워드 감지를 잠깐 꺼두면 자기 목소리에 반응하는 걸 막을 수 있음)
"""
import os
import queue
import threading

import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PCM_RATE = 24000  # OpenAI TTS 'pcm' 출력: 24kHz, 16bit, mono


def load_api_key():
    if os.getenv('OPENAI_API_KEY'):
        return os.getenv('OPENAI_API_KEY')
    try:
        from ament_index_python.packages import get_package_share_directory
        from dotenv import load_dotenv
        load_dotenv(os.path.join(get_package_share_directory('seedling_llm'), 'resource', '.env'))
    except Exception:  # noqa: BLE001
        pass
    return os.getenv('OPENAI_API_KEY')


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.declare_parameter('model', 'gpt-4o-mini-tts')
        self.declare_parameter('voice', 'nova')
        self.declare_parameter('device_index', -1)
        self.model = self.get_parameter('model').value
        self.voice = self.get_parameter('voice').value
        dev = int(self.get_parameter('device_index').value)

        from openai import OpenAI
        key = load_api_key()
        if not key:
            raise RuntimeError('OPENAI_API_KEY를 찾을 수 없습니다')
        self.client = OpenAI(api_key=key, timeout=15.0)

        self.pa = pyaudio.PyAudio()
        self.out = self.pa.open(format=pyaudio.paInt16, channels=1, rate=PCM_RATE, output=True,
                                output_device_index=None if dev < 0 else dev)
        self.q = queue.Queue()
        self.state_pub = self.create_publisher(String, '/tts/state', 10)
        self.create_subscription(String, '/tts/text', self.on_text, 10)
        self.create_subscription(String, '/robot/stop', self.on_stop, 10)
        threading.Thread(target=self.loop, daemon=True).start()
        self.get_logger().info(f'TTS 노드 시작 (model={self.model}, voice={self.voice})')

    def on_text(self, msg):
        if msg.data.strip():
            self.q.put(msg.data.strip())

    def on_stop(self, msg):
        # 정지하면 밀린 안내는 버림 ("정지합니다"는 control_node 가 그 뒤에 보냄)
        if '"stop"' in msg.data:
            while not self.q.empty():
                try:
                    self.q.get_nowait()
                except queue.Empty:
                    break

    def loop(self):
        while rclpy.ok():
            text = self.q.get()
            self.state_pub.publish(String(data='speaking'))
            try:
                with self.client.audio.speech.with_streaming_response.create(
                        model=self.model, voice=self.voice, input=text,
                        response_format='pcm') as resp:
                    for chunk in resp.iter_bytes(4096):
                        self.out.write(chunk)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'TTS 오류: {e}')
            if self.q.empty():
                self.state_pub.publish(String(data='idle'))

    def destroy_node(self):
        try:
            self.out.stop_stream()
            self.out.close()
            self.pa.terminate()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
