import json
import os
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
from dotenv import load_dotenv
from ament_index_python.packages import get_package_share_directory

ACTIONS = ("plant", "harvest", "stop", "unknown")

STOP_KEYWORDS = ("멈춰", "정지", "스톱", "그만", "stop")
PLANT_HINTS = ("묘목", "며목", "모목", "묘묵", "미목", "나무", "심어", "심자", "심기")

SYSTEM_PROMPT = """너는 묘목심기 로봇의 음성 명령 해석기다.
사용자 문장을 보고 아래 action 중 하나만 골라 JSON으로만 답한다.

- plant: 묘목/나무/씨앗을 심으라는 요청 (예: "심어줘", "나무 하나 심자")
- harvest: 열매/과일을 수확하거나 따라는 요청 (예: "열매 따줘", "수확해")
- stop: 멈춤/정지/취소 요청
- unknown: 위에 해당하지 않거나 의미가 불명확한 경우

음성 인식 결과라 오타나 띄어쓰기 오류가 있을 수 있으니 문맥으로 판단한다.
비슷한 발음도 같은 뜻으로 본다 (예: "며목", "모목", "묘묵", "미목" = 묘목 / "시머", "심어요" = 심어).
명령 뒤에 상관없는 말이 붙어 있어도 명령 부분만 보고 고른다 (예: "며목심어 아 약간 발음" → plant).
출력 형식: {"action": "<plant|harvest|stop|unknown>"}"""


def load_api_key():
    # 1) 환경변수  2) 패키지 resource/.env
    if os.getenv("OPENAI_API_KEY"):
        return os.getenv("OPENAI_API_KEY")
    env_path = os.path.join(
        get_package_share_directory("seedling_llm"), "resource", ".env"
    )
    load_dotenv(env_path)
    return os.getenv("OPENAI_API_KEY")


class LLMNode(Node):
    def __init__(self):
        super().__init__("llm_node")

        api_key = load_api_key()
        if not api_key:
            raise RuntimeError("OPENAI_API_KEY를 찾을 수 없습니다")
        self.client = OpenAI(api_key=api_key, timeout=10.0)
        self.model = "gpt-4o-mini"

        self.sub = self.create_subscription(String, "/voice/text", self.on_text, 10)
        self.pub = self.create_publisher(String, "/llm/command", 10)

        self.get_logger().info(f"LLM node started (model={self.model})")

    def on_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f"입력: {text}")

        # 비상정지는 LLM 거치지 않고 즉시 발행
        if any(k in text.replace(" ", "").lower() for k in STOP_KEYWORDS):
            out = json.dumps({"action": "stop", "text": text}, ensure_ascii=False)
            self.pub.publish(String(data=out))
            self.get_logger().info(f"즉시 정지 발행: {out}")
            return

        threading.Thread(target=self.process_text, args=(text,), daemon=True).start()

    def process_text(self, text: str):
        try:
            action = self.ask_llm(text)
        except Exception as e:
            self.get_logger().error(f"LLM 오류: {e}")
            action = "unknown"

        # LLM 이 못 알아들어도 '심' + 묘목 비슷한 말이 있으면 심기로 봄 (발음 오인식 대비)
        t = text.replace(" ", "")
        if action == "unknown" and "심" in t and any(k in t for k in PLANT_HINTS):
            self.get_logger().info(f"발음 보정: '{text}' → plant")
            action = "plant"

        cmd = {"action": action, "text": text}
        out = json.dumps(cmd, ensure_ascii=False)
        self.pub.publish(String(data=out))
        self.get_logger().info(f"발행: {out}")

    def ask_llm(self, text: str) -> str:
        resp = self.client.chat.completions.create(
            model=self.model,
            temperature=0,
            response_format={"type": "json_object"},
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": text},
            ],
        )
        raw = resp.choices[0].message.content
        self.get_logger().info(f"LLM 응답: {raw}")

        action = str(json.loads(raw).get("action", "unknown")).lower().strip()
        return action if action in ACTIONS else "unknown"


def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
