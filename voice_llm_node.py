#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
음성·LLM 명령 노드 (Whisper + Claude)

흐름:
  UI 마이크 녹음 ──(/ui/command, voice_audio)──▶ Whisper(음성→글자)
  UI 글자 입력   ──(/ui/command, voice)────────▶ ┐
                                                 ▼
                         Claude(글자→로봇 명령 해석, 도구 호출)
                                                 ▼
  UI 승인 카드 ◀──(/ui/status, plan)── 이 노드
  UI 승인 버튼 ──(/ui/command, approve)──▶ 이 노드 ──(/ui/command, plant 등)──▶ 명령 처리 노드 ──▶ 로봇

안전 규칙:
  - 로봇을 움직이는 명령은 반드시 UI에서 승인해야 실행됩니다.
  - "멈춰/정지"는 승인 없이 즉시 일시정지합니다.

준비:
  pip install anthropic openai-whisper
  sudo apt install ffmpeg
  export ANTHROPIC_API_KEY="발급받은 키"
"""
import base64
import json
import os
import queue
import re
import tempfile
import threading
import time
import traceback
import uuid

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

# =====================================================================
# 1. 설정
# =====================================================================
CLAUDE_MODEL = 'claude-haiku-5-5'   # 빠르고 저렴한 모델. 오류가 나면 콘솔에서 쓸 수 있는 모델 이름으로 바꾸세요
USE_FAKE_LLM = False                # True: API 키 없이 간단한 규칙으로 흉내 (연결 시험용)

WHISPER_MODEL = 'small'             # tiny, base, small, medium (클수록 정확하지만 느림)
WHISPER_LANG = 'ko'

CMD_TOPIC = '/ui/command'
STATUS_TOPIC = '/ui/status'

DEFAULT_SPACING = 150               # 간격을 말하지 않았을 때 (mm)
PLAN_TIMEOUT_SEC = 120              # 승인 대기 시간
TASK_START_TIMEOUT = 8              # 명령을 보낸 뒤 작업이 시작되길 기다리는 시간
TASK_DONE_TIMEOUT = 900             # 한 작업이 끝나길 기다리는 최대 시간

STOP_WORDS = re.compile(r'멈춰|멈춤|정지|스톱|그만')
# Whisper가 조용한 소리를 듣고 지어내는 흔한 문장
WHISPER_GHOSTS = {'감사합니다', '시청해주셔서 감사합니다', '시청해 주셔서 감사합니다', '구독과 좋아요 부탁드립니다', 'MBC 뉴스'}

# 로봇이 할 수 있는 일 (Claude에게 알려주는 도구 목록)
TOOLS = [
    {'name': 'plant_seedlings',
     'description': '식재 구역을 촬영해 빈 칸을 찾고, 간격을 지켜 묘목을 심는다.',
     'input_schema': {'type': 'object', 'properties': {
         'count': {'type': 'integer', 'minimum': 0, 'maximum': 6,
                   'description': '심을 그루 수. 개수를 말하지 않았거나 "전부", "모두", "빈 칸 다"면 0'},
         'spacing_mm': {'type': 'integer', 'minimum': 100, 'maximum': 300,
                        'description': '최소 간격(mm). 사용자가 말한 경우에만 넣는다'}},
         'required': ['count']}},
    {'name': 'scan_cells',
     'description': '식재 구역을 촬영해 빈 칸과 묘목이 있는 칸만 확인한다. 심지는 않는다.',
     'input_schema': {'type': 'object', 'properties': {}}},
    {'name': 'classify_berries',
     'description': '열매 구역을 촬영해 익은 열매(빨강)와 덜 익은 열매(초록)를 나눈다.',
     'input_schema': {'type': 'object', 'properties': {}}},
    {'name': 'harvest_berries',
     'description': '익은 열매를 집어 수확함으로 옮긴다. 덜 익은 열매는 수확하지 않는다.',
     'input_schema': {'type': 'object', 'properties': {
         'berry_ids': {'type': 'array', 'items': {'type': 'integer'},
                       'description': '특정 열매 번호를 말한 경우에만 넣는다. 없으면 익은 열매 전부'}}}},
    {'name': 'go_home',
     'description': '로봇팔을 기본 자세(홈 위치)로 돌려놓는다.',
     'input_schema': {'type': 'object', 'properties': {}}},
    {'name': 'set_gripper',
     'description': '그리퍼를 열거나 닫는다.',
     'input_schema': {'type': 'object', 'properties': {
         'state': {'type': 'string', 'enum': ['open', 'close']}}, 'required': ['state']}},
    {'name': 'set_speed',
     'description': '로봇 이동 속도를 퍼센트로 바꾼다.',
     'input_schema': {'type': 'object', 'properties': {
         'percent': {'type': 'integer', 'minimum': 10, 'maximum': 100}}, 'required': ['percent']}},
    {'name': 'resume_robot',
     'description': '일시정지한 작업을 다시 시작한다.',
     'input_schema': {'type': 'object', 'properties': {}}},
]

SYSTEM_PROMPT = """당신은 농업 실습장의 두산 협동로봇(로봇팔 1대)에 내리는 한국어 음성 명령을 해석합니다.
작업대: A 식재 구역(2줄 × 3칸, 칸 번호 1~6), 묘목 공급대, B 모형 열매 구역, 수확함.

규칙:
1. 로봇을 움직이는 요청이면 알맞은 도구를 호출하세요. 여러 작업을 말하면 말한 순서대로 여러 도구를 호출하세요.
2. 상태 질문, 인사, 고맙다는 말에는 도구 없이 아래 현재 상태를 근거로 한두 문장으로 답하세요.
3. 뜻이 애매하거나 이 로봇이 할 수 없는 일이면 도구를 호출하지 말고 무엇을 원하는지 짧게 되물으세요.
4. 음성 인식 오류가 있을 수 있습니다(예: "시머줘"는 "심어줘"). 뜻이 분명할 때만 바로잡으세요.
5. 현재 상태에 없는 정보는 지어내지 말고 모른다고 하세요.
6. 답은 존댓말로, 짧고 따뜻하게 쓰세요.

현재 상태(JSON):
{state}"""


# =====================================================================
# 2. 명령 변환과 설명
# =====================================================================
def tool_to_cmd(name, args):
    """Claude 도구 호출 → 명령 처리 노드가 이해하는 명령"""
    if name == 'plant_seedlings':
        count = max(0, min(6, int(args.get('count') or 0)))
        spacing = max(100, min(300, int(args.get('spacing_mm') or DEFAULT_SPACING)))
        return {'type': 'plant', 'count': count, 'spacing_mm': spacing}
    if name == 'scan_cells':
        return {'type': 'scan'}
    if name == 'classify_berries':
        return {'type': 'classify'}
    if name == 'harvest_berries':
        ids = [int(i) for i in (args.get('berry_ids') or [])]
        return {'type': 'harvest', 'berry_ids': ids} if ids else {'type': 'harvest'}
    if name == 'go_home':
        return {'type': 'home'}
    if name == 'set_gripper':
        return {'type': 'gripper', 'state': 'close' if args.get('state') == 'close' else 'open'}
    if name == 'set_speed':
        return {'type': 'speed', 'percent': max(10, min(100, int(args.get('percent') or 60)))}
    if name == 'resume_robot':
        return {'type': 'resume'}
    return None


def describe(c):
    t = c['type']
    if t == 'plant':
        n = f"{c['count']}그루" if c['count'] else '빈 칸 모두'
        return f"묘목 {n} 식재(간격 {c['spacing_mm']}mm)"
    if t == 'harvest':
        return f"열매 {', '.join(map(str, c['berry_ids']))}번 수확" if c.get('berry_ids') else '익은 열매 모두 수확'
    if t == 'gripper':
        return '그리퍼 ' + ('닫기' if c['state'] == 'close' else '열기')
    if t == 'speed':
        return f"속도 {c['percent']}%로 변경"
    return {'scan': '빈 칸 판별', 'classify': '성숙도 분류', 'home': '홈 위치로 이동', 'resume': '작업 재개'}[t]


# 명령 처리 노드가 실행하는 "작업" (끝날 때까지 기다려야 하는 것)
TASK_TYPES = {'scan', 'plant', 'classify', 'harvest', 'home', 'gripper'}


# =====================================================================
# 3. 노드
# =====================================================================
class VoiceLlmNode(Node):
    def __init__(self):
        super().__init__('voice_llm_node')
        self.cmd_pub = self.create_publisher(String, CMD_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(String, CMD_TOPIC, self.on_command, 10)
        self.create_subscription(String, STATUS_TOPIC, self.on_status, 10)

        self.jobs = queue.Queue()           # 음성 인식·LLM 호출은 느려서 별도 스레드에서 처리
        self.plans = {}                      # 승인 대기 중인 계획 {id: (명령 목록, 만든 시각)}
        self.lock = threading.Lock()
        self.busy = None                     # 명령 처리 노드의 현재 작업 이름
        self.busy_changed = threading.Condition()
        self.started = 0                     # 작업이 시작된 횟수
        self.finished = 0                    # 작업이 끝난 횟수
        self.estop = threading.Event()
        self.scene = {}
        self.recent_logs = []
        self.whisper = None
        self.claude = None

    # --- 보내기 ---
    def _pub(self, pub, data):
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        pub.publish(msg)

    def send_status(self, **data):
        self._pub(self.status_pub, data)

    def reply(self, text):
        self.send_status(type='reply', text=text)
        self.get_logger().info(f'답변: {text}')

    def send_cmd(self, cmd):
        self._pub(self.cmd_pub, cmd)

    # --- 받기 ---
    def on_command(self, msg):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        t = d.get('type')
        if t in ('voice', 'voice_audio'):
            self.jobs.put(d)
        elif t == 'approve':
            self.approve(d.get('plan_id'))
        elif t == 'reject':
            with self.lock:
                self.plans.pop(d.get('plan_id'), None)
        elif t == 'estop':
            self.estop.set()
            with self.lock:
                self.plans.clear()
        elif t == 'estop_reset':
            self.estop.clear()

    def on_status(self, msg):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        t = d.get('type')
        if t == 'state':
            with self.busy_changed:
                new = d.get('busy')
                if new and not self.busy:
                    self.started += 1
                if self.busy and not new:
                    self.finished += 1
                self.busy = new
                self.busy_changed.notify_all()
        elif t == 'scene':
            self.scene = d
        elif t == 'log' and d.get('text'):
            self.recent_logs = (self.recent_logs + [d['text']])[-5:]

    # --- 준비 ---
    def load_models(self):
        try:
            import whisper
            import torch
            self.get_logger().info(f'Whisper {WHISPER_MODEL} 모델을 불러오는 중...')
            self.whisper = whisper.load_model(WHISPER_MODEL)
            self.fp16 = torch.cuda.is_available()
            self.get_logger().info('Whisper 준비 완료')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Whisper를 불러오지 못했습니다. 글자 입력만 쓸 수 있어요: {e}')
        if not USE_FAKE_LLM:
            try:
                from anthropic import Anthropic
                if not os.environ.get('ANTHROPIC_API_KEY'):
                    raise RuntimeError('ANTHROPIC_API_KEY 환경변수가 없습니다')
                self.claude = Anthropic()
                self.get_logger().info(f'Claude 준비 완료 ({CLAUDE_MODEL})')
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'Claude를 준비하지 못했습니다: {e}')

    # --- 작업 스레드 ---
    def run(self):
        while rclpy.ok():
            try:
                job = self.jobs.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if job['type'] == 'voice_audio':
                    text = self.transcribe(job)
                    if not text:
                        continue
                    self.send_status(type='heard', text=text)
                else:
                    text = (job.get('text') or '').strip()
                if text:
                    self.interpret(text)
            except Exception as e:  # noqa: BLE001
                self.reply(f'처리 중 문제가 생겼어요: {e}')
                self.get_logger().error(traceback.format_exc())

    def transcribe(self, job):
        if self.whisper is None:
            self.reply('음성 인식(Whisper)이 준비되지 않았어요. 글자로 입력해 주세요.')
            return ''
        raw = base64.b64decode(job.get('data', ''))
        ext = '.ogg' if 'ogg' in job.get('mime', '') else '.mp4' if 'mp4' in job.get('mime', '') else '.webm'
        with tempfile.NamedTemporaryFile(suffix=ext, delete=False) as f:
            f.write(raw)
            path = f.name
        try:
            result = self.whisper.transcribe(path, language=WHISPER_LANG, fp16=self.fp16)
        finally:
            os.remove(path)
        text = result.get('text', '').strip()
        if not text or text.strip('. ') in WHISPER_GHOSTS:
            self.reply('잘 들리지 않았어요. 마이크 가까이에서 다시 말해 주세요.')
            return ''
        self.get_logger().info(f'들은 말: {text}')
        return text

    def interpret(self, text):
        # 멈춤은 LLM을 거치지 않고 바로 (안전 우선)
        if STOP_WORDS.search(text):
            self.send_cmd({'type': 'pause'})
            self.reply('일시정지했어요. 이어서 하려면 "재개"를 누르거나 "다시 시작해"라고 말해 주세요.')
            return

        cmds, answer = self.ask_llm(text)
        if not cmds:
            self.reply(answer or '무엇을 하면 될지 잘 모르겠어요. 예: "묘목 두 그루 심어줘"')
            return
        plan_id = uuid.uuid4().hex[:8]
        with self.lock:
            self.plans[plan_id] = (cmds, time.time())
        summary = ' → '.join(describe(c) for c in cmds)
        self.send_status(type='plan', id=plan_id, summary=summary, detail=answer or '')

    def ask_llm(self, text):
        if USE_FAKE_LLM:
            return self.fake_llm(text)
        if self.claude is None:
            return [], 'LLM(Claude)이 준비되지 않았어요. 노드 터미널의 오류를 확인해 주세요.'
        state = {'robot_busy': self.busy, 'estop': self.estop.is_set(),
                 'cells_1_to_6': [c.get('s') for c in self.scene.get('cells', [])],
                 'berries': [{'id': b['id'], 'ripe': b['ripe'], 'picked': b['picked']}
                             for b in self.scene.get('berries', [])],
                 'supply_left': self.scene.get('supply'), 'harvested': self.scene.get('harvested'),
                 'recent_logs': self.recent_logs}
        resp = self.claude.messages.create(
            model=CLAUDE_MODEL, max_tokens=600,
            system=SYSTEM_PROMPT.format(state=json.dumps(state, ensure_ascii=False)),
            tools=TOOLS, messages=[{'role': 'user', 'content': text}])
        cmds, texts = [], []
        for block in resp.content:
            if block.type == 'tool_use':
                c = tool_to_cmd(block.name, block.input)
                if c:
                    cmds.append(c)
            elif block.type == 'text' and block.text.strip():
                texts.append(block.text.strip())
        return cmds, ' '.join(texts)

    def fake_llm(self, text):
        """API 없이 연결을 시험하기 위한 간단한 규칙"""
        nums = {'한': 1, '하나': 1, '두': 2, '둘': 2, '세': 3, '셋': 3, '네': 4, '넷': 4}
        m = re.search(r'(\d+|한|하나|두|둘|세|셋|네|넷)\s*(그루|개|칸)', text)
        n = (int(m.group(1)) if m.group(1).isdigit() else nums[m.group(1)]) if m else 0
        if re.search(r'수확|따', text):
            return [{'type': 'harvest'}], ''
        if re.search(r'심|식재', text):
            return [{'type': 'plant', 'count': n, 'spacing_mm': DEFAULT_SPACING}], ''
        if re.search(r'분류|익었|성숙', text):
            return [{'type': 'classify'}], ''
        if re.search(r'홈|원위치|제자리', text):
            return [{'type': 'home'}], ''
        if re.search(r'상태|어때', text):
            return [], f'지금 로봇은 {self.busy or "대기"} 중이에요.'
        return [], ''

    # --- 승인 후 실행 ---
    def approve(self, plan_id):
        with self.lock:
            item = self.plans.pop(plan_id, None)
        if not item:
            self.reply('승인할 작업을 찾지 못했어요. 다시 말해 주세요.')
            return
        cmds, created = item
        if time.time() - created > PLAN_TIMEOUT_SEC:
            self.reply('승인 시간이 지나 취소했어요. 다시 말해 주세요.')
            return
        if self.estop.is_set():
            self.reply('비상정지 상태라 실행하지 않았어요. 해제한 뒤 다시 말해 주세요.')
            return
        if self.busy:
            self.reply(f'지금 "{self.busy}" 작업 중이에요. 끝난 뒤 다시 말해 주세요.')
            return
        threading.Thread(target=self.execute, args=(cmds,), daemon=True).start()

    def execute(self, cmds):
        for i, c in enumerate(cmds, 1):
            if self.estop.is_set():
                self.reply('비상정지로 남은 작업을 취소했어요.')
                return
            with self.busy_changed:
                s0, f0 = self.started, self.finished
            self.send_cmd(c)
            if c['type'] not in TASK_TYPES:          # 속도, 재개처럼 바로 끝나는 명령
                continue
            if i == len(cmds):                       # 마지막 작업은 기다릴 필요 없음
                return
            if not self.wait_task(s0, f0):
                self.reply(f'"{describe(c)}" 작업이 끝나지 않아 다음 작업을 멈췄어요.')
                return

    def wait_task(self, s0, f0):
        """보낸 작업이 시작됐다가 끝날 때까지 기다림"""
        with self.busy_changed:
            if not self.busy_changed.wait_for(lambda: self.started > s0, timeout=TASK_START_TIMEOUT):
                return False
            ok = self.busy_changed.wait_for(lambda: self.finished > f0, timeout=TASK_DONE_TIMEOUT)
        return ok and not self.estop.is_set()


# =====================================================================
# 4. 시작
# =====================================================================
def main():
    rclpy.init()
    node = VoiceLlmNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    node.load_models()
    node.get_logger().info('음성·LLM 노드 시작')
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
