#!/usr/bin/env python3
"""묘목 심기 프로젝트 중앙관제(컨트롤) 노드 — 관제 UI v2 (seedling_console_v2) 규약.

  /llm/command  (웨이크워드 → llm_node: plant|stop|unknown)  ─┐
  /ui/command   (관제 UI 버튼 · voice_llm_node 가 승인 후 보냄) ─┼─▶ control_node ──▶ /robot/task (로봇 노드)
  /vision/planting_slots (depth_grid_node S1~S4)              ─┘        │   ◀── /robot/result
                                                                       ├──▶ /robot/stop
                                                                       ├──▶ /ui/status (관제 UI)
                                                                       └──▶ /tts/text

자리 이름 (UI 그림: 윗줄 G H, 아랫줄 F I, 왼쪽에 보관 묘목 C·D·E)
  UI F = 비전 S1 = C-CENTER     UI G = 비전 S2 = D-CENTER
  UI I = 비전 S3 = E-CENTER     UI H = 비전 S4 = F-CENTER
자리를 정하지 않으면 C → D → E → F-CENTER (UI F → G → I → H) 순서로 빈 곳만 채움.

UI 의 'voice', 'voice_audio', 'approve', 'reject' 는 voice_llm_node 가 처리함.
"""
import json
import queue
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 묘목을 잡는 곳 (이 순서로 하나씩 꺼내 씀). 로봇 노드 PICK 과 이름이 같아야 함
PICK_ORDER = ['C', 'D', 'E']

# UI 자리 ↔ 비전 칸 ↔ 실제 이름 (채우는 순서대로)
UI_TO_SLOT = {'F': 'S1', 'G': 'S2', 'I': 'S3', 'H': 'S4'}
SLOT_TO_UI = {v: k for k, v in UI_TO_SLOT.items()}
SLOT_NAME = {'S1': 'C-CENTER', 'S2': 'D-CENTER', 'S3': 'E-CENTER', 'S4': 'F-CENTER'}
TOTAL_SEEDLINGS = 4                          # 작업대에 있는 묘목은 모두 4그루 (판 + 보관)
PICK_NUM = {'C': '5번', 'D': '6번', 'E': '7번'}   # 보관 묘목 화면 번호
NUM_TO_PICK = {'5': 'C', '6': 'D', '7': 'E'}
NUM_TO_UI = {'1': 'H', '2': 'I', '3': 'G', '4': 'F'}   # 화면 번호(왼쪽위 1, 오른쪽위 2, 왼쪽아래 3, 오른쪽아래 4) → UI 자리
SLOT_NUM = {'S4': '1번', 'S3': '2번', 'S2': '3번', 'S1': '4번'}   # 화면 번호 (비전 S1~S4 와 다름)
FILL_ORDER = ['S1', 'S2', 'S3', 'S4']
UI_SPOTS = ['F', 'G', 'H', 'I']

VISION_STATE_TO_UI = {'EMPTY': 'empty', 'OCCUPIED': 'planted', 'UNKNOWN': 'unknown'}

JOB_NAMES = {'plant': '묘목 심기', 'scan': '빈 자리 확인', 'home': '홈 위치 이동', 'gripper': '그리퍼'}


def label(slot_id):
    """'S2' → '2번' (화면 사분면 번호)"""
    return SLOT_NUM.get(slot_id, slot_id)


class Aborted(Exception):
    """정지 명령으로 작업 중단"""


class TaskFailed(Exception):
    """작업 실패 (사유를 메시지로 가짐)"""


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # ---------- 파라미터 ----------
        p = self.declare_parameter
        p('min_valid_fraction', 0.90)    # 이보다 depth 유효 비율이 낮은 칸은 안 고름
        p('vision_timeout_sec', 8.0)     # 새 비전 결과를 기다리는 최대 시간
        p('vision_settle_sec', 2.5)      # 로봇이 빠진 뒤 depth 평균창(20프레임)이 갱신될 시간
        p('verify_after_plant', True)    # 심은 뒤 비전으로 OCCUPIED 확인
        p('default_speed', 0.6)
        p('tts_enabled', True)
        p('stop_latch', True)            # True: 정지 후 UI '비상정지 해제' 전까지 새 작업 거부
        p('timeout_home', 30.0)
        p('timeout_plant', 150.0)        # 잡기→MID→놓기 한 번
        p('timeout_gripper', 15.0)
        p('timeout_water', 400.0)        # 물뿌리개 잡기 → 자리마다 물 주기 → 제자리
        p('water_after_plant', True)     # 다 심은 뒤 A 물뿌리개로 심은 자리에 물 주기

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.min_valid = float(g('min_valid_fraction'))
        self.vision_timeout = float(g('vision_timeout_sec'))
        self.vision_settle = float(g('vision_settle_sec'))
        self.verify = bool(g('verify_after_plant'))
        self.speed = float(g('default_speed'))
        self.tts_enabled = bool(g('tts_enabled'))
        self.stop_latch = bool(g('stop_latch'))

        # ---------- 통신 ----------
        self.ui_pub = self.create_publisher(String, '/ui/status', 10)
        self.tts_pub = self.create_publisher(String, '/tts/text', 10)
        self.task_pub = self.create_publisher(String, '/robot/task', 10)
        self.stop_pub = self.create_publisher(String, '/robot/stop', 10)

        self.create_subscription(String, '/llm/command', self.on_llm, 10)
        self.create_subscription(String, '/ui/command', self.on_ui, 10)
        self.create_subscription(String, '/vision/planting_slots', self.on_vision, 10)
        self.create_subscription(String, '/robot/result', self.on_robot_result, 10)
        self.create_subscription(String, '/voice/event', self.on_voice_event, 10)

        # ---------- 상태 ----------
        self.lock = threading.Lock()
        self.jobs = queue.Queue()
        self.abort = threading.Event()
        self.paused = threading.Event()
        self.busy = None                 # 진행 중인 작업 이름
        self.vision = None               # 마지막 /vision/planting_slots payload
        self.vision_rx = 0.0             # 마지막 수신 시각 (monotonic)
        self.vision_cond = threading.Condition()
        self.pending = {}                # robot task id -> {'event', 'result'}
        self.picked = set()              # 이미 옮긴 보관 묘목 (C/D/E)
        self.job_supply = None           # 작업 시작 때 카메라로 본 보관 묘목
        self.scanned = False
        self.live_last = None            # UI 로 마지막에 보낸 자리 상태 (실시간 동기화)
        self.live_seen = None            # 직전 검사 때 본 상태 (두 번 연속 같아야 반영)
        self.create_timer(1.0, self.live_sync)

        self.worker = threading.Thread(target=self.run_jobs, daemon=True)
        self.worker.start()
        self.get_logger().info('control_node 시작: /llm/command, /ui/command 대기 중')

    # =================================================================
    # 출력 도우미
    # =================================================================
    def ui(self, **data):
        self.ui_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def log(self, text, level='info'):
        self.ui(type='log', text=text, level=level)
        (self.get_logger().warn if level in ('warn', 'danger') else self.get_logger().info)(text)

    def say(self, text):
        """음성 안내 (UI 채팅 말풍선에도 표시)"""
        if self.tts_enabled:
            self.tts_pub.publish(String(data=text))
        self.ui(type='reply', text=text)

    def step(self, index, state=None):
        self.ui(type='step', index=index, state=state)

    def send_state(self):
        self.ui(type='state', busy=self.busy)

    def vision_supply(self):
        """카메라로 본 보관 묘목 (있는 것 목록). 비전 정보가 없거나 애매하면 None"""
        sup = (self.vision or {}).get('supply')
        if not sup or time.monotonic() - self.vision_rx > 3.0:
            return None
        st = {s.get('id'): s.get('status') for s in sup}
        if any(st.get(p) not in ('PRESENT', 'EMPTY') for p in PICK_ORDER):
            return None
        return [p for p in PICK_ORDER if st[p] == 'PRESENT']

    def supply(self):
        """남은 보관 묘목. 작업 중에는 (팔이 가려서) 시작 때 본 것에서 옮긴 것을 빼고, 쉴 때는 카메라 그대로"""
        if self.busy and self.job_supply is not None:
            return [p for p in self.job_supply if p not in self.picked]
        return self.idle_supply()

    def idle_supply(self):
        vs = self.vision_supply()
        base = vs if vs is not None else [p for p in PICK_ORDER if p not in self.picked]
        n = self.supply_count()
        if n is None or len(base) == n:
            return base
        # 묘목은 모두 4그루: 남은 수 = 4 - 판에 심어진 수 (카메라 보관 자리 판단보다 이걸 우선)
        rest = [p for p in PICK_ORDER if p not in base]
        return (base + rest)[:n] if n > len(base) else base[:n]

    def supply_count(self):
        """판(1~4번)에 심어진 수로 센 남은 보관 묘목 수. 비전이 없거나 애매하면 None"""
        if self.vision is None or time.monotonic() - self.vision_rx > 3.0:
            return None
        st = [s.get('status') for s in self.slots()]
        if len(st) != 4 or any(s not in ('OCCUPIED', 'EMPTY') for s in st):
            return None
        return max(0, min(len(PICK_ORDER), TOTAL_SEEDLINGS - st.count('OCCUPIED')))

    def spot_states(self):
        """비전 결과 → {'F': {'s': 'empty', 'conf': None}, ...}"""
        out = {}
        for s in self.slots():
            ui_id = SLOT_TO_UI.get(s.get('id'))
            if ui_id:
                out[ui_id] = {'s': VISION_STATE_TO_UI.get(s.get('status'), 'unknown'), 'conf': None}
        return out

    def live_sync(self):
        """작업 중이 아닐 때 카메라에 보이는 대로 UI 자리 상태를 맞춘다 (사람이 묘목을 빼거나 넣어도 반영).
        로봇 작업 중에는 팔 그림자 때문에 건너뛰고, 2초 연속 같은 상태일 때만 보낸다."""
        if self.busy or self.vision is None or time.monotonic() - self.vision_rx > 3.0:
            self.live_seen = None
            return
        now = {k: v['s'] for k, v in self.spot_states().items() if v['s'] != 'unknown'}
        now['supply'] = self.supply()
        if now != self.live_seen:
            self.live_seen = now
            return
        if now != self.live_last:
            self.live_last = now
            self.scanned = True
            self.send_scene()

    def send_scene(self):
        spots = self.spot_states()
        if not self.scanned:   # 아직 확인 전이면 심어진 자리만 보여 줌 (UI 원본과 같게)
            spots = {k: v for k, v in spots.items() if v['s'] == 'planted'}
        self.ui(type='scene', spots=spots, supply=self.supply())

    # =================================================================
    # 입력 콜백 (ROS 스레드: 오래 걸리는 일은 하지 않는다)
    # =================================================================
    def on_llm(self, msg):
        """웨이크워드(HELLO KERNEL) → llm_node 경로: 승인 없이 바로 실행"""
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.log(f'LLM 명령을 읽을 수 없음: {msg.data}', 'warn')
            return
        action = str(cmd.get('action', 'unknown')).lower()
        text = cmd.get('text', '')
        if text:
            self.ui(type='heard', text=text)
        self.get_logger().info(f'LLM 명령: {action} ({text})')

        if action == 'stop':
            self.do_stop('음성 정지')
        elif action == 'plant':
            self.submit({'type': 'plant', 'count': 0}, source='voice')   # 0 = 빈 자리 모두
        else:
            self.say('잘 못 알아들었어요. "묘목 심어"라고 말해 주세요.')

    def on_ui(self, msg):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.log(f'읽을 수 없는 UI 명령: {msg.data}', 'warn')
            return
        t = cmd.get('type')
        if t == 'estop':
            self.do_stop('비상정지')
        elif t == 'estop_reset':
            self.abort.clear()
            self.log('비상정지를 해제했습니다. 주변 안전을 확인한 뒤 홈 위치로 이동하세요.', 'warn')
        elif t == 'pause':
            self.paused.set()
            self.stop_pub.publish(String(data=json.dumps({'mode': 'pause'})))
            self.log('일시정지했습니다.', 'warn')
        elif t == 'resume':
            self.paused.clear()
            self.stop_pub.publish(String(data=json.dumps({'mode': 'resume'})))
            self.log('작업을 다시 시작합니다.', 'ok')
        elif t == 'speed':
            self.speed = min(1.0, max(0.1, float(cmd.get('percent', 60)) / 100.0))
            self.log(f'속도를 {round(self.speed * 100)}%로 바꿨습니다.')
        elif t == 'hello':
            self.log('컨트롤 노드와 연결되었습니다 (실제 비전 · 로봇 노드).', 'ok')
            self.send_state()
            self.send_scene()
        elif t == 'reset_counts':
            if self.busy:
                self.log('작업 중에는 초기화할 수 없습니다.', 'warn')
            else:
                self.picked = set()
                self.send_scene()
                self.log('보관 묘목 5·6·7번을 다시 채운 것으로 기록을 되돌렸습니다.', 'ok')
        elif t in JOB_NAMES:
            self.submit(cmd, source='ui')
        # voice / voice_audio / approve / reject 는 voice_llm_node 가 처리

    def on_voice_event(self, msg):
        if msg.data == 'wakeup':
            self.log('🎙 웨이크워드 감지, 듣는 중…')
        elif msg.data == 'stt_failed':
            self.log('음성 인식에 실패했습니다.', 'warn')

    def on_vision(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.vision_cond:
            self.vision = data
            self.vision_rx = time.monotonic()
            self.vision_cond.notify_all()

    def on_robot_result(self, msg):
        try:
            res = json.loads(msg.data)
        except json.JSONDecodeError:
            self.log(f'로봇 결과를 읽을 수 없음: {msg.data}', 'warn')
            return
        with self.lock:
            entry = self.pending.get(res.get('id'))
        if entry is None:
            return  # 다른 작업(이미 타임아웃/정지된 것)의 응답
        entry['result'] = res
        entry['event'].set()

    # =================================================================
    # 작업 접수 / 정지
    # =================================================================
    def submit(self, cmd, source):
        t = cmd['type']
        if self.abort.is_set():
            self.log('비상정지 상태입니다. 해제한 뒤 다시 시도하세요.', 'warn')
            if source == 'voice':
                self.say('정지 상태라서 실행할 수 없어요.')
            return
        if self.busy or not self.jobs.empty():
            now = f'“{self.busy}” 작업이' if self.busy else '앞선 작업이 아직'
            self.log(f'{now} 진행 중입니다. 끝난 뒤 다시 시도하세요.', 'warn')
            if source == 'voice':
                self.say('지금 다른 작업 중이에요. 끝나면 다시 말해 주세요.')
            return
        self.jobs.put(cmd)

    def do_stop(self, reason):
        self.abort.set()
        self.paused.clear()
        while not self.jobs.empty():
            try:
                self.jobs.get_nowait()
            except queue.Empty:
                break
        self.stop_pub.publish(String(data=json.dumps({'mode': 'stop'})))
        with self.lock:
            for entry in self.pending.values():
                entry['event'].set()
        with self.vision_cond:
            self.vision_cond.notify_all()
        self.log(f'{reason}! 로봇에 정지 명령을 보냈습니다.', 'danger')
        self.say('정지합니다.')

    # =================================================================
    # 작업 스레드
    # =================================================================
    def run_jobs(self):
        while rclpy.ok():
            try:
                cmd = self.jobs.get(timeout=0.2)
            except queue.Empty:
                if not self.stop_latch and self.abort.is_set():
                    self.abort.clear()
                continue
            t = cmd['type']
            self.busy = JOB_NAMES[t]
            self.send_state()
            try:
                getattr(self, 'job_' + t)(cmd)
            except Aborted:
                self.log(f'비상정지로 “{self.busy}” 작업이 중단되었습니다.', 'danger')
            except TaskFailed as e:
                self.log(f'“{self.busy}” 실패: {e}', 'danger')
                self.say(f'작업에 실패했어요. {e}')
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'예상하지 못한 오류: {e!r}')
                self.log(f'“{self.busy}” 오류: {e}', 'danger')
            finally:
                self.busy = None
                self.send_state()
                self.send_scene()

    def check(self):
        if self.abort.is_set():
            raise Aborted()
        while self.paused.is_set():
            time.sleep(0.1)
            if self.abort.is_set():
                raise Aborted()

    # ---------- 로봇 ----------
    def robot(self, action, slot=None, **extra):
        """로봇 노드에 작업 하나를 보내고 끝날 때까지 기다린다."""
        self.check()
        task_id = uuid.uuid4().hex[:8]
        entry = {'event': threading.Event(), 'result': None}
        with self.lock:
            self.pending[task_id] = entry
        task = {'id': task_id, 'action': action, 'speed': self.speed}
        if slot is not None:
            task['slot'] = slot
        task.update(extra)
        self.task_pub.publish(String(data=json.dumps(task, ensure_ascii=False)))
        self.get_logger().info(f'→ robot: {task}')

        timeout = float(self.get_parameter(f'timeout_{action}').value)
        deadline = time.monotonic() + timeout
        try:
            while not entry['event'].wait(0.1):
                if self.abort.is_set():
                    raise Aborted()
                if self.paused.is_set():          # 일시정지 동안은 시간 안 셈
                    deadline += 0.1
                if time.monotonic() > deadline:
                    raise TaskFailed(f'로봇이 {timeout:.0f}초 안에 “{action}”을 끝내지 못했습니다')
            self.check()
            res = entry['result'] or {}
            if not res.get('ok', False):
                raise TaskFailed(res.get('error') or f'로봇 “{action}” 실패')
            return res
        finally:
            with self.lock:
                self.pending.pop(task_id, None)

    # ---------- 비전 ----------
    def slots(self):
        return (self.vision or {}).get('slots', [])

    def wait_fresh_vision(self):
        """지금부터 settle 초 이후에 들어온 비전 결과를 기다린다(로봇 팔 그림자 제거)."""
        after = time.monotonic() + self.vision_settle
        deadline = after + self.vision_timeout
        with self.vision_cond:
            while self.vision_rx < after:
                self.check()
                left = deadline - time.monotonic()
                if left <= 0:
                    raise TaskFailed('비전 결과(/vision/planting_slots)가 들어오지 않습니다')
                self.vision_cond.wait(min(left, 0.2))
        self.check()
        return self.slots()

    def empty_slots(self, slots):
        """EMPTY 칸을 C → D → E → F-CENTER 순서로"""
        ok = {s.get('id') for s in slots
              if s.get('status') == 'EMPTY' and s.get('valid_fraction', 0) >= self.min_valid}
        return [sid for sid in FILL_ORDER if sid in ok]

    # =================================================================
    # 작업들
    # =================================================================
    def job_home(self, cmd):
        self.ui(type='focus', kind='home')
        self.robot('home')
        self.log('홈 위치에 도착했습니다.', 'ok')

    def job_gripper(self, cmd):
        state = 'close' if cmd.get('state') == 'close' else 'open'
        self.robot('gripper', state=state)
        self.ui(type='gripper', state=state, holding=False)
        self.log('그리퍼 닫힘' if state == 'close' else '그리퍼 열림')

    def scan(self):
        self.step(0, 'run')
        self.ui(type='focus', kind='home')
        self.robot('home')                       # 카메라 시야에서 팔 빼기
        slots = self.wait_fresh_vision()
        self.scanned = True
        self.send_scene()
        empty = self.empty_slots(slots)
        names = ', '.join(label(s) for s in empty) or '없음'
        self.log(f'빈 자리 확인 완료: empty {len(empty)}곳 ({names})', 'ok')
        self.step(0, 'done')
        return slots

    def job_scan(self, cmd):
        self.step('reset')
        self.scan()

    def job_plant(self, cmd):
        count = max(0, int(cmd.get('count') or 0))   # 0 = 빈 자리 모두
        wanted = []                                   # 자리: UI 글자(F/G/H/I) 또는 화면 번호(1~4)
        for s in cmd.get('spots') or []:
            s = str(s).strip().upper().rstrip('번')
            s = NUM_TO_UI.get(s, s)
            if s in UI_TO_SLOT and UI_TO_SLOT[s] not in wanted:
                wanted.append(UI_TO_SLOT[s])
        picks = []                                    # 보관 묘목: C/D/E 또는 화면 번호(5~7)
        for p in cmd.get('picks') or []:
            p = str(p).strip().upper().rstrip('번')
            p = NUM_TO_PICK.get(p, p)
            if p in PICK_ORDER and p not in picks:
                picks.append(p)
        self.step('reset')
        self.job_supply = None
        self.say('묘목 심기를 시작할게요. 빈 자리를 확인하고 있어요.')
        slots = self.scan()                      # HOME 에서 비전 확인 (보관 묘목도 같이 봄)
        vs = self.vision_supply()
        if vs is not None:                       # 잡으러 갈 곳: 카메라(깊이)로 실제 묘목이 보이는 5·6·7번만
            self.job_supply = vs
            self.log(f'보관 묘목 확인: {", ".join(PICK_NUM[p] for p in vs) or "없음"}', 'ok')
        else:
            self.job_supply = self.idle_supply()
            self.log('보관 묘목을 카메라로 확인하지 못해 남은 수로 셉니다.', 'warn')
        self.picked = set()
        missing = [p for p in picks if p not in self.job_supply]
        if missing:
            self.step(1, 'fail')
            names = ', '.join(PICK_NUM[p] for p in missing)
            self.say(f'{names} 묘목이 보관 자리에 없어요.')
            raise TaskFailed(f'{names} 묘목이 보관 자리에 없습니다')
        left = len(self.job_supply)
        if left <= 0:
            self.step(1, 'fail')
            self.say('남은 묘목이 없어요.')
            raise TaskFailed('남은 묘목이 없습니다. 보관 자리에 묘목을 채워 주세요')
        self.send_scene()

        # 1. 심을 자리 고르기
        self.step(1, 'run')
        empty = self.empty_slots(slots)
        if wanted:
            full = [s for s in wanted if s not in empty]
            if full:
                names = ', '.join(SLOT_NUM[s] for s in full)
                self.log(f'{", ".join(label(s) for s in full)} 자리에 묘목이 있어 심을 수 없습니다.', 'warn')
                self.say(f'{names} 자리에는 묘목이 있어서 심을 수 없어요.')
            targets = [s for s in wanted if s in empty]
            if not targets:
                self.step(1, 'fail')
                self.robot('home')
                return
        else:
            targets = empty
        if not targets:
            self.step(1, 'fail')
            self.say('빈 자리가 없어서 홈에서 기다릴게요.')
            self.log('심을 수 있는 빈 자리가 없습니다. HOME 대기', 'warn')
            return
        if not count and picks and not wanted:
            count = len(picks)                   # "5번 묘목 심어줘" → 한 그루
        n = min(count or len(targets), len(targets), left)
        if n < len(targets) and n == left:
            self.log(f'남은 묘목이 {left}개라 {left}곳만 심습니다.', 'warn')
        targets = targets[:n]
        # 자리마다 잡을 묘목: 말한 묘목 먼저, 나머지는 남은 것 차례로
        rest = [p for p in self.supply() if p not in picks]
        pick_for = (picks + rest)[:len(targets)]
        self.log(f'심을 자리: {", ".join(label(s) for s in targets)}', 'ok')
        self.say(f'{len(targets)}곳에 심을게요.')
        self.step(1, 'done')

        # 2. 식재: 묘목 하나씩 C→D→E 에서 잡아 MID 를 거쳐 빈 자리에 놓기
        self.step(2, 'run')
        for i, (slot, pick) in enumerate(zip(targets, pick_for), 1):
            ui_id = SLOT_TO_UI[slot]
            self.ui(type='focus', kind='supply', id=pick)
            self.say(f'{PICK_NUM[pick]} 묘목을 {SLOT_NUM[slot]} 자리에 심을게요.')
            self.log(f'묘목 {i}/{len(targets)}: {PICK_NUM[pick]} 묘목 → {label(slot)} 자리')
            try:
                self.robot('plant', slot=slot, pick=pick)
            except TaskFailed:
                self.step(2, 'fail')
                raise
            self.picked.add(pick)
            self.ui(type='gripper', state='open', holding=False, supply=self.supply())
            self.ui(type='spot', id=ui_id, state='planted', supply=self.supply())
            self.log(f'{PICK_NUM[pick]} 묘목 → {label(slot)} 자리 식재 완료 ({i}/{len(targets)})', 'ok')
        self.step(2, 'done')

        # 2-1. 물 주기: A 물뿌리개를 잡아 이번에 심은 자리마다 물 주고 제자리에 둠
        if bool(self.get_parameter('water_after_plant').value):
            self.say('이제 물을 줄게요.')
            self.log(f'물 주기: {", ".join(label(s) for s in targets)} 자리', 'info')
            self.robot('water', slots=list(targets))
            self.log('물 주기 완료', 'ok')

        # 3. 결과 확인 (HOME 에서 다시 촬영)
        self.step(3, 'run')
        self.ui(type='focus', kind='home')
        self.robot('home')
        if not self.verify:
            self.step(3, 'done')
            self.say('묘목 심기를 완료했어요.')
            return
        slots = self.wait_fresh_vision()
        self.send_scene()
        state = {s.get('id'): s.get('status') for s in slots}
        missed = [s for s in targets if state.get(s) != 'OCCUPIED']
        if missed:
            self.step(3, 'fail')
            names = ', '.join(label(s) for s in missed)
            self.log(f'결과 확인: {names} 자리에서 묘목이 보이지 않습니다. 직접 확인해 주세요.', 'warn')
            self.say(f'{", ".join(SLOT_NUM[s] for s in missed)} 자리는 확인이 안 돼요. 직접 봐 주세요.')
        else:
            self.step(3, 'done')
            planted = sum(v == 'OCCUPIED' for v in state.values())
            self.log(f'결과 확인 완료: {len(targets)}그루를 심었고, 지금 {planted}/4자리에 묘목이 있습니다.', 'ok')
            self.say('묘목 심기를 완료했어요.')


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
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
