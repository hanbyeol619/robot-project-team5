#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
묘목 식재 컨트롤러 노드 (관제 UI 명령 처리)

작업대:  보관 묘목 C·D·E (3개) ──▶ 식재 자리 F·G·H·I 중 빈 자리(empty)에 무작위로 옮겨 심기
         A 물뿌리개, B 모종삽은 아직 동작에 쓰지 않음 (UI에도 표시하지 않음)

흐름:  관제 UI ──(/ui/command, JSON)──▶ 이 노드 ──▶ 두산 로봇 + 카메라(YOLO)
       관제 UI ◀──(/ui/status,  JSON)── 이 노드 (진행 상황, 자리 상태)

UI로 보내는 주요 메시지:
  {"type":"spot","id":"G","state":"planted"}      ← 내려놓는 순간 UI에 나무가 생김
  {"type":"scene","spots":{"F":"empty",...},"supply":["D","E"]}
  {"type":"state","busy":"묘목 식재"} / {"type":"state","busy":null}

처음에는 DRY_RUN = True 로 두고 실행하세요.
"""
import json
import queue
import random
import threading
import time
import traceback

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

# =====================================================================
# 1. 설정: 팀 환경에 맞게 이 부분만 고치면 됩니다
# =====================================================================
DRY_RUN = True            # True: 로봇을 움직이지 않고 기록만 남김 (처음엔 꼭 True)
USE_FAKE_VISION = True    # True: 카메라/YOLO 없이 가짜 감지 결과 사용

ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'     # 실제 로봇 모델명으로 바꾸세요
TOOL_NAME = None          # 예: 'Tool Weight_2FG' (티칭 펜던트에 등록된 이름, 없으면 None)
TCP_NAME = None           # 예: '2FG_TCP'

CMD_TOPIC = '/ui/command'
STATUS_TOPIC = '/ui/status'

LIN_VEL, LIN_ACC = 100.0, 100.0   # 직선 이동 mm/s, mm/s² (UI 속도 %가 곱해짐)
JNT_VEL, JNT_ACC = 30.0, 30.0     # 관절 이동 deg/s, deg/s²

HOME_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
SCAN_J = HOME_J                   # 식재 자리를 촬영하는 자세
TOOL_ORIENT = (0.0, 180.0, 0.0)   # 그리퍼가 바닥을 향하는 자세 (rx, ry, rz)
SAFE_Z = 300.0                    # 자리 사이를 이동할 때 높이 (mm)

# ⚠ 아래 좌표는 모두 예시입니다. 티칭 펜던트로 실제 위치를 재서 바꾸세요 (로봇 좌표 x, y mm)
SPOTS_XY = {                      # 식재 자리 4곳 (UI 그림: 윗줄 G H, 아랫줄 F I)
    'G': (450.0, 0.0),   'H': (450.0, 160.0),
    'F': (610.0, 0.0),   'I': (610.0, 160.0),
}
PLANT_Z = 120.0                   # 묘목을 내려놓는 높이
SUPPLY_XY = {                     # 보관 묘목 3개 (이 순서대로 집어 감)
    'C': (610.0, -200.0), 'D': (530.0, -200.0), 'E': (450.0, -200.0),
}
SUPPLY_ORDER = ['C', 'D', 'E']
SUPPLY_PICK_Z = 100.0
# A 물뿌리개, B 모종삽 위치 (지금은 사용하지 않음)
TOOLS_XY = {'A': (300.0, 0.0), 'B': (300.0, 160.0)}

# 그리퍼: (디지털 출력 번호, 값) 목록. 그리퍼 종류마다 다릅니다
GRIP_CLOSE_OUTPUTS = [(1, 1), (2, 0)]
GRIP_OPEN_OUTPUTS = [(1, 0), (2, 1)]
GRIP_WAIT_SEC = 0.8

# 비전 (USE_FAKE_VISION = False 일 때 사용)
YOLO_MODEL = 'best.pt'
CAMERA_INDEX = 0                  # 카메라를 ROS 토픽으로만 쓴다면 이 부분을 토픽 구독으로 바꿔야 함
CONF_MIN = 0.5
SEEDLING_CLASSES = {'seedling', 'tree'}   # YOLO 학습 때 쓴 클래스 이름으로 바꾸세요
# 카메라 화면에서 각 자리의 영역 (픽셀 x1, y1, x2, y2)
SPOT_ROIS = {
    'G': (240, 100, 360, 220), 'H': (380, 100, 500, 220),
    'F': (240, 240, 360, 360), 'I': (380, 240, 500, 360),
}

SPOT_IDS = ['F', 'G', 'H', 'I']
TASK_NAMES = {'scan': '빈 자리 확인', 'plant': '묘목 식재', 'home': '홈 위치 이동', 'gripper': '그리퍼'}


class Aborted(Exception):
    """비상정지로 작업을 멈출 때 쓰는 신호"""


# =====================================================================
# 2. UI와 주고받는 노드
# =====================================================================
class UiBridge(Node):
    def __init__(self):
        super().__init__('seedling_controller')
        self.pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(String, CMD_TOPIC, self.on_command, 10)
        self.worker = None

    def send(self, **data):
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.pub.publish(msg)

    def log(self, text, level='info'):
        self.send(type='log', text=text, level=level)
        (self.get_logger().warn if level in ('warn', 'danger') else self.get_logger().info)(text)

    def on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.log(f'읽을 수 없는 명령입니다: {msg.data}', 'warn')
            return
        if self.worker:
            self.worker.handle(cmd)


# =====================================================================
# 3. 로봇 (두산 DSR 함수 감싸기)
# =====================================================================
class Robot:
    def __init__(self, ui):
        self.ui = ui
        self.ratio = 0.6
        self.dry = DRY_RUN
        if self.dry:
            return

        import DR_init
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        self.dsr_node = rclpy.create_node('ui_robot_worker', namespace=ROBOT_ID)
        DR_init.__dsr__node = self.dsr_node

        import DSR_ROBOT2 as dr
        try:
            from DR_common2 import posx
        except ImportError:
            posx = dr.posx
        self.dr, self.posx = dr, posx
        if TOOL_NAME:
            dr.set_tool(TOOL_NAME)
        if TCP_NAME:
            dr.set_tcp(TCP_NAME)

        # 멈춤·재개·정지는 동작 중에도 바로 보내야 해서 서비스로 따로 호출
        self.cli_stop = self.cli_pause = self.cli_resume = None
        try:
            from dsr_msgs2.srv import MoveStop, MovePause, MoveResume
            self.MoveStop, self.MovePause, self.MoveResume = MoveStop, MovePause, MoveResume
            self.cli_stop = ui.create_client(MoveStop, f'/{ROBOT_ID}/motion/move_stop')
            self.cli_pause = ui.create_client(MovePause, f'/{ROBOT_ID}/motion/move_pause')
            self.cli_resume = ui.create_client(MoveResume, f'/{ROBOT_ID}/motion/move_resume')
        except ImportError:
            ui.log('dsr_msgs2를 불러오지 못해 화면의 정지·일시정지가 로봇에 전달되지 않습니다.', 'danger')

    # --- 이동 ---
    def move(self, x, y, z):
        if self.dry:
            time.sleep(0.4 / self.ratio)
            return
        self.dr.movel(self.posx(x, y, z, *TOOL_ORIENT),
                      vel=LIN_VEL * self.ratio, acc=LIN_ACC * self.ratio)

    def movej(self, j):
        if self.dry:
            time.sleep(0.6 / self.ratio)
            return
        self.dr.movej(j, vel=JNT_VEL * self.ratio, acc=JNT_ACC * self.ratio)

    def grip(self, close):
        if not self.dry:
            for idx, val in (GRIP_CLOSE_OUTPUTS if close else GRIP_OPEN_OUTPUTS):
                self.dr.set_digital_output(idx, self.dr.ON if val else self.dr.OFF)
        time.sleep(GRIP_WAIT_SEC if not self.dry else 0.2)

    # --- 즉시 명령 (다른 스레드에서 호출됨) ---
    def _call(self, cli, req, name):
        if self.dry:
            return
        if cli is None or not cli.service_is_ready():
            self.ui.log(f'로봇 {name} 서비스를 찾지 못했습니다. 티칭 펜던트를 사용하세요.', 'danger')
            return
        cli.call_async(req)

    def stop(self):
        if self.dry:
            return
        req = None
        if self.cli_stop:
            req = self.MoveStop.Request()
            req.stop_mode = 1      # 빠른 정지 (값의 의미는 두산 문서로 확인 필요)
        self._call(self.cli_stop, req, '정지')

    def pause(self):
        if not self.dry:
            self._call(self.cli_pause, self.MovePause.Request() if self.cli_pause else None, '일시정지')

    def resume(self):
        if not self.dry:
            self._call(self.cli_resume, self.MoveResume.Request() if self.cli_resume else None, '재개')


# =====================================================================
# 4. 비전 (YOLO) / 가짜 비전
# =====================================================================
class Vision:
    fake = False

    def __init__(self):
        import cv2
        from ultralytics import YOLO
        self.model = YOLO(YOLO_MODEL)
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            raise RuntimeError(f'카메라 {CAMERA_INDEX}번을 열 수 없습니다')

    def _frame(self):
        ok, frame = False, None
        for _ in range(5):                 # 쌓여 있던 오래된 화면 버리기
            ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError('카메라 화면을 읽지 못했습니다')
        return frame

    def spot_status(self):
        r = self.model(self._frame(), conf=CONF_MIN, verbose=False)[0]
        dets = []
        for box in r.boxes:
            if r.names[int(box.cls[0])] in SEEDLING_CLASSES:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                dets.append(((x1 + x2) / 2, (y1 + y2) / 2, float(box.conf[0])))
        result = {}
        for sid, (x1, y1, x2, y2) in SPOT_ROIS.items():
            inside = [d for d in dets if x1 <= d[0] <= x2 and y1 <= d[1] <= y2]
            result[sid] = ({'s': 'planted', 'conf': round(max(d[2] for d in inside) * 100)} if inside
                           else {'s': 'empty', 'conf': None})
        return result

    def mark_planted(self, sid): pass
    def reset(self): pass


class FakeVision:
    """카메라 없이 UI 연결을 시험하기 위한 가짜 감지 결과 (처음엔 모든 자리가 empty)"""
    fake = True

    def __init__(self):
        self.reset()

    def reset(self):
        self.planted = set()

    def spot_status(self):
        time.sleep(1.0)
        return {sid: ({'s': 'planted', 'conf': random.randint(92, 99)} if sid in self.planted
                      else {'s': 'empty', 'conf': None}) for sid in SPOT_IDS}

    def mark_planted(self, sid):
        self.planted.add(sid)


# =====================================================================
# 5. 작업 처리
# =====================================================================
class Worker:
    def __init__(self, ui, robot, vision):
        self.ui, self.robot, self.vision = ui, robot, vision
        self.q = queue.Queue()
        self.estop = threading.Event()
        self.paused = threading.Event()
        self.busy = None
        self.reset_state()

    def reset_state(self):
        self.spots = {sid: {'s': 'empty', 'conf': None} for sid in SPOT_IDS}
        self.scanned = False
        self.supply = list(SUPPLY_ORDER)
        self.vision.reset()

    # --- UI 명령 받기 (ROS 콜백 스레드) ---
    def handle(self, cmd):
        t = cmd.get('type')
        if t == 'estop':
            self.estop.set()
            self.paused.clear()
            while not self.q.empty():
                self.q.get_nowait()
            self.robot.stop()
            self.ui.log('비상정지! 로봇 정지 명령을 보냈습니다.', 'danger')
        elif t == 'estop_reset':
            self.estop.clear()
            self.ui.log('비상정지를 해제했습니다. 로봇 쪽 복구 상태는 티칭 펜던트에서 확인하세요.', 'warn')
        elif t == 'pause':
            self.paused.set()
            self.robot.pause()
            self.ui.log('일시정지했습니다.', 'warn')
        elif t == 'resume':
            self.paused.clear()
            self.robot.resume()
            self.ui.log('작업을 다시 시작합니다.', 'ok')
        elif t == 'speed':
            self.robot.ratio = min(1.0, max(0.1, float(cmd.get('percent', 60)) / 100))
            self.ui.log(f'속도를 {round(self.robot.ratio * 100)}%로 바꿨습니다.')
        elif t == 'hello':
            mode = '시험 모드(로봇 안 움직임)' if DRY_RUN else '실제 로봇 모드'
            vis = '가짜 비전' if self.vision.fake else 'YOLO 카메라'
            self.ui.log(f'컨트롤러와 연결되었습니다: {mode}, {vis}', 'ok')
            self.ui.send(type='state', busy=self.busy)
            self.send_scene()
        elif t == 'reset_counts':
            if self.busy:
                self.ui.log('작업 중에는 초기화할 수 없습니다.', 'warn')
            else:
                self.reset_state()
                self.send_scene()
                self.ui.log('보관 묘목 C·D·E를 다시 채운 것으로 기록을 되돌렸습니다.')
        elif t in TASK_NAMES:
            if self.estop.is_set():
                self.ui.log('비상정지 상태입니다. 해제한 뒤 다시 시도하세요.', 'warn')
            elif self.busy or not self.q.empty():
                now = f'“{self.busy}” 작업이' if self.busy else '앞선 작업이 아직'
                self.ui.log(f'{now} 진행 중입니다. 끝난 뒤 다시 시도하세요.', 'warn')
            else:
                self.q.put(cmd)
        # voice, voice_audio, approve, reject 는 음성(LLM) 노드가 처리

    # --- 작업 실행 (메인 스레드) ---
    def run(self):
        while rclpy.ok():
            try:
                cmd = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            name = TASK_NAMES[cmd['type']]
            self.busy = name
            self.ui.send(type='state', busy=name)
            try:
                getattr(self, 'task_' + cmd['type'])(cmd)
            except Aborted:
                self.ui.log(f'비상정지로 “{name}” 작업이 중단되었습니다.', 'danger')
            except Exception as e:  # noqa: BLE001
                self.ui.log(f'“{name}” 실패: {e}', 'danger')
                self.ui.get_logger().error(traceback.format_exc())
            finally:
                self.busy = None
                self.ui.send(type='state', busy=None)
                self.send_scene()

    def check(self):
        if self.estop.is_set():
            raise Aborted()
        while self.paused.is_set():
            time.sleep(0.1)
            if self.estop.is_set():
                raise Aborted()

    def send_scene(self):
        spots = self.spots if self.scanned else {sid: v for sid, v in self.spots.items() if v['s'] == 'planted'}
        self.ui.send(type='scene', spots=spots, supply=self.supply)

    def focus(self, kind, sid=None):
        self.ui.send(type='focus', kind=kind, id=sid)

    def step(self, index, state=None):
        self.ui.send(type='step', index=index, state=state)

    def pick(self, x, y, z):
        self.robot.grip(False)
        self.check(); self.robot.move(x, y, SAFE_Z)
        self.check(); self.robot.move(x, y, z)
        self.check(); self.robot.grip(True)
        self.check(); self.robot.move(x, y, SAFE_Z)

    def place(self, x, y, z):
        self.check(); self.robot.move(x, y, SAFE_Z)
        self.check(); self.robot.move(x, y, z)
        self.check(); self.robot.grip(False)
        self.check(); self.robot.move(x, y, SAFE_Z)

    # ---------- 작업들 ----------
    def task_home(self, cmd):
        self.focus('home')
        self.check(); self.robot.grip(False)
        self.check(); self.robot.movej(HOME_J)
        self.ui.log('홈 위치에 도착했습니다.', 'ok')

    def task_gripper(self, cmd):
        close = cmd.get('state') == 'close'
        self.robot.grip(close)
        self.ui.send(type='gripper', state='close' if close else 'open', holding=False)
        self.ui.log('그리퍼 닫힘' if close else '그리퍼 열림')

    def task_scan(self, cmd):
        if cmd.get('type') == 'scan':
            self.step('reset')
        self.step(0, 'run')
        self.focus('home')
        self.check(); self.robot.movej(SCAN_J)
        self.check(); self.spots = self.vision.spot_status()
        self.scanned = True
        self.send_scene()
        empty = [sid for sid in SPOT_IDS if self.spots[sid]['s'] == 'empty']
        self.ui.log(f'빈 자리 확인 완료: empty {len(empty)}곳 ({", ".join(empty) or "없음"})', 'ok')
        self.step(0, 'done')

    def task_plant(self, cmd):
        count = int(cmd.get('count') or 0)
        wanted = [s.upper() for s in (cmd.get('spots') or []) if str(s).upper() in SPOT_IDS]
        self.step('reset')
        self.task_scan(cmd)

        # 자리 고르기: 지정한 자리가 있으면 그중 빈 곳, 없으면 빈 자리 중 무작위
        self.step(1, 'run')
        empty = [sid for sid in SPOT_IDS if self.spots[sid]['s'] == 'empty']
        if wanted:
            skipped = [s for s in wanted if s not in empty]
            if skipped:
                self.ui.log(f'{", ".join(skipped)} 자리는 이미 묘목이 있어 건너뜁니다.', 'warn')
            candidates = [s for s in wanted if s in empty]
        else:
            candidates = random.sample(empty, len(empty))
        if not candidates:
            self.step(1, 'fail')
            raise RuntimeError('심을 수 있는 빈 자리가 없습니다')
        if not self.supply:
            self.step(1, 'fail')
            raise RuntimeError('남은 묘목이 없습니다. 채운 뒤 “작업대 초기화”를 누르세요')
        n = min(count or len(candidates), len(candidates), len(self.supply))
        targets = candidates[:n]
        self.ui.log(f'심을 자리: {", ".join(targets)}', 'ok')
        self.step(1, 'done')

        # 옮겨 심기
        self.step(2, 'run')
        for i, sid in enumerate(targets, 1):
            src = self.supply[0]
            self.focus('supply', src)
            self.pick(*SUPPLY_XY[src], SUPPLY_PICK_Z)
            self.supply.pop(0)
            # 집는 순간: UI에서 보관 나무가 사라지고 팔 끝에 들림
            self.ui.send(type='gripper', state='close', holding=True, supply=self.supply)
            self.focus('spot', sid)
            self.place(*SPOTS_XY[sid], PLANT_Z)
            self.spots[sid] = {'s': 'planted', 'conf': None}
            self.vision.mark_planted(sid)
            self.ui.send(type='gripper', state='open', holding=False)
            self.ui.send(type='spot', id=sid, state='planted', supply=self.supply)   # ← UI에 나무 생김
            self.ui.log(f'묘목 {src} → {sid} 자리 식재 완료 ({i}/{len(targets)})')
        self.step(2, 'done')

        # 결과 확인
        self.step(3, 'run')
        self.focus('home')
        self.check(); self.robot.movej(SCAN_J)
        self.check(); self.spots = self.vision.spot_status()
        self.send_scene()
        missed = [sid for sid in targets if self.spots[sid]['s'] != 'planted']
        if missed:
            self.step(3, 'fail')
            self.ui.log(f'결과 확인: {", ".join(missed)} 자리에서 묘목이 보이지 않습니다. 직접 확인해 주세요.', 'warn')
        else:
            self.step(3, 'done')
            planted = sum(v['s'] == 'planted' for v in self.spots.values())
            self.ui.log(f'결과 확인 완료: {len(targets)}그루를 심었고, 지금 {planted}/4자리에 묘목이 있습니다.', 'ok')


# =====================================================================
# 6. 시작
# =====================================================================
def main():
    rclpy.init()
    ui = UiBridge()
    robot = Robot(ui)

    if USE_FAKE_VISION:
        vision = FakeVision()
    else:
        try:
            vision = Vision()
        except Exception as e:  # noqa: BLE001
            ui.get_logger().error(f'비전 준비 실패, 가짜 비전으로 대신합니다: {e}')
            vision = FakeVision()

    worker = Worker(ui, robot, vision)
    ui.worker = worker

    executor = SingleThreadedExecutor()
    executor.add_node(ui)
    threading.Thread(target=executor.spin, daemon=True).start()

    ui.get_logger().info(f'묘목 식재 컨트롤러 시작 (DRY_RUN={DRY_RUN}, 가짜 비전={vision.fake})')
    try:
        worker.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        ui.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
