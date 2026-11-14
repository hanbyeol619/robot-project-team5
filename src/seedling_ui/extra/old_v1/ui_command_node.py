#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
묘목 식재·열매 수확 관제 UI 명령 처리 노드

흐름:  관제 UI ──(/ui/command, JSON)──▶ 이 노드 ──▶ 두산 로봇 + 카메라(YOLO)
       관제 UI ◀──(/ui/status,  JSON)── 이 노드 (진행 상황, 감지 결과)

처음에는 DRY_RUN = True 로 두고 실행하세요.
로봇을 움직이지 않고 UI ↔ 노드 연결만 시험할 수 있습니다.
"""
import json
import math
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

# 속도 (UI의 속도 %가 여기에 곱해집니다)
LIN_VEL, LIN_ACC = 100.0, 100.0   # 직선 이동 mm/s, mm/s²
JNT_VEL, JNT_ACC = 30.0, 30.0     # 관절 이동 deg/s, deg/s²

# 자세 (관절 각도, 도)
HOME_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
SCAN_A_J = HOME_J                 # 식재 구역을 촬영하는 자세
SCAN_B_J = HOME_J                 # 열매 구역을 촬영하는 자세

# 그리퍼가 바닥을 향하는 자세 (rx, ry, rz). 설치 방향에 따라 다릅니다
TOOL_ORIENT = (0.0, 180.0, 0.0)
SAFE_Z = 300.0                    # 칸 사이를 이동할 때 높이 (mm)

# ⚠ 아래 좌표는 모두 예시입니다. 티칭 펜던트로 실제 위치를 재서 바꾸세요
# 식재 칸 6곳 (로봇 좌표 x, y mm). 순서: 윗줄 왼→오, 아랫줄 왼→오 (UI 칸 순서와 같음)
CELLS_XY = [
    (450.0, -160.0), (450.0, 0.0), (450.0, 160.0),
    (610.0, -160.0), (610.0, 0.0), (610.0, 160.0),
]
PLANT_Z = 120.0                   # 묘목을 내려놓는 높이

# 묘목 공급대 자리 (이 순서대로 하나씩 집어 갑니다)
SUPPLY_XY = [(250.0, -300.0), (250.0, -240.0), (250.0, -180.0),
             (250.0, -120.0), (250.0, -60.0), (250.0, 0.0)]
SUPPLY_PICK_Z = 100.0

BASKET_XYZ = (300.0, 350.0, 200.0)   # 수확함 위치(내려놓는 높이 포함)
BERRY_PICK_Z = 150.0                 # 열매를 집는 높이

# 그리퍼: (디지털 출력 번호, 값) 목록. 그리퍼 종류마다 다릅니다
GRIP_CLOSE_OUTPUTS = [(1, 1), (2, 0)]
GRIP_OPEN_OUTPUTS = [(1, 0), (2, 1)]
GRIP_WAIT_SEC = 0.8

# 비전 (USE_FAKE_VISION = False 일 때 사용)
YOLO_MODEL = 'best.pt'
CAMERA_INDEX = 0
CONF_MIN = 0.5
SEEDLING_CLASSES = {'seedling'}       # YOLO 학습 때 쓴 클래스 이름으로 바꾸세요
RIPE_CLASSES = {'ripe', 'red'}
UNRIPE_CLASSES = {'unripe', 'green'}
# 카메라 화면에서 식재 칸 6곳의 영역 (픽셀 x1, y1, x2, y2), CELLS_XY와 같은 순서
CELL_ROIS = [
    (100, 100, 220, 220), (240, 100, 360, 220), (380, 100, 500, 220),
    (100, 240, 220, 360), (240, 240, 360, 360), (380, 240, 500, 360),
]
BERRY_ROI = (700, 80, 1220, 640)      # 카메라 화면에서 열매 구역 (픽셀)
# 픽셀 → 로봇 좌표 변환 기준점 4쌍 (작업대 위 같은 높이, 같은 촬영 자세에서 측정)
CALIB_PIXELS = [(700, 80), (1220, 80), (1220, 640), (700, 640)]
CALIB_ROBOT = [(400.0, 150.0), (400.0, 400.0), (650.0, 400.0), (650.0, 150.0)]

TASK_NAMES = {
    'scan': '빈 칸 판별', 'plant': '묘목 식재', 'classify': '성숙도 분류',
    'harvest': '열매 수확', 'home': '홈 위치 이동', 'gripper': '그리퍼',
}


class Aborted(Exception):
    """비상정지로 작업을 멈출 때 쓰는 신호"""


# =====================================================================
# 2. UI와 주고받는 노드
# =====================================================================
class UiBridge(Node):
    def __init__(self):
        super().__init__('ui_command_node')
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
        import numpy as np
        from ultralytics import YOLO
        self.cv2, self.np = cv2, np
        self.model = YOLO(YOLO_MODEL)
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            raise RuntimeError(f'카메라 {CAMERA_INDEX}번을 열 수 없습니다')
        self.H = cv2.getPerspectiveTransform(np.float32(CALIB_PIXELS), np.float32(CALIB_ROBOT))

    def _frame(self):
        ok, frame = False, None
        for _ in range(5):                 # 쌓여 있던 오래된 화면 버리기
            ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError('카메라 화면을 읽지 못했습니다')
        return frame

    def _detect(self):
        r = self.model(self._frame(), conf=CONF_MIN, verbose=False)[0]
        out = []
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            out.append((r.names[int(box.cls[0])], float(box.conf[0]), (x1 + x2) / 2, (y1 + y2) / 2))
        return out

    def cell_status(self):
        dets = [d for d in self._detect() if d[0] in SEEDLING_CLASSES]
        result = []
        for x1, y1, x2, y2 in CELL_ROIS:
            inside = [d for d in dets if x1 <= d[2] <= x2 and y1 <= d[3] <= y2]
            if inside:
                result.append({'s': 'planted', 'conf': round(max(d[1] for d in inside) * 100)})
            else:
                result.append({'s': 'empty', 'conf': None})
        return result

    def berries(self):
        bx1, by1, bx2, by2 = BERRY_ROI
        out = []
        for cls, conf, cx, cy in self._detect():
            if cls not in RIPE_CLASSES | UNRIPE_CLASSES or not (bx1 <= cx <= bx2 and by1 <= cy <= by2):
                continue
            x, y = self.cv2.perspectiveTransform(self.np.float32([[[cx, cy]]]), self.H)[0][0]
            out.append({'u': (cx - bx1) / (bx2 - bx1), 'v': (cy - by1) / (by2 - by1),
                        'x': float(x), 'y': float(y), 'ripe': cls in RIPE_CLASSES,
                        'conf': round(conf * 100), 'picked': False})
        out.sort(key=lambda b: (round(b['v'], 1), b['u']))
        for i, b in enumerate(out):
            b['id'] = i
        return out

    def mark_planted(self, i): pass
    def mark_picked(self, i): pass
    def reset(self): pass


class FakeVision:
    """카메라 없이 UI 연결을 시험하기 위한 가짜 감지 결과"""
    fake = True
    BERRIES = [(0.125, 0.19, True), (0.464, 0.216, False), (0.786, 0.279, False),
               (0.107, 0.631, True), (0.721, 0.748, True)]

    def __init__(self):
        self.reset()

    def reset(self):
        self.planted, self.picked = {0, 4}, set()

    def cell_status(self):
        time.sleep(1.0)
        return [{'s': 'planted', 'conf': random.randint(92, 99)} if i in self.planted
                else {'s': 'empty', 'conf': None} for i in range(6)]

    def berries(self):
        time.sleep(1.0)
        return [{'id': i, 'u': u, 'v': v, 'x': 400 + v * 250, 'y': 150 + u * 250, 'ripe': r,
                 'conf': random.randint(88, 99), 'picked': i in self.picked}
                for i, (u, v, r) in enumerate(self.BERRIES)]

    def mark_planted(self, i): self.planted.add(i)
    def mark_picked(self, i): self.picked.add(i)


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
        self.cells = [{'s': 'unknown', 'conf': None} for _ in range(6)]
        self.berries = []
        self.supply_used = 0
        self.harvested = 0
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
            self.ui.log(f'명령 처리 노드와 연결되었습니다: {mode}, {vis}', 'ok')
            self.ui.send(type='state', busy=self.busy)
            self.send_scene()
        elif t == 'reset_counts':
            if self.busy:
                self.ui.log('작업 중에는 초기화할 수 없습니다.', 'warn')
            else:
                self.reset_state()
                self.send_scene()
                self.ui.log('공급대와 수확 기록을 처음 상태로 되돌렸습니다.')
        elif t in TASK_NAMES:
            if self.estop.is_set():
                self.ui.log('비상정지 상태입니다. 해제한 뒤 다시 시도하세요.', 'warn')
            elif self.busy or not self.q.empty():
                now = f'“{self.busy}” 작업이' if self.busy else '앞선 작업이 아직'
                self.ui.log(f'{now} 진행 중입니다. 끝난 뒤 다시 시도하세요.', 'warn')
            else:
                self.q.put(cmd)
        # voice, approve 는 음성(LLM) 노드가 처리하므로 여기서는 무시

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
                for c in self.cells:
                    if c['s'] == 'block':
                        c['s'] = 'empty'
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
        self.ui.send(type='scene', cells=self.cells,
                     berries=[{k: b[k] for k in ('id', 'u', 'v', 'ripe', 'conf', 'picked')}
                              for b in self.berries],
                     supply=len(SUPPLY_XY) - self.supply_used, harvested=self.harvested)

    def focus(self, kind, index=None):
        self.ui.send(type='focus', kind=kind, index=index)

    def step(self, index, state=None):
        self.ui.send(type='step', index=index, state=state)

    # 집기 / 놓기
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
        self.ui.log('그리퍼 닫힘' if close else '그리퍼 열림')

    def task_scan(self, cmd):
        if cmd.get('type') == 'scan':
            self.step('reset')
        self.step(0, 'run')
        self.focus('home')
        self.check(); self.robot.movej(SCAN_A_J)
        self.check(); self.cells = self.vision.cell_status()
        self.send_scene()
        empty = sum(c['s'] == 'empty' for c in self.cells)
        self.ui.log(f'빈 영역 판별 완료: 묘목 {6 - empty}칸, 빈 칸 {empty}칸', 'ok')
        self.step(0, 'done')

    def task_plant(self, cmd):
        count = int(cmd.get('count') or 0)
        spacing = float(cmd.get('spacing_mm') or 150)
        self.step('reset')
        self.task_scan(cmd)

        # 간격 계산
        self.step(1, 'run')
        fixed = [CELLS_XY[i] for i, c in enumerate(self.cells) if c['s'] == 'planted']
        targets = []
        for i, c in enumerate(self.cells):
            if c['s'] != 'empty':
                continue
            others = fixed + [CELLS_XY[j] for j in targets]
            if any(math.dist(CELLS_XY[i], p) < spacing for p in others):
                c['s'] = 'block'
                continue
            if count == 0 or len(targets) < count:
                targets.append(i)
        self.send_scene()
        blocked = sum(c['s'] == 'block' for c in self.cells)
        if blocked:
            self.ui.log(f'간격 {spacing:.0f}mm 기준으로 {blocked}칸은 너무 가까워 건너뜁니다.', 'warn')
        if not targets:
            self.step(1, 'fail')
            raise RuntimeError('심을 수 있는 빈 칸이 없습니다')
        if count and len(targets) < count:
            self.ui.log(f'조건에 맞는 칸이 {len(targets)}칸이라 {len(targets)}그루만 심습니다.', 'warn')
        self.ui.log(f'심을 칸 {len(targets)}곳을 정했습니다.', 'ok')
        self.step(1, 'done')

        # 식재
        self.step(2, 'run')
        for n, i in enumerate(targets, 1):
            if self.supply_used >= len(SUPPLY_XY):
                self.step(2, 'fail')
                raise RuntimeError('공급대에 묘목이 없습니다. 채운 뒤 “작업대 초기화”를 누르세요')
            sx, sy = SUPPLY_XY[self.supply_used]
            self.focus('supply')
            self.pick(sx, sy, SUPPLY_PICK_Z)
            self.supply_used += 1
            self.send_scene()
            self.focus('cell', i)
            self.place(*CELLS_XY[i], PLANT_Z)
            self.cells[i] = {'s': 'planted', 'conf': None}
            self.vision.mark_planted(i)
            self.send_scene()
            self.ui.log(f'묘목 {n}/{len(targets)} 식재 완료')
        self.step(2, 'done')

        # 결과 확인
        self.step(3, 'run')
        self.focus('home')
        self.check(); self.robot.movej(SCAN_A_J)
        self.check(); self.cells = self.vision.cell_status()
        self.send_scene()
        missed = [i + 1 for i in targets if self.cells[i]['s'] != 'planted']
        if missed:
            self.step(3, 'fail')
            self.ui.log(f'결과 확인: {missed}번 칸에서 묘목이 보이지 않습니다. 직접 확인해 주세요.', 'warn')
        else:
            self.step(3, 'done')
            planted = sum(c['s'] == 'planted' for c in self.cells)
            self.ui.log(f'결과 확인 완료: {len(targets)}그루를 심었고, 지금 {planted}/6칸이 차 있습니다.', 'ok')

    def task_classify(self, cmd):
        self.focus('berry_view')
        self.check(); self.robot.movej(SCAN_B_J)
        self.check(); self.berries = self.vision.berries()
        self.send_scene()
        left = [b for b in self.berries if not b['picked']]
        ripe = sum(b['ripe'] for b in left)
        self.ui.log(f'성숙도 분류 완료: 익음 {ripe}개, 덜 익음 {len(left) - ripe}개', 'ok')

    def task_harvest(self, cmd):
        if not DRY_RUN and self.vision.fake:
            raise RuntimeError('가짜 비전 좌표로는 실제 수확을 하지 않습니다. USE_FAKE_VISION = False로 바꾸세요')
        if not any(not b['picked'] for b in self.berries):
            self.task_classify(cmd)
        ids = cmd.get('berry_ids')
        todo = [b for b in self.berries if b['ripe'] and not b['picked'] and (not ids or b['id'] in ids)]
        if not todo:
            raise RuntimeError('수확할 익은 열매가 없습니다')
        for n, b in enumerate(todo, 1):
            self.focus('berry', b['id'])
            self.pick(b['x'], b['y'], BERRY_PICK_Z)
            b['picked'] = True
            self.vision.mark_picked(b['id'])
            self.send_scene()
            self.focus('basket')
            self.place(*BASKET_XYZ)
            self.harvested += 1
            self.send_scene()
            self.ui.log(f'열매 {n}/{len(todo)} 수확 완료')
        self.focus('home')
        self.check(); self.robot.movej(HOME_J)
        self.ui.log(f'수확 완료: 이번에 {len(todo)}개, 누적 {self.harvested}개', 'ok')


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

    # UI 메시지는 별도 스레드에서 받고, 로봇 동작은 메인 스레드에서 실행
    executor = SingleThreadedExecutor()
    executor.add_node(ui)
    threading.Thread(target=executor.spin, daemon=True).start()

    ui.get_logger().info(f'관제 UI 명령 처리 노드 시작 (DRY_RUN={DRY_RUN}, 가짜 비전={vision.fake})')
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
