"""묘목심기 로봇 노드 (두산 e0509) — control_node 의 mock_robot_node 자리에 들어가요.

받는 것   /robot/task  (JSON)  {"id", "action", "speed", "slot": "S1"~"S4", "pick": "C"/"D"/"E"(없어도 됨)}
          /robot/stop  (JSON)  {"mode": "stop" | "pause" | "resume"}
보내는 것 /robot/result (JSON) {"id", "action", "ok", "error"?, "pick"?, "slot"?}

action
  home      HOME 자세로 이동 (처음·끝에 카메라 시야에서 팔 빼기)
  plant     묘목 하나 옮기기:
              잡는 곳(C→D→E 차례) 20cm 위 → 내려가 잡기 → 20cm 위 → MID
              → 빈자리(○-CENTER) 20cm 위 → 내려가 놓기 → 20cm 위 → MID
            묘목 잡기 전에는 빠르게, 잡고 있을 때는 천천히 움직여요.
  dig / water / press   지금은 로봇 동작 없음 (바로 성공으로 답해요)
  gripper   {"state": "open"/"close"}
  reset_picks  잡는 순서를 처음(C)으로 되돌림

비전 칸 → 놓는 곳:  S1 = C-CENTER, S2 = D-CENTER, S3 = E-CENTER, S4 = F-CENTER

실행
  ros2 run robot_control seedling_robot
  ros2 run robot_control seedling_robot --ros-args -p check_only:=true   ← 계산만, 안 움직임
"""
import json
import queue
import threading
import time

import rclpy
import DR_init
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ── 티칭 자세 (관절 각도, deg) — Creator3D 에서 찍은 값 ──────────────────
HOME = [0, -33.98, 114.14, 0, 99.83, 0]
MID = [0, 37.15, 15.87, 0, 126.97, 0]

PICK = {   # 묘목을 잡는 높이의 자세
    "C": [-20.44, -16, 116, 0, 79.99, -20.44],
    "D": [10.09, -18.16, 117.52, -0.01, 80.63, 10.08],
    "E": [40.89, -5.69, 107.67, -0.01, 78.02, 40.88],
}
PICK_ORDER = ["C", "D", "E"]

PLACE = {  # 묘목을 놓는 높이의 자세 (비전 칸 번호 → 위치)
    # 베이스 좌표 (fkin): C(408,-92) D(401,172) E(658,-96) F(633,149) mm
    # ※ 처음 받은 코드에서 D-CENTER 와 E-CENTER 자세 이름이 서로 바뀌어 있었어요 (11-12 실제로 확인)
    "S1": [-12.75, 8.27, 93.63, 0, 78.1, -12.74],     # C-CENTER (가까운 아래)
    "S2": [23.29, 10.59, 89.57, -0.05, 79.75, 23.28],  # D-CENTER (가까운 위)  ← 원래 'E-CENTER' 로 적힌 값
    "S3": [-8.33, 54.06, 22.67, 0.01, 103.22, -8.34],  # E-CENTER (먼 아래)    ← 원래 'D-CENTER' 로 적힌 값
    "S4": [13.23, 49.07, 31.6, -0.05, 99.12, 13.2],    # F-CENTER (먼 위)
}
# 가까운 자리(S1=C-CENTER 화면 4번, S2=D-CENTER 화면 3번, x≈400mm)는 MID(x≈523mm)까지 갔다 오면
# 지나쳐서 되돌아오는 동작이 생겨요 → MID 와 같은 높이·y 에서 x 만 이 값으로 줄인 경유점을 써요 (mm)
NEAR_VIA_X = {"S1": 380.0, "S2": 380.0, "S3": 440.0, "S4": 440.0}   # 먼 자리도 MID(523)에서 당김: MID 위치는 높이면 J5 한계 넘음
# 경유점 높이 올림 (mm): 묘목 들고 지나갈 때 다른 묘목 손잡이에 닿지 않게 (11-14, MID z≈554 → 634)
VIA_DZ = 40.0   # 4cm (x=380 → J5≈121°, x=440 → J5≈125°, 한계 130°)

# 놓는 높이 보정 (mm, 티칭 자세보다 아래로): 화면 1번(S4 F-CENTER)·2번(S3 E-CENTER) 2cm 낮춤 (11-14)
PLACE_DZ = {"S3": -20.0, "S4": -20.0}

# 화면(UI) 번호로 이름 붙임 (11-14): S1 C-CENTER=4번, S2 D-CENTER=3번, S3 E-CENTER=2번, S4 F-CENTER=1번
PLACE_NAME = {"S1": "4번 자리", "S2": "3번 자리", "S3": "2번 자리", "S4": "1번 자리"}
PICK_NAME = {"C": "5번 묘목", "D": "6번 묘목", "E": "7번 묘목"}

# ── 물 주기 (11-14) ──────────────────────────────────────────────
WATER_CAN = [-55.52, 41.78, 74.34, 0, 63.88, -55.52]   # A 물뿌리개 잡는 자세 (fkin ≈ x330, y-481, z146)
WATER_DY = -200.0     # 나무 위치에서 -y 쪽으로 (mm) — 11-14 10cm → 30cm → 20cm
# 먼 자리(화면 1번 S4 F-CENTER, 2번 S3 E-CENTER)는 -y 로 빼면 팔이 안 닿아서 (11-14)
# 그리퍼를 위에서 볼 때 시계방향 90° 돌려 주둥이를 +x 로 향하게 하고 -x 쪽으로 뺌
WATER_ROTATED = {"S3", "S4"}
WATER_DX_ROTATED = -200.0   # 나무 위치에서 -x 쪽으로 (mm)
WATER_YAW_ROTATED = -90.0   # 위에서 볼 때 시계방향 90° (베이스 z축 기준)
WATER_DZ = 70.0       # 나무를 놓는 높이보다 위로 (mm)
WATER_TILT = [0.0, 0.0, 0.0, -90.0, 3.0, 90.0]   # 기울이기: 공구 X축으로 3° (ZYZ 상대 회전). 반대로 기울면 3 → -3
WATER_APPROACH = 70.0  # 물 주는 곳 위 7cm 에서 내려옴 (11-14: 20cm 위면 C-CENTER 에서 J5 131° 로 한계 초과)
WATER_SQUEEZES = 2    # 기울인 채 꽉 잡았다 푸는 횟수
WATER_SEC = 2.0       # 기울인 채 기다리는 시간 (초)

# 그리퍼 잡는 세기 = 닫는 위치 (0 열림 ~ 700 끝까지 닫힘). 작을수록 살살 잡음 (11-14: 700 은 너무 셈)
# 묘목은 원래대로 끝까지 닫음 (gripper/close)
GRIP_CAN = 450        # 물뿌리개 들고 다닐 때
GRIP_CAN_TILT = 470   # 기울일 때 살짝 더 꽉
WATER_DIRECT = {"S1", "S2"}   # 화면 4·3번(가까운 자리)은 물 주러 갈 때 경유점 없이 바로

LIFT_MM = 200.0     # 잡기/놓기 전에 위로 올라가 있는 높이 (20cm)
GRIP_WAIT = 1.0     # 그리퍼 동작 후 기다리는 시간 (초)

# 속도 (speed=1.0 일 때 값. control_node 가 보내는 speed(기본 0.6)를 곱해요)
FAST_J = 60.0       # 빈손 관절 이동 (deg/s)
SLOW_J = 20.0       # 묘목 들고 관절 이동 (deg/s)
FAST_L = 200.0      # 빈손 직선 이동 (mm/s)
SLOW_L = 60.0       # 묘목 들고 직선 이동 (mm/s)
DOWN_L = 80.0       # 빈손으로 잡으러 내려갈 때 (mm/s)


class Stopped(Exception):
    """정지 명령으로 중단"""


class Failed(Exception):
    """동작 실패 (사유)"""


class RobotIO(Node):
    """토픽 송수신 + 정지/일시정지 — 로봇이 움직이는 동안에도 따로 돌아요."""

    def __init__(self, tasks):
        super().__init__("seedling_robot_io")
        self.tasks = tasks
        self.stopped = threading.Event()
        self.paused = threading.Event()
        self.result_pub = self.create_publisher(String, "/robot/result", 10)
        self.create_subscription(String, "/robot/task", self.on_task, 10)
        self.create_subscription(String, "/robot/stop", self.on_stop, 10)
        from dsr_msgs2.srv import MoveStop, MovePause, MoveResume
        self.MoveStop = MoveStop
        self.stop_cli = self.create_client(MoveStop, f"/{ROBOT_ID}/motion/move_stop")
        self.pause_cli = self.create_client(MovePause, f"/{ROBOT_ID}/motion/move_pause")
        self.resume_cli = self.create_client(MoveResume, f"/{ROBOT_ID}/motion/move_resume")

    def on_task(self, msg):
        try:
            task = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"읽을 수 없는 task: {msg.data}")
            return
        self.tasks.put(task)

    def on_stop(self, msg):
        try:
            mode = json.loads(msg.data).get("mode")
        except (json.JSONDecodeError, AttributeError):
            return
        if mode == "stop":
            self.paused.clear()
            self.stopped.set()
            while not self.tasks.empty():
                try:
                    self.reply(self.tasks.get_nowait(), False, "정지 명령으로 취소")
                except queue.Empty:
                    break
            self.call(self.stop_cli, "stop")
        elif mode == "pause":
            self.paused.set()
            self.call(self.pause_cli, "pause")
        elif mode == "resume":
            self.paused.clear()
            self.call(self.resume_cli, "resume")
        self.get_logger().warn(f"/robot/stop: {mode}")

    def call(self, client, name):
        if not client.service_is_ready():
            self.get_logger().error(f"move_{name} 서비스가 없어요 — 필요하면 비상정지 버튼을 쓰세요")
            return
        req = client.srv_type.Request()
        if name == "stop":
            req.stop_mode = 1   # DR_QSTOP (빠른 정지, 전원 유지)
        client.call_async(req)

    def reply(self, task, ok, error=None, **extra):
        res = {"id": task.get("id"), "action": task.get("action"), "ok": ok}
        if error:
            res["error"] = error
        res.update(extra)
        self.result_pub.publish(String(data=json.dumps(res, ensure_ascii=False)))
        self.get_logger().info(f"결과: {res}")


class SeedlingRobot:
    """실제 동작 — 메인 스레드에서 DSR_ROBOT2 로 로봇을 움직여요."""

    def __init__(self, dsr_node, io, dr, guard, check_only):
        self.node, self.io, self.dr, self.guard = dsr_node, io, dr, guard
        self.check_only = check_only
        self.next_pick = 0
        self.grip = {
            "open": dsr_node.create_client(Trigger, "gripper/open"),
            "close": dsr_node.create_client(Trigger, "gripper/close"),
        }
        self.grip_pub = dsr_node.create_publisher(Int32, "gripper/position_cmd", 10)

    # ---------- 도우미 ----------
    def log(self, text):
        self.io.get_logger().info(text)

    def check(self):
        if self.io.stopped.is_set():
            raise Stopped()
        while self.io.paused.is_set():
            time.sleep(0.1)
            if self.io.stopped.is_set():
                raise Stopped()

    def fk(self, j, name):
        x = self.dr.fkin(j, self.dr.DR_BASE)
        if isinstance(x, int):
            raise Failed(f"{name} 자세의 좌표 계산(fkin)에 실패했어요")
        return [float(v) for v in x]

    def above(self, j, name, dz=0.0):
        """티칭 자세 j 바로 위 LIFT_MM 지점 → (아래 좌표, 위 좌표, 위 관절각)"""
        bad = self.guard.joint_violations(j)
        if bad:
            raise Failed(f"{name} 자세가 관절 한계를 넘어요: {', '.join(bad)}")
        low = self.fk(j, name)
        high = list(low)
        high[2] += LIFT_MM          # 위 지점은 티칭 높이 기준 20cm 위 그대로
        low[2] += dz                # 놓는 높이만 보정
        sol = self.dr.get_solution_space(j)
        if isinstance(sol, int) and sol < 0:
            raise Failed(f"{name} 자세의 solution space 를 못 구했어요")
        ok, hj = self.guard.solve_ik(high, sol)
        if not ok:
            raise Failed(f"{name} 위 {LIFT_MM / 10:.0f}cm 지점에 로봇이 닿을 수 없어요")
        bad = self.guard.joint_violations(hj)
        if bad:
            raise Failed(f"{name} 위 지점이 관절 한계를 넘어요: {', '.join(bad)}")
        jump = max(abs(a - b) for a, b in zip(hj, j))
        if jump > 60:   # 같은 자세 계열이면 20cm 올라가도 관절이 크게 안 바뀌어요
            raise Failed(f"{name} 위 지점의 관절값이 이상해요 (최대 {jump:.0f}° 차이)")
        ok, why = self.guard.check_line(high, low)
        if not ok:
            raise Failed(f"{name} 로 내려가는 길: {why}")
        return low, high, [float(v) for v in hj]

    def via(self, slot):
        """자리별 경유점 (관절각, 이름). 가까운 자리는 MID 에서 -x 로 당긴 지점."""
        x = self.fk(MID, "MID")
        x[0] = NEAR_VIA_X.get(slot, x[0])
        x[2] += VIA_DZ
        name = f"경유점(x={x[0]:.0f}, z={x[2]:.0f})"
        if x[0] == self.fk(MID, "MID")[0] and VIA_DZ == 0:
            return MID, "MID"
        sol = self.dr.get_solution_space(MID)
        if isinstance(sol, int) and sol < 0:
            raise Failed("MID 자세의 solution space 를 못 구했어요")
        ok, vj = self.guard.solve_ik(x, sol)
        if not ok:
            raise Failed(f"{name} 에 로봇이 닿을 수 없어요")
        bad = self.guard.joint_violations(vj)
        if bad:
            raise Failed(f"{name} 이 관절 한계를 넘어요: {', '.join(bad)}")
        jump = max(abs(a - b) for a, b in zip(vj, MID))
        if jump > 60:
            raise Failed(f"{name} 의 관절값이 이상해요 (MID 와 최대 {jump:.0f}° 차이)")
        return [float(v) for v in vj], name

    def movej(self, j, vel, what):
        self.check()
        self.log(f"  관절 이동 → {what} ({vel:.0f}°/s)")
        if self.check_only:
            return
        res = self.dr.movej(self.dr.posj(*j), vel=vel, acc=vel * 2)
        self.after_move(res, what)

    def movel(self, x, vel, what):
        self.check()
        self.log(f"  직선 이동 → {what} ({vel:.0f}mm/s)")
        if self.check_only:
            return
        res = self.dr.movel(self.dr.posx(*x), vel=[vel, 30.0], acc=[vel * 2, 60.0],
                            ref=self.dr.DR_BASE)
        self.after_move(res, what)

    def after_move(self, res, what):
        self.dr.mwait()
        if self.io.stopped.is_set():
            raise Stopped()
        if res != 0:
            raise Failed(f"{what} 이동을 로봇이 거부했어요 (결과 {res}) — 보호정지·한계를 확인하세요")

    def grip_to(self, pos, what):
        """그리퍼를 pos(0~700) 까지만 닫기 → 세기 조절"""
        self.check()
        self.log(f"  그리퍼 {what} (위치 {pos})")
        if self.check_only:
            return
        self.grip_pub.publish(Int32(data=int(pos)))
        time.sleep(GRIP_WAIT + 1.0)   # 그리퍼 노드가 제어기에 명령을 보내고 손가락이 움직일 시간

    def gripper(self, name):
        self.check()
        self.log(f"  그리퍼 {name}")
        if self.check_only:
            return
        cli = self.grip[name]
        if not cli.wait_for_service(timeout_sec=3.0):
            raise Failed(f"gripper/{name} 서비스를 찾을 수 없어요")
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or not res.success:
            raise Failed(f"그리퍼 {name} 실패 ({getattr(res, 'message', '응답 없음')})")
        time.sleep(GRIP_WAIT)

    # ---------- 작업 ----------
    def run(self, task):
        action = task.get("action")
        speed = min(1.0, max(0.1, float(task.get("speed", 0.6))))
        if action == "home":
            self.movej(HOME, FAST_J * speed, "HOME")
            return {}
        if action == "plant":
            return self.plant(task, speed)
        if action in ("dig", "press"):
            self.log(f"  {action}: 로봇 동작 없음")
            return {}
        if action == "water":
            return self.water(task, speed)
        if action == "gripper":
            self.gripper("close" if task.get("state") == "close" else "open")
            return {}
        if action == "reset_picks":
            self.next_pick = 0
            return {}
        raise Failed(f"모르는 action: {action}")

    def plant(self, task, speed):
        slot = task.get("slot")
        if slot not in PLACE:
            raise Failed(f"놓을 칸을 모르겠어요: {slot}")
        pick = task.get("pick")
        if pick is None:
            if self.next_pick >= len(PICK_ORDER):
                raise Failed("잡을 묘목이 더 없어요 (C·D·E 모두 옮김)")
            pick = PICK_ORDER[self.next_pick]
        if pick not in PICK:
            raise Failed(f"잡는 곳을 모르겠어요: {pick}")
        place = PLACE_NAME[slot]

        # 움직이기 전에 경로를 모두 계산·확인
        bad = self.guard.joint_violations(MID)
        if bad:
            raise Failed("MID 자세가 관절 한계를 넘어요")
        p_low, p_high, p_high_j = self.above(PICK[pick], PICK_NAME[pick])
        q_low, q_high, q_high_j = self.above(PLACE[slot], place, PLACE_DZ.get(slot, 0.0))
        via_j, via_name = self.via(slot)
        self.log(f"{PICK_NAME[pick]} → {place} ({slot}) 시작")

        fj, sj = FAST_J * speed, SLOW_J * speed
        fl, sl, dl = FAST_L * speed, SLOW_L * speed, DOWN_L * speed

        # 1) 빈손: 빠르게 잡으러 가기
        self.gripper("open")
        self.movej(p_high_j, fj, f"{PICK_NAME[pick]} 위 20cm")
        self.movel(p_low, dl, f"{PICK_NAME[pick]} 잡는 높이")
        self.gripper("close")
        holding = True
        try:
            # 2) 묘목 들고: 천천히
            self.movel(p_high, sl, f"{PICK_NAME[pick]} 위 20cm")
            self.movej(via_j, sj, via_name)
            self.movej(q_high_j, sj, f"{place} 위 20cm")
            self.movel(q_low, sl, f"{place} 놓는 높이")
            self.gripper("open")
            holding = False
        except (Stopped, Failed) as e:
            if holding and isinstance(e, Failed):
                raise Failed(f"{e} ({PICK_NAME[pick]}을 잡은 채로 멈췄어요)") from e
            raise
        # 다음 묘목은 C→D→E 차례 (control 이 pick 을 정해 보냈으면 그 다음부터)
        self.next_pick = PICK_ORDER.index(pick) + 1

        # 3) 빈손: 빠르게 MID 로 (다음 묘목은 여기서 바로 잡으러 감)
        self.movel(q_high, fl, f"{place} 위 20cm")
        self.movej(via_j, fj, via_name)
        self.log(f"{PICK_NAME[pick]} → {place} 완료")
        return {"pick": pick, "slot": slot}

    def water(self, task, speed):
        """A 물뿌리개를 잡아 심은 자리마다 물을 주고 다시 A 에 둔다"""
        slots = [s for s in (task.get("slots") or []) if s in PLACE]
        if not slots:
            return {"slots": []}
        # 움직이기 전에 모두 계산·확인
        a_low, a_high, a_high_j = self.above(WATER_CAN, "물뿌리개(A)")
        can_rot = a_low[3:]                         # 물뿌리개를 잡은 방향 그대로 들고 다님
        plan = []
        for slot in slots:
            place = PLACE_NAME[slot]
            p = self.fk(PLACE[slot], place)
            if slot in WATER_ROTATED:
                rot = list(can_rot); rot[0] += WATER_YAW_ROTATED      # ZYZ 첫 값 = 베이스 z축 회전
                w = [p[0] + WATER_DX_ROTATED, p[1], p[2] + WATER_DZ] + rot
                where = f"x {WATER_DX_ROTATED / 10:+.0f}cm"
            else:
                w = [p[0], p[1] + WATER_DY, p[2] + WATER_DZ] + list(can_rot)
                where = f"y {WATER_DY / 10:+.0f}cm"
            w_high = list(w); w_high[2] = w[2] + WATER_APPROACH   # 위에서 내려오는 높이 (높을수록 J5 가 많이 꺾임)
            sol = self.dr.get_solution_space(PLACE[slot])
            if isinstance(sol, int) and sol < 0:
                raise Failed(f"{place} 자세의 solution space 를 못 구했어요")
            ok, wj = self.guard.solve_ik(w_high, sol)
            if ok:   # 팔이 다 펴진 경계에서는 IK 가 엉뚱한 답을 줘서 fkin 으로 다시 확인
                back = self.fk(wj, place)
                ok = max(abs(back[k] - w_high[k]) for k in range(3)) < 5.0 and abs(wj[2]) > 5.0
            if not ok:
                raise Failed(f"{place} 물 주는 위치({where}, z +{WATER_DZ / 10:.0f}cm)에 팔이 닿지 않아요")
            bad = self.guard.joint_violations(wj)
            if bad:
                raise Failed(f"{place} 물 주는 위치가 관절 한계를 넘어요: {', '.join(bad)}")
            ok, why = self.guard.check_line(w_high, w)
            if not ok:
                raise Failed(f"{place} 물 주는 위치로 내려가는 길: {why}")
            plan.append((slot, place, w, w_high, [float(v) for v in wj], self.via(slot)))

        fj, sj = FAST_J * speed, SLOW_J * speed
        fl, sl, dl = FAST_L * speed, SLOW_L * speed, DOWN_L * speed
        self.log(f"물 주기 시작: {', '.join(PLACE_NAME[s] for s in slots)}")

        # 1) 빈손: 물뿌리개 잡기
        self.gripper("open")
        self.movej(a_high_j, fj, "물뿌리개 위 20cm")
        self.movel(a_low, dl, "물뿌리개 잡는 높이")
        self.grip_to(GRIP_CAN, "물뿌리개 잡기")
        try:
            # 2) 물뿌리개 들고 (천천히) 자리마다 물 주기
            self.movel(a_high, sl, "물뿌리개 들어 올림")
            for slot, place, w, w_high, wj, (via_j, via_name) in plan:
                if slot not in WATER_DIRECT:          # 먼 자리만 경유점을 거침
                    self.movej(via_j, sj, via_name)
                self.movej(wj, sj, f"{place} 물 주는 곳 위")
                self.movel(w, sl, f"{place} 물 주는 곳")
                self.check()
                self.log(f"  기울여 물 주기 ({WATER_SEC:.0f}초)")
                if not self.check_only:
                    res = self.dr.movel(self.dr.posx(*WATER_TILT), vel=[sl, 30.0 * speed], acc=[sl * 2, 60.0 * speed],
                                        ref=self.dr.DR_TOOL, mod=self.dr.DR_MV_MOD_REL)
                    self.after_move(res, "기울이기")
                for k in range(1, WATER_SQUEEZES + 1):     # 기울인 채 "꽉 → 풀기" 를 여러 번
                    self.grip_to(GRIP_CAN_TILT, f"꽉 ({k}/{WATER_SQUEEZES})")
                    if not self.check_only:
                        time.sleep(WATER_SEC)
                    self.grip_to(GRIP_CAN, f"풀기 ({k}/{WATER_SQUEEZES})")
                self.movel(w, sl, f"{place} 물뿌리개 바로 세우기")
                self.movel(w_high, sl, f"{place} 물 주는 곳 위")
            # 3) 물뿌리개 제자리(A)에 두기
            if plan[-1][0] not in WATER_DIRECT:
                last_via = plan[-1][5]
                self.movej(last_via[0], sj, last_via[1])
            self.movej(a_high_j, sj, "물뿌리개 자리 위 20cm")
            self.movel(a_low, sl, "물뿌리개 내려놓는 높이")
            self.gripper("open")
        except Failed as e:
            raise Failed(f"{e} (물뿌리개를 든 채로 멈췄어요)") from e
        self.movel(a_high, fl, "물뿌리개 위 20cm")
        self.log("물 주기 완료")
        return {"slots": slots}


def main(args=None):
    # Ctrl+C 를 파이썬이 직접 받게 (rclpy 가 먼저 받으면 통신이 끊겨 멈춤 명령을 못 보내요)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    dsr_node = rclpy.create_node("seedling_robot", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node
    import DSR_ROBOT2 as dr
    from robot_control.robot_guard import Guard

    tasks = queue.Queue()
    io = RobotIO(tasks)
    io.declare_parameter("check_only", False)
    check_only = bool(io.get_parameter("check_only").value)
    ex = SingleThreadedExecutor()
    ex.add_node(io)
    threading.Thread(target=ex.spin, daemon=True).start()

    guard = Guard(dsr_node, dr)
    robot = SeedlingRobot(dsr_node, io, dr, guard, check_only)
    try:
        guard.wait_for_driver()
        if not check_only:
            dr.set_robot_mode(dr.ROBOT_MODE_AUTONOMOUS)
        io.get_logger().info(
            "seedling_robot 준비 완료" + (" (check_only: 계산만, 안 움직임)" if check_only else ""))
        while rclpy.ok():
            try:
                task = tasks.get(timeout=0.2)
            except queue.Empty:
                continue
            io.stopped.clear()   # 새 작업이 오면 정지 상태 풀기 (mock 과 같음)
            io.get_logger().info(f"작업: {task}")
            try:
                extra = robot.run(task)
                io.reply(task, True, **(extra or {}))
            except Stopped:
                io.reply(task, False, "정지 명령으로 중단")
            except Failed as e:
                io.reply(task, False, str(e))
            except Exception as e:  # noqa: BLE001
                io.get_logger().error(f"예상하지 못한 오류: {e!r}")
                io.reply(task, False, f"로봇 노드 오류: {e}")
    except KeyboardInterrupt:
        io.stopped.set()
        io.call(io.stop_cli, "stop")
        time.sleep(0.3)
    finally:
        ex.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
