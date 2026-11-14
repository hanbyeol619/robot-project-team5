import sys
import time
import rclpy
import DR_init
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from std_msgs.msg import Int32

from robot_control.program_runner import parse_program, describe, GRIP_OPEN_FROM
from robot_control.robot_guard import Guard

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

VEL = [50.0, 50.0]   # movel 속도 [mm/s, deg/s] — 프로그램의 vel 이 더 빨라도 이 값까지만
ACC = [100.0, 60.0]
VELJ = 30.0          # movej 관절 속도 (deg/s) — 프로그램의 vel 이 더 빨라도 이 값까지만
ACCJ = VELJ * 2      # movej 관절 가속도 (deg/s^2)
GRIP_WAIT = 1.0      # 그리퍼 동작 후 대기 시간 (초)
GRIP_PERCENT = True  # True: gripper(60) 처럼 적은 % 만큼 벌려요 (gripper_node 의 gripper/position_cmd 사용)
                     # False: 예전처럼 95 이상 = 열기, 그보다 작으면 = 끝까지 닫기
GRIP_TIMEOUT = 8.0   # % 명령 뒤 그리퍼가 끝났다고 알려 줄 때까지 기다리는 최대 시간 (초)
REPEAT = 1           # 전체 반복 횟수

# ────────────────────────────────────────────────────────────
# 방법 1) PROGRAM: Creator3D 「코드 보기」 의 코드를 """ """ 사이에 그대로 붙여넣으세요.
#   movej(posj(...)), movel(posx(...)), gripper(퍼센트), wait(초), home(), vel = 30 을 읽어요.
#   gripper(퍼센트) 는 벌림 % 예요: 100 = 다 열기, 0 = 꽉 닫기, 60 = 60% 벌림 (Creator3D 의 「벌림」과 같아요).
#   GRIP_PERCENT = False 로 두면 예전처럼 95 이상 = 열기, 그보다 작으면 = 끝까지 닫기만 해요.
#   PROGRAM 에 움직이는 줄이 하나라도 있으면 아래 SEQUENCE 대신 PROGRAM 을 실행해요.
#   (vel = 20 처럼 설정만 있으면 SEQUENCE 를 실행해요)
PROGRAM = """
movej(posj(-53.42, 32.43, 48.6, 0, 98.97, -53.41), vel=vel)  # A
movej(posj(-44.27, 59.93, 41.08, 0, 78.99, -44.28), vel=vel)  # B
movel(posx(493.32, -480.91, 217.44, 0, 180, -0.01), vel=vel)  # B_up
movel(posx(359.39, -480.93, 217.44, 0, 180, -0.01), vel=vel)  # B_2
movel(posx(346.48, -480.94, 217.52, -56.69, 179.99, -56.7), vel=vel)  # B_22
movel(posx(346.47, -480.93, 109.66, -56.69, 179.99, -56.7), vel=vel)  # B_down
movel(posx(346.46, -480.91, 132.03, -56.69, 179.99, -56.7), vel=vel)  # B_up2
movej(posj(-20.44, -16, 116, 0, 79.99, -20.44), vel=vel)  # C
movel(posx(224.65, -101.72, 322.32, 0, 180, 0), vel=vel)  # C_down
movel(posx(224.67, -101.73, 297.54, 0, 180, 0), vel=vel)  # C_down2
movel(posx(224.67, -101.73, 317.38, -26.82, 179.99, -26.82), vel=vel)  # C_up
movej(posj(5.56, -20.93, 107.13, -0.01, 93.79, 5.56), vel=vel)  # D
movej(posj(43.6, -6.86, 96.43, -0.01, 90.42, 43.6), vel=vel)  # E
movej(posj(13.26, 46.78, 31.99, -0.02, 101.17, 13.25), vel=vel)  # F
movej(posj(-48.21, 51.13, 60.48, 0, 68.41, -48.2), vel=vel)  # B_Down
gripper(25)  # B_Gripper_Close
movej(posj(-48.21, 51.13, 60.48, 0, 68.41, -48.2), vel=vel)  # P9
movej(posj(-48.21, 45.19, 47.63, 0, 87.2, -48.2), vel=vel)  # B_Up
movej(posj(-33.06, 27.53, 88.82, 0.01, 63.68, -33.05), vel=vel)  # B_Y-1
movej(posj(-33.06, 27.53, 88.82, -25, 63.68, -33.05), vel=vel)  # B_Rotateㅎㅋ_들기
movej(posj(-26.02, 22.87, 95.71, -24.98, 64.3, -25.7), vel=vel)  # B_Y-2
movel(posx(461.71, -265.47, 210.25, -117.51, 157.63, -128.74), vel=vel)  # B_Y-2_2
movej(posj(-33.06, 27.53, 88.82, -25, 63.68, -33.05), vel=vel)  # B_Rotate
movel(posx(394.26, -265.5, 167.18, -117.52, 157.63, -128.75), vel=vel)  # B_Y-2_down
movej(posj(-22.85, 31.32, 82.96, -23.91, 69.39, -24.82), vel=vel)  # B_X-1
movej(posj(-22.85, 31.32, 82.96, -23.91, 69.39, -24.82), vel=vel)  # P16
movel(posx(461.71, -202.78, 210.23, -117.51, 157.62, -128.74), vel=vel)  # B_X-1_2
movej(posj(-16.38, 28.38, 87.52, -23.38, 70.26, -18.41), vel=vel)  # B_Y-2_3
movel(posx(461.67, -202.85, 177.47, -117.52, 157.63, -128.75), vel=vel)  # B_Y-2_3_down
movel(posx(461.65, -202.86, 168.2, -117.53, 157.62, -128.74), vel=vel)  # B_Y-2_3_down2
movel(posx(461.69, -202.87, 157.75, -117.52, 157.62, -128.74), vel=vel)  # B_Y-2_3_down3
movej(posj(-16.38, 32.33, 90.52, -24.58, 63.9, -15.34), vel=vel)  # B_Down-2
movej(posj(-16.38, 32.34, 90.52, 8.5, 63.9, -15.34), vel=vel)  # B_Rotate-2_파기
movej(posj(-40.95, 49.14, 32.82, 4.44, 107.07, -34.74), vel=vel)  # B_Y-3
movej(posj(-40.95, 49.14, 32.82, -26.5, 107.07, -34.74), vel=vel)  # B_Rotate_버리기
movej(posj(-48.21, 51.13, 60.48, 0, 68.41, -48.2), vel=vel)  # P26
gripper(100)  # B_Gripper_Open
movej(posj(-40.95, 49.14, 32.82, 4.44, 107.07, -34.74), vel=vel)  # P24
movej(posj(-40.95, 49.14, 32.82, -26.5, 107.07, -34.74), vel=vel)  # P25
movej(posj(-16.38, 32.34, 90.52, 8.5, 63.9, -15.34), vel=vel)  # P23
movej(posj(-16.38, 32.33, 90.52, -24.58, 63.9, -15.34), vel=vel)  # P22
movej(posj(-16.38, 28.38, 87.52, -23.38, 70.26, -18.41), vel=vel)  # P18
movej(posj(-26.02, 22.87, 95.71, -24.98, 64.3, -25.7), vel=vel)  # P13
movej(posj(-33.06, 27.53, 88.82, -25, 63.68, -33.05), vel=vel)  # P12
movej(posj(-33.06, 23.18, 82.11, 0.01, 74.73, -33.05), vel=vel)  # P11
movej(posj(0, 0, 90, 0, 90, 0), vel=vel)  # P10
gripper(25)  # B_Gripper
gripper(100)  # 놓기1
movej(posj(-48.21, 51.13, 60.48, 0, 68.41, -48.2), vel=vel)  # B_DOWN
movej(posj(-48.21, 46.07, 52.81, 0, 81.11, -48.2), vel=vel)  # P7
movej(posj(-55.52, 41.78, 74.34, 0, 63.88, -55.52), vel=vel)  # P8
movej(posj(-24.36, -18.79, 119.26, 0, 79.53, -24.36), vel=vel)  # C_Down
movej(posj(-13.52, 0.19, 102.06, 0, 77.76, -13.52), vel=vel)  # C-1
movej(posj(23.05, 6.94, 91.7, 0, 81.37, 23.05), vel=vel)  # D-Center
gripper(0)  # 잡기1
movel(posx(247.89, -43.26, 325.76, -12.36, 179.99, -12.36), vel=vel)  # C_2
movel(posx(247.92, 12.92, 325.78, -43.45, 179.99, -43.45), vel=vel)  # C_22
movej(posj(10.09, -18.16, 117.52, -0.01, 80.63, 10.08), vel=vel)  # P5
movel(posx(224.68, -101.73, 437.56, 0, 180, 0), vel=vel)  # C_Down_up
movel(posx(285.62, -101.76, 437.52, 0, 180, 0), vel=vel)  # C_Down_2
movel(posx(360.56, -101.76, 437.5, 0, 180, 0), vel=vel)  # C_Down_22
movel(posx(418.07, -101.68, 437.52, 0, 180, 0), vel=vel)  # C_Down_23
movel(posx(418.02, -101.75, 344.65, 168.68, 179.99, 168.68), vel=vel)  # C_Down_down
movej(posj(-13.68, 10.51, 93.28, 0, 76.22, -13.68), vel=vel)  # C_화분 심기
gripper(0)  # 잡기2
gripper(0)  # 잡기3
gripper(0)  # 잡기4
gripper(0)  # 잡기5
gripper(0)  # 잡기6
gripper(0)  # 잡기7
gripper(0)  # 잡기8
movej(posj(0, 37.15, 15.87, 0, 126.97, 0), vel=vel)  # P1
movel(posx(493.3, -480.89, 141.36, 138.09, 179.99, 138.08), vel=vel)  # P1_down
movel(posx(493.32, -480.91, 172.85, 0, 180, -0.01), vel=vel)  # P1_up
movej(posj(-44.27, 59.93, 41.08, 0, 78.99, -44.28), vel=vel)  # P4
movej(posj(-8.33, 54.06, 22.67, 0.01, 103.22, -8.34), vel=vel)  # D-CENTER
movel(posx(658.17, -96.34, 336.54, 2.27, 179.95, 2.25), vel=vel)  # D-CENTER_up
movel(posx(652.88, -95.57, 366.93, -4.03, 179.88, -4.04), vel=vel)  # D-CENTER_up2
movel(posx(652.89, 12.05, 366.85, -4.15, 179.89, -4.16), vel=vel)  # D-CENTER_2
movel(posx(509.76, 12.07, 366.91, -4.06, 179.89, -4.07), vel=vel)  # D-CENTER_22
movel(posx(509.79, 130.1, 366.91, -5.88, 179.88, -5.9), vel=vel)  # D-CENTER_23
"""

# 방법 2) SEQUENCE: 직접 적기
#   [x, y, z, rx, ry, rz]  → 베이스 기준 절대좌표로 직선 이동 (mm, deg)  = movel
#   ("j", [j1, ..., j6])    → 관절 각도로 이동 (deg)                       = movej
#   "close" / "open"       → 그리퍼 닫기 / 열기
#   ("wait", 초)           → 대기
#
# 좌표 얻는 법: PROGRAM 과 SEQUENCE 를 모두 비워둔 채 실행하면 현재 위치만 출력하고 끝나요.
SEQUENCE = [
    "open",                                   # 원본: gripper(50) 반만 열기 → 서비스는 열기/닫기만 있음
]

# 관절 한계 보호: 실행 전에 모든 단계를 미리 계산해서, 한 곳이라도 한계(J2·J3·J5 ±130°)를
# 넘으면 아예 시작하지 않아요. 실행 중에도 movel 은 실제 위치에서 경로를 다시 확인해요.
#   ros2 run robot_control move_basic --check   → 확인만 하고 움직이지 않음
# ────────────────────────────────────────────────────────────


def sequence_to_steps(seq):
    """SEQUENCE 형식 → parse_program 과 같은 단계 목록"""
    steps = []
    for n, s in enumerate(seq, 1):
        if isinstance(s, str):
            if s not in ("open", "close"):
                raise ValueError(f"SEQUENCE {n}번째: 알 수 없는 명령 '{s}'")
            st = {"kind": "grip", "g": 100.0 if s == "open" else 0.0}
        elif isinstance(s, tuple) and s[0] == "j":
            st = {"kind": "movej", "pos": [float(v) for v in s[1]], "vel": VELJ, "acc": ACCJ}
        elif isinstance(s, tuple) and s[0] == "wait":
            st = {"kind": "wait", "s": float(s[1])}
        elif isinstance(s, (list, tuple)) and len(s) == 6:
            st = {"kind": "movel", "pos": [float(v) for v in s], "vel": VEL[0], "acc": ACC[0]}
        else:
            raise ValueError(f"SEQUENCE {n}번째를 이해할 수 없어요: {s}")
        st.update(line=n, src=str(s), note="")
        steps.append(st)
    return steps


def call_gripper(node, client, name):
    """그리퍼 서비스 호출 (open / close)"""
    if not client.wait_for_service(timeout_sec=3.0):
        node.get_logger().error(f"gripper/{name} 서비스를 찾을 수 없음")
        return False
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    res = future.result()
    if res is None:
        node.get_logger().error(f"gripper/{name} 응답 없음")
        return False
    print(f"그리퍼 {name}: success={res.success}, msg='{res.message}'")
    time.sleep(GRIP_WAIT)
    return res.success


def pct_to_stroke(g):
    """벌림 % → RH-P12-RN 위치값 (gripper_node: 0 = 열림 … 700 = 닫힘). Creator3D 와 같은 환산."""
    return int(round((100.0 - max(0.0, min(100.0, g))) * 7.0))


def grip_text(g):
    if not GRIP_PERCENT:
        return f"그리퍼 {'열기' if g >= GRIP_OPEN_FROM else '닫기'} (원본 {g:g}%)"
    if g >= 100:
        return "그리퍼 열기 (100%)"
    if g <= 0:
        return "그리퍼 닫기 (0%)"
    return f"그리퍼 벌림 {g:g}% (위치 {pct_to_stroke(g)}/700)"


def grip_percent(node, pub, seen, g):
    """gripper(%) 를 gripper/position_cmd 로 보내고, gripper_node 가 gripper/stroke 로 그 값을 알려 줄 때까지 기다려요.
    gripper_node 는 실제 그리퍼가 명령을 끝낸 뒤에야 stroke 를 바꾸므로, 그 값이 바뀌면 끝난 거예요."""
    target = pct_to_stroke(g)
    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        node.get_logger().error("gripper/position_cmd 를 받는 노드가 없음 (gripper_node 실행 확인)")
        return False
    t0 = time.time()
    while seen["stroke"] is None and time.time() - t0 < 1.5:   # 지금 값 (gripper/stroke 는 0.1초마다 나와요)
        rclpy.spin_once(node, timeout_sec=0.1)
    same = seen["stroke"] == target
    pub.publish(Int32(data=target))
    if same:   # 이미 그 값이라 바뀌는 걸 볼 수 없어요: 명령이 끝날 만큼만 기다려요
        time.sleep(2.0 + GRIP_WAIT)
        print(f"그리퍼 위치 {target}: 이미 그 값이라 정해진 시간만 기다렸어요")
        return True
    t0 = time.time()
    while time.time() - t0 < GRIP_TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.1)
        if seen["stroke"] == target:
            print(f"그리퍼 위치 {target} 완료 ({time.time() - t0:.1f}초)")
            time.sleep(GRIP_WAIT)
            return True
    node.get_logger().error(f"그리퍼가 {GRIP_TIMEOUT:g}초 안에 위치 {target} 을(를) 끝내지 못함 (gripper_node 로그 확인)")
    return False


def stop_robot(node, client):
    """로봇에 멈춤 명령 (빠른 정지, 전원은 유지). Ctrl+C 로 중단했을 때 하던 이동을 멈춰요."""
    try:
        from dsr_msgs2.srv import MoveStop
        if not client.wait_for_service(timeout_sec=1.0):
            print("⚠ 멈춤 서비스를 찾지 못했어요 — 비상정지 버튼을 쓰세요")
            return
        req = MoveStop.Request()
        req.stop_mode = 1   # DR_QSTOP
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        print("로봇에 멈춤 명령을 보냈어요")
    except Exception as e:   # noqa: BLE001 - 중단 중이라 무슨 일이든 알리기만
        print(f"⚠ 멈춤 명령을 보내지 못했어요 ({e}) — 비상정지 버튼을 쓰세요")


def main(args=None):
    check_only = "--check" in sys.argv
    # Ctrl+C 를 파이썬이 직접 받게 해요 (rclpy 가 먼저 받으면 통신이 끊겨 멈춤 명령을 못 보내요)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    import DSR_ROBOT2 as dr
    from DSR_ROBOT2 import movel, posx, get_current_posx, get_current_posj, movej, posj, mwait, fkin
    from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS, DR_BASE

    guard = Guard(node, dr)
    from dsr_msgs2.srv import MoveStop
    stop_cli = node.create_client(MoveStop, "motion/move_stop")
    started = False   # Enter 를 눌러 움직이기 시작했는지

    # 그리퍼 서비스 클라이언트 (네임스페이스 dsr01 기준 → /dsr01/gripper/...)
    grippers = {
        "open": node.create_client(Trigger, "gripper/open"),
        "close": node.create_client(Trigger, "gripper/close"),
    }
    # % 명령 (gripper_node: gripper/position_cmd 0~700) 과 그 결과 (gripper/stroke)
    grip_pub = node.create_publisher(Int32, "gripper/position_cmd", 10)
    grip_seen = {"stroke": None}
    node.create_subscription(Int32, "gripper/stroke", lambda m: grip_seen.update(stroke=m.data), 10)

    try:
        guard.wait_for_driver()

        cur = list(get_current_posx()[0])        # [x, y, z, rx, ry, rz]
        print("현재 위치 (복사해서 SEQUENCE에 붙여넣기):")
        print("    [" + ", ".join(f"{v:.1f}" for v in cur) + "],")
        print("    (\"j\", [" + ", ".join(f"{v:.1f}" for v in get_current_posj()) + "]),")

        try:
            steps = parse_program(PROGRAM, default_vel=VELJ, gripper=True) if PROGRAM.strip() else []
            source = "PROGRAM"
            if not steps:   # 비었거나 vel = 20 같은 설정만 있으면 SEQUENCE 를 써요
                if PROGRAM.strip():
                    print("PROGRAM 에 움직이는 줄이 없어서 SEQUENCE 를 실행해요")
                steps, source = sequence_to_steps(SEQUENCE), "SEQUENCE"
        except ValueError as e:
            print(f"코드 오류: {e}")
            return
        if not steps:
            print(f"{source}가 비어 있어서 이동하지 않고 종료합니다.")
            return

        # ── 미리 확인: 모든 단계를 순서대로 따라가며 관절 한계 계산 (로봇은 안 움직임) ──
        print(f"\n{source} {len(steps)}단계를 미리 확인하는 중...")
        problems = []
        pose_x = cur
        for i, st in enumerate(steps, 1):
            if st["kind"] == "movej":
                bad = guard.joint_violations(st["pos"])
                if bad:
                    problems.append(f"  {i}단계 ({st['line']}번째 줄) movej: " + ", ".join(bad))
                fk = fkin(st["pos"], DR_BASE)
                if not isinstance(fk, int):        # 실패하면 -1 (int), 성공하면 배열
                    pose_x = [float(v) for v in fk]
            elif st["kind"] == "movel":
                ok, why = guard.check_line(pose_x, st["pos"])
                if not ok:
                    problems.append(f"  {i}단계 ({st['line']}번째 줄) movel: {why}")
                pose_x = list(st["pos"])
        for i, st in enumerate(steps, 1):
            print(f"  {i:3d}. {grip_text(st['g']) if st['kind'] == 'grip' else describe(st)}")
        if problems:
            print("\n⚠ 관절 한계에 걸리는 단계가 있어서 시작하지 않아요:")
            print("\n".join(problems))
            print("(값을 고치거나, 그 앞에 movej 로 자세를 바꿔 주세요)")
            return
        print("\n✅ 모든 단계가 관절 한계 안쪽이에요")
        if check_only:
            print("--check: 확인만 하고 끝냅니다 (로봇은 움직이지 않았어요)")
            return

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        input(f"\n위 동작을 {REPEAT}회 실행합니다. Enter = 시작, Ctrl+C = 취소 ")
        started = True

        for r in range(REPEAT):
            for n, st in enumerate(steps, 1):
                tag = f"[{r+1}/{REPEAT}] {n}/{len(steps)}"
                k = st["kind"]
                if k == "grip":
                    g = st["g"]
                    print(f"{tag} {grip_text(g)}")
                    if GRIP_PERCENT and 0 < g < 100:   # 중간 값: 그 % 만큼
                        ok = grip_percent(node, grip_pub, grip_seen, g)
                    else:                               # 다 열기 / 꽉 닫기 (또는 GRIP_PERCENT = False)
                        name = "open" if g >= (100 if GRIP_PERCENT else GRIP_OPEN_FROM) else "close"
                        ok = call_gripper(node, grippers[name], name)
                    if not ok:
                        print(f"{tag} 그리퍼가 실패해서 여기서 멈춰요 (못 잡은 채로 옮기지 않게) — 그리퍼 서비스와 전원을 확인하세요")
                        return
                    continue
                if k == "wait":
                    print(f"{tag} {st['s']:g}초 대기")
                    time.sleep(st["s"])
                    continue
                if k == "skip":
                    continue
                if k == "movej":
                    bad = guard.joint_violations(st["pos"])
                    if bad:
                        print(f"{tag} 관절 한계로 멈춤: {', '.join(bad)}")
                        return
                    v = min(st["vel"], VELJ)
                    print(f"{tag} 관절 이동 → {st['pos']}")
                    res = movej(posj(*st["pos"]), vel=v, acc=min(st["acc"], v * 2))
                else:  # movel: 실제 위치에서 경로를 다시 확인
                    ok, why = guard.check_line(list(get_current_posx()[0]), st["pos"])
                    if not ok:
                        print(f"{tag} 관절 한계로 멈춤: {why}")
                        return
                    v = min(st["vel"], VEL[0])
                    print(f"{tag} 직선 이동 → {st['pos']}")
                    res = movel(posx(*st["pos"]), vel=[v, VEL[1]], acc=[min(st["acc"], v * 2), ACC[1]], ref=DR_BASE)
                mwait()
                if res != 0:
                    print(f"{tag} 로봇이 이동을 거부했어요 (결과 {res}) — 로봇 상태(보호정지·한계)를 확인하세요")
                    return

        print("완료")

    except KeyboardInterrupt:
        print("\n중단")
        if started:
            stop_robot(node, stop_cli)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
