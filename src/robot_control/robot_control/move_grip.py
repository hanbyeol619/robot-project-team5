# 11월 7일 버전 move_basic.py (VS Code 기록에서 복구): P1↔P2 왕복 후 그리퍼 닫기 → 열기
import time
import rclpy
import DR_init
from std_srvs.srv import Trigger

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

REPEAT = 3          # 왕복 횟수
OFFSET_X = 50.0
OFFSET_Y = 100.0    # mm, 두 번째 점까지 거리
VEL = [100.0, 100.0]  # [mm/s, deg/s] 처음엔 낮게
ACC = [100.0, 60.0]
GRIP_WAIT = 1.0     # 그리퍼 동작 후 대기 시간 (초)


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


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import movel, posx, get_current_posx
    from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS, DR_BASE

    # 그리퍼 서비스 클라이언트 (네임스페이스 dsr01 기준 → /dsr01/gripper/...)
    grip_open = node.create_client(Trigger, "gripper/open")
    grip_close = node.create_client(Trigger, "gripper/close")

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    cur = get_current_posx()[0]        # [x, y, z, rx, ry, rz]
    print(f"현재 위치: {cur}")

    p1 = posx(*cur)
    p2 = posx(cur[0] + OFFSET_X, cur[1] + OFFSET_Y, cur[2], cur[3], cur[4], cur[5])

    try:
        for i in range(REPEAT):
            print(f"[{i+1}/{REPEAT}] → P2")
            movel(p2, vel=VEL, acc=ACC, ref=DR_BASE)
            print(f"[{i+1}/{REPEAT}] → P1")
            movel(p1, vel=VEL, acc=ACC, ref=DR_BASE)

        # 마지막에 잡고 → 열기
        print("그리퍼 닫기 (잡기)")
        call_gripper(node, grip_close, "close")
        print("그리퍼 열기")
        call_gripper(node, grip_open, "open")

    except KeyboardInterrupt:
        print("중단")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()