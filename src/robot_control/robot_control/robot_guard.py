"""두산 e0509 관절 한계 보호 (mini_jog_ 와 같은 규칙). move_basic 같은 스크립트에서 씀.

- wait_for_driver(): 서비스가 '발견'될 때까지 기다림. DSR_ROBOT2 는 발견 전에 보낸 요청이 사라지면
  영원히 기다리기 때문에 꼭 필요해요.
- joint_violations(posj): 한계를 넘는 관절 목록
- check_line(시작 posx, 목표 posx): movel 경로를 잘게 나눠 컨트롤러의 역기구학(ikin)으로 관절값을 미리 계산
"""
import rclpy

from robot_control.program_runner import line_points

# 이 컨트롤러는 J3, J5 가 ±135° 를 넘으면 서보를 끄고 멈춥니다. 실제보다 5° 안쪽에서 막아요.
# J2 의 실제 한계는 확인 못 해서 같이 보수적으로 막아요.
JOINT_LIMITS = [
    (-355.0, 355.0),  # J1
    (-130.0, 130.0),  # J2
    (-130.0, 130.0),  # J3  (실제 135)
    (-355.0, 355.0),  # J4
    (-130.0, 130.0),  # J5  (실제 135)
    (-355.0, 355.0),  # J6
]


class Guard:
    def __init__(self, node, dr, limits=JOINT_LIMITS, step_mm=10.0, step_deg=2.0):
        self.node, self.dr, self.limits = node, dr, limits
        self.step_mm, self.step_deg = step_mm, step_deg
        self.ik_ready = False

    def wait_for_driver(self):
        needed = {
            "get_current_posj": self.dr._ros2_get_current_posj,
            "get_current_posx": self.dr._ros2_get_current_posx,
            "move_joint": self.dr._ros2_movej,
            "move_line": self.dr._ros2_movel,
        }
        for name, client in needed.items():
            while not client.wait_for_service(timeout_sec=2.0):
                print(f"[대기] {name} 서비스를 기다리는 중 — 두산 드라이버(ros2 launch)가 켜져 있나요?")
        self.ik_ready = self.dr._ros2_ikin.wait_for_service(timeout_sec=3.0) and \
            self.dr._ros2_get_current_solution_space.wait_for_service(timeout_sec=3.0)
        if not self.ik_ready:
            print("[주의] ikin 서비스가 없어 movel 경로의 관절 한계는 미리 확인하지 못해요")
        print("드라이버 서비스 확인 완료")

    def joint_violations(self, j):
        out = []
        for k, (v, (lo, hi)) in enumerate(zip(j, self.limits)):
            if v < lo or v > hi:
                out.append(f"J{k + 1} {v:.1f}° (허용 {lo:g}~{hi:g})")
        return out

    def solve_ik(self, x, sol_space):
        from dsr_msgs2.srv import Ikin
        req = Ikin.Request()
        req.pos = [float(v) for v in x]
        req.sol_space = int(sol_space)
        req.ref = 0  # DR_BASE
        future = self.dr._ros2_ikin.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        res = future.result()
        if res is None or not res.success:
            return False, None
        return True, list(res.conv_posj)

    def check_line(self, start, target):
        """→ (괜찮은지, 이유)"""
        if not self.ik_ready:
            return True, ""
        pts, _, _ = line_points(start, target, self.step_mm, self.step_deg)
        sol = self.dr.get_current_solution_space()
        for t, p in pts:
            ok, j = self.solve_ik(p, sol)
            where = f"경로 {t * 100:.0f}% 지점 (X={p[0]:.0f}, Y={p[1]:.0f}, Z={p[2]:.0f})"
            if not ok:
                return False, f"{where}에 로봇이 닿을 수 없어요"
            bad = self.joint_violations(j)
            if bad:
                return False, f"{where}에서 관절 한계를 넘어요: " + ", ".join(bad)
        return True, ""
