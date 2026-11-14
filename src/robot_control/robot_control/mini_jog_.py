import math
import os
import re
import rclpy
import DR_init
import tkinter as tk
from tkinter import messagebox

from robot_control.program_runner import parse_program, describe, line_points, zyz_to_quat, quat_angle


# Configuration for a single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
# 펜던트에 등록된 툴/TCP 이름을 쓸 때만 채우세요 (비워 두면 설정하지 않음)
ROBOT_TCP = ""
ROBOT_TOOL = ""

VELOCITY, ACC = 30, 30   # 처음엔 낮게

# ── 관절 한계 보호 ────────────────────────────────────────────
# 이 컨트롤러는 J3, J5 가 ±135° 를 넘으면 서보를 끄고 멈춥니다 (오늘 두 번 걸림).
# 실제 한계보다 5° 안쪽으로 막습니다. J2 의 실제 한계는 확인 못 해서 보수적으로 같이 막아요.
JOINT_LIMITS = [
    (-355.0, 355.0),  # J1
    (-130.0, 130.0),  # J2
    (-130.0, 130.0),  # J3  (실제 135)
    (-355.0, 355.0),  # J4
    (-130.0, 130.0),  # J5  (실제 135)
    (-355.0, 355.0),  # J6
]
MAX_JOINT_STEP = 30.0     # movej 한 번에 어떤 관절이든 이보다 많이 바뀌면 물어봄 (deg)
MAX_LINEAR_STEP = 100.0   # movel 한 번 이동 거리가 이보다 길면 물어봄 (mm)
MAX_ROT_STEP = 30.0       # movel 한 번에 Rx/Ry/Rz 가 이보다 많이 바뀌면 물어봄 (deg)
PATH_CHECK_MM = 10.0      # movel 경로를 이 간격(mm)으로 나눠 관절값을 미리 계산해 확인

# Initialize DR_init with robot parameters
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    # Initialize the ROS node (여러 개 떠도 이름이 안 겹치게 PID를 붙임)
    rclpy.init(args=args)
    node = rclpy.create_node(f"mini_jog_{os.getpid()}", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # Import robot functions
    try:
        from DSR_ROBOT2 import (
            movej,
            movel,
            get_current_posx,
            parallel_axis,
            get_current_posj,
            get_current_solution_space,
            mwait,
            set_tool, set_tcp,
            DR_BASE,
            DR_AXIS_Z,
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        return

    # DSR_ROBOT2 는 서비스가 '발견'되기 전에 요청을 보내면 그 요청이 사라지고 영원히 기다립니다.
    # 그래서 쓰는 서비스들이 보일 때까지 먼저 기다립니다 (드라이버가 꺼져 있으면 여기서 알려줌).
    import DSR_ROBOT2 as _dr
    needed = {
        "get_current_posj": _dr._ros2_get_current_posj,
        "get_current_posx": _dr._ros2_get_current_posx,
        "move_joint": _dr._ros2_movej,
        "move_line": _dr._ros2_movel,
        "move_wait": _dr._ros2_move_wait,
    }
    for name, client in needed.items():
        while not client.wait_for_service(timeout_sec=2.0):
            print(f"[대기] {name} 서비스를 기다리는 중 — 두산 드라이버(ros2 launch)가 켜져 있나요?")
    print("드라이버 서비스 확인 완료")

    # 역기구학(ikin) 서비스: movel 경로의 관절값을 미리 계산하는 데 씀
    from dsr_msgs2.srv import Ikin
    ik_client = _dr._ros2_ikin
    ik_ready = ik_client.wait_for_service(timeout_sec=3.0)
    if not ik_ready:
        print("[주의] ikin 서비스가 없어 movel 경로의 관절 한계는 미리 확인하지 못해요 (이동 거리 제한만 적용)")

    # ── 관절 한계 보호 함수 ────────────────────────────────────
    def joint_violations(j):
        """한계를 넘는 관절 목록 → ['J5 132.0° (허용 -130~130)', ...]"""
        out = []
        for k, (v, (lo, hi)) in enumerate(zip(j, JOINT_LIMITS)):
            if v < lo or v > hi:
                out.append(f"J{k + 1} {v:.1f}° (허용 {lo:g}~{hi:g})")
        return out

    def solve_ik(x, sol_space):
        """posx → (성공여부, posj). 서비스 응답이 없으면 (False, None)"""
        req = Ikin.Request()
        req.pos = [float(v) for v in x]
        req.sol_space = int(sol_space)
        req.ref = DR_BASE
        future = ik_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        res = future.result()
        if res is None or not res.success:
            return False, None
        return True, list(res.conv_posj)

    def ang_diff(a, b):
        """b - a 를 -180~180 으로 (각도가 ±180 을 넘나들 때 대비)"""
        return (b - a + 180.0) % 360.0 - 180.0

    def check_line(start, target):
        """movel 경로를 PATH_CHECK_MM 간격으로 나눠 관절 한계를 미리 확인 → (괜찮은지, 이유)"""
        if not ik_ready:
            return True, ""
        # 자세는 오일러각을 그냥 나누면 틀려서(예: 0.3,-180,0.3 과 175.8,180,175.8 은 같은 자세) 최단 회전으로 나눔
        pts, _, _ = line_points(start, target, step_mm=PATH_CHECK_MM, step_deg=2.0)
        sol = get_current_solution_space()
        for t, p in pts:
            ok, j = solve_ik(p, sol)
            where = f"경로 {t * 100:.0f}% 지점 (X={p[0]:.0f}, Y={p[1]:.0f}, Z={p[2]:.0f})"
            if not ok:
                return False, f"{where}에 로봇이 닿을 수 없어요"
            bad = joint_violations(j)
            if bad:
                return False, f"{where}에서 관절 한계를 넘어요:\n  " + "\n  ".join(bad)
        return True, ""

    prog = {"steps": [], "i": 0, "running": False}   # 아래 「프로그램」 칸의 실행 상태

    def refuse(title, text):
        print(f"[막음] {title}: {text}")
        messagebox.showwarning(title, text + "\n\n이동하지 않았어요. 칸의 값은 현재 위치로 되돌렸어요.")

    # Tool과 TCP 설정 (이름을 적었을 때만)
    if ROBOT_TOOL:
        set_tool(ROBOT_TOOL)
    if ROBOT_TCP:
        set_tcp(ROBOT_TCP)

    def create_label_entry(root, text, row, col):
        label = tk.Label(root, text=text)
        label.grid(row=row + 1, column=col - 1, padx=10, pady=5)
        entry = tk.Entry(root, width=10)
        entry.insert(0, str(1))
        entry.grid(row=row + 1, column=col)
        return entry

    def create_position_entries(root, labels, default_values, col, min_val=5, max_val=50):
        data_entries = []
        for i, (label_text, default_value) in enumerate(zip(labels, default_values)):
            label = tk.Label(root, text=label_text)
            label.grid(row=i + 1, column=col, padx=10, pady=5)

            if i <= 5:
                entry = tk.Entry(root, width=10)
                entry.insert(0, str(round(default_value, 3)))
                entry.grid(row=i + 1, column=col + 1, padx=10, pady=5)
                data_entries.append(entry)
            else:
                entry = tk.Scale(
                    root, from_=min_val, to=max_val, orient=tk.HORIZONTAL, length=200
                )
                entry.set(default_value)
                entry.grid(row=i + 1, column=col + 1, padx=10, pady=5, columnspan=3)
                data_entries.append(entry)

        return data_entries

    def create_increment_buttons(
        root, joint_data, axes_data, increment_j_value, increment_x_value
    ):
        for i in range(6):
            tk.Button(
                root,
                text="+",
                command=lambda i=i: increment_joint(
                    axes_data, joint_data, i, float(increment_j_value.get())
                ),
            ).grid(row=i + 1, column=2, padx=2, pady=5)
            tk.Button(
                root,
                text="-",
                command=lambda i=i: increment_joint(
                    axes_data, joint_data, i, -float(increment_j_value.get())
                ),
            ).grid(row=i + 1, column=3, padx=2, pady=5)
            tk.Button(
                root,
                text="+",
                command=lambda i=i: increment_linear(
                    axes_data, joint_data, i, float(increment_x_value.get())
                ),
            ).grid(row=i + 1, column=7, padx=2, pady=5)
            tk.Button(
                root,
                text="-",
                command=lambda i=i: increment_linear(
                    axes_data, joint_data, i, -float(increment_x_value.get())
                ),
            ).grid(row=i + 1, column=8, padx=2, pady=5)

    def create_control_buttons(root, joint_data, axes_data):
        tk.Button(
            root, text="movej", command=lambda: move_by_joint(axes_data, joint_data)
        ).grid(row=9, column=1, columnspan=1, pady=10)
        tk.Button(
            root, text="copy", command=lambda: copy_to_clipboard(root, joint_data, "posj")
        ).grid(row=9, column=2, columnspan=1, pady=10)

        tk.Button(
            root, text="movel", command=lambda: move_by_line(axes_data, joint_data)
        ).grid(row=9, column=6, columnspan=1, pady=10)
        tk.Button(
            root, text="copy", command=lambda: copy_to_clipboard(root, axes_data, "posx")
        ).grid(row=9, column=7, columnspan=1, pady=10)

        tk.Button(
            root,
            text="z-axis alignment",
            command=lambda: z_axis_alignment(axes_data, joint_data),
        ).grid(row=11, column=2, columnspan=4, pady=10)

    # Joint and Linear Movement Functions
    def refresh(entries_x, entries_j):
        """칸들을 로봇의 현재 값으로 되돌림"""
        update_entries(get_current_posj(), entries_j)
        update_entries(get_current_posx()[0], entries_x)

    def move_by_joint(entries_x, entries_j):
        if prog["running"]:
            return
        inputs = [float(entry.get()) for entry in entries_j]
        pos = inputs[:-2]

        bad = joint_violations(pos)
        if bad:
            refresh(entries_x, entries_j)
            refuse("관절 한계", "목표 자세가 관절 한계를 넘어요:\n  " + "\n  ".join(bad))
            return
        cur = list(get_current_posj())
        step = max(abs(a - b) for a, b in zip(pos, cur))
        if step > MAX_JOINT_STEP and not messagebox.askyesno(
            "큰 이동", f"관절이 한 번에 최대 {step:.1f}° 움직여요 (기준 {MAX_JOINT_STEP:g}°).\n정말 이동할까요?"
        ):
            refresh(entries_x, entries_j)
            return

        is_success = movej(pos, vel=inputs[-2], acc=inputs[-1])
        mwait()
        update_entries(get_current_posx()[0], entries_x)
        update_entries(get_current_posj(), entries_j)

        print("current posx : ", get_current_posx())
        print("current posj : ", get_current_posj())
        print("is_success:", is_success)

    def move_by_line(entries_x, entries_j):
        inputs = [float(entry.get()) for entry in entries_x]
        pos = inputs[:-2]

        if prog["running"]:
            return
        cur = list(get_current_posx()[0])
        dist = math.dist(cur[:3], pos[:3])
        rot = quat_angle(zyz_to_quat(*cur[3:]), zyz_to_quat(*pos[3:]))
        if (dist > MAX_LINEAR_STEP or rot > MAX_ROT_STEP) and not messagebox.askyesno(
            "큰 이동",
            f"한 번에 {dist:.0f} mm, 회전 {rot:.0f}° 움직여요 "
            f"(기준 {MAX_LINEAR_STEP:g} mm / {MAX_ROT_STEP:g}°).\n"
            "직선 이동은 관절이 크게 꺾일 수 있어요. 정말 이동할까요?",
        ):
            refresh(entries_x, entries_j)
            return
        ok, why = check_line(cur, pos)
        if not ok:
            refresh(entries_x, entries_j)
            refuse("관절 한계 (직선 경로)", why)
            return

        is_success = movel(pos, vel=inputs[-2], acc=inputs[-1], ref=DR_BASE)
        mwait()
        update_entries(get_current_posj(), entries_j)
        update_entries(get_current_posx()[0], entries_x)

        print("current posx : ", get_current_posx())
        print("current posj : ", get_current_posj())
        print("is_success:", is_success)

    # Utility to update entry fields
    def update_entries(values, entries):
        for i, val in enumerate(values):
            if val is not None:   # 0.0도 화면에 반영 (안 그러면 예전 값이 남아 다음 이동에 쓰임)
                entries[i].delete(0, tk.END)
                entries[i].insert(0, str(round(val, 3)))

    def copy_to_clipboard(root, entries, type_name):
        root.clipboard_clear()
        clipboard_text = (
            f"{type_name}(["
            + ", ".join([str(entry.get()) for entry in entries[:6]])
            + "])"
        )
        root.clipboard_append(clipboard_text)
        print(f"copy to clipboard: {clipboard_text}")
        root.update()

    def z_axis_alignment(axes_data, joint_data):
        vect = [0, 0, -1]
        parallel_axis(vect, DR_AXIS_Z, DR_BASE)
        mwait()
        print("move to z-axis alignment")
        update_entries(get_current_posj(), joint_data)
        update_entries(get_current_posx()[0], axes_data)

    # Increment functions for joint and linear positions
    def increment_joint(entries_x, entries_j, idx, increment):
        if prog["running"]:
            return
        temp = float(entries_j[idx].get())
        print("temp : ", temp, idx)
        entries_j[idx].delete(0, tk.END)
        entries_j[idx].insert(0, str(round(temp + increment, 3)))
        move_by_joint(entries_x, entries_j)

    def increment_linear(entries_x, entries_j, idx, increment):
        if prog["running"]:
            return
        temp = float(entries_x[idx].get())
        print("temp : ", temp, idx)
        entries_x[idx].delete(0, tk.END)
        entries_x[idx].insert(0, str(round(temp + increment, 3)))
        move_by_line(entries_x, entries_j)

    # UI setup
    root = tk.Tk()
    root.title("Pos 설정")

    # Joint and Axis Position Data(row: 1~8, Joint and Axis Entries)
    print("create_position_entries")
    joint_data, axes_data = create_position_entries(
        root,
        labels=["J1", "J2", "J3", "J4", "J5", "J6", "Speed", "Acc"],
        default_values=get_current_posj() + [VELOCITY, ACC],
        col=0,
        max_val=50,
    ), create_position_entries(
        root,
        labels=["X", "Y", "Z", "Rx", "Ry", "Rz", "Speed", "Acc"],
        default_values=get_current_posx()[0] + [VELOCITY, ACC],
        col=5,
        max_val=100,
    )

    # Control Buttons(row: [9, 11], movej, movel, copy, z-axis alignment)
    print("control buttons")
    create_control_buttons(root, joint_data, axes_data)

    # Increment Input Fields(row: 9, Increment Entry)
    increment_j_value = create_label_entry(root, "Joint Increment", 9, 1)
    increment_x_value = create_label_entry(root, "Linear Increment", 9, 6)
    # Increment Buttons(row: 1~7, +- buttons)
    print("increment buttons")
    create_increment_buttons(
        root, joint_data, axes_data, increment_j_value, increment_x_value
    )

    # ── 프로그램 실행: Creator3D 코드(movej / movel / gripper ...)를 붙여넣어 순서대로 실행 ──
    # 단계마다 위의 관절 한계 보호를 그대로 거칩니다. 속도는 코드의 vel 과 위 Speed 슬라이더 중 작은 값.
    frame = tk.LabelFrame(root, text="프로그램 — Creator3D 코드를 붙여넣으세요 (gripper 는 건너뜀)")
    frame.grid(row=12, column=0, columnspan=9, padx=10, pady=10, sticky="ew")
    text = tk.Text(frame, height=10, width=95, font=("monospace", 9), undo=True)
    scroll = tk.Scrollbar(frame, command=text.yview)
    text.configure(yscrollcommand=scroll.set)
    text.grid(row=0, column=0, columnspan=5, sticky="nsew")
    scroll.grid(row=0, column=5, sticky="ns")
    text.tag_configure("cur", background="#fff2a8")
    text.tag_configure("bad", background="#ffc9c9")
    status = tk.Label(frame, text="코드를 붙여넣고 「코드 확인」을 누르세요", anchor="w", justify="left", wraplength=700)
    status.grid(row=2, column=0, columnspan=6, sticky="ew", pady=(4, 0))

    def set_status(msg):
        status.config(text=msg)
        print("[프로그램]", msg)

    def mark(line, tag="cur"):
        text.tag_remove("cur", "1.0", "end")
        text.tag_remove("bad", "1.0", "end")
        if line:
            text.tag_add(tag, f"{line}.0", f"{line}.end")
            text.see(f"{line}.0")

    def on_edit(_e=None):
        if text.edit_modified():
            prog.update(steps=[], i=0)   # 코드를 고치면 다시 읽어야 함
            text.edit_modified(False)
    text.bind("<<Modified>>", on_edit)

    def load_program():
        try:
            steps = parse_program(text.get("1.0", "end"), default_vel=float(joint_data[6].get()))
        except ValueError as e:
            m = re.match(r"(\d+)번째", str(e))
            mark(int(m.group(1)) if m else None, "bad")
            set_status(f"코드 오류: {e}")
            return False
        if not steps:
            set_status("실행할 명령이 없어요")
            return False
        prog.update(steps=steps, i=0)
        moves = sum(s["kind"] in ("movej", "movel") for s in steps)
        skips = sum(s["kind"] == "skip" for s in steps)
        bad = [s for s in steps if s["kind"] == "movej" and joint_violations(s["pos"])]
        msg = f"{len(steps)}단계 읽음 (이동 {moves}, 건너뜀 {skips})"
        if bad:
            mark(bad[0]["line"], "bad")
            msg += f" — ⚠ {bad[0]['line']}번째 줄 movej 가 관절 한계를 넘어요: {', '.join(joint_violations(bad[0]['pos']))}"
        else:
            mark(None)
            msg += " — 「한 단계」 또는 「▶ 전체 실행」"
        set_status(msg)
        return True

    def run_step(st):
        """한 단계 실행 → (성공, 이유)"""
        if st["kind"] == "movej":
            bad = joint_violations(st["pos"])
            if bad:
                return False, "관절 한계: " + ", ".join(bad)
            v = min(st["vel"], float(joint_data[6].get()))
            a = min(st["acc"], v * 2)
            res = movej(st["pos"], vel=v, acc=a)
        elif st["kind"] == "movel":
            cur = list(get_current_posx()[0])
            ok, why = check_line(cur, st["pos"])
            if not ok:
                return False, why.replace("\n", " ")
            v = min(st["vel"], float(axes_data[6].get()))
            a = min(st["acc"], v * 2)
            res = movel(st["pos"], vel=v, acc=a, ref=DR_BASE)
        else:
            return True, ""   # wait 은 tick 에서 기다림, skip 은 그냥 넘어감
        mwait()
        refresh(axes_data, joint_data)
        if res != 0:
            return False, f"로봇이 이동을 거부했어요 (결과 {res}) — 로봇 상태(보호정지·한계)를 확인하세요"
        return True, ""

    def set_running(on):
        prog["running"] = on
        for b in (btn_load, btn_step, btn_run):
            b.config(state=tk.DISABLED if on else tk.NORMAL)
        btn_stop.config(state=tk.NORMAL if on else tk.DISABLED)

    def do_one():
        steps, i = prog["steps"], prog["i"]
        if i >= len(steps):
            set_running(False)
            mark(None)
            set_status(f"프로그램 끝 ({len(steps)}단계)")
            return False
        st = steps[i]
        mark(st["line"])
        set_status(f"{i + 1}/{len(steps)} · {st['line']}번째 줄 · {describe(st)}")
        root.update_idletasks()
        ok, why = run_step(st)
        if not ok:
            mark(st["line"], "bad")
            set_running(False)
            set_status(f"{i + 1}/{len(steps)}단계({st['line']}번째 줄)에서 멈춤: {why}")
            messagebox.showwarning("프로그램 멈춤", f"{st['line']}번째 줄에서 멈췄어요.\n\n{why}\n\n로봇은 이 줄을 실행하지 않았어요.")
            return False
        prog["i"] = i + 1
        return True

    def tick():
        if not prog["running"]:
            return
        steps, i = prog["steps"], prog["i"]
        delay = int(steps[i]["s"] * 1000) if i < len(steps) and steps[i]["kind"] == "wait" else 50
        if do_one():
            root.after(delay, tick)

    def step_once():
        if prog["running"]:
            return
        if not prog["steps"] and not load_program():
            return
        do_one()

    def run_all():
        if prog["running"]:
            return
        steps = prog["steps"]
        if steps and 0 < prog["i"] < len(steps):
            ans = messagebox.askyesnocancel("이어서 실행", f"{prog['i'] + 1}단계부터 이어서 할까요?\n(아니오 = 처음부터)")
            if ans is None:
                return
            if not ans and not load_program():
                return
        elif not load_program():
            return
        steps = prog["steps"]
        first = next((s for s in steps[prog["i"]:] if s["kind"] in ("movej", "movel")), None)
        warn = ""
        if first and first["kind"] == "movej":
            step = max(abs(a - b) for a, b in zip(first["pos"], list(get_current_posj())))
            warn = f"\n첫 이동에서 관절이 최대 {step:.0f}° 움직여요."
        if not messagebox.askyesno(
            "프로그램 실행",
            f"{len(steps) - prog['i']}단계를 실행해요.{warn}\n"
            "단계마다 관절 한계를 확인하고, 걸리면 그 줄 앞에서 멈춰요.\n"
            "「■ 정지」는 지금 하던 동작이 끝난 뒤 멈춰요. 급하면 비상정지 버튼!\n\n시작할까요?",
        ):
            return
        set_running(True)
        root.after(10, tick)

    def stop_program():
        if prog["running"]:
            set_running(False)
            set_status(f"정지함 ({prog['i']}/{len(prog['steps'])}단계까지 실행) — 「▶ 전체 실행」으로 이어서 할 수 있어요")

    btn_load = tk.Button(frame, text="코드 확인", command=load_program)
    btn_step = tk.Button(frame, text="한 단계", command=step_once)
    btn_run = tk.Button(frame, text="▶ 전체 실행", command=run_all)
    btn_stop = tk.Button(frame, text="■ 정지", command=stop_program, state=tk.DISABLED)
    for c, b in enumerate((btn_load, btn_step, btn_run, btn_stop)):
        b.grid(row=1, column=c, padx=4, pady=6, sticky="w")

    # Run UI
    root.mainloop()


if __name__ == "__main__":
    main()
