#!/usr/bin/env python3
"""가짜 로봇 노드 — 실제 로봇 노드가 완성되기 전 control_node 시험용.

/robot/task 를 받아 일정 시간 기다린 뒤 /robot/result 로 성공을 돌려준다.
/robot/stop 의 stop/pause/resume 도 흉내 낸다.
실제 로봇 노드는 이 파일과 똑같은 토픽·메시지 형식만 맞추면 된다 (README 참고).

옵션 파라미터
  fail_action  : 이 action 이 오면 일부러 실패 (예: 'water') — 실패 처리 시험용
  fake_vision  : True 면 /vision/planting_slots 도 가짜로 발행 (카메라 없이 시험)
"""
import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DURATION = {  # speed 1.0 기준 소요 시간(초)
    'home': 1.5, 'dig': 2.0, 'plant': 2.5, 'water': 1.5, 'press': 1.5,
    'harvest': 3.0, 'gripper': 0.5,
}


class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot_node')
        self.declare_parameter('fail_action', '')
        self.declare_parameter('fake_vision', False)
        self.fail_action = self.get_parameter('fail_action').value
        self.fake_vision = bool(self.get_parameter('fake_vision').value)

        self.pub = self.create_publisher(String, '/robot/result', 10)
        self.create_subscription(String, '/robot/task', self.on_task, 10)
        self.create_subscription(String, '/robot/stop', self.on_stop, 10)
        self.stopped = threading.Event()
        self.paused = threading.Event()
        self.busy = threading.Lock()

        # 가짜 비전: S1 OCCUPIED, 나머지 EMPTY 로 시작, plant 가 끝나면 OCCUPIED 로 바뀜
        self.slots = {'S1': 'OCCUPIED', 'S2': 'EMPTY', 'S3': 'EMPTY', 'S4': 'EMPTY'}
        if self.fake_vision:
            self.vis_pub = self.create_publisher(String, '/vision/planting_slots', 10)
            self.create_timer(1.0, self.publish_vision)
        self.get_logger().warn(f'MOCK 로봇 실행 중 (fake_vision={self.fake_vision}) — 실제 로봇은 움직이지 않음')

    def on_task(self, msg):
        task = json.loads(msg.data)
        threading.Thread(target=self.execute, args=(task,), daemon=True).start()

    def execute(self, task):
        with self.busy:
            self.stopped.clear()
            action = task.get('action')
            speed = max(0.1, float(task.get('speed', 0.6)))
            self.get_logger().info(f'실행: {action} slot={task.get("slot")} xyz={task.get("camera_xyz_m")}')
            remain = DURATION.get(action, 1.0) / speed
            while remain > 0:
                if self.stopped.is_set():
                    self.reply(task, False, '정지 명령으로 중단')
                    return
                time.sleep(0.05)
                if not self.paused.is_set():
                    remain -= 0.05
            if action == self.fail_action:
                self.reply(task, False, f'MOCK: {action} 일부러 실패')
                return
            if action == 'plant' and task.get('slot') in self.slots:
                self.slots[task['slot']] = 'OCCUPIED'
            self.reply(task, True)

    def reply(self, task, ok, error=None):
        res = {'id': task.get('id'), 'action': task.get('action'), 'ok': ok}
        if error:
            res['error'] = error
        self.pub.publish(String(data=json.dumps(res, ensure_ascii=False)))
        self.get_logger().info(f'결과: {res}')

    def on_stop(self, msg):
        mode = json.loads(msg.data).get('mode')
        if mode == 'stop':
            self.paused.clear()
            self.stopped.set()
        elif mode == 'pause':
            self.paused.set()
        elif mode == 'resume':
            self.paused.clear()
        self.get_logger().warn(f'/robot/stop: {mode}')

    def publish_vision(self):
        centers = {'S1': [-0.1, -0.1], 'S2': [0.1, -0.1], 'S3': [-0.1, 0.1], 'S4': [0.1, 0.1]}
        payload = {
            'schema_version': '0.1', 'mode': 'depth_grid',
            'frame_id': 'camera_color_optical_frame', 'robot_base_calibrated': False,
            'slots': [{'id': k, 'status': v, 'valid_fraction': 1.0, 'raised_fraction': 0.0,
                       'camera_xyz_m': centers[k] + [0.6]} for k, v in self.slots.items()],
        }
        self.vis_pub.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = MockRobot()
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
