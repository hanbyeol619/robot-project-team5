"""Read camera-frame EMPTY planting slots; print only, never command robot."""
import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EmptySlotListener(Node):
    def __init__(self):
        super().__init__('empty_slot_listener')
        self.sub = self.create_subscription(
            String, '/vision/planting_slots', self.callback, 10
        )
        self.get_logger().info('Listening to /vision/planting_slots (CAMERA FRAME ONLY)')

    def callback(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f'Invalid JSON: {e}')
            return
        if data.get('frame_id') != 'camera_color_optical_frame':
            self.get_logger().warn(f"Unexpected frame: {data.get('frame_id')}")
            return
        if data.get('robot_base_calibrated') is not False:
            self.get_logger().warn('Unexpected coordinate calibration flag; not using coordinates')
            return
        empty = []
        for item in data.get('slots', []):
            if item.get('status') != 'EMPTY' or item.get('valid_fraction', 0) < 0.95:
                continue
            xyz = item.get('camera_xyz_m')
            if not isinstance(xyz, list) or len(xyz) != 3 or not all(
                isinstance(val, (float, int)) and math.isfinite(val) for val in xyz
            ):
                continue
            empty.append((item.get('id', '?'), xyz))
        self.get_logger().info(f'Empty slots: {len(empty)}')
        for slot_id, (x, y, z) in empty:
            self.get_logger().info(
                f'{slot_id} | X={x:.4f}m Y={y:.4f}m Z={z:.4f}m (CAMERA ONLY)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = EmptySlotListener()
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
