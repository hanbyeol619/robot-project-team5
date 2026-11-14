"""Demo consumer for /vision/objects. Never sends robot motion commands."""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ObjectListener(Node):
    def __init__(self):
        super().__init__('agri_object_listener')
        self.last_print = 0.0
        self.create_subscription(String, '/vision/objects', self.on_objects, 10)
        self.get_logger().info('Listening to /vision/objects (NO robot motion)')

    def on_objects(self, msg):
        # Avoid printing large diagnostic records 5 times per second.
        now = time.monotonic()
        if now - self.last_print < 1.0:
            return
        self.last_print = now
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Malformed JSON received')
            return
        frame = result.get('frame_id', 'unknown')
        mode = result.get('mode', 'unknown')
        self.get_logger().info(f'mode={mode} frame={frame}')
        for obj in result.get('objects', []):
            xyz = obj.get('camera_xyz_m')
            if xyz is None:
                self.get_logger().info(f"  {obj['id']}: NO DEPTH")
                continue
            note = 'REFERENCE ONLY' if not obj.get('presence_verified') else 'YOLO'
            self.get_logger().info(
                f"  {obj['id']} {obj['label']}: "
                f"u,v={obj['center_px_full']}  Z={obj['depth_m']:.3f}m "
                f"XYZ={[round(x, 3) for x in xyz]} ({note})")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectListener()
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
