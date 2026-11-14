import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

COLOR_TOPIC = '/camera/camera/color/image_raw'
DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
INFO_TOPIC = '/camera/camera/color/camera_info'

VIEW_WIDTH = 640
MAX_DEPTH_M = 2.0


class LiveCamera(Node):
    def __init__(self):
        super().__init__('live_camera_view')
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.info = None
        self.selected = None

        self.create_subscription(
            Image, COLOR_TOPIC, self.color_cb,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Image, DEPTH_TOPIC, self.depth_cb,
            qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, INFO_TOPIC, self.info_cb,
            qos_profile_sensor_data
        )

        cv2.namedWindow('RealSense RGB + Depth')
        cv2.setMouseCallback(
            'RealSense RGB + Depth', self.on_mouse
        )
        print('Waiting for RGB and Depth...')
        print('Click RGB image to inspect XYZ. Press Q to quit.')

    def color_cb(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8'
        )

    def depth_cb(self, msg):
        raw = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough'
        )
        if msg.encoding == '16UC1':
            self.depth = raw.astype(np.float32) / 1000.0
        elif msg.encoding == '32FC1':
            self.depth = raw.astype(np.float32)

    def info_cb(self, msg):
        self.info = msg

    def on_mouse(self, event, x, y, flags, userdata):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.color is None or x >= VIEW_WIDTH:
            return

        h, w = self.color.shape[:2]
        view_h = round(h * VIEW_WIDTH / w)
        if not (0 <= y < view_h):
            return

        u = min(int(x * w / VIEW_WIDTH), w - 1)
        v = min(int(y * h / view_h), h - 1)
        self.selected = (u, v)

    def show(self):
        if self.color is None or self.depth is None:
            return

        rgb = self.color.copy()
        depth = self.depth

        h, w = rgb.shape[:2]
        if depth.shape != (h, w):
            print('RGB/Depth resolution mismatch')
            return

        valid = np.isfinite(depth) & (depth > 0)
        clipped = np.clip(
            np.nan_to_num(depth, nan=0.0),
            0, MAX_DEPTH_M
        )

        gray = (clipped / MAX_DEPTH_M * 255).astype(
            np.uint8
        )
        depth_color = cv2.applyColorMap(
            gray, cv2.COLORMAP_TURBO
        )
        depth_color[~valid] = (0, 0, 0)

        u, v = self.selected or (w // 2, h // 2)
        z = float(depth[v, u])

        if np.isfinite(z) and z > 0:
            distance_text = f'Pixel ({u},{v}) Depth: {z:.3f} m'
        else:
            distance_text = f'Pixel ({u},{v}) Depth: INVALID'

        if (self.info is not None
                and self.info.width == w
                and self.info.height == h
                and np.isfinite(z) and z > 0):
            k = self.info.k
            if k[0] > 0 and k[4] > 0:
                x = (u - k[2]) * z / k[0]
                y = (v - k[5]) * z / k[4]
                distance_text += (
                    f' | XYZ: {x:.3f}, {y:.3f}, {z:.3f} m'
                )

        cv2.circle(rgb, (u, v), 6, (0, 255, 0), 2)
        cv2.circle(depth_color, (u, v), 6, (255, 255, 255), 2)

        view_h = round(h * VIEW_WIDTH / w)
        target = (VIEW_WIDTH, view_h)

        rgb = cv2.resize(rgb, target)
        depth_color = cv2.resize(depth_color, target)

        combined = np.hstack((rgb, depth_color))
        cv2.putText(
            combined, distance_text, (10, view_h - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.52,
            (255, 255, 255), 2
        )
        cv2.imshow('RealSense RGB + Depth', combined)


def main():
    rclpy.init()
    node = LiveCamera()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            node.show()
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
