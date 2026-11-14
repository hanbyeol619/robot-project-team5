
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraROI(Node):
    def __init__(self):
        super().__init__('camera_roi')

        self.bridge = CvBridge()
        self.frame = None

        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(
            msg, 'bgr8'
        )


def main():
    rclpy.init()
    node = CameraROI()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            if node.frame is None:
                continue

            cv2.imshow('Camera', node.frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('s'):
                roi = cv2.selectROI(
                    'Select Work Area',
                    node.frame,
                    False,
                    False
                )

                x, y, w, h = map(int, roi)

                if w > 0 and h > 0:
                    cropped = node.frame[
                        y:y+h, x:x+w
                    ]

                    cv2.imwrite(
                        '/home/jun/ros2_ws/work_area.png',
                        cropped
                    )

                    print(
                        f'ROI: x={x}, y={y}, '
                        f'w={w}, h={h}'
                    )

                    cv2.imshow(
                        'Work Area',
                        cropped
                    )

            elif key == ord('q'):
                break

    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

