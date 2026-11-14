"""Stage 1: RealSense aligned RGB-D -> object samples -> ROS topics.

manual mode = four fixed reference boxes (NOT automatic detection).
yolo mode = real dynamic detection when an appropriately trained model exists.
Positions are camera optical frame only: never send directly to the robot.
"""
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .geometry import bbox_to_full, pixel_to_camera_xyz, robust_depth_m


class AgriVisionNode(Node):
    def __init__(self):
        super().__init__('agri_vision_node')
        self.declare_parameter('mode', 'manual')
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('max_rate_hz', 5.0)
        self.declare_parameter('depth_scale_m_per_unit', 0.001)
        self.declare_parameter('depth_patch_radius', 4)
        self.declare_parameter('sync_slop_sec', 0.12)
        self.declare_parameter('config_path', '')

        self.mode = self.get_parameter('mode').value
        if self.mode not in ('manual', 'yolo'):
            raise ValueError('mode must be manual or yolo')
        config_path = self.get_parameter('config_path').value
        if not config_path:
            config_path = os.path.join(
                get_package_share_directory('vision'), 'config', 'objects.json')
        with open(os.path.expanduser(config_path), 'r', encoding='utf-8') as f:
            self.config = json.load(f)
        self.roi = list(map(int, self.config['roi']))
        self.reference_size = list(map(int, self.config['reference_image_size']))
        self.model = None
        if self.mode == 'yolo':
            model_path = os.path.expanduser(self.get_parameter('model_path').value)
            if not Path(model_path).is_file():
                raise FileNotFoundError('YOLO model not found: ' + model_path)
            from ultralytics import YOLO  # optional: only needed in yolo mode
            self.model = YOLO(model_path)

        self.bridge = CvBridge()
        self.K = None
        self.D = []
        self.distortion_model = 'plumb_bob'
        self.info_size = None
        self.last_processed_at = 0.0
        self.last_synced_at = 0.0
        self.last_notice_at = 0.0
        self.frame_counter = 0

        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.camera_info_cb, qos_profile_sensor_data)
        self.rgb_sub = Subscriber(
            self, Image, '/camera/camera/color/image_raw',
            qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw',
            qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=15,
            slop=float(self.get_parameter('sync_slop_sec').value))
        self.sync.registerCallback(self.on_rgbd)

        self.data_pub = self.create_publisher(String, '/vision/objects', 10)
        self.pose_pub = self.create_publisher(PoseArray, '/vision/object_poses', 10)
        self.image_pub = self.create_publisher(Image, '/vision/debug_image', 10)
        self.create_timer(5.0, self.check_stream)
        self.get_logger().info(
            f'mode={self.mode}, roi={self.roi}, '
            '3D output is CAMERA OPTICAL FRAME, NOT ROBOT BASE')

    def camera_info_cb(self, msg):
        self.K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = list(msg.d)
        self.distortion_model = msg.distortion_model
        self.info_size = (int(msg.width), int(msg.height))

    def check_stream(self):
        if self.last_synced_at == 0.0:
            self.get_logger().warn(
                'No synchronized RGB+aligned-depth yet; check camera topics and QoS')
        elif time.monotonic() - self.last_synced_at > 5:
            self.get_logger().warn('RGB-D stream stopped or unsynchronized')
        if self.K is None:
            self.get_logger().warn('No CameraInfo received yet')

    def get_candidates(self, bgr):
        rx, ry, rw, rh = self.roi
        if self.mode == 'manual':
            # Fixed reference rectangles; existence / identity is NOT verified!
            return [
                {'id': obj['id'], 'label': obj['label'],
                 'bbox': bbox_to_full(obj['bbox_roi'], self.roi),
                 'confidence': None, 'presence_verified': False}
                for obj in self.config['objects']
            ]

        crop = bgr[ry:ry+rh, rx:rx+rw]
        result = self.model.predict(
            crop, conf=float(self.get_parameter('confidence').value),
            verbose=False)[0]
        found = []
        if result.boxes is None:
            return found
        for i, box in enumerate(result.boxes):
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            bbox = [int(x1+rx), int(y1+ry), int(x2+rx), int(y2+ry)]
            label = str(result.names[int(box.cls[0])])
            found.append({
                'id': f'frame_{i+1:02d}', # NOT a persistent tracking ID
                'label': label, 'bbox': bbox,
                'confidence': round(float(box.conf[0]), 4),
                'presence_verified': True
            })
        return found

    def on_rgbd(self, rgb_msg, depth_msg):
        self.last_synced_at = time.monotonic()
        hz = max(0.1, float(self.get_parameter('max_rate_hz').value))
        if self.last_synced_at - self.last_processed_at < 1.0 / hz:
            return
        self.last_processed_at = self.last_synced_at
        if self.K is None:
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge error: {exc}')
            return

        h, w = bgr.shape[:2]
        rx, ry, rw, rh = self.roi
        if list([w, h]) != self.reference_size:
            self.get_logger().error(
                f'Unexpected RGB size {(w, h)}; expected {self.reference_size}. '
                'Adjust config before using stored ROI and boxes')
            return
        if depth.shape[:2] != (h, w):
            self.get_logger().error(
                f'RGB size {(w, h)} differs from aligned Depth shape {depth.shape[:2]}')
            return
        if self.info_size != (w, h):
            self.get_logger().warn('CameraInfo resolution does not match RGB resolution')
            return
        if not (0 <= rx and 0 <= ry and rx+rw <= w and ry+rh <= h):
            self.get_logger().error('Configured ROI is outside RGB image')
            return

        self.frame_counter += 1
        frame_id = rgb_msg.header.frame_id or 'camera_color_optical_frame'
        overlay = bgr.copy()
        cv2.rectangle(overlay, (rx, ry), (rx+rw, ry+rh), (255, 200, 0), 2)
        detections = []
        poses = PoseArray()
        poses.header = rgb_msg.header
        poses.header.frame_id = frame_id
        for candidate in self.get_candidates(bgr):
            x1, y1, x2, y2 = candidate['bbox']
            u = int(round((x1+x2)/2))
            v = int(round((y1+y2)/2))
            z = robust_depth_m(
                depth, u, v, encoding=depth_msg.encoding,
                scale_m_per_unit=float(self.get_parameter('depth_scale_m_per_unit').value),
                patch_radius=int(self.get_parameter('depth_patch_radius').value))
            xyz = None
            if z is not None:
                try:
                    xyz = pixel_to_camera_xyz(
                        u, v, z, self.K, self.D, self.distortion_model)
                except ValueError as exc:
                    self.get_logger().warn(str(exc))
            record = dict(candidate)
            record.update({
                'center_px_full': [u, v],
                'center_px_roi': [u-rx, v-ry],
                'depth_m': z,
                'camera_xyz_m': xyz,
                'depth_valid': xyz is not None
            })
            detections.append(record)
            color = (0, 200, 0) if xyz is not None else (0, 0, 255)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
            cv2.circle(overlay, (u, v), 4, (0, 0, 255), -1)
            z_str = f'{z:.3f}m' if z is not None else 'NO DEPTH'
            title = f'{candidate["id"]} {candidate["label"]} Z={z_str}'
            cv2.putText(overlay, title, (x1, max(22, y1-8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.47, color, 1, cv2.LINE_AA)
            if xyz is not None:
                p = Pose()
                p.position.x, p.position.y, p.position.z = xyz
                p.orientation.w = 1.0  # placeholder, object orientation unknown
                poses.poses.append(p)

        payload = {
            'schema_version': '0.1', 'mode': self.mode,
            'source': 'manual_reference_boxes' if self.mode == 'manual' else 'yolo',
            'frame_id': frame_id,
            'stamp': {'sec': rgb_msg.header.stamp.sec,
                      'nanosec': rgb_msg.header.stamp.nanosec},
            'depth_encoding': depth_msg.encoding,
            'depth_scale_m_per_unit': float(
                self.get_parameter('depth_scale_m_per_unit').value)
                if depth_msg.encoding == '16UC1' else None,
            'roi_px': self.roi,
            'objects': detections
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.data_pub.publish(msg)
        self.pose_pub.publish(poses)
        if self.image_pub.get_subscription_count() > 0:
            imgmsg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            imgmsg.header = rgb_msg.header
            self.image_pub.publish(imgmsg)

        if self.frame_counter % 25 == 0:
            good = sum(d['depth_valid'] for d in detections)
            self.get_logger().info(f'{len(detections)} regions / {good} valid 3D samples')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AgriVisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
