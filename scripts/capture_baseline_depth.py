#!/usr/bin/env python3
"""Record an empty planting-bed depth baseline from ROS 2 RealSense aligned depth.

ROS 2 Jazzy / RealSense D455F; saves a 2-D ROI baseline in metres, plus valid ratios.
This records the EMPTY surface, not seedling detection or robot-base coordinates.
"""
import time
import warnings
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
ROI = (465, 56, 386, 478)  # x, y, width, height in FULL color image pixels
FRAMES = 50
MIN_VALID_RATIO = 0.70
TIMEOUT_SECONDS = 35
OUTPUT = Path.home() / 'Agriculture_Ws' / 'data' / 'baseline_depth.npz'
PREVIEW = Path.home() / 'Agriculture_Ws' / 'data' / 'baseline_depth_preview.png'


class BaselineCapture(Node):
    def __init__(self):
        super().__init__('baseline_depth_capture')
        self.bridge = CvBridge()
        self.samples = []
        self.frame_shape = None
        self.frame_id = ''
        self.enc = None
        self.error = None
        self.sub = self.create_subscription(
            Image, TOPIC, self.on_depth, qos_profile_sensor_data
        )
        self.get_logger().info(
            f'빈 작업면 측정 시작: {FRAMES} frames, ROI={ROI}; objects MUST be removed'
        )

    def on_depth(self, msg):
        if len(self.samples) >= FRAMES or self.error:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth.ndim != 2:
                raise ValueError(f'Expected single-channel depth, got {depth.shape}')

            if msg.encoding == '16UC1':
                # RealSense ROS aligned 16UC1 convention: millimetres -> metres
                depth_m = depth.astype(np.float32) * 0.001
            elif msg.encoding == '32FC1':
                depth_m = depth.astype(np.float32)
            else:
                raise ValueError(f'Unexpected depth encoding: {msg.encoding}')

            x, y, w, h = ROI
            image_h, image_w = depth_m.shape
            if x < 0 or y < 0 or x + w > image_w or y + h > image_h:
                raise ValueError(
                    f'ROI {ROI} outside aligned-depth image {image_w}x{image_h}. '
                    'Camera resolution or ROI may have changed.'
                )
            if self.frame_shape is not None and depth_m.shape != self.frame_shape:
                raise ValueError('Depth image resolution changed during capture.')
            self.frame_shape = depth_m.shape
            self.frame_id = msg.header.frame_id
            self.enc = msg.encoding

            patch = depth_m[y:y+h, x:x+w]
            valid = np.isfinite(patch) & (patch > 0.10) & (patch < 6.0)
            sample = np.where(valid, patch, np.nan)
            self.samples.append(sample.copy())
            if len(self.samples) % 10 == 0 or len(self.samples) == 1:
                self.get_logger().info(f'Captured {len(self.samples)}/{FRAMES} frames')
        except Exception as exc:
            self.error = str(exc)
            self.get_logger().error(self.error)


def main():
    rclpy.init()
    node = BaselineCapture()
    try:
        start = time.monotonic()
        while (rclpy.ok() and len(node.samples) < FRAMES and not node.error
               and time.monotonic() - start < TIMEOUT_SECONDS):
            rclpy.spin_once(node, timeout_sec=0.2)

        if node.error:
            raise RuntimeError(node.error)
        if len(node.samples) < FRAMES:
            raise RuntimeError(
                f'Only {len(node.samples)}/{FRAMES} frames received. '
                'Confirm RealSense node and aligned-depth topic are running.'
            )

        frames = np.stack(node.samples, axis=0)
        valid_ratio = np.isfinite(frames).mean(axis=0).astype(np.float32)
        with warnings.catch_warnings():
            warnings.filterwarnings('ignore', message='All-NaN slice encountered')
            baseline = np.nanmedian(frames, axis=0).astype(np.float32)
        baseline[valid_ratio < MIN_VALID_RATIO] = np.nan
        good = np.isfinite(baseline)

        if not good.any():
            raise RuntimeError('No valid baseline pixels in ROI; check depth coverage.')

        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            OUTPUT,
            baseline_depth_m=baseline,
            valid_ratio=valid_ratio,
            roi_px=np.array(ROI, dtype=np.int32),
            full_image_size_wh=np.array(node.frame_shape[::-1], dtype=np.int32),
            frame_id=np.array(node.frame_id),
            depth_encoding=np.array(node.enc),
            frame_count=np.array(FRAMES),
            min_valid_ratio=np.array(MIN_VALID_RATIO),
        )

        # False-color ROI preview: display only, not depth source data.
        lo, hi = np.percentile(baseline[good], [2, 98])
        if hi - lo < 0.005:
            hi = lo + 0.005
        intensity = (255 * np.clip((baseline - lo) / (hi - lo), 0, 1))
        image8 = np.nan_to_num(intensity, nan=0).astype(np.uint8)
        color = cv2.applyColorMap(image8, cv2.COLORMAP_TURBO)
        color[~good] = (0, 0, 0)
        cv2.imwrite(str(PREVIEW), color)

        node.get_logger().info(f'SAVED: {OUTPUT}')
        node.get_logger().info(f'PREVIEW: {PREVIEW}')
        node.get_logger().info(
            f'ROI valid pixels: {good.mean()*100:.1f}% | '
            f'median depth: {np.median(baseline[good]):.3f} m | '
            f'frame: {node.frame_id}'
        )
    except Exception as exc:
        node.get_logger().error(f'Baseline capture failed: {exc}')
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
