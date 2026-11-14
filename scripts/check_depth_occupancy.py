#!/usr/bin/env python3
"""ROS 2 Jazzy / RealSense D455F: compare a live depth image with EMPTY baseline.

This is an occupancy prototype, NOT a seedling classifier or robot target generator.
Reads baseline_depth.npz from the previous capture script (pixel-wise 50-frame median).
Captures 20 aligned-depth frames, detects raised areas and saves debug overlays.
"""
from pathlib import Path
import time
import warnings

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
BASELINE_FILE = Path.home() / 'Agriculture_Ws' / 'data' / 'baseline_depth.npz'
OUTPUT_IMAGE = Path.home() / 'Agriculture_Ws' / 'data' / 'depth_occupancy_preview.png'
OUTPUT_DATA = Path.home() / 'Agriculture_Ws' / 'data' / 'depth_occupancy_mask.png'
FRAMES = 20
MIN_HEIGHT_M = 0.040        # 4 cm above the previously captured empty surface
MIN_COMPONENT_AREA_PX = 350
MIN_CURRENT_VALID_RATIO = 0.65
TIMEOUT_SECONDS = 30


class DepthCapture(Node):
    def __init__(self, roi_px, expected_wh):
        super().__init__('depth_occupancy_check')
        self.roi_px = tuple(int(v) for v in roi_px)
        self.expected_wh = tuple(int(v) for v in expected_wh)
        self.bridge = CvBridge()
        self.frames = []
        self.error = None
        self.subscription = self.create_subscription(
            Image, TOPIC, self.callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            f'Capturing {FRAMES} aligned depth frames; ROI={self.roi_px}. '
            'Keep camera fixed. Place ONE test object on the bed.'
        )

    def callback(self, msg):
        if self.error or len(self.frames) >= FRAMES:
            return
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if raw.ndim != 2:
                raise RuntimeError(f'Depth image must be single-channel, got {raw.shape}')
            if tuple(raw.shape[::-1]) != self.expected_wh:
                raise RuntimeError(
                    f'Image size {raw.shape[::-1]} differs from baseline '
                    f'{self.expected_wh}; restore camera stream resolution.'
                )
            if msg.encoding == '16UC1':
                depth_m = raw.astype(np.float32) * 0.001
            elif msg.encoding == '32FC1':
                depth_m = raw.astype(np.float32)
            else:
                raise RuntimeError(f'Unsupported depth encoding: {msg.encoding}')

            x, y, w, h = self.roi_px
            roi = depth_m[y:y + h, x:x + w]
            if roi.shape != (h, w):
                raise RuntimeError('ROI is outside current depth image')
            valid = np.isfinite(roi) & (roi > 0.10) & (roi < 6.0)
            self.frames.append(np.where(valid, roi, np.nan).copy())
            if len(self.frames) in (1, 10, FRAMES):
                self.get_logger().info(f'Captured {len(self.frames)}/{FRAMES}')
        except Exception as err:
            self.error = str(err)


def analyze(baseline, frames, roi):
    # Median over frames reduces noise and intermittent missing pixels.
    with warnings.catch_warnings():
        warnings.filterwarnings('ignore', message='All-NaN slice encountered')
        live = np.nanmedian(frames, axis=0).astype(np.float32)
    live_valid = np.isfinite(frames).mean(axis=0) >= MIN_CURRENT_VALID_RATIO
    good = np.isfinite(baseline) & np.isfinite(live) & live_valid
    delta = np.full(baseline.shape, np.nan, dtype=np.float32)
    delta[good] = baseline[good] - live[good]  # positive means raised toward camera
    mask = np.zeros(baseline.shape, dtype=np.uint8)
    mask[good & (delta >= MIN_HEIGHT_M)] = 255
    # Remove isolated depth artifacts; link small breaks in an object.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(
        (c for c in contours if cv2.contourArea(c) >= MIN_COMPONENT_AREA_PX),
        key=lambda c: cv2.boundingRect(c)[0]
    )

    # Visualize baseline-referenced height. Dark = near ground; warmer = raised.
    height_cm = np.nan_to_num(delta, nan=0.0) * 100.0
    gray = np.uint8(np.clip(height_cm / 20.0, 0.0, 1.0) * 255.0)
    preview = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    preview[~good] = (25, 25, 25)
    clean_mask = np.zeros(mask.shape, dtype=np.uint8)
    detections = []
    rx, ry, _, _ = roi
    for i, contour in enumerate(contours, 1):
        cv2.drawContours(clean_mask, [contour], -1, 255, thickness=cv2.FILLED)
        x, y, w, h = cv2.boundingRect(contour)
        M = cv2.moments(contour)
        if M['m00'] == 0:
            continue
        cx = int(round(M['m10'] / M['m00']))
        cy = int(round(M['m01'] / M['m00']))
        component_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(component_mask, [contour], -1, 255, thickness=cv2.FILLED)
        pick = (component_mask > 0) & good
        med_height_cm = float(np.median(delta[pick]) * 100)
        med_depth_m = float(np.median(live[pick]))
        detections.append((i, (cx + rx, cy + ry), med_depth_m, med_height_cm))
        cv2.drawContours(preview, [contour], -1, (0, 255, 0), 2)
        cv2.circle(preview, (cx, cy), 4, (255, 255, 255), -1)
        cv2.putText(preview, f'ID{i} +{med_height_cm:.1f}cm',
                    (x, max(18, y - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.50,
                    (255, 255, 255), 2, cv2.LINE_AA)
    return detections, preview, clean_mask, float(good.mean() * 100)


def main():
    if not BASELINE_FILE.is_file():
        raise SystemExit(f'Baseline file missing: {BASELINE_FILE}')
    with np.load(BASELINE_FILE) as saved:
        baseline = saved['baseline_depth_m'].astype(np.float32)
        roi = saved['roi_px'].astype(int)
        full_wh = saved['full_image_size_wh'].astype(int)
    if baseline.shape != (int(roi[3]), int(roi[2])):
        raise SystemExit('Baseline ROI shape is invalid; redo baseline acquisition.')

    rclpy.init()
    node = DepthCapture(roi, full_wh)
    try:
        start = time.monotonic()
        while rclpy.ok() and len(node.frames) < FRAMES and not node.error:
            if time.monotonic() - start > TIMEOUT_SECONDS:
                raise RuntimeError(f'Timed out at {len(node.frames)}/{FRAMES} frames')
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.error:
            raise RuntimeError(node.error)
        data = np.stack(node.frames, axis=0)
        detections, preview, mask, valid_pct = analyze(baseline, data, roi)
        cv2.imwrite(str(OUTPUT_IMAGE), preview)
        cv2.imwrite(str(OUTPUT_DATA), mask)
        print(f'VALID PIXELS: {valid_pct:.1f}%')
        print(f'RAISED REGIONS: {len(detections)}  (threshold >= {MIN_HEIGHT_M*100:.0f}cm)')
        for idx, pixel, z, height_cm in detections:
            print(f'  ID{idx}: full_pixel={pixel}, median_depth={z:.3f}m, '
                  f'height_above_baseline={height_cm:.1f}cm')
        print(f'PREVIEW: {OUTPUT_IMAGE}')
        print(f'MASK: {OUTPUT_DATA}')
        print('NOTE: Regions indicate height changes, NOT confirmed seedlings.')
    except Exception as err:
        node.get_logger().error(str(err))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
