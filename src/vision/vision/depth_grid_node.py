#!/usr/bin/env python3
"""ROS 2 Jazzy / RealSense D455F: depth-only occupancy in a fixed 2x2 planting grid.

Requires an EMPTY-bed baseline captured by capture_baseline_depth.py.
Deliberately publishes CAMERA frame coordinates, never robot base targets.
No YOLO and no robot motion.
"""
import json
import time
import warnings
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

BASELINE_PATH = Path.home() / 'Agriculture_Ws' / 'data' / 'baseline_depth.npz'
PREVIEW_PATH = Path.home() / 'Agriculture_Ws' / 'data' / 'depth_grid_preview.png'
DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
INFO_TOPIC = '/camera/camera/color/camera_info'
RESULT_TOPIC = '/vision/planting_slots'
DEBUG_TOPIC = '/vision/planting_grid_debug'

# Positions are the FOUR MANUALLY MARKED SLOTS in ROI coordinates (not detected seedlings).
# Set these to true planting centers after testing.
SLOTS_ROI = [
    ('S1', 63, 108),
    ('S2', 265, 104),
    ('S3', 83, 286),
    ('S4', 277, 280),
]

WINDOW_FRAMES = 20
SLOT_RADIUS_PX = 45
RAISED_HEIGHT_M = 0.040
OCCUPIED_FRACTION = 0.08
EMPTY_FRACTION = 0.02
MIN_VALID_FRACTION = 0.75
MAX_GLOBAL_RAISED_FRACTION = 0.25
MAX_STALE_SECONDS = 3.0


# 보관 묘목 C·D·E (화면 번호 5·6·7) 자리: 판(ROI) 바깥 위쪽. 묘목이 있으면 잎 윗부분이 카메라에 가까움
# (11-14 측정: 있으면 하위 5% 깊이 ≈ 0.80m, 없으면 ≈ 0.98m)
SUPPLY_PX = {'C': (535, 55), 'D': (653, 60), 'E': (787, 60)}   # 원본 1280x720 화면 좌표
SUPPLY_HALF = (25, 12)                                          # 살펴볼 네모 크기 (가로, 세로 반)
SUPPLY_PRESENT_M = 0.90                                         # 이보다 가까운 게 있으면 묘목 있음


def classify_slots(baseline, current_stack, slot_positions=SLOTS_ROI):
    """Pure NumPy/OpenCV classification, convenient for offline tests."""
    with warnings.catch_warnings():
        warnings.filterwarnings('ignore', message='All-NaN slice encountered')
        current = np.nanmedian(current_stack, axis=0)
    current_valid = np.isfinite(current_stack).mean(axis=0) >= 0.65
    good = np.isfinite(baseline) & np.isfinite(current) & current_valid
    delta = baseline - current  # raised objects are CLOSER to camera
    raw_raised = good & (delta >= RAISED_HEIGHT_M)
    binary = (raw_raised.astype(np.uint8) * 255)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    raised = binary > 0
    h, w = baseline.shape
    yy, xx = np.ogrid[:h, :w]
    global_valid = float(good.mean())
    global_raised = float(np.count_nonzero(raised & good) / max(1, np.count_nonzero(good)))
    global_problem = (global_valid < 0.70 or global_raised > MAX_GLOBAL_RAISED_FRACTION)
    readings = []
    for slot_id, cx, cy in slot_positions:
        if not (0 <= cx < w and 0 <= cy < h):
            raise ValueError(f'{slot_id}: location {(cx,cy)} outside ROI {(w,h)}')
        disk = (xx - cx) ** 2 + (yy - cy) ** 2 <= SLOT_RADIUS_PX ** 2
        total = int(np.count_nonzero(disk))
        valid = int(np.count_nonzero(disk & good))
        frac_valid = valid / max(total, 1)
        frac_raised = float(np.count_nonzero(disk & raised & good) / max(valid, 1))
        if global_problem or frac_valid < MIN_VALID_FRACTION:
            state = 'UNKNOWN'
        elif frac_raised >= OCCUPIED_FRACTION:
            state = 'OCCUPIED'
        elif frac_raised <= EMPTY_FRACTION:
            state = 'EMPTY'
        else:
            state = 'UNKNOWN'
        # Get bed surface depth, not object-top depth, using the EMPTY baseline.
        patch = baseline[max(0,cy-4):min(h,cy+5),max(0,cx-4):min(w,cx+5)]
        finite = patch[np.isfinite(patch)]
        surface_depth_m = float(np.median(finite)) if finite.size else None
        if surface_depth_m is None:
            state = 'UNKNOWN'
        readings.append(dict(id=slot_id, center_roi_px=[cx,cy], status=state,
                             raised_fraction=round(frac_raised, 4),
                             valid_fraction=round(frac_valid, 4),
                             baseline_surface_depth_m=surface_depth_m))
    return readings, delta, good, raised, global_valid, global_raised


def to_camera_xyz(u, v, depth_m, k):
    fx, fy, cx, cy = float(k[0]),float(k[4]),float(k[2]),float(k[5])
    if fx <= 0 or fy <= 0:
        raise ValueError('camera_info K is invalid')
    return [round((u-cx)*depth_m/fx, 4),
            round((v-cy)*depth_m/fy, 4), round(depth_m,4)]


class DepthGridNode(Node):
    def __init__(self):
        super().__init__('depth_grid_node')
        if not BASELINE_PATH.exists():
            raise RuntimeError(f'Missing baseline file: {BASELINE_PATH}')
        with np.load(BASELINE_PATH) as saved:
            self.baseline = saved['baseline_depth_m'].astype(np.float32)
            self.roi = [int(x) for x in saved['roi_px']]
            self.full_wh = [int(x) for x in saved['full_image_size_wh']]
            self.baseline_frame = str(saved['frame_id'])
        if self.baseline.shape != (self.roi[3], self.roi[2]):
            raise ValueError('Baseline ROI dimensions are inconsistent')
        self.bridge = CvBridge()
        self.frames = deque(maxlen=WINDOW_FRAMES)
        self.supply_frames = deque(maxlen=WINDOW_FRAMES)   # 보관 묘목 깊이 기록
        self.latest_depth_mono_time = None
        self.info = None
        self.latest_header = None
        self.warned = False
        self.error = None
        self.pub = self.create_publisher(String, RESULT_TOPIC, 10)
        self.debug_pub = self.create_publisher(Image, DEBUG_TOPIC, 10)
        self.sub_depth = self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, qos_profile_sensor_data)
        self.sub_info = self.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0, self.on_timer)
        self.get_logger().info(f'Depth Grid started: ROI={self.roi}, 4 slots, baseline={BASELINE_PATH}')
        self.get_logger().warn('OUTPUT IS CAMERA OPTICAL FRAME. NOT ROBOT BASE. DO NOT COMMAND ROBOT.')

    def on_info(self, msg):
        if [msg.width,msg.height] != self.full_wh:
            self.error = f'camera_info size {(msg.width,msg.height)} != baseline size {self.full_wh}'
            return
        self.info = msg

    def on_depth(self, msg):
        if self.error:
            return
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if list(raw.shape[::-1]) != self.full_wh:
                raise ValueError(f'Depth shape {raw.shape} incompatible with baseline {self.full_wh}')
            if self.baseline_frame and msg.header.frame_id != self.baseline_frame:
                raise ValueError(f'Frame changed: {msg.header.frame_id} != {self.baseline_frame}')
            if msg.encoding == '16UC1':
                depth = raw.astype(np.float32)*0.001
            elif msg.encoding == '32FC1':
                depth = raw.astype(np.float32)
            else:
                raise ValueError(f'Unsupported Depth encoding {msg.encoding}')
            x,y,w,h = self.roi
            patch = depth[y:y+h,x:x+w]
            valid = np.isfinite(patch)&(patch>0.10)&(patch<6.0)
            self.frames.append(np.where(valid,patch,np.nan).copy())
            sup = {}
            for sid, (sx, sy) in SUPPLY_PX.items():
                hx, hy = SUPPLY_HALF
                p = depth[max(0,sy-hy):sy+hy, max(0,sx-hx):sx+hx]
                p = p[np.isfinite(p)&(p>0.10)&(p<6.0)]
                sup[sid] = float(np.percentile(p,5)) if p.size > 50 else None
            self.supply_frames.append(sup)
            self.latest_depth_mono_time = time.monotonic()
            self.latest_header = msg.header
        except Exception as exc:
            self.error = str(exc)
            self.get_logger().error(self.error)

    def supply_status(self):
        """최근 프레임의 보관 묘목 상태: PRESENT / EMPTY / UNKNOWN"""
        out = []
        for sid in SUPPLY_PX:
            vals = [f[sid] for f in self.supply_frames if f.get(sid) is not None]
            if len(vals) < max(3, len(self.supply_frames)//2):
                st, near = 'UNKNOWN', None
            else:
                near = float(np.median(vals))
                st = 'PRESENT' if near < SUPPLY_PRESENT_M else 'EMPTY'
            out.append({'id': sid, 'status': st, 'near_depth_m': None if near is None else round(near,3)})
        return out

    def on_timer(self):
        if self.error:
            self.get_logger().error(f'Cannot publish: {self.error}')
            return
        if self.info is None or len(self.frames) < WINDOW_FRAMES:
            if not self.warned:
                self.get_logger().info('Waiting for CameraInfo and 20 Depth frames...')
                self.warned = True
            return
        if time.monotonic() - self.latest_depth_mono_time > MAX_STALE_SECONDS:
            self.get_logger().warn('Depth frames are stale; not publishing new target data')
            return
        self.warned = False
        readings, delta, good, raised, global_valid, global_raised = classify_slots(
            self.baseline, np.stack(self.frames,axis=0)
        )
        rx,ry,_,_ = self.roi
        for obj in readings:
            cx,cy = obj['center_roi_px']
            u,v = rx+cx,ry+cy
            obj['center_full_px'] = [u,v]
            obj['camera_xyz_m'] = (to_camera_xyz(u,v,obj['baseline_surface_depth_m'], self.info.k)
                if obj['baseline_surface_depth_m'] is not None and obj['status']!='UNKNOWN' else None)
        payload = {
            'schema_version':'0.1', 'mode':'depth_grid',
            'frame_id':self.latest_header.frame_id,
            'stamp':{'sec':self.latest_header.stamp.sec,'nanosec':self.latest_header.stamp.nanosec},
            'coordinate_unit':'m',
            'robot_base_calibrated':False,
            'grid_size':len(readings),
            'global_valid_fraction':round(global_valid,4),
            'global_raised_fraction':round(global_raised,4),
            'slots':readings,
            'supply':self.supply_status(),
        }
        msg = String()
        msg.data = json.dumps(payload,ensure_ascii=False)
        self.pub.publish(msg)
        self.get_logger().info(' '.join(f"{v['id']}={v['status']}({v['raised_fraction']:.1%})" for v in readings))
        # Debug imagery: warm colors represent objects closer than baseline.
        intensity = np.uint8(np.clip(np.nan_to_num(delta,nan=0.0)/0.20,0.0,1.0)*255)
        debug = cv2.applyColorMap(intensity,cv2.COLORMAP_TURBO)
        debug[~good] = (15,15,15)
        colors={'EMPTY':(0,255,0),'OCCUPIED':(0,0,255),'UNKNOWN':(0,255,255)}
        for v in readings:
            cx,cy=v['center_roi_px']
            cv2.circle(debug,(cx,cy),SLOT_RADIUS_PX,colors[v['status']],2)
            cv2.circle(debug,(cx,cy),3,(255,255,255),-1)
            cv2.putText(debug,f"{v['id']} {v['status']}", (max(0,cx-43),max(14,cy-49)),
                        cv2.FONT_HERSHEY_SIMPLEX,0.42,colors[v['status']],1,cv2.LINE_AA)
        out=self.bridge.cv2_to_imgmsg(debug,encoding='bgr8')
        out.header=self.latest_header
        self.debug_pub.publish(out)
        cv2.imwrite(str(PREVIEW_PATH),debug)


def main():
    rclpy.init()
    node = None
    try:
        node = DepthGridNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'ERROR: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
