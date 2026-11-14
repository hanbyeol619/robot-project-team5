"""Pure image/depth geometry, testable without ROS."""
import cv2
import numpy as np


def robust_depth_m(depth_image, u, v, *, encoding='16UC1',
                   scale_m_per_unit=0.001, patch_radius=4,
                   min_m=0.2, max_m=5.0):
    """Median of valid depth pixels around (u,v); returns None when invalid."""
    h, w = depth_image.shape[:2]
    if not (0 <= u < w and 0 <= v < h):
        return None
    window = depth_image[max(0, v-patch_radius):min(h, v+patch_radius+1),
                         max(0, u-patch_radius):min(w, u+patch_radius+1)]
    values = window.astype(np.float64)
    if encoding == '16UC1':
        values = values * scale_m_per_unit
    elif encoding == '32FC1':
        pass  # ROS convention: floating point depth in meters
    else:
        raise ValueError(f'Unsupported depth encoding: {encoding}')
    valid = values[np.isfinite(values) & (values >= min_m) & (values <= max_m)]
    if valid.size < 3:
        return None
    return float(np.median(valid))


def pixel_to_camera_xyz(u, v, z_m, K, D=None, distortion_model='plumb_bob'):
    """3D position in the color optical frame, meters.

    Assumes depth image is aligned to color pixels and z_m is optical Z.
    """
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    if K[0, 0] == 0 or K[1, 1] == 0:
        raise ValueError('CameraInfo has invalid fx/fy')
    uv = np.array([[[float(u), float(v)]]], dtype=np.float64)
    D = np.asarray(D if D is not None else [], dtype=np.float64)
    if D.size and np.any(D != 0):
        if distortion_model not in ('plumb_bob', 'rational_polynomial'):
            raise ValueError(f'Unsupported distortion model: {distortion_model}')
        xy = cv2.undistortPoints(uv, K, D)[0, 0]
        xn, yn = float(xy[0]), float(xy[1])
    else:
        xn = (u - K[0, 2]) / K[0, 0]
        yn = (v - K[1, 2]) / K[1, 1]
    return [float(xn * z_m), float(yn * z_m), float(z_m)]


def bbox_to_full(bbox_roi, roi):
    rx, ry = roi[:2]
    x1, y1, x2, y2 = bbox_roi
    return [int(x1+rx), int(y1+ry), int(x2+rx), int(y2+ry)]
