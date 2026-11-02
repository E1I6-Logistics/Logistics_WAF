"""Experimental two-opening detector and geometry; no ROS or motion side effects.

The model uses the FRONT opening corners, not the back wall/floor shadows.
Only an approximately front-facing pallet with both openings visible is supported.
"""
from dataclasses import dataclass
from itertools import combinations
import math

import cv2
import numpy as np
import yaml


def load_config(path):
    with open(path, encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)
    positive = ('opening_width', 'opening_height', 'center_web_width',
                'target_body_gap', 'fork_projection', 'min_opening_height_px',
                'max_reprojection_error_px', 'max_pose_range', 'min_pose_range',
                'max_pallet_yaw_deg', 'lateral_tolerance', 'gap_tolerance',
                'yaw_tolerance_deg', 'near_guard_gap', 'min_body_gap',
                'max_linear_speed', 'max_angular_speed', 'max_image_age',
                'max_info_age', 'lost_timeout', 'alignment_timeout', 'startup_timeout')
    for key in positive:
        if not isinstance(cfg.get(key), (float, int)) or not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if cfg['max_linear_speed'] > 0.04 or cfg['max_angular_speed'] > 0.25:
        raise ValueError('Prototype speed limits: linear <= 0.04, angular <= 0.25')
    if not 0 <= cfg['dark_threshold'] <= 255:
        raise ValueError('dark_threshold must be between 0 and 255')
    if not isinstance(cfg['stable_frames'], int) or cfg['stable_frames'] < 3:
        raise ValueError('stable_frames must be an integer >= 3')
    roi = cfg['roi']
    if len(roi) != 4 or not (0 <= roi[0] < roi[2] <= 1 and 0 <= roi[1] < roi[3] <= 1):
        raise ValueError('roi must be normalized [left, top, right, bottom]')
    if not cfg['min_body_gap'] < cfg['target_body_gap'] < cfg['near_guard_gap']:
        raise ValueError('Required: min_body_gap < target_body_gap < near_guard_gap')
    if cfg['min_pose_range'] >= cfg['max_pose_range']:
        raise ValueError('Invalid pose range')
    if cfg.get('detector', 'contour') not in ('contour', 'yolo_pose'):
        raise ValueError('detector must be contour or yolo_pose')
    for key in ('pose_confidence', 'keypoint_confidence'):
        value = cfg.get(key, 0.7)
        if not isinstance(value, (float, int)) or not math.isfinite(value) or not 0 < value <= 1:
            raise ValueError(f'{key} must be in (0, 1]')
    return cfg


def validate_motion_config(cfg):
    if cfg['extrinsics_verified'] is not True:
        raise ValueError('Measure camera TF and set extrinsics_verified: true before --execute')
    front = cfg.get('body_front_x')
    if not isinstance(front, (int, float)) or not math.isfinite(front) or not 0 < front < 0.5:
        raise ValueError('Set measured body_front_x in meters (0 < x < 0.5)')
    if cfg['cmd_vel_topic'] != '/cmd_vel_nav':
        raise ValueError('This prototype requires /cmd_vel_nav through the Nav2 safety chain')
    # This controller aligns outside the entrance; it cannot perform insertion.
    # Include the configured completion tolerance, not only the nominal target.
    if (cfg['target_body_gap']-cfg['gap_tolerance'] <= cfg['fork_projection'] or
            cfg['min_body_gap'] <= cfg['fork_projection']):
        raise ValueError('Pre-insertion alignment requires target_body_gap - gap_tolerance '
                         'and min_body_gap to exceed measured fork_projection; revise gap settings')


def opening_model(cfg):
    """x=image right, y=down, z=into pallet; origin at pair midpoint, mid-height."""
    w, h, web = (cfg[k] for k in ('opening_width', 'opening_height', 'center_web_width'))
    points = []
    for left in (-web / 2 - w, web / 2):
        points.extend([(left, -h/2, 0), (left+w, -h/2, 0),
                       (left+w, h/2, 0), (left, h/2, 0)])
    return np.array(points, dtype=np.float64)


@dataclass
class Detection:
    corners: np.ndarray
    rotation: np.ndarray
    translation: np.ndarray
    error: float


def estimate_pose(corners, k, d, cfg):
    model = opening_model(cfg)
    corners = np.asarray(corners, dtype=np.float64).reshape(8, 2)
    if not all(np.isfinite(a).all() for a in (corners, k, d)):
        return None
    try:
        result = cv2.solvePnPGeneric(model, corners, k, d, flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return None
    if not result[0]:
        return None
    candidates = []
    for rvec, tvec in zip(result[1], result[2]):
        rotation = cv2.Rodrigues(rvec)[0]
        translation = tvec.reshape(3)
        depths = (model @ rotation.T + translation)[:, 2]
        if not np.isfinite(translation).all() or np.min(depths) <= 0:
            continue
        if not cfg['min_pose_range'] < translation[2] < cfg['max_pose_range']:
            continue
        if rotation[2, 2] < 0.5:  # Reject back-facing/grazing solutions.
            continue
        projected = cv2.projectPoints(model, rvec, tvec, k, d)[0].reshape(8, 2)
        error = float(np.sqrt(np.mean(np.sum((projected-corners)**2, axis=1))))
        if math.isfinite(error) and error <= cfg['max_reprojection_error_px']:
            candidates.append(Detection(corners, rotation, translation, error))
    candidates.sort(key=lambda det: det.error)
    if not candidates:
        return None
    if len(candidates) > 1:
        a, b = candidates[:2]
        angle = math.acos(float(np.clip(a.rotation[:, 2] @ b.rotation[:, 2], -1, 1)))
        # Coplanar pose can have two equally plausible, materially different normals.
        if b.error-a.error < 0.15 and angle > math.radians(10):
            return None
    return candidates[0]


def detect_openings(image, k, d, cfg):
    """Reject incomplete contours, multiple plausible pairs, and poor PnP fits."""
    height, width = image.shape[:2]
    x0, y0, x1, y1 = (int(v*s) for v, s in zip(cfg['roi'], (width, height, width, height)))
    gray = cv2.cvtColor(image[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    mask = cv2.threshold(gray, cfg['dark_threshold'], 255, cv2.THRESH_BINARY_INV)[1]
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    quads = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if x <= 1 or y <= 1 or x+w >= mask.shape[1]-1 or y+h >= mask.shape[0]-1:
            continue
        if h < cfg['min_opening_height_px'] or not 1.7 < w/h < 7.0:
            continue
        if cv2.contourArea(contour) < 0.65*w*h:
            continue
        polygon = cv2.approxPolyDP(contour, 0.025*cv2.arcLength(contour, True), True)
        if len(polygon) != 4 or not cv2.isContourConvex(polygon):
            continue
        p = polygon.reshape(4, 2).astype(np.float64)
        # Limited roll: split the two top and two bottom vertices, then sort by x.
        p = p[np.argsort(p[:, 1])]
        top = p[:2][np.argsort(p[:2, 0])]
        bottom = p[2:][np.argsort(p[2:, 0])]
        quad = np.array([top[0], top[1], bottom[1], bottom[0]]) + [x0, y0]
        quads.append((quad, w, h))
    matches = []
    for first, second in combinations(quads, 2):
        left, right = sorted((first, second), key=lambda q: q[0][:, 0].mean())
        lp, lw, lh = left
        rp, rw, rh = right
        if max(lw, rw)/min(lw, rw) > 1.5 or max(lh, rh)/min(lh, rh) > 1.5:
            continue
        if abs(lp[:, 1].mean()-rp[:, 1].mean()) > 0.8*max(lh, rh):
            continue
        gap = rp[:, 0].min()-lp[:, 0].max()
        if not 0.3 < gap/((lw+rw)/2) < 1.0:
            continue
        det = estimate_pose(np.vstack((lp, rp)), k, d, cfg)
        if det is not None:
            matches.append(det)
    # No semantic classifier is present: never select among multiple plausible pallets.
    return matches[0] if len(matches) == 1 else None


def quaternion_matrix(q):
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def target_in_base(det, rotation, translation, cfg):
    center = rotation @ det.translation + translation
    normal = rotation @ det.rotation[:, 2]  # Points INTO pallet.
    if not np.isfinite(center).all() or not np.isfinite(normal).all():
        raise ValueError('Non-finite pose')
    if abs(normal[2]) > math.sin(math.radians(20)):
        raise ValueError('Pallet face is not vertical; check corners and TF')
    normal = normal[:2] / np.linalg.norm(normal[:2])
    return face_target(center, math.atan2(normal[1], normal[0]), cfg)


# Model axes (x right, y down, z into pallet) in base axes for a vertical face at yaw 0.
FACE_AXES = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def vertical_face_target(det, k, d, rotation, translation, cfg):
    """Refit only (x, y, z, yaw) with the face held vertical in base_link.

    The 8 points barely constrain face tilt, and a free tilt trades off with yaw.
    """
    model, corners = opening_model(cfg), det.corners.ravel()

    def residual(p):
        c, s = math.cos(p[3]), math.sin(p[3])
        face = model @ (np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ FACE_AXES).T + p[:3]
        # Transform explicitly: cv2.Rodrigues rounds tiny rotations to zero.
        camera = (face-translation) @ rotation
        return cv2.projectPoints(camera, np.zeros(3), np.zeros(3), k, d)[0].ravel()-corners

    normal = rotation @ det.rotation[:, 2]
    p = np.array([*(rotation @ det.translation + translation), math.atan2(normal[1], normal[0])])
    r, damping = residual(p), 1e-3
    for _ in range(30):
        jac = np.column_stack([(residual(p+step)-r)/step.sum() for step in np.eye(4)*1e-6])
        h = jac.T @ jac
        trial = p-np.linalg.solve(h+damping*np.diag(np.diag(h)+1e-12), jac.T @ r)
        r_trial = residual(trial)
        if r_trial @ r_trial >= r @ r:
            damping *= 10
            continue
        done = np.max(np.abs(trial-p)) < 1e-9
        p, r, damping = trial, r_trial, damping*.3
        if done:
            break
    rms = float(np.sqrt(np.mean(np.sum(r.reshape(-1, 2)**2, axis=1))))
    if not np.isfinite(p).all() or rms > cfg['max_reprojection_error_px']:
        raise ValueError(f'Vertical-face fit rejected: rms={rms:.2f}px')
    return face_target(p[:3], float(p[3]), cfg)


def face_target(center, yaw, cfg):
    normal = np.array([math.cos(yaw), math.sin(yaw)])
    if center[0] <= 0 or abs(yaw) > math.radians(cfg['max_pallet_yaw_deg']):
        raise ValueError('Pallet is behind robot or too oblique')
    target = center[:2] - normal*(cfg['body_front_x']+cfg['target_body_gap'])
    gap = float((center[:2]-[cfg['body_front_x'], 0]) @ normal)
    return float(target[0]), float(target[1]), yaw, gap


def control_step(target, cfg):
    """Return (v, w, state). Complete mode can align inside the near guard.

    Forward-only approach: at the target distance, finish heading correction
    without driving past the target to chase lateral error. Residual lateral
    error is not success; the runner's no-progress watchdog handles that case.
    """
    x, y, yaw, gap = target
    if not all(math.isfinite(v) for v in target):
        return 0.0, 0.0, 'INVALID'
    aligned = abs(y) <= cfg['lateral_tolerance'] and abs(yaw) <= math.radians(cfg['yaw_tolerance_deg'])
    if gap < cfg['min_body_gap']:
        return 0.0, 0.0, 'TOO_CLOSE'
    if aligned and abs(gap-cfg['target_body_gap']) <= cfg['gap_tolerance']:
        return 0.0, 0.0, 'ALIGNED'
    complete = cfg.get('alignment_mode') == 'complete'
    if not complete and gap <= cfg['near_guard_gap'] and not aligned:
        return 0.0, 0.0, 'NEED_REPOSITION'
    if complete and not aligned and (x <= 0 or gap <= cfg['target_body_gap']+cfg['gap_tolerance']):
        # Polar steering atan2(y, x) becomes singular at the docking point
        # and can request a large turn after passing it. Use the face normal
        # here instead, even when the desired point is slightly behind us.
        omega = float(np.clip(1.5*yaw, -cfg['max_angular_speed'], cfg['max_angular_speed']))
        return 0.0, omega, 'ALIGNING'
    if x <= 0:
        return 0.0, 0.0, 'NEED_REPOSITION'
    alpha = math.atan2(y, x)
    beta = math.atan2(math.sin(yaw-alpha), math.cos(yaw-alpha))
    # Waffle transmits int32(linear.x * 100) to OpenCR. Smaller positive
    # commands become zero; floor permitted forward motion to one unit.
    min_forward_command = 0.01
    velocity = min(cfg['max_linear_speed'], 0.3*math.hypot(x, y))
    velocity *= max(0.0, math.cos(alpha))
    if abs(alpha) > math.radians(35) or velocity <= 0.0:
        velocity = 0.0
    else:
        if cfg['max_linear_speed'] < min_forward_command:
            return 0.0, 0.0, 'INVALID'
        velocity = max(min_forward_command, velocity)
    omega = float(np.clip(1.5*alpha-0.6*beta, -cfg['max_angular_speed'], cfg['max_angular_speed']))
    return velocity, omega, 'ALIGNING'


class PoseGate:
    """Require consecutive stable fresh detections; a jump clears the track."""
    def __init__(self, required):
        self.required = required
        self.reset()

    def reset(self):
        self.previous = None
        self.count = 0

    def accept(self, detection):
        if self.previous is not None:
            delta = np.linalg.norm(detection.translation-self.previous.translation)
            dot = detection.rotation[:, 2] @ self.previous.rotation[:, 2]
            if delta > 0.03 or dot < math.cos(math.radians(8)):
                self.reset()
                return False
        self.previous = detection
        self.count += 1
        return self.count >= self.required
