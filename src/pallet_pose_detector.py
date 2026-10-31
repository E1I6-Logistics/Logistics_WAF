"""Learned 8-corner detector; preserves the existing geometry and motion gates."""
from pathlib import Path

import cv2
import numpy as np

from pallet_vision import estimate_pose, Detection
from pallet_pose_refinement import raw_candidates, refine_candidates, accepted_refined_candidates
from pallet_pose_schema import KEYPOINT_NAMES


def detection_from_keypoints(points, confidence, image_shape, k, d, cfg):
    points = np.asarray(points, dtype=float)
    confidence = np.asarray(confidence, dtype=float)
    if points.shape != (8, 2) or confidence.shape != (8,):
        return None
    if not np.isfinite(points).all() or not np.isfinite(confidence).all():
        return None
    if np.any(confidence < cfg.get('keypoint_confidence', 0.7)):
        return None
    height, width = image_shape[:2]
    low = np.array(cfg['roi'][:2])*[width, height]
    high = np.array(cfg['roi'][2:])*[width, height]
    if np.any(points <= low) or np.any(points >= high):
        return None
    for quad in points.reshape(2, 4, 2):
        if not cv2.isContourConvex(quad.astype(np.float32)):
            return None
        # Expected TL, TR, BR, BL order in the approximately frontal view.
        if cv2.contourArea(quad.astype(np.float32), oriented=True) <= 0:
            return None
        if not (quad[0, 0] < quad[1, 0] and quad[3, 0] < quad[2, 0]):
            return None
        if min(quad[3, 1]-quad[0, 1], quad[2, 1]-quad[1, 1]) < cfg['min_opening_height_px']:
            return None
    if points[:4, 0].mean() >= points[4:, 0].mean():
        return None
    if cfg.get('pose_refine_lm', False):
        seeds = raw_candidates(points, k, d, cfg)
        refined = refine_candidates(seeds, points, k, d, cfg)
        accepted = accepted_refined_candidates(refined, cfg)
        if not accepted:
            return None
        best = accepted[0]
        return Detection(points, np.array(best['rotation']), np.array(best['translation_m']), best['rms_px'])
    return estimate_pose(points, k, d, cfg)


class PalletPoseDetector:
    def __init__(self, cfg):
        weights = Path(cfg.get('pose_model') or '').expanduser()
        if not weights.is_file() or weights.suffix != '.pt':
            raise ValueError('pose_model must point to an existing trained .pt checkpoint')
        from ultralytics import YOLO
        self.model = YOLO(str(weights), task='pose')
        if self.model.task != 'pose' or self.model.names != {0: 'pallet'}:
            raise ValueError('Expected a custom single-class pallet pose model')
        shape = getattr(self.model.model.model[-1], 'kpt_shape', None)
        if list(shape or []) != [8, 3]:
            raise ValueError('Expected exactly 8 keypoints with visibility')
        if getattr(self.model.model, 'kpt_names', None) != {0: list(KEYPOINT_NAMES)}:
            raise ValueError('Checkpoint keypoint names/order do not match the PnP model')
        self.cfg = cfg
        self.preview_points = []

    def __call__(self, image, k, d, cfg):
        self.preview_points = []
        result = self.model.predict(image, imgsz=cfg.get('pose_imgsz', 640),
                                    conf=cfg.get('pose_confidence', 0.5),
                                    device=cfg.get('pose_device', 'cpu'), verbose=False)[0]
        if result.keypoints is None or result.keypoints.conf is None:
            return None
        points = result.keypoints.xy.cpu().numpy()
        scores = result.keypoints.conf.cpu().numpy()
        self.preview_points = list(zip(points, scores))
        # Never pick a target by confidence when multiple pallets intersect ROI.
        h, w = image.shape[:2]
        low = np.array(cfg['roi'][:2])*[w, h]
        high = np.array(cfg['roi'][2:])*[w, h]
        boxes = result.boxes.xyxy.cpu().numpy()
        candidates = [i for i, box in enumerate(boxes)
                      if np.all(box[:2] < high) and np.all(box[2:] > low)]
        if len(candidates) != 1:
            return None
        i = candidates[0]
        return detection_from_keypoints(points[i], scores[i], image.shape, k, d, cfg)
