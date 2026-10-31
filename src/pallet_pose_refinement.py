"""IPPE+LM pose helpers. No ROS or movement side effects."""
import math
import cv2
import numpy as np
from pallet_vision import opening_model

def raw_candidates(points, k, d, cfg):
    """Positive-depth IPPE candidates without the controller's RMS cutoff."""
    points = np.asarray(points, dtype=float)
    if points.shape != (8, 2) or not np.isfinite(points).all():
        return []
    model = opening_model(cfg)
    try:
        solution = cv2.solvePnPGeneric(model, points, k, d, flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return []
    candidates = []
    for rv, tv in zip(solution[1], solution[2]):
        rotation, translation = cv2.Rodrigues(rv)[0], tv.reshape(3)
        if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
            continue
        if np.min((model@rotation.T+translation)[:, 2]) <= 0:
            continue
        projected = cv2.projectPoints(model, rv, tv, k, d)[0].reshape(8, 2)
        error = float(np.sqrt(np.mean(np.sum((projected-points)**2, axis=1))))
        if math.isfinite(error):
            candidates.append(dict(rotation=rotation.tolist(), translation_m=translation.tolist(),
                                   rms_px=error, projected_points=projected.tolist()))
    return sorted(candidates, key=lambda c: c['rms_px'])


def refine_candidates(candidates, points, k, d, cfg):
    """Refine each IPPE seed independently; retain failures only for diagnostics."""
    model = opening_model(cfg)
    refined = []
    for seed in candidates:
        candidate = dict(seed, ippe_rms_px=seed['rms_px'], refinement_ok=False)
        try:
            rv = cv2.Rodrigues(np.asarray(seed['rotation'], dtype=float))[0]
            tv = np.asarray(seed['translation_m'], dtype=float).reshape(3, 1).copy()
            rv, tv = cv2.solvePnPRefineLM(
                model, np.asarray(points, dtype=float), k, d, rv, tv,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-12))
            rotation, translation = cv2.Rodrigues(rv)[0], tv.reshape(3)
            if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
                raise ValueError('Non-finite LM result')
            if np.min((model @ rotation.T + translation)[:, 2]) <= 0:
                raise ValueError('LM result behind camera')
            projected = cv2.projectPoints(model, rv, tv, k, d)[0].reshape(8, 2)
            error = float(np.sqrt(np.mean(np.sum((projected-points)**2, axis=1))))
            if not math.isfinite(error) or error > seed['rms_px'] + 1e-8:
                raise ValueError('LM did not preserve or reduce reprojection error')
            candidate.update(rotation=rotation.tolist(), translation_m=translation.tolist(),
                             rms_px=error, projected_points=projected.tolist(), refinement_ok=True)
        except (cv2.error, ValueError) as exc:
            candidate['refinement_error'] = str(exc)
        refined.append(candidate)
    return sorted(refined, key=lambda c: (not c['refinement_ok'], c['rms_px']))


def keypoint_geometry_valid(points, image_shape, cfg):
    """Image-space geometry checks shared by preview and optional LM detector."""
    h, w = image_shape[:2]
    low, high = np.array(cfg['roi'][:2])*[w, h], np.array(cfg['roi'][2:])*[w, h]
    if np.any(points <= low) or np.any(points >= high):
        return False
    for quad in points.reshape(2, 4, 2):
        if not cv2.isContourConvex(quad.astype(np.float32)):
            return False
        if cv2.contourArea(quad.astype(np.float32), oriented=True) <= 0:
            return False
        if not (quad[0, 0] < quad[1, 0] and quad[3, 0] < quad[2, 0]):
            return False
        if min(quad[3, 1]-quad[0, 1], quad[2, 1]-quad[1, 1]) < cfg['min_opening_height_px']:
            return False
    return bool(points[:4, 0].mean() < points[4:, 0].mean())


def ambiguous_candidates(candidates):
    if len(candidates) < 2:
        return False
    a, b = candidates[:2]
    angle = math.degrees(math.acos(float(np.clip(
        np.array(a['rotation'])[:, 2] @ np.array(b['rotation'])[:, 2], -1, 1))))
    return abs(b['rms_px']-a['rms_px']) < .15 and angle > 10


def accepted_refined_candidates(candidates, cfg):
    # Failed refinement must never silently become an accepted raw IPPE result.
    if not candidates or any(not c['refinement_ok'] for c in candidates):
        return []
    valid = [c for c in candidates
             if cfg['min_pose_range'] < c['translation_m'][2] < cfg['max_pose_range']
             and c['rotation'][2][2] >= .5
             and c['rms_px'] <= cfg['max_reprojection_error_px']]
    return [] if ambiguous_candidates(valid) else valid


