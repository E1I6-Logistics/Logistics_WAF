"""Offline labelled-image diagnostic. No ROS imports or robot commands.

The source manifest comes from prepare_pallet_pose.py. Unknown train/test overlap
is explicitly recorded; these diagnostics are not held-out performance claims.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from pallet_vision import load_config, estimate_pose, opening_model
from pallet_pose_detector import PalletPoseDetector, detection_from_keypoints


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        p.error('Use a new output directory')
    cfg = load_config(args.config)
    detector = PalletPoseDetector(cfg)
    records = json.loads(args.manifest.read_text())
    args.output.mkdir(parents=True)
    (args.output/'overlays').mkdir()
    frames_cache, errors, timings, results = {}, [], [], []
    for record in records:
        source = Path(record['image'])
        image = cv2.imread(str(source))
        start = time.monotonic()
        pred = detector.model.predict(image, conf=cfg['pose_confidence'],
                                      imgsz=cfg['pose_imgsz'], device=cfg['pose_device'], verbose=False)[0]
        duration = time.monotonic()-start
        timings.append(duration)
        item = dict(image=str(source), inference_seconds=duration,
                    detections=len(pred.boxes), pose_accepted=False, label_pose_accepted=False)
        if len(pred.boxes) == 1 and pred.keypoints is not None and len(record['rows']) == 1:
            xy = pred.keypoints.xy[0].cpu().numpy()
            conf = pred.keypoints.conf[0].cpu().numpy()
            truth = np.array(record['rows'][0][5:]).reshape(8,3)
            expected = truth[:,:2]*[image.shape[1],image.shape[0]]
            visible = truth[:,2] == 2
            distances = np.linalg.norm(xy-expected,axis=1)
            errors.extend(distances[visible].tolist())
            item.update(keypoints=xy.tolist(), confidence=conf.tolist(),
                        keypoint_errors_px=distances.tolist())
            raw = Path(record['raw'])
            session = raw.parent.parent
            if session not in frames_cache:
                frames_cache[session] = {r['file']:r for r in map(json.loads,(session/'frames.jsonl').read_text().splitlines())}
            frame = frames_cache[session][str(raw.relative_to(session))]
            info_id = frame['camera_info_id']
            if info_id is not None:
                info = json.loads((session/f'camera_info_{info_id}.json').read_text())
                k, d = np.array(info['k']).reshape(3,3), np.array(info['d'])
                det = detection_from_keypoints(xy,conf,image.shape,k,d,cfg)
                # Diagnostic only: report unconstrained PnP residual without
                # relaxing the production pose acceptance threshold.
                model_points = opening_model(cfg)
                solutions = cv2.solvePnPGeneric(model_points, xy.astype(float), k, d,
                                                flags=cv2.SOLVEPNP_IPPE)
                residuals = [float(np.sqrt(np.mean(np.sum((cv2.projectPoints(
                    model_points, rv, tv, k, d)[0].reshape(8, 2)-xy)**2, axis=1))))
                    for rv, tv in zip(solutions[1], solutions[2])]
                item['minimum_pnp_reprojection_px'] = min(residuals) if residuals else None
                truth_pose = estimate_pose(expected,k,d,cfg) if visible.all() else None
                item['label_pose_accepted'] = truth_pose is not None
                if det is not None:
                    item.update(pose_accepted=True, camera_translation_m=det.translation.tolist(), reprojection_px=det.error)
            for i, (a,b) in enumerate(zip(xy,expected)):
                cv2.circle(image,tuple(np.rint(b).astype(int)),3,(255,0,255),1)
                cv2.circle(image,tuple(np.rint(a).astype(int)),2,(0,255,0),-1)
                cv2.putText(image,str(i),tuple(np.rint(a).astype(int)+[0,-5]),cv2.FONT_HERSHEY_SIMPLEX,.3,(0,255,0),1)
        cv2.putText(image,f"detections={len(pred.boxes)} pose={item['pose_accepted']}",
                    (10,25),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,180,255),1)
        cv2.imwrite(str(args.output/'overlays'/source.name),image)
        results.append(item)
    report = dict(model=cfg['pose_model'], model_sha256=hashlib.sha256(Path(cfg['pose_model']).read_bytes()).hexdigest(),
                  keypoint_names=detector.model.model.kpt_names,
                  images=len(results), single_detection_images=sum(r['detections']==1 for r in results),
                  pose_accepted_images=sum(r['pose_accepted'] for r in results),
                  label_pose_accepted_images=sum(r['label_pose_accepted'] for r in results),
                  mean_keypoint_error_px=float(np.mean(errors)) if errors else None,
                  p95_keypoint_error_px=float(np.percentile(errors,95)) if errors else None,
                  median_inference_ms=float(np.median(timings[1:] or timings)*1000),
                  configured_max_reprojection_px=cfg['max_reprojection_error_px'],
                  validation_status='Train/test overlap unknown; diagnostic only. No measured physical pose ground truth.',
                  overlays='Green=model; magenta=manual labels; numbers=keypoint indices.', results=results)
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='results'},indent=2))


if __name__ == '__main__':
    main()
