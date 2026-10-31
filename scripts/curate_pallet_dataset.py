"""Read-only raw-data audit and review sheets for the fixed 640x480 capture setup.

Never modifies raw files. Clustering is an image-similarity heuristic, not a
trained pallet detector. Human visual review is needed before selecting copies.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def hash_tree(root):
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(root.rglob('*')) if p.is_file()}


def analyze(raw, output, sessions, roi_bounds, review_bounds, threshold):
    output.mkdir(parents=True, exist_ok=False)
    (output/'raw_sha256_before.json').write_text(json.dumps(hash_tree(raw), indent=2)+'\n')
    rows, anchors, groups = [], [], []
    x1, y1, x2, y2 = roi_bounds
    for session in sessions:
        source_rows = [json.loads(s) for s in (raw/session/'frames.jsonl').read_text().splitlines()]
        for original in source_rows:
            source = raw/session/original['file']
            image = cv2.imread(str(source))
            row = dict(original, source=str(source), session=session,
                       sha256=hashlib.sha256(source.read_bytes()).hexdigest())
            if image is None or image.shape != (480, 640, 3):
                row.update(group=None, issue='decode_or_resolution')
                rows.append(row)
                continue
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Fixed region covers observed pallet positions. Excludes most upper
            # background movement, but hands can still affect similarity.
            roi = gray[y1:y2, x1:x2]
            feature = cv2.resize(cv2.GaussianBlur(roi, (3, 3), 0), (130, 28)).astype(np.float32)
            feature -= np.median(feature)
            row['front_region_sharpness'] = float(cv2.Laplacian(roi, cv2.CV_64F).var())
            if anchors:
                distances = np.array([np.abs(a-feature).mean() for a in anchors])
                closest = int(np.argmin(distances))
                distance = float(distances[closest])
            else:
                closest, distance = 0, float('inf')
            if distance > threshold:
                closest = len(anchors)
                anchors.append(feature)
                groups.append([])
                distance = 0.0
            row.update(group=closest, distance_to_anchor=distance)
            groups[closest].append(len(rows))
            rows.append(row)
    representatives = []
    for group, indices in enumerate(groups):
        eligible = [i for i in indices if rows[i]['camera_info_id'] is not None]
        best = max(eligible or indices, key=lambda i: rows[i]['front_region_sharpness'])
        representatives.append(dict(group=group, members=len(indices), row_index=best, **{
            k: rows[best][k] for k in ('source', 'session', 'elapsed_s', 'front_region_sharpness')}))
    (output/'audit.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (output/'groups.json').write_text(json.dumps(representatives, indent=2)+'\n')
    # Contact sheets are derived review views; training copies remain byte-identical.
    rx1, ry1, rx2, ry2 = review_bounds
    tile_w, tile_h = rx2-rx1, ry2-ry1+40
    for start in range(0, len(representatives), 8):
        batch = representatives[start:start+8]
        canvas = np.full((4*tile_h, 2*tile_w, 3), 245, np.uint8)
        for j, rep in enumerate(batch):
            image = cv2.imread(rep['source'])
            crop = image[ry1:ry2, rx1:rx2]
            y, x = (j//2)*tile_h, (j%2)*tile_w
            canvas[y+30:y+30+crop.shape[0], x:x+tile_w] = crop
            label = f'G{rep["group"]:03d} n={rep["members"]} {rep["session"][-2:]} t={rep["elapsed_s"]:.1f}s'
            cv2.putText(canvas, label, (x+8,y+22), cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,0),1)
        cv2.imwrite(str(output/f'review_{start//8:02d}.jpg'),canvas)
    report = {'input_frames':len(rows),'groups':len(groups),'sessions':sessions,
              'roi':roi_bounds,'review_roi':review_bounds,
              'descriptor_shape':[130,28],'mean_absolute_difference_threshold':threshold,
              'representative_rule':'highest front-region Laplacian variance, prefer matching CameraInfo',
              'limits':'Similarity and sharpness are heuristics; group representatives must be visually reviewed.'}
    (output/'method.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--sessions',nargs='+',default=['20261028_training_01','20261028_training_02'])
    parser.add_argument('--roi',type=int,nargs=4,default=[60,310,580,420],metavar=('X1','Y1','X2','Y2'))
    parser.add_argument('--review-roi',type=int,nargs=4,default=[60,280,580,440],metavar=('X1','Y1','X2','Y2'))
    parser.add_argument('--threshold',type=float,default=4.0)
    args=parser.parse_args()
    for x1,y1,x2,y2 in (args.roi,args.review_roi):
        if not (0 <= x1 < x2 <= 640 and 0 <= y1 < y2 <= 480):
            parser.error('Regions must lie within the 640x480 frame')
    if not np.isfinite(args.threshold) or args.threshold <= 0:
        parser.error('Threshold must be finite and positive')
    raw=args.raw.resolve()
    for session in args.sessions:
        if Path(session).name != session or not (raw/session/'frames.jsonl').is_file():
            parser.error('Each session must name an existing directory directly under raw')
    analyze(raw,args.output.resolve(),args.sessions,args.roi,args.review_roi,args.threshold)


if __name__=='__main__':
    main()
