"""Audit LabelMe annotations and optionally export a session-disjoint YOLO dataset.

Source images/annotations are read-only. Unknown provenance, invalid labels,
missing annotations and duplicate source images prevent export. Run --audit-only
before choosing held-out capture sessions. Formal accuracy still needs new scenes.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
import sys

import cv2
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from pallet_pose_schema import KEYPOINT_NAMES, FLIP_INDEX, labelme_rows


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(source, raw, output, val_sessions=(), test_sessions=(), audit_only=False):
    if output.exists():
        raise ValueError('Output must be a new directory')
    if set(val_sessions) & set(test_sessions):
        raise ValueError('Validation/test sessions overlap')
    index = {}
    for path in sorted(raw.glob('*/frames/*.jpg')):
        index.setdefault(digest(path), []).append(path)
    records, errors, seen = [], [], set()
    images = sorted((source/'images').glob('*.jpg'))
    if not images:
        raise ValueError('No JPEG images in source/images')
    for path in images:
        try:
            annotation = source/'annotations'/f'{path.stem}.json'
            document = json.loads(annotation.read_text())
            image = cv2.imread(str(path))
            if image is None or image.shape[:2] != (document['imageHeight'], document['imageWidth']):
                raise ValueError('Image/annotation dimensions disagree')
            rows = labelme_rows(document)
            sha = digest(path)
            if sha in seen:
                raise ValueError('Duplicate source image')
            seen.add(sha)
            origins = index.get(sha, [])
            sessions = {p.parent.parent.name for p in origins}
            if len(sessions) != 1:
                raise ValueError('Missing or ambiguous raw capture session')
            session = sessions.pop()
            split = 'test' if session in test_sessions else 'val' if session in val_sessions else 'train'
            records.append(dict(image=str(path.resolve()), annotation=str(annotation.resolve()),
                                annotation_sha256=digest(annotation), sha256=sha,
                                raw=str(origins[0].resolve()), session=session, split=split, rows=rows))
        except (ValueError, KeyError, TypeError, OSError) as exc:
            errors.append(dict(image=str(path), error=str(exc)))
    orphaned = sorted(p.name for p in (source/'annotations').glob('*.json')
                      if not (source/'images'/f'{p.stem}.jpg').exists())
    if orphaned:
        errors.append(dict(error='Orphan annotations', files=orphaned))
    report = dict(images=len(images), valid_images=len(records), errors=errors,
                  sessions=dict(Counter(r['session'] for r in records)),
                  splits=dict(Counter(r['split'] for r in records)),
                  objects=sum(len(r['rows']) for r in records),
                  partial_objects=sum(any(row[i] == 0 for i in range(7, 29, 3))
                                      for r in records for row in r['rows']),
                  audit_only=audit_only,
                  limitation='Structural checks do not verify semantic label accuracy or physical pose accuracy.')
    if not audit_only:
        existing = {r['session'] for r in records}
        if not set(val_sessions).issubset(existing) or not set(test_sessions).issubset(existing):
            raise ValueError('Requested holdout session not found in valid labels')
        if errors or not all(report['splits'].get(s) for s in ('train', 'val', 'test')):
            raise ValueError(f'Export requires valid labels and nonempty train/val/test: {report}')
    output.mkdir(parents=True)
    (output/'audit.json').write_text(json.dumps(report, indent=2)+'\n')
    (output/'manifest.json').write_text(json.dumps(records, indent=2)+'\n')
    if not audit_only:
        for r in records:
            image_dir, label_dir = output/'images'/r['split'], output/'labels'/r['split']
            image_dir.mkdir(parents=True, exist_ok=True)
            label_dir.mkdir(parents=True, exist_ok=True)
            image_path = Path(r['image'])
            shutil.copy2(image_path, image_dir/image_path.name)
            (label_dir/f'{image_path.stem}.txt').write_text(''.join(
                ' '.join(f'{v:.9g}' for v in row)+'\n' for row in r['rows']))
        config = dict(path=str(output.resolve()), train='images/train', val='images/val', test='images/test',
                      names={0:'pallet'}, kpt_shape=[8,3], flip_idx=FLIP_INDEX,
                      kpt_names={0:list(KEYPOINT_NAMES)})
        (output/'data.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source', 'raw', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--val-sessions', nargs='+', default=[])
    parser.add_argument('--test-sessions', nargs='+', default=[])
    parser.add_argument('--audit-only', action='store_true')
    args = parser.parse_args()
    try:
        report = prepare(args.source, args.raw, args.output, args.val_sessions, args.test_sessions, args.audit_only)
    except ValueError as exc:
        parser.exit(2, str(exc)+'\n')
    print(json.dumps(report, indent=2))
    return bool(report['errors'])


if __name__ == '__main__':
    sys.exit(main())
