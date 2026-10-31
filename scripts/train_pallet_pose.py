"""Train and evaluate a custom 8-point pallet model in a separate ML environment."""
import argparse
import json
from pathlib import Path
import platform

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--model', default='yolo11n-pose.pt')
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--batch', type=int, default=8)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a new output directory')
    if min(args.epochs, args.batch, args.imgsz) <= 0:
        parser.error('epochs, batch and imgsz must be positive')
    data = yaml.safe_load(args.data.read_text())
    if data.get('kpt_shape') != [8, 3] or data.get('names') != {0: 'pallet'}:
        parser.error('Expected single-class pallet data with 8 keypoints')
    audit = json.loads((args.data.parent/'audit.json').read_text())
    if audit['audit_only'] or audit['errors'] or not all(audit['splits'].get(s) for s in ('train','val','test')):
        parser.error('Run prepare_pallet_pose.py with disjoint train/val/test sessions first')
    import torch
    import ultralytics
    from ultralytics import YOLO
    if args.device != 'cpu' and not torch.cuda.is_available():
        parser.error('CUDA unavailable. Fix the GPU environment or explicitly choose --device cpu')
    model = YOLO(args.model, task='pose')
    model.train(data=str(args.data.resolve()), device=args.device, epochs=args.epochs,
                batch=args.batch, imgsz=args.imgsz, workers=0, seed=29,
                project=str(args.output.resolve().parent), name=args.output.name,
                exist_ok=False, patience=20, deterministic=True,
                mosaic=0.0, mixup=0.0, degrees=5.0, translate=0.05,
                scale=0.15, fliplr=0.0, flipud=0.0)
    run = Path(model.trainer.save_dir)
    best = run/'weights/best.pt'
    metrics = YOLO(str(best)).val(data=str(args.data.resolve()), split='test',
                                device=args.device, imgsz=args.imgsz, workers=0,
                                project=str(run), name='heldout_test')
    report = dict(best=str(best), python=platform.python_version(),
                  torch=torch.__version__, ultralytics=ultralytics.__version__,
                  device=args.device, data=str(args.data.resolve()),
                  metrics={k: float(v) for k,v in metrics.results_dict.items()},
                  limitation='Pose mAP does not certify millimeter distance/yaw or real robot alignment.')
    (run/'training_report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
