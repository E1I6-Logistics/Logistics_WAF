"""Display learned pallet keypoints and diagnostic PnP estimates. Never moves a robot.

Live: source /opt/ros/jazzy/setup.bash; ROS_DOMAIN_ID=11 <ML-python> this_file.py
Offline: --image frame.jpg --camera-info camera_info.json --headless --output DIR
IPPE candidates are refined with LM using shared optional detector helpers.
Orange estimates are shown even when the preview geometry filter rejects them.
No velocity publisher, action client, camera service or --execute option exists.
"""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys
import time
from zoneinfo import ZoneInfo

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from pallet_pose_detector import PalletPoseDetector
from pallet_pose_refinement import (raw_candidates, refine_candidates, keypoint_geometry_valid,
                                    ambiguous_candidates, accepted_refined_candidates)
from pallet_vision import load_config, opening_model, quaternion_matrix

WINDOW = 'Pallet model preview - OBSERVE ONLY | Q quit | TAB next object | S save'
ORANGE, GREEN, RED, WHITE = (0, 190, 255), (90, 220, 110), (80, 80, 255), (225, 230, 235)


def camera_matrix(info, image_shape):
    if image_shape[:2] != (info['height'], info['width']):
        raise ValueError('Image/CameraInfo resolution mismatch')
    if info.get('distortion_model', 'plumb_bob') not in ('plumb_bob', 'rational_polynomial'):
        raise ValueError('Unsupported distortion model')
    roi = info.get('roi', {})
    if info.get('binning_x', 0) > 1 or info.get('binning_y', 0) > 1 or roi.get('x_offset', 0) or roi.get('y_offset', 0):
        raise ValueError('Cropped/binned calibration is unsupported')
    k, d = np.asarray(info['k'], dtype=float).reshape(3, 3), np.asarray(info['d'], dtype=float)
    if not np.isfinite(k).all() or not np.isfinite(d).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('Invalid camera calibration')
    return k, d


def base_estimate(candidate, transform, cfg):
    """Diagnostic coordinates only; no control computation or motion approval."""
    rotation = np.asarray(transform['rotation'], dtype=float)
    translation = np.asarray(transform['translation_m'], dtype=float)
    center = rotation@np.array(candidate['translation_m'])+translation
    normal = rotation@np.array(candidate['rotation'])[:, 2]
    length = float(np.linalg.norm(normal[:2]))
    if not np.isfinite(center).all() or not np.isfinite(normal).all() or length < 1e-6:
        return None
    planar_normal = normal[:2]/length
    yaw = math.degrees(math.atan2(planar_normal[1], planar_normal[0]))
    body = cfg.get('body_front_x')
    gap = float((center[:2]-[body, 0])@planar_normal) if body is not None else None
    return dict(center_xyz_m=center.tolist(), yaw_deg=yaw,
                face_tilt_deg=math.degrees(math.asin(float(np.clip(normal[2], -1, 1)))),
                body_to_face_normal_gap_m=gap,
                body_forward_gap_m=float(center[0]-body) if body is not None else None,
                mounting_verified=cfg['extrinsics_verified'])


def analyze_object(points, confidence, image_shape, k, d, cfg, transform=None):
    points, confidence = np.asarray(points, dtype=float), np.asarray(confidence, dtype=float)
    result = dict(geometry_pass=False, warnings=[], candidates=[])
    if points.shape != (8, 2) or confidence.shape != (8,) or not np.isfinite(points).all() or not np.isfinite(confidence).all():
        result['warnings'].append('Invalid keypoints');return result
    result['min_keypoint_confidence'] = float(confidence.min())
    if np.any(confidence < cfg['keypoint_confidence']):
        result['warnings'].append('Low keypoint confidence: pose hidden');return result
    h, w = image_shape[:2]
    if np.any(points < 0) or np.any(points >= [w, h]):
        result['warnings'].append('Keypoints outside image: pose hidden');return result
    seeds = raw_candidates(points, k, d, cfg)
    if not seeds:
        result['warnings'].append('PnP has no finite positive-depth solution');return result
    result['pose_method'] = 'IPPE + LM'
    result['candidates'] = refine_candidates(seeds, points, k, d, cfg)
    if any(not c['refinement_ok'] for c in result['candidates']):
        result['warnings'].append('LM failed: geometry rejected; fallback is diagnostic only')
    image_geometry_ok = keypoint_geometry_valid(points, image_shape, cfg)
    if not image_geometry_ok:
        result['warnings'].append('Keypoint ROI / shape / size check failed')
    accepted = accepted_refined_candidates(result['candidates'], cfg)
    result['geometry_pass'] = bool(image_geometry_ok and accepted)
    if result['geometry_pass']:
        index = next(i for i,c in enumerate(result['candidates']) if c is accepted[0])
        result['candidates'].insert(0, result['candidates'].pop(index))
    else:
        result['warnings'].append('Preview geometry filter REJECTED')
    best = result['candidates'][0]
    if best['rms_px'] > cfg['max_reprojection_error_px']:
        result['warnings'].append('Reprojection error above limit')
    if len(result['candidates']) > 1:
        a, b = result['candidates'][:2]
        angle = math.degrees(math.acos(float(np.clip(np.array(a['rotation'])[:, 2]@np.array(b['rotation'])[:, 2], -1, 1))))
        result['candidate_normal_difference_deg'] = angle
        if b['rms_px']-a['rms_px'] < .15 and angle > 10:
            result['warnings'].append('Ambiguous plane orientation')
    if transform is not None:
        result['base'] = base_estimate(best, transform, cfg)
        if not cfg['extrinsics_verified']:
            result['warnings'].append('Body values use unverified nominal mounting')
    return result


def infer(image, info, detector, cfg, transform=None):
    k, d = camera_matrix(info, image.shape)
    start = time.monotonic()
    prediction = detector.model.predict(image, imgsz=cfg['pose_imgsz'], conf=cfg['pose_confidence'],
                                        device=cfg['pose_device'], verbose=False)[0]
    objects = []
    if prediction.keypoints is not None and prediction.keypoints.conf is not None:
        for box, score, points, confidence in zip(prediction.boxes.xyxy.cpu().numpy(),
                prediction.boxes.conf.cpu().numpy(), prediction.keypoints.xy.cpu().numpy(),
                prediction.keypoints.conf.cpu().numpy()):
            result = analyze_object(points, confidence, image.shape, k, d, cfg, transform)
            result.update(box=box.tolist(), box_confidence=float(score), points=points.tolist(),
                          keypoint_confidence=confidence.tolist())
            objects.append(result)
    return dict(objects=objects, inference_ms=(time.monotonic()-start)*1000,
                camera_frame=info.get('frame_id', ''), body_tf_available=transform is not None)


def render(image, report, cfg, selected=0, status='LIVE'):
    h, w = image.shape[:2]
    canvas = np.full((max(h, 760), w+550, 3), (26, 24, 22), np.uint8)
    canvas[:h, :w] = image
    x0, y0, x1, y1 = (int(v*s) for v, s in zip(cfg['roi'], (w, h, w, h)))
    cv2.rectangle(canvas, (x0, y0), (x1, y1), (100, 100, 100), 1)
    objects = report.get('objects', [])
    index = selected % len(objects) if objects else 0
    for i, obj in enumerate(objects):
        color = GREEN if obj['geometry_pass'] else ORANGE
        box = np.rint(obj['box']).astype(int)
        cv2.rectangle(canvas, tuple(box[:2]), tuple(box[2:]), color, 2 if i==index else 1)
        cv2.putText(canvas, f'#{i+1} {obj["box_confidence"]:.2f}', (box[0], max(15, box[1]-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
        points = np.asarray(obj['points'])
        for j,(point,score) in enumerate(zip(points, obj['keypoint_confidence'])):
            if score < cfg['keypoint_confidence'] or not np.isfinite(point).all():continue
            pixel=tuple(np.rint(point).astype(int));cv2.circle(canvas,pixel,3,color,-1)
            cv2.putText(canvas,str(j),(pixel[0]+3,pixel[1]-4),cv2.FONT_HERSHEY_SIMPLEX,.35,color,1)
        if i == index and obj['candidates']:
            for p in obj['candidates'][0]['projected_points']:
                if np.isfinite(p).all() and 0 <= p[0] < w and 0 <= p[1] < h:
                    cv2.drawMarker(canvas,tuple(np.rint(p).astype(int)),(255,100,255),cv2.MARKER_CROSS,6,1)
    lines=[('PALLET MODEL - OBSERVE ONLY',WHITE),(status+' | IPPE + LM',ORANGE),
           (f'Target body gap: {cfg["target_body_gap"]*1000:.0f} mm',WHITE),
           (f'Inference: {report.get("inference_ms",0):.1f} ms | objects: {len(objects)}',WHITE)]
    if objects:
        obj=objects[index]
        lines += [(f'Viewing #{index+1} / {len(objects)} (TAB changes)',WHITE),
                  (f'Keypoint confidence min: {obj.get("min_keypoint_confidence",0):.2f}',WHITE),
                  ('Geometry filter: '+('PASS - not motion approval' if obj['geometry_pass'] else 'REJECTED - diagnostic estimate only'),GREEN if obj['geometry_pass'] else ORANGE)]
        if obj['candidates']:
            c=obj['candidates'][0];right,down,forward=np.array(c['translation_m'])*1000
            lines += [(f'IPPE seed RMS: {c.get("ippe_rms_px", c["rms_px"]):.2f} px',WHITE),
                      (('LM refined' if c.get('refinement_ok') else 'IPPE fallback')+' pose shown',WHITE),
                      (f'PnP RMS: {c["rms_px"]:.2f} px (limit {cfg["max_reprojection_error_px"]:.2f})',ORANGE),
                      (f'Camera forward Z: {forward:.1f} mm',WHITE),
                      (f'Camera right X: {right:+.1f} mm',WHITE),
                      (f'Camera down Y: {down:+.1f} mm',WHITE)]
            base=obj.get('base')
            if base:
                gap=base['body_to_face_normal_gap_m']
                lines += [(f'Body face gap (normal): {gap*1000:.1f} mm' if gap is not None else 'Body gap: body_front_x missing',WHITE),
                          (f'Pallet center lateral (+left): {base["center_xyz_m"][1]*1000:+.1f} mm',WHITE),
                          (f'Face yaw (+left): {base["yaw_deg"]:+.1f} deg',WHITE),
                          (f'Face tilt: {base["face_tilt_deg"]:+.1f} deg',WHITE)]
            else:lines += [('Body gap/yaw: TF unavailable',ORANGE)]
            if len(obj['candidates']) > 1:
                alt=obj['candidates'][1]
                lines += [(f'Alt pose: Z={alt["translation_m"][2]*1000:.1f} mm, RMS={alt["rms_px"]:.2f}',ORANGE)]
        for warning in obj['warnings']:lines.append((warning,ORANGE))
    else:lines += [('No usable detection / estimates hidden',ORANGE)]
    lines += [('Dots = model | magenta crosses = reprojection',WHITE),
              ('Q / Esc quit | TAB next object | S save',WHITE)]
    for i,(line,color) in enumerate(lines):
        cv2.putText(canvas,line,(w+14,26+25*i),cv2.FONT_HERSHEY_SIMPLEX,.49,color,1,cv2.LINE_AA)
    return canvas


def save_snapshot(output, canvas, report, cfg):
    output.mkdir(parents=True, exist_ok=True)
    stem=datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S_%f')
    path=output/f'{stem}.png'
    if not cv2.imwrite(str(path),canvas):raise OSError(f'Could not save {path}')
    record=dict(report,model=cfg['pose_model'],target_body_gap_m=cfg['target_body_gap'],
                body_front_x_m=cfg['body_front_x'],motion_enabled=False)
    path.with_suffix('.json').write_text(json.dumps(record,indent=2,allow_nan=False)+'\n')
    print(f'Saved {path}',flush=True)


def live(args, cfg, detector):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, CompressedImage
    from tf2_ros import Buffer, TransformListener, TransformException
    rclpy.init();node=Node('pallet_pose_preview');buffer=Buffer();listener=TransformListener(buffer,node)
    latest={}
    node.create_subscription(CompressedImage,cfg['image_topic'],lambda m:latest.update(image=(m,time.monotonic())),qos_profile_sensor_data)
    node.create_subscription(CameraInfo,cfg['camera_info_topic'],lambda m:latest.update(info=(m,time.monotonic())),qos_profile_sensor_data)
    start=time.monotonic();stamp=None;selected=0;last_log=0.;last_frame=None
    canvas=render(np.zeros((480,640,3),np.uint8),{},cfg,status='Waiting for camera and CameraInfo')
    report={'objects':[],'status':'Waiting for camera'}
    if not args.headless:cv2.namedWindow(WINDOW,cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok() and (args.seconds==0 or time.monotonic()-start<args.seconds):
            rclpy.spin_once(node,timeout_sec=.02)
            status=None
            try:
                if 'image' not in latest or 'info' not in latest:raise ValueError('Waiting for camera and CameraInfo')
                m,received=latest['image'];info,info_received=latest['info'];now=time.monotonic()
                age=(node.get_clock().now().nanoseconds-Time.from_msg(m.header.stamp).nanoseconds)/1e9
                if not -.05 <= age <= cfg['max_image_age'] or now-received>cfg['max_image_age']:raise ValueError('STALE CAMERA - estimates hidden')
                info_age=(node.get_clock().now().nanoseconds-Time.from_msg(info.header.stamp).nanoseconds)/1e9
                if not -.05<=info_age<=cfg['max_info_age'] or now-info_received>cfg['max_info_age']:raise ValueError('Stale CameraInfo - estimates hidden')
                if not m.header.frame_id or m.header.frame_id!=info.header.frame_id:raise ValueError('Image/CameraInfo frame mismatch')
                current=Time.from_msg(m.header.stamp).nanoseconds
                if stamp is not None and current<stamp:raise ValueError('Non-monotonic image timestamp')
                if current!=stamp:
                    stamp=current;frame=cv2.imdecode(np.frombuffer(m.data,np.uint8),cv2.IMREAD_COLOR)
                    if frame is None:raise ValueError('JPEG decode failed')
                    transform=None;tf_error=None
                    try:
                        tf=buffer.lookup_transform(cfg['base_frame'],m.header.frame_id,Time.from_msg(m.header.stamp)).transform
                        q,p=tf.rotation,tf.translation
                        transform={'rotation':quaternion_matrix([q.x,q.y,q.z,q.w]),'translation_m':[p.x,p.y,p.z]}
                    except TransformException as exc:tf_error=str(exc)
                    ci={'k':list(info.k),'d':list(info.d),'width':info.width,'height':info.height,'distortion_model':info.distortion_model,'frame_id':info.header.frame_id,'binning_x':info.binning_x,'binning_y':info.binning_y,'roi':{'x_offset':info.roi.x_offset,'y_offset':info.roi.y_offset}}
                    report=infer(frame,ci,detector,cfg,transform)
                    age_after=(node.get_clock().now().nanoseconds-current)/1e9
                    if age_after>cfg['max_image_age']:raise ValueError('Frame became stale during inference')
                    report.update(stamp_ns=current,source_age_seconds=age_after,status='LIVE',tf_error=tf_error)
                    last_frame=frame
                if last_frame is not None:canvas=render(last_frame,report,cfg,selected)
            except (ValueError,cv2.error) as exc:
                status=str(exc);report={'objects':[],'status':status}
                blank=np.zeros((480,640,3),np.uint8) if last_frame is None else last_frame//3
                canvas=render(blank,report,cfg,status=status)
            if time.monotonic()-last_log>1:
                print(f'{report.get("status","LIVE")}: objects={len(report.get("objects",[]))}',flush=True);last_log=time.monotonic()
            if not args.headless:
                cv2.imshow(WINDOW,canvas);key=cv2.waitKey(1)&0xff
                if key in (27,ord('q')):break
                if key==9:selected+=1
                if key==ord('s'):save_snapshot(args.output,canvas,report,cfg)
                if cv2.getWindowProperty(WINDOW,cv2.WND_PROP_VISIBLE)<1:break
        if args.seconds>0:save_snapshot(args.output,canvas,report,cfg)
    finally:
        node.destroy_node();rclpy.shutdown()
        if not args.headless:cv2.destroyAllWindows()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'config/pallet_alignment_yolo.yaml')
    p.add_argument('--model',type=Path)
    p.add_argument('--device',default=None)
    p.add_argument('--headless',action='store_true')
    p.add_argument('--seconds',type=float,default=0,help='Live duration; 0 runs until Q/Ctrl+C')
    p.add_argument('--output',type=Path,default=ROOT/'output/pallet_pose_preview')
    p.add_argument('--image',type=Path,help='Offline JPEG instead of ROS')
    p.add_argument('--camera-info',type=Path,help='Captured CameraInfo JSON for offline image')
    args=p.parse_args()
    if not math.isfinite(args.seconds) or args.seconds<0:p.error('--seconds must be finite and nonnegative')
    if args.headless and not args.image and args.seconds==0:p.error('Headless live preview requires --seconds')
    if bool(args.image)!=bool(args.camera_info):p.error('--image and --camera-info must be used together')
    if not args.headless and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):p.error('No desktop display; use --headless --seconds 10')
    cfg=load_config(args.config)
    if args.model:cfg['pose_model']=str(args.model.resolve())
    if args.device:cfg['pose_device']=args.device
    detector=PalletPoseDetector(cfg)
    if args.image:
        frame=cv2.imread(str(args.image))
        if frame is None:p.error('Cannot decode --image')
        report=infer(frame,json.loads(args.camera_info.read_text()),detector,cfg)
        report['status']='OFFLINE - body TF unavailable'
        canvas=render(frame,report,cfg,status=report['status'])
        save_snapshot(args.output,canvas,report,cfg)
        if not args.headless:
            cv2.imshow(WINDOW,canvas);cv2.waitKey(0);cv2.destroyAllWindows()
    else:
        live(args,cfg,detector)


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:pass
