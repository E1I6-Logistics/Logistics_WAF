"""Record compressed ROS camera images and metadata; never publish or move robot.

JPEGs are saved verbatim. preview.avi is derived for playback only; frame times in
frames.jsonl are authoritative. No labels or train/validation splits are invented.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, CompressedImage


class Collector(Node):
    def __init__(self, args):
        super().__init__('pallet_dataset_collector')
        self.args = args
        self.root = args.output.resolve()
        self.root.mkdir(parents=True, exist_ok=False)
        (self.root/'frames').mkdir()
        self.manifest = (self.root/'frames.jsonl').open('w', encoding='utf-8')
        self.count = 0
        self.bytes_saved = 0
        self.rejected = 0
        self.first_time = None
        self.last_time = None
        self.last_stamp = -1
        self.size = None
        self.camera_info_id = None
        self.camera_info_frame = None
        self.camera_info_size = None
        self.info_ids = set()
        self.samples = []
        self.create_subscription(CameraInfo, args.camera_info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, args.image_topic, self.on_image, qos_profile_sensor_data)

    def on_info(self, msg):
        payload = {key: list(getattr(msg, key)) for key in ('k', 'd', 'r', 'p')}
        payload.update(width=msg.width, height=msg.height, frame_id=msg.header.frame_id,
                       distortion_model=msg.distortion_model, binning_x=msg.binning_x,
                       binning_y=msg.binning_y,
                       roi={key: getattr(msg.roi, key) for key in
                            ('x_offset', 'y_offset', 'width', 'height', 'do_rectify')})
        encoded = json.dumps(payload, sort_keys=True).encode()
        identity = hashlib.sha256(encoded).hexdigest()[:16]
        if identity not in self.info_ids:
            (self.root/f'camera_info_{identity}.json').write_bytes(encoded)
            self.info_ids.add(identity)
        self.camera_info_id = identity
        self.camera_info_frame = msg.header.frame_id
        self.camera_info_size = (msg.width, msg.height)

    def on_image(self, msg):
        received = time.monotonic()
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if self.count >= self.args.max_frames or self.bytes_saved >= self.args.max_mb*1024**2:
            return
        if self.first_time is not None and received-self.first_time >= self.args.seconds:
            return
        if stamp <= self.last_stamp or not -1.0 <= age <= 2.0:
            self.rejected += 1
            return
        data = bytes(msg.data)
        if not data.startswith(b'\xff\xd8'):
            self.rejected += 1
            return
        frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.rejected += 1
            return
        size = (frame.shape[1], frame.shape[0])
        if self.size is not None and size != self.size:
            raise RuntimeError('Image resolution changed during capture; start a new session')
        self.size = size
        if self.first_time is None:
            self.first_time = received
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        thumb = cv2.resize(gray, (80, 60))
        change = None if not self.samples else float(np.mean(np.abs(thumb.astype(float)-self.samples[-1])))
        self.samples[:] = [thumb.astype(float)]
        name = f'frames/{self.count:06d}_{stamp}.jpg'
        (self.root/name).write_bytes(data)
        info_matches = self.camera_info_frame == msg.header.frame_id and self.camera_info_size == size
        row = dict(file=name, stamp_ns=stamp, received_utc=datetime.now(timezone.utc).isoformat(),
                   elapsed_s=received-self.first_time, source_age_s=age, frame_id=msg.header.frame_id,
                   camera_info_id=self.camera_info_id if info_matches else None,
                   width=size[0], height=size[1], bytes=len(data),
                   brightness=float(gray.mean()), sharpness=float(cv2.Laplacian(gray, cv2.CV_64F).var()),
                   thumbnail_change=change)
        self.manifest.write(json.dumps(row)+'\n')
        self.manifest.flush()
        self.count += 1
        self.bytes_saved += len(data)
        self.last_stamp, self.last_time = stamp, received
        if self.count == 1 or self.count % 150 == 0:
            self.get_logger().info(f'Saved {self.count} JPEGs, {self.bytes_saved/1024**2:.1f} MiB: {self.root}')

    def finish(self, reason):
        self.manifest.close()
        rows = [json.loads(line) for line in (self.root/'frames.jsonl').read_text().splitlines()]
        seconds = 0 if self.count < 2 else self.last_time-self.first_time
        fps = (self.count-1)/seconds if seconds > 0 else 0
        video_written = False
        if self.count >= 2:
            video = cv2.VideoWriter(str(self.root/'preview.avi'), cv2.VideoWriter_fourcc(*'MJPG'), fps, self.size)
            try:
                if video.isOpened():
                    for row in rows:
                        frame = cv2.imread(str(self.root/row['file']))
                        if frame is None:
                            raise RuntimeError(f'Cannot read saved JPEG: {row["file"]}')
                        video.write(frame)
                    video_written = True
            finally:
                video.release()
        result = dict(reason=reason, frames=self.count, jpeg_bytes=self.bytes_saved,
                      observed_duration_s=seconds, observed_fps=fps, rejected_frames=self.rejected,
                      resolution=self.size, preview_video=video_written,
                      frames_with_matching_camera_info=sum(r['camera_info_id'] is not None for r in rows),
                      image_topic=self.args.image_topic, camera_info_topic=self.args.camera_info_topic,
                      labels_created=False, robot_commands_sent=False)
        (self.root/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2), flush=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New session directory (must not exist)')
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--startup-timeout', type=float, default=15)
    parser.add_argument('--max-frames', type=int, default=1800)
    parser.add_argument('--max-mb', type=float, default=200)
    parser.add_argument('--image-topic', default='/camera/image_raw/compressed')
    parser.add_argument('--camera-info-topic', default='/camera/camera_info')
    args = parser.parse_args()
    if any(not math.isfinite(x) or x <= 0 for x in
           (args.seconds, args.startup_timeout, args.max_frames, args.max_mb)):
        parser.error('Capture limits must be finite and positive')
    if args.output.exists():
        parser.error('Output directory already exists; choose a new session name')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    reason = 'interrupted'
    try:
        node = Collector(args)
        start = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if node.first_time is None:
                if now-start > args.startup_timeout:
                    reason = 'no_fresh_images'
                    break
            elif now-node.first_time >= args.seconds:
                reason = 'duration_complete'
                break
            elif now-node.last_time > 5:
                reason = 'stream_lost'
                break
            if node.count >= args.max_frames or node.bytes_saved >= args.max_mb*1024**2:
                reason = 'storage_limit'
                break
    except KeyboardInterrupt:
        reason = 'user_interrupted'
    except Exception:
        reason = 'error'
        raise
    finally:
        try:
            if node is not None:
                result = node.finish(reason)
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
    return 0 if result['frames'] > 0 and reason == 'duration_complete' else 2


if __name__ == '__main__':
    raise SystemExit(main())
