#!/usr/bin/env python3
"""Orbbec depth-only ROS publisher: no color, MJPEG, AI or point cloud.

Uses the depth optical frame's own intrinsics, not RGB intrinsics.
Relies on the existing map -> base_link -> optical TF; mounting geometry is
not invented or broadcast here.
"""
import argparse
import math
import signal
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from pyorbbecsdk import Pipeline, Config, OBSensorType, OBFormat

DEPTH_TOPIC = '/camera/depth/image_raw'
INFO_TOPIC = '/camera/registered_color/camera_info'


def measurement_stamp(frame):
    """Require SDK host-epoch capture timestamp; never substitute reception time.

    The global timestamp is the hardware capture timestamp mapped into host
    clock time. Zero/unavailable or non-epoch timestamps are rejected rather
    than silently publishing an incorrect ROS timestamp.
    """
    ts_us = int(frame.get_global_timestamp_us())
    if not (1_577_836_800_000_000 <= ts_us < 4_102_444_800_000_000):
        raise ValueError('Orbbec global capture timestamp unavailable or not Unix epoch')
    return ts_us // 1_000_000, (ts_us % 1_000_000) * 1000


def depth_to_meters(frame):
    width, height = int(frame.get_width()), int(frame.get_height())
    if width <= 0 or height <= 0 or frame.get_format() != OBFormat.Y16:
        raise ValueError('Expected positive-sized Y16 depth frame')
    raw = np.frombuffer(frame.get_data(), dtype='<u2')
    if raw.size != width * height:
        raise ValueError(f'Invalid depth payload length: {raw.size}, expected {width * height}')
    scale_mm = float(frame.get_depth_scale())
    if not math.isfinite(scale_mm) or scale_mm <= 0:
        raise ValueError(f'Invalid Orbbec depth scale: {scale_mm}')
    # Orbbec Y16 raw * SDK scale = millimeters. 32FC1 is in meters.
    data = raw.astype('<f4') * (scale_mm / 1000.0)
    data[raw == 0] = np.nan
    return data, width, height


def camera_info_from_profile(profile, width, height, frame_id, stamp):
    intrinsic = profile.get_intrinsic()
    if int(intrinsic.width) != width or int(intrinsic.height) != height:
        raise ValueError('Depth intrinsics resolution differs from frame resolution')
    fx, fy = float(intrinsic.fx), float(intrinsic.fy)
    cx, cy = float(intrinsic.cx), float(intrinsic.cy)
    if any(not math.isfinite(v) for v in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
        raise ValueError('Invalid Orbbec depth intrinsics')
    info = CameraInfo()
    info.header.stamp = stamp
    info.header.frame_id = frame_id
    info.width, info.height = width, height
    info.distortion_model = 'plumb_bob'
    info.d = [0.0] * 5
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    # If SDK reports actual distortion, do not fabricate a zero-distortion model.
    distortion = profile.get_distortion()
    coeffs = [float(getattr(distortion, key)) for key in ('k1', 'k2', 'p1', 'p2', 'k3')]
    if any(not math.isfinite(v) for v in coeffs):
        raise ValueError('Invalid Orbbec distortion coefficients')
    info.d = coeffs
    return info


class DepthPublisher(Node):
    def __init__(self, frame_id, width, height, fps):
        super().__init__('amr_orbbec_depth_only')
        self.frame_id = frame_id
        self.pipeline = Pipeline()
        device = self.pipeline.get_device()
        if not device.is_global_timestamp_supported():
            raise RuntimeError(
                'Orbbec device does not support global capture timestamps; '
                'cannot satisfy measurement-time header.stamp requirement')
        device.enable_global_timestamp(True)
        self.config = Config()
        profiles = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        self.profile = profiles.get_video_stream_profile(width, height, OBFormat.Y16, fps)
        self.config.enable_stream(self.profile)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.depth_pub = self.create_publisher(Image, DEPTH_TOPIC, qos)
        self.info_pub = self.create_publisher(CameraInfo, INFO_TOPIC, qos)
        self.started = False
        self.pipeline.start(self.config)
        self.started = True
        self.get_logger().info(
            f'Depth only: {width}x{height}@{fps}, frame={frame_id}; '
            'YOLO/RGB/MJPEG disabled; external map->base_link->optical TF required'
        )

    def run(self):
        missing_stamp = 0
        while rclpy.ok():
            frames = self.pipeline.wait_for_frames(1000)
            if frames is None:
                continue
            frame = frames.get_depth_frame()
            if frame is None:
                continue
            try:
                sec, nsec = measurement_stamp(frame)
            except ValueError as exc:
                missing_stamp += 1
                if missing_stamp >= 30:
                    raise RuntimeError(
                        'No valid globally synchronized capture timestamps for 30 frames. '
                        'Enable Orbbec global timestamp / device clock synchronization; '
                        'refusing receipt-time substitution.'
                    ) from exc
                continue
            missing_stamp = 0
            data, width, height = depth_to_meters(frame)
            stamp = self.get_clock().now().to_msg()
            stamp.sec, stamp.nanosec = sec, nsec
            profile = frame.get_stream_profile().as_video_stream_profile()
            info = camera_info_from_profile(profile, width, height, self.frame_id, stamp)
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.width, msg.height = width, height
            msg.encoding = '32FC1'
            msg.is_bigendian = 0
            msg.step = width * 4
            msg.data = data.tobytes(order='C')
            self.info_pub.publish(info)
            self.depth_pub.publish(msg)

    def close(self):
        if self.started:
            self.pipeline.stop()
            self.started = False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frame-id', default='camera_depth_optical_frame')
    parser.add_argument('--width', type=int, default=960)
    parser.add_argument('--height', type=int, default=600)
    parser.add_argument('--fps', type=int, default=20)
    args = parser.parse_args()
    if not args.frame_id or args.frame_id.startswith('/'):
        parser.error('--frame-id must be a relative TF frame name')
    rclpy.init()
    node = None
    try:
        node = DepthPublisher(args.frame_id, args.width, args.height, args.fps)
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'[CAMERA ERROR] {exc}', file=sys.stderr, flush=True)
        raise
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
