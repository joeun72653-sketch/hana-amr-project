#!/usr/bin/env python3

import os
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from ultralytics import YOLO
from pyorbbecsdk import (
    Pipeline, Config, OBSensorType, OBAlignMode, OBFormat
)

MODEL_PATH = os.path.expanduser(
    "~/amr_openvino/models/epoch22_openvino_model"
)

HOST = "0.0.0.0"
PORT = 8090
TARGET_FPS = 10
JPEG_QUALITY = 70
ROTATE_CAMERA = True

MIN_DEPTH_M = 0.3
MAX_DEPTH_M = 10.0
ROI_FRACTION = 0.4

latest_frame = None
frame_lock = threading.Lock()
stop_event = threading.Event()


def estimate_distance(depth_m, box):
    """객체 박스 중앙 40% 영역의 깊이 중앙값."""
    h, w = depth_m.shape

    x1, y1, x2, y2 = map(float, box)
    x1, x2 = np.clip([x1, x2], 0, w)
    y1, y2 = np.clip([y1, y2], 0, h)

    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2

    half_w = (x2 - x1) * ROI_FRACTION / 2
    half_h = (y2 - y1) * ROI_FRACTION / 2

    xa = max(0, int(cx - half_w))
    xb = min(w, int(cx + half_w))
    ya = max(0, int(cy - half_h))
    yb = min(h, int(cy + half_h))

    if xb <= xa or yb <= ya:
        return None

    roi = depth_m[ya:yb, xa:xb]

    valid = roi[
        np.isfinite(roi) &
        (roi >= MIN_DEPTH_M) &
        (roi <= MAX_DEPTH_M)
    ]

    if valid.size < 20:
        return None

    if valid.size / roi.size < 0.4:
        return None

    q25, median, q75 = np.percentile(
        valid, [25, 50, 75]
    )

    if q75 - q25 > max(0.15, median * 0.1):
        return None

    return float(median)


def camera_worker():
    global latest_frame

    pipeline = None
    started = False

    try:
        print("[AI] OpenVINO 모델 로딩", flush=True)
        model = YOLO(MODEL_PATH, task="detect")

        pipeline = Pipeline()
        config = Config()

        depth_profiles = pipeline.get_stream_profile_list(
            OBSensorType.DEPTH_SENSOR
        )
        color_profiles = pipeline.get_stream_profile_list(
            OBSensorType.COLOR_SENSOR
        )

        depth_profile = depth_profiles.get_video_stream_profile(
            640, 480, OBFormat.Y16, 30
        )
        color_profile = color_profiles.get_video_stream_profile(
            640, 480, OBFormat.MJPG, 30
        )

        config.enable_stream(depth_profile)
        config.enable_stream(color_profile)
        config.set_align_mode(OBAlignMode.SW_MODE)

        pipeline.start(config)
        started = True

        print("[CAMERA] RGB + Depth 시작", flush=True)

        last_processed = 0.0

        while not stop_event.is_set():
            frames = pipeline.wait_for_frames(1000)

            if frames is None:
                continue

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if color_frame is None or depth_frame is None:
                continue

            now = time.monotonic()
            if now - last_processed < 1.0 / TARGET_FPS:
                continue
            last_processed = now

            # MJPEG RGB 디코딩
            color_bytes = np.frombuffer(
                color_frame.get_data(), dtype=np.uint8
            )
            color = cv2.imdecode(
                color_bytes, cv2.IMREAD_COLOR
            )

            if color is None:
                continue

            dw = depth_frame.get_width()
            dh = depth_frame.get_height()

            raw = np.frombuffer(
                depth_frame.get_data(), dtype=np.uint16
            ).copy()

            if raw.size != dw * dh:
                print("[DEPTH] 프레임 크기 오류", flush=True)
                continue

            depth_raw = raw.reshape(dh, dw)

            # Orbbec Y16 깊이값: SDK scale 적용 후 mm -> m
            scale = depth_frame.get_depth_scale()
            depth_m = (
                depth_raw.astype(np.float32)
                * scale / 1000.0
            )

            # 동일 해상도 확인.
            # SW D2C가 실제로 정렬했는지는 별도 검증 필요.
            aligned_shape = (
                color.shape[:2] == depth_m.shape
            )

            if ROTATE_CAMERA:
                color = cv2.rotate(
                    color, cv2.ROTATE_180
                )
                depth_m = cv2.rotate(
                    depth_m, cv2.ROTATE_180
                )

            results = model.predict(
                source=color,
                imgsz=640,
                device="cpu",
                conf=0.25,
                verbose=False
            )

            result = results[0]
            annotated = result.plot()

            for box in result.boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                cls_id = int(box.cls[0])
                name = result.names[cls_id]

                if aligned_shape:
                    distance = estimate_distance(
                        depth_m, xyxy
                    )
                else:
                    distance = None

                x1, y1, x2, y2 = map(int, xyxy)

                label = (
                    f"{name} Z~{distance:.2f}m"
                    if distance is not None
                    else f"{name} Z:N/A"
                )

                text_y = min(
                    annotated.shape[0] - 8,
                    max(18, y2 + 20)
                )

                cv2.putText(
                    annotated,
                    label,
                    (max(0, x1), text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2
                )

            cv2.putText(
                annotated,
                "Depth Z estimate - NOT safety rated",
                (8, annotated.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (255, 255, 255),
                1
            )

            success, encoded = cv2.imencode(
                ".jpg",
                annotated,
                [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )

            if success:
                with frame_lock:
                    latest_frame = encoded.tobytes()

    except Exception as e:
        print("[CAMERA ERROR]", repr(e), flush=True)

    finally:
        stop_event.set()
        if started:
            pipeline.stop()
        print("[CAMERA] 종료", flush=True)


class CameraHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        path = self.path.split("?")[0]

        if path == "/health":
            with frame_lock:
                ready = latest_frame is not None

            body = b"CAMERA_OK" if ready else b"CAMERA_NOT_READY"
            self.send_response(200 if ready else 503)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path != "/stream.mjpg":
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=frame"
        )
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

        try:
            while not stop_event.is_set():
                with frame_lock:
                    frame = latest_frame

                if frame is None:
                    time.sleep(0.1)
                    continue

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(
                    b"Content-Type: image/jpeg\r\n"
                )
                self.wfile.write(
                    f"Content-Length: {len(frame)}\r\n\r\n".encode()
                )
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()

                time.sleep(1 / TARGET_FPS)

        except (
            BrokenPipeError,
            ConnectionResetError,
            TimeoutError
        ):
            pass


def main():
    if not os.path.isdir(MODEL_PATH):
        raise FileNotFoundError(MODEL_PATH)

    worker = threading.Thread(
        target=camera_worker,
        daemon=True
    )
    worker.start()

    server = ThreadingHTTPServer(
        (HOST, PORT), CameraHandler
    )

    print("[SERVER] MJPEG:", PORT, flush=True)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        server.server_close()
        worker.join(timeout=3)


if __name__ == "__main__":
    main()
