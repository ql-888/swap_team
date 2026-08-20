#!/usr/bin/env python3
"""Open the wrist D435i and/or global Gemini 336L color camera."""

import argparse
import sys
from dataclasses import dataclass
from typing import Callable

import cv2


@dataclass
class Camera:
    name: str
    read: Callable[[], object]
    stop: Callable[[], None]


def open_d435i(width: int, height: int, fps: int) -> Camera:
    try:
        import pyrealsense2 as rs
    except ImportError as error:
        raise RuntimeError("D435i requires pyrealsense2. Install librealsense Python bindings first.") from error

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    profile = pipeline.start(config)
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    print(
        f"D435i opened: {intrinsics.width}x{intrinsics.height} @ {fps} FPS, "
        f"fx={intrinsics.fx:.3f}, fy={intrinsics.fy:.3f}"
    )

    def read():
        frames = pipeline.wait_for_frames(2000)
        frame = frames.get_color_frame()
        if not frame:
            raise RuntimeError("No color frame received from D435i")
        import numpy as np

        return np.asanyarray(frame.get_data()).copy()

    return Camera("D435i wrist camera", read, pipeline.stop)


def open_gemini(width: int, height: int, fps: int, use_default_profile: bool) -> Camera:
    try:
        from pyorbbecsdk import Config, Context, OBFormat, OBLogLevel, OBSensorType, Pipeline
    except ImportError as error:
        raise RuntimeError("Gemini 336L requires pyorbbecsdk (pyorbbecsdk2).") from error

    context = Context()
    context.set_logger_level(OBLogLevel.WARNING)
    if context.query_devices().get_count() == 0:
        raise RuntimeError("No Orbbec Gemini 336L device found")

    pipeline = Pipeline()
    try:
        profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        if use_default_profile:
            profile = profiles.get_default_video_stream_profile()
        else:
            profile = profiles.get_video_stream_profile(width, height, OBFormat.RGB, fps)
        config = Config()
        config.enable_stream(profile)
        pipeline.start(config)
    except Exception:
        try:
            pipeline.stop()
        except Exception:
            pass
        raise

    print(f"Gemini 336L opened: {profile.get_width()}x{profile.get_height()} @ {profile.get_fps()} FPS")

    def read():
        import numpy as np

        frames = pipeline.wait_for_frames(2000)
        if frames is None or frames.get_color_frame() is None:
            raise RuntimeError("No color frame received from Gemini 336L")
        frame = frames.get_color_frame()
        width_px, height_px = frame.get_width(), frame.get_height()
        data = np.asanyarray(frame.get_data())
        frame_format = frame.get_format()
        if frame_format == OBFormat.RGB:
            return cv2.cvtColor(data.reshape(height_px, width_px, 3), cv2.COLOR_RGB2BGR)
        if frame_format == OBFormat.BGR:
            return data.reshape(height_px, width_px, 3).copy()
        if frame_format == OBFormat.MJPG:
            return cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame_format == OBFormat.YUYV:
            return cv2.cvtColor(data.reshape(height_px, width_px, 2), cv2.COLOR_YUV2BGR_YUY2)
        raise RuntimeError(f"Unsupported Gemini color format: {frame_format}")

    return Camera("Gemini 336L global camera", read, pipeline.stop)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", choices=("d435i", "gemini", "both"), default="both")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--gemini-requested-profile",
        action="store_true",
        help="Request --width/--height/--fps for Gemini instead of using its default color profile.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cameras: list[Camera] = []
    try:
        if args.camera in ("d435i", "both"):
            cameras.append(open_d435i(args.width, args.height, args.fps))
        if args.camera in ("gemini", "both"):
            cameras.append(
                open_gemini(args.width, args.height, args.fps, not args.gemini_requested_profile)
            )

        print("Press Q or Esc in an image window to quit.")
        while True:
            for camera in cameras:
                image = camera.read()
                if image is not None:
                    cv2.imshow(camera.name, image)
            if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                return 0
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print(f"Camera startup/read failed: {error}", file=sys.stderr)
        return 1
    finally:
        for camera in reversed(cameras):
            try:
                camera.stop()
            except Exception:
                pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
