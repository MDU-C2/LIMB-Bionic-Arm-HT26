"""Open a live RGB preview from a connected Luxonis OAK camera."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "recording"))

from record_oak_pose import configure_depthai_runtime


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="Exit after this many frames; 0 keeps the preview open.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Validate frame delivery without opening a window (requires --frames).",
    )
    return parser.parse_args()


def main() -> None:
    configure_depthai_runtime()
    import cv2
    import depthai as dai

    args = parse_args()
    if args.width < 1 or args.height < 1 or args.frames < 0:
        raise ValueError("width and height must be positive; frames cannot be negative")
    if args.headless and args.frames == 0:
        raise ValueError("--headless requires a positive --frames value")

    available = dai.Device.getAllAvailableDevices()
    if not available:
        raise RuntimeError("No OAK camera found. Check the USB cable and reconnect the camera.")

    device = dai.Device(available[0])
    print(f"Connected: {device.getDeviceName()} ({device.getDeviceId()})")
    with dai.Pipeline(device) as pipeline:
        camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        frames = camera.requestOutput((args.width, args.height)).createOutputQueue()
        pipeline.start()

        print("RGB stream started. Press Q or Esc to close the preview.")
        received = 0
        while pipeline.isRunning():
            frame = frames.get().getCvFrame()
            received += 1
            if not args.headless:
                cv2.imshow("AURORA - OAK-D Lite RGB", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
            if args.frames and received >= args.frames:
                break
        if args.frames:
            print(
                f"PASS: received {received}/{args.frames} RGB frames "
                f"at {args.width}x{args.height}."
            )


if __name__ == "__main__":
    try:
        main()
    finally:
        try:
            import cv2

            cv2.destroyAllWindows()
        except ImportError:
            pass
