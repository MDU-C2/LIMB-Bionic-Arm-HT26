"""Open a live RGB preview from a connected Luxonis OAK camera."""

from __future__ import annotations

import argparse

import cv2
import depthai as dai


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.width < 1 or args.height < 1:
        raise ValueError("width and height must be positive")

    available = dai.Device.getAllAvailableDevices()
    if not available:
        raise RuntimeError("No OAK camera found. Check the USB cable and reconnect the camera.")

    device = dai.Device(available[0])
    print(f"Connected: {device.getDeviceName()} ({device.getDeviceId()})")
    with dai.Pipeline(device) as pipeline:
        camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        frames = camera.requestOutput((args.width, args.height)).createOutputQueue()
        pipeline.start()

        print("Live RGB preview started. Press Q or Esc to close.")
        while pipeline.isRunning():
            frame = frames.get().getCvFrame()
            cv2.imshow("AURORA - OAK-D Lite RGB", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    finally:
        cv2.destroyAllWindows()
