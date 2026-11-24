"""Synthetic orientation checks for AURORA IMU preprocessing.

Run from the Preprocessing directory:
    python test_imu_orientation.py

Requires the updated IMUProcessor and IMUConfig from filters.py.
"""

import math

from filters import IMUProcessor, IMUConfig


def test_orientation():
    processor = IMUProcessor(IMUConfig())

    # Calibrate gyro with stationary samples (deg/s).
    processor.calibrate([(0.0, 0.0, 0.0)] * 50)

    dt_ns = 10_000_000  # 100 Hz

    # Test 1: Stationary, upright IMU for 2 seconds.
    for i in range(200):
        state = processor.update(
            timestamp_ns=i * dt_ns,
            accel=(0.0, 0.0, 1.0),
            gyro=(0.0, 0.0, 0.0),
        )

    assert state.orientation_valid
    assert state.orientation_quat is not None
    w, x, y, z = state.orientation_quat
    assert abs(w - 1.0) < 0.01
    print("PASS: Stationary orientation")

    # Test 2: Reset and rotate +90 degrees about sensor Z in 1 s.
    processor.reset_filters()

    for i in range(101):
        state = processor.update(
            timestamp_ns=i * dt_ns,
            accel=(0.0, 0.0, 1.0),
            gyro=(0.0, 0.0, 90.0),
        )

    assert state.orientation_valid
    assert state.orientation_quat is not None
    w, x, y, z = state.orientation_quat

    yaw = math.degrees(
        math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
    )

    print(f"Yaw after rotation: {yaw:.2f} deg")
    assert abs(yaw - 90.0) < 1.0
    print("PASS: Known 90-degree rotation")

    # Test 3: Initialize with the accelerometer facing upside down.
    processor.reset_filters()
    state = processor.update(
        timestamp_ns=0,
        accel=(0.0, 0.0, -1.0),
        gyro=(0.0, 0.0, 0.0),
    )

    assert state.orientation_valid
    assert state.orientation_quat is not None
    print("PASS: Upside-down initialization")


if __name__ == "__main__":
    test_orientation()
