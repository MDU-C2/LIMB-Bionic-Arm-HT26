# Sensors

Shared sensor input for recording, monitoring and live control.

- `ble_input.py`, `serial_input.py`: sensor connections and live samples.
- `emg_packets.py`, `imu_packets.py`: pure wire decoders with explicit legacy rules.
- `ble_protocol.py`, `serial_protocol.py`, `clock_sync.py`: packet accounting and timing.
- `settings.py`, `ble_discovery.py`: common defaults and current/earlier board discovery.
- `fusion.py`, `imu_twist.py`: arm orientation, alignment and camera correction.
- `arm_alignment.py`: fit functional sensor-to-arm rotations for subject recordings.
- `reference_capture.py`: bounded live reference capture, fitting and calibration provenance.
- `emg_filter.py`, `emg_protocol.py`: EMG filtering, calibration and grip activation.
- `emg_window.py`: causal windows and features shared by training and live models.
- `fusion_comparison.py`: offline planar filter comparisons with an independent reference.
- `imu_protocol.py`: serial channel assembly, sample times and independent expiry.
- `camera/`: camera setup, arm and hand tracking, cup detection and geometry.
- `replay_input.py`: recorded cup trials without connected hardware.

The second IMU is mounted on the forearm. Its packet role is still `wrist`
to stay compatible with firmware and earlier recordings.

Connection workers own acquisition. Camera workers publish complete observations;
control consumes them under one state lock without waiting for inference. Raw BLE
files have a separate bounded writer. Queue overruns and worker failures are
reported; they must not be treated as a complete capture.
