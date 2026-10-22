# Sensors

Shared sensor input for recording, monitoring and live control.

- `ble_input.py`, `serial_input.py`: sensor connections and live samples.
- `ble_protocol.py`, `serial_protocol.py`, `clock_sync.py`: shared decoding and timing.
- `settings.py`, `ble_discovery.py`: common defaults and current/earlier board discovery.
- `fusion.py`, `imu_twist.py`: arm orientation, alignment and camera correction.
- `emg_filter.py`, `emg_protocol.py`: EMG filtering, calibration and grip activation.
- `imu_protocol.py`: current and older IMU packet formats.
- `camera/`: camera setup, arm and hand tracking, cup detection and geometry.
- `replay_input.py`: recorded cup trials without connected hardware.

The second IMU is mounted on the forearm. Its packet role is still `wrist`
to stay compatible with firmware and earlier recordings.
