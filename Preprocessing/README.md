# AURORA – Preprocessing
## Overview

Preprocessing converts recorded **EMG, IMU and camera data** into filtered, structured sensor states for future sensor fusion and Learning from Demonstration (LfD). The current version runs **offline on recorded trials**; real-time execution on the Jetson AGX Orin is planned.

```text
EMG + 2 IMUs (BLE) ─┐
                    ├── Preprocessing ──> Sensor states ──> Future sensor fusion
Camera (OAK-D Lite) ┘
```

## File Functions

| File | Purpose |
|---|---|
| `main.py`                     | Selects trials, runs preprocessing, displays plots and saves results. |
| `filters.py`                  | Filters EMG and IMU signals and provides reusable EMA filtering.      |
| `vision.py`                   | Processes arm positions, joint angles and finger tracking.            |
| `sensor_state.py`             | Defines the output data structures shared with future sensor fusion.  |
| `reconstruct_timestamps.py`   | Syncs EMG and IMU data that has drifted from each others timeline     |


## Preprocessing

**EMG**
- Subtracts a baseline, estimates muscle activity using absolute deviation and smooths it with EMA.
- Checks timestamps and invalid measurements; optional normalization and activation thresholds require calibration.

**IMU (two sensors)**
- Corrects gyroscope bias and smooths acceleration and angular velocity with EMA.
- Checks timestamps and saturation; orientation estimation is **not yet implemented**.

**Camera**
- Validates and smooths shoulder, elbow and wrist 3D positions with EMA.
- Smooths finger-bending estimates and flags lost tracking; grip closure requires open/closed-hand calibration.

## How to Run

Install plotting support if needed: `python -m pip install matplotlib`
From the project root, run:

```bash
python Preprocessing/main.py
```
| Menu | Action |
|---|---|
| **1. Preview EMG + IMU**      | Compare raw and filtered EMG, acceleration and gyroscope signals for both IMUs. |
| **2. Preview camera**         | Compare raw/filtered shoulder, elbow and wrist XYZ, inspect finger curl and tracking validity. |
| **3. Process one recording**  | Select a subject/trial and save processed data, plots and a summary. |
| **4. Process all recordings** | Process all discovered trials and save their outputs. |
| **5. Exit**                   | Close the program. |

Camera XYZ preview is shown as **displacement from the first valid point (cm)** in the updated local plotting code; saved 3D positions remain in metres.

## Technical Information
### Folder structure
```text
project/
├── Preprocessing/
│   ├── main.py
│   ├── filters.py
│   ├── vision.py
│   └── sensor_state.py
└── outputs/
    ├── recordings/SXX/TXXX.../
    │   ├── ble/emg.csv, imu.csv
    │   └── camera/pose.json
    └── processed/SXX/TXXX.../
```

### Input → output

|       Input       |                   Processed output                                |
|-------------------|-------------------------------------------------------------------|
| `ble/emg.csv`     | `EMGState`: muscle activity, validity and optional activation.    |
| `ble/imu.csv`     | `IMUState` per sensor: filtered acceleration/gyro and validity.   |
| `camera/pose.json`| `CameraState`: arm positions/angles and finger tracking.          |

The program saves `emg_processed.csv`, `imu_sensor_0_processed.csv`, `imu_sensor_1_processed.csv`, `camera_processed.csv`, `preprocessing_events.jsonl`, `summary.json` and preview PNGs where data is available. **Raw recordings are not overwritten.** Camera data is optional.

### Important settings and code changes
- `main.py`: input/output paths, preview menu and temporary calibration window (`PREVIEW_CALIBRATION_S = 0.5`).
- `filters.py`: `EMGConfig` (activity smoothing, timestamp gaps) and `IMUConfig` (EMA smoothing, timestamp gaps).
- `vision.py`: `VisionConfig` (position/finger smoothing, visibility and outlier limits).
- `sensor_state.py`: add/change fields passed to future fusion; update the corresponding processor at the same time.
- All processing uses timestamps in **nanoseconds**; IMU acceleration is in **g**, gyro in **°/s**, camera positions in **m**.

### Current limitations:
Calibration uses a provisional initial recording window, sensor streams are not yet fully synchronized into combined `SensorState` packets, and the program does not perform sensor fusion or live acquisition. For trials where camera pose and observation counts differ, ensure the local `vision.py` matches records by timestamp rather than list index.

## Debugging & Calibration

### Completed
- [x] **EMG Timing:** Fixed sample timestamp reconstruction. No sample time gaps reported in the latest test.
- [x] **EMG Preprocessing:** 0.5 s baseline, absolute deviation and 50 ms EMA. Continuous muscle activity without binary classification.
- [x] **IMU Mapping:** Verified `sensor=0` = Wrist/Forearm and `sensor=1` = Shoulder/Upper Arm.
- [x] **IMU Calibration:** Gyroscope bias correction using the first 0.5 s. Acceleration retains gravity.
- [x] **IMU Filtering:** Reviewed EMA filtering and gyroscope saturation handling.
- [x] **Camera Coordinates:** Verified XYZ coordinate convention, units and negative Z-values.
- [x] **Camera Timing:** Reviewed RGB/depth timestamp matching and frame handling.
- [x] **Camera Position Filtering:** Reviewed XYZ smoothing, validity checks and position-jump handling.

### Remaining
- [ ] **EMG Quality:** Further validate electrode placement, signal variability and movement artifacts across participants.
- [ ] **IMU Orientation:** Implement orientation estimation and verify sensor-to-arm alignment.
- [ ] **Camera Depth:** Investigate systematic Z-axis jumps in raw stereo depth measurements.
- [ ] **Finger Tracking:** Validate finger curl, EMA smoothing and tracking loss caused by occlusion.
- [ ] **Grip Calibration:** Establish open/closed reference values and validate grip estimation.
- [ ] **Synchronization:** Validate BLE/camera timing, sampling rates and processing delays.
- [ ] **Sensor Fusion:** Combine processed EMG, IMU and camera data to estimate arm movement and hand states.
- [ ] **Final Validation:** Test several recordings, tune parameters and verify real-time performance on Jetson AGX Orin.