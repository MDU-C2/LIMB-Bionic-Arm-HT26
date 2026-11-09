"""Plot a saved AURORA BLE recording."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt


def read_emg(path: Path):
    times = []
    values = []

    with path.open("r", newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)

        for row in reader:
            times.append(
                int(row["synced_host_monotonic_ns"])
            )
            values.append(
                float(row["value"])
            )

    return times, values


def read_imu(path: Path):
    data = {
        0: {
            "time": [],
            "accel_x": [],
            "accel_y": [],
            "accel_z": [],
            "gyro_x": [],
            "gyro_y": [],
            "gyro_z": [],
        },
        1: {
            "time": [],
            "accel_x": [],
            "accel_y": [],
            "accel_z": [],
            "gyro_x": [],
            "gyro_y": [],
            "gyro_z": [],
        },
    }

    with path.open("r", newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)

        for row in reader:
            sensor = int(row["sensor"])

            if sensor not in data:
                continue

            data[sensor]["time"].append(
                int(row["synced_host_monotonic_ns"])
            )

            data[sensor]["accel_x"].append(float(row["accel_x"]))
            data[sensor]["accel_y"].append(float(row["accel_y"]))
            data[sensor]["accel_z"].append(float(row["accel_z"]))

            data[sensor]["gyro_x"].append(float(row["gyro_x"]))
            data[sensor]["gyro_y"].append(float(row["gyro_y"]))
            data[sensor]["gyro_z"].append(float(row["gyro_z"]))

    return data


def normalize_time_ns(values, start_ns):
    return [
        (value - start_ns) / 1_000_000_000
        for value in values
    ]


def plot_recording(session: Path) -> None:

    emg_path = session / "emg.csv"
    imu_path = session / "imu.csv"

    if not emg_path.exists():
        raise FileNotFoundError(
            f"Missing EMG file: {emg_path}"
        )

    if not imu_path.exists():
        raise FileNotFoundError(
            f"Missing IMU file: {imu_path}"
        )

    emg_time_ns, emg_values = read_emg(emg_path)
    imu = read_imu(imu_path)

    all_start_times = []

    if emg_time_ns:
        all_start_times.append(emg_time_ns[0])

    for sensor in (0, 1):
        if imu[sensor]["time"]:
            all_start_times.append(
                imu[sensor]["time"][0]
            )

    if not all_start_times:
        raise RuntimeError(
            "No sensor samples found in recording."
        )

    start_ns = min(all_start_times)

    emg_time = normalize_time_ns(
        emg_time_ns,
        start_ns,
    )

    imu_time = {
        sensor: normalize_time_ns(
            imu[sensor]["time"],
            start_ns,
        )
        for sensor in (0, 1)
    }

    fig, axes = plt.subplots(
        5,
        1,
        figsize=(13, 10),
        sharex=True,
    )

    fig.suptitle(
        f"AURORA BLE Recording\n{session.name}"
    )

    # --------------------------------------------------------
    # EMG
    # --------------------------------------------------------

    axes[0].plot(
        emg_time,
        emg_values,
    )

    axes[0].set_title(
        "EMG - Raw ADC"
    )

    axes[0].set_ylabel(
        "ADC"
    )

    axes[0].grid(True)

    # --------------------------------------------------------
    # IMU 1 accelerometer
    # --------------------------------------------------------

    axes[1].plot(
        imu_time[0],
        imu[0]["accel_x"],
        label="X",
    )

    axes[1].plot(
        imu_time[0],
        imu[0]["accel_y"],
        label="Y",
    )

    axes[1].plot(
        imu_time[0],
        imu[0]["accel_z"],
        label="Z",
    )

    axes[1].set_title(
        "IMU 1 - Accelerometer"
    )

    axes[1].set_ylabel(
        "g"
    )

    axes[1].legend()
    axes[1].grid(True)

    # --------------------------------------------------------
    # IMU 1 gyroscope
    # --------------------------------------------------------

    axes[2].plot(
        imu_time[0],
        imu[0]["gyro_x"],
        label="X",
    )

    axes[2].plot(
        imu_time[0],
        imu[0]["gyro_y"],
        label="Y",
    )

    axes[2].plot(
        imu_time[0],
        imu[0]["gyro_z"],
        label="Z",
    )

    axes[2].set_title(
        "IMU 1 - Gyroscope"
    )

    axes[2].set_ylabel(
        "deg/s"
    )

    axes[2].legend()
    axes[2].grid(True)

    # --------------------------------------------------------
    # IMU 2 accelerometer
    # --------------------------------------------------------

    axes[3].plot(
        imu_time[1],
        imu[1]["accel_x"],
        label="X",
    )

    axes[3].plot(
        imu_time[1],
        imu[1]["accel_y"],
        label="Y",
    )

    axes[3].plot(
        imu_time[1],
        imu[1]["accel_z"],
        label="Z",
    )

    axes[3].set_title(
        "IMU 2 - Accelerometer"
    )

    axes[3].set_ylabel(
        "g"
    )

    axes[3].legend()
    axes[3].grid(True)

    # --------------------------------------------------------
    # IMU 2 gyroscope
    # --------------------------------------------------------

    axes[4].plot(
        imu_time[1],
        imu[1]["gyro_x"],
        label="X",
    )

    axes[4].plot(
        imu_time[1],
        imu[1]["gyro_y"],
        label="Y",
    )

    axes[4].plot(
        imu_time[1],
        imu[1]["gyro_z"],
        label="Z",
    )

    axes[4].set_title(
        "IMU 2 - Gyroscope"
    )

    axes[4].set_ylabel(
        "deg/s"
    )

    axes[4].set_xlabel(
        "Time [s]"
    )

    axes[4].legend()
    axes[4].grid(True)

    plt.tight_layout()
    plt.show()


def select_recording(recordings_root: Path) -> Path | None:

    if not recordings_root.exists():
        print(
            f"Recording directory does not exist:\n"
            f"{recordings_root}"
        )
        return None

    sessions = sorted(
        [
            path
            for path in recordings_root.iterdir()
            if path.is_dir()
        ],
        reverse=True,
    )

    if not sessions:
        print(
            "No recordings found."
        )
        return None

    print()
    print(
        "Available recordings:"
    )
    print()

    for index, session in enumerate(
        sessions,
        start=1,
    ):
        print(
            f"{index}. {session.name}"
        )

    print()

    selection = input(
        "Select recording: "
    ).strip()

    try:
        index = int(selection) - 1
    except ValueError:
        print(
            "Invalid selection."
        )
        return None

    if index < 0 or index >= len(sessions):
        print(
            "Invalid selection."
        )
        return None

    return sessions[index]


def main() -> int:

    recordings_root = (
        Path(__file__).resolve().parents[2]
        / "Recordings"
    )

    session = select_recording(
        recordings_root
    )

    if session is None:
        return 1

    try:
        plot_recording(
            session
        )

    except (
        FileNotFoundError,
        RuntimeError,
        ValueError,
    ) as error:

        print(
            f"Could not plot recording: {error}"
        )

        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(
        main()
    )