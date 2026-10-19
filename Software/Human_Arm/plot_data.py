
import pandas as pd
import matplotlib.pyplot as plt

from pathlib import Path

# ==========================================
# CONFIGURATION
# ==========================================

BASE_DIR = Path(__file__).resolve().parent
RECORDING_FOLDER = BASE_DIR / "recordings"

IMU_AXES = ["ax", "ay", "az", "gx", "gy", "gz"]


# ==========================================
# SELECT RECORDING
# ==========================================

def select_recording():

    files = sorted(
        RECORDING_FOLDER.glob("recording_*.csv"),
        key=lambda f: f.stat().st_mtime,
        reverse=True
    )

    if not files:
        print("No recordings found!")
        return None

    print("\n========== RECORDINGS ==========\n")

    for i, file in enumerate(files, start=1):

        size_kb = file.stat().st_size / 1024

        print(
            f"{i}. {file.name} ({size_kb:.1f} KB)"
        )

    print("\n0. Exit")

    while True:

        choice = input("\nSelect recording: ").strip()

        if choice == "0":
            return None

        if choice.isdigit():

            index = int(choice)

            if 1 <= index <= len(files):
                return files[index - 1]

        print("Invalid selection. Try again.")


# ==========================================
# LOAD CSV
# ==========================================

def load_recording(filename):

    print(f"\nLoading: {filename.name}")

    # Automatically detect comma or semicolon
    data = pd.read_csv(
        filename,
        sep=None,
        engine="python"
    )

    # Convert numerical columns
    for column in data.columns:

        data[column] = pd.to_numeric(
            data[column],
            errors="coerce"
        )

    # Reconstruct common ESP32 start time
    # timestamp_s = (timestamp_us - t0) / 1e6

    if (
        "timestamp_us" in data.columns
        and "timestamp_s" in data.columns
    ):

        valid = data.dropna(
            subset=["timestamp_us", "timestamp_s"]
        )

        if not valid.empty:

            first = valid.iloc[0]

            t0 = (
                first["timestamp_us"]
                - first["timestamp_s"] * 1_000_000
            )

        else:
            t0 = None

    else:
        t0 = None

    return data, t0


# ==========================================
# DETECT AVAILABLE SENSORS
# ==========================================

def has_data(data, column):

    return (
        column in data.columns
        and data[column].notna().any()
    )


def available_sensors(data):

    return {
        "EMG": has_data(data, "emg_raw"),
        "IMU1": any(
            has_data(data, f"imu1_{axis}")
            for axis in IMU_AXES
        ),
        "IMU2": any(
            has_data(data, f"imu2_{axis}")
            for axis in IMU_AXES
        )
    }


# ==========================================
# SELECT PLOT MODE
# ==========================================

def select_plot_mode(data):

    available = available_sensors(data)

    print("\n========== AVAILABLE DATA ==========\n")

    for sensor, exists in available.items():

        status = "Available" if exists else "No data"

        print(f"{sensor}: {status}")

    print("\n========== PLOT MODE ==========\n")

    print("1. EMG")
    print("2. IMU 1")
    print("3. IMU 2")
    print("4. All available sensors")
    print("0. Exit")

    while True:

        choice = input("\nSelect plot mode: ").strip()

        if choice in ["0", "1", "2", "3", "4"]:
            return choice

        print("Invalid selection.")


# ==========================================
# GET SENSOR TIME
# ==========================================

def get_time(data, sensor, t0):

    # EMG uses the main time column
    if sensor == "EMG":

        return data["timestamp_s"]

    # IMU uses its original acquisition timestamp
    column = f"{sensor.lower()}_timestamp_us"

    if t0 is not None and column in data.columns:

        return (
            data[column] - t0
        ) / 1_000_000

    # Fallback for older CSV files
    return data["timestamp_s"]


# ==========================================
# PLOT EMG
# ==========================================

def plot_emg(ax, data):

    if not has_data(data, "emg_raw"):
        return False

    emg = data.dropna(
        subset=["timestamp_s", "emg_raw"]
    )

    if emg.empty:
        return False

    ax.plot(
        emg["timestamp_s"],
        emg["emg_raw"],
        label="EMG",
        linewidth=0.8
    )

    ax.set_title("EMG")
    ax.set_ylabel("ADC Raw")
    ax.grid(True)

    return True


# ==========================================
# PLOT IMU
# ==========================================

def plot_imu(ax, data, sensor, t0, measurement):

    prefix = sensor.lower()

    if measurement == "acc":
        axes = ["ax", "ay", "az"]
        title = f"{sensor} - Accelerometer"
        ylabel = "Acceleration"

    else:
        axes = ["gx", "gy", "gz"]
        title = f"{sensor} - Gyroscope"
        ylabel = "Angular velocity"

    time = get_time(data, sensor, t0)

    plotted = False

    for axis in axes:

        column = f"{prefix}_{axis}"

        if not has_data(data, column):
            continue

        valid = (
            time.notna()
            & data[column].notna()
        )

        if not valid.any():
            continue

        ax.plot(
            time[valid],
            data.loc[valid, column],
            label=axis.upper(),
            linewidth=0.9
        )

        plotted = True

    if plotted:

        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.legend()
        ax.grid(True)

    return plotted


# ==========================================
# CREATE PLOTS
# ==========================================

def create_plots(data, t0, mode, filename):

    available = available_sensors(data)

    selected = []

    if mode == "1":
        selected = ["EMG"]

    elif mode == "2":
        selected = ["IMU1"]

    elif mode == "3":
        selected = ["IMU2"]

    elif mode == "4":
        selected = [
            sensor
            for sensor, exists in available.items()
            if exists
        ]

    # Remove sensors without actual data
    selected = [
        sensor
        for sensor in selected
        if available.get(sensor, False)
    ]

    if not selected:

        print("\nNo data available for selected sensor.")
        return

    # One EMG plot, two plots per IMU
    plot_count = sum(
        1 if sensor == "EMG" else 2
        for sensor in selected
    )

    fig, axes = plt.subplots(
        plot_count,
        1,
        figsize=(13, 3 * plot_count),
        sharex=True,
        squeeze=False
    )

    axes = axes.flatten()

    index = 0

    for sensor in selected:

        if sensor == "EMG":

            plot_emg(
                axes[index],
                data
            )

            index += 1

        else:

            plot_imu(
                axes[index],
                data,
                sensor,
                t0,
                "acc"
            )

            index += 1

            plot_imu(
                axes[index],
                data,
                sensor,
                t0,
                "gyro"
            )

            index += 1

    axes[-1].set_xlabel("Time [s]")

    fig.suptitle(
        filename.name,
        fontsize=12
    )

    fig.tight_layout()

    plt.show()


# ==========================================
# MAIN
# ==========================================

def main():

    while True:

        filename = select_recording()

        if filename is None:
            break

        try:

            data, t0 = load_recording(filename)

            print(f"Rows: {len(data)}")

            mode = select_plot_mode(data)

            if mode == "0":
                continue

            create_plots(
                data,
                t0,
                mode,
                filename
            )

        except Exception as error:

            print(f"\nError: {error}")

        print("\nReturning to recording selection...")


if __name__ == "__main__":
    main()
