
import serial
import csv
from pathlib import Path
from datetime import datetime
from bisect import bisect_left
from statistics import median

# ==========================================
# CONFIGURATION
# ==========================================

PORT = "COM9"
BAUD = 115200

RECORDING_FOLDER = Path("recordings")

# Maximum allowed difference when matching
# IMU samples without an EMG timebase.
DEFAULT_TOLERANCE_US = 5000

COLUMNS = [
    "timestamp_s",
    "timestamp_us",
    "emg_raw",

    "imu1_timestamp_us",
    "imu1_ax",
    "imu1_ay",
    "imu1_az",
    "imu1_gx",
    "imu1_gy",
    "imu1_gz",

    "imu2_timestamp_us",
    "imu2_ax",
    "imu2_ay",
    "imu2_az",
    "imu2_gx",
    "imu2_gy",
    "imu2_gz"
]

# ==========================================
# DATA STORAGE
# ==========================================

emg_data = []
imu1_data = []
imu2_data = []

# ==========================================
# PARSE INCOMING DATA
# ==========================================

def parse_line(line):

    parts = line.strip().split(",")

    try:

        # Old EMG format:
        # timestamp,adc
        if len(parts) == 2:
            timestamp = int(parts[0])
            adc = int(parts[1])

            emg_data.append((timestamp, adc))
            return

        sensor = parts[0].upper()

        # New EMG format:
        # EMG,timestamp,adc
        if sensor == "EMG" and len(parts) == 3:

            timestamp = int(parts[1])
            adc = int(parts[2])

            emg_data.append((timestamp, adc))

        # IMU format:
        # IMU1,timestamp,ax,ay,az,gx,gy,gz
        elif sensor in ("IMU1", "IMU2") and len(parts) == 8:

            timestamp = int(parts[1])

            values = [
                float(x) for x in parts[2:]
            ]

            sample = (timestamp, *values)

            if sensor == "IMU1":
                imu1_data.append(sample)

            else:
                imu2_data.append(sample)

    except ValueError:
        # Ignore boot messages or invalid lines
        pass


# ==========================================
# SERIAL RECORDING
# ==========================================

def record():

    print("Connecting to ESP32...")

    with serial.Serial(
        PORT,
        BAUD,
        timeout=1
    ) as ser:

        print("Recording started!")
        print("Press Ctrl+C to stop.\n")

        try:

            while True:

                line = ser.readline().decode(
                    "utf-8",
                    errors="ignore"
                ).strip()

                if line:
                    parse_line(line)

        except KeyboardInterrupt:
            print("\nRecording stopped.")

    print(f"EMG samples:   {len(emg_data)}")
    print(f"IMU1 samples:  {len(imu1_data)}")
    print(f"IMU2 samples:  {len(imu2_data)}")


# ==========================================
# CREATE COMMON TIME AXIS
# ==========================================

def create_rows():

    all_timestamps = (
        [x[0] for x in emg_data] +
        [x[0] for x in imu1_data] +
        [x[0] for x in imu2_data]
    )

    if not all_timestamps:
        return []

    # Common start time
    t0 = min(all_timestamps)

    rows = []

    # EMG determines the main time axis
    for timestamp, value in sorted(emg_data):

        rows.append({
            "timestamp_us": timestamp,
            "emg_raw": value
        })

    # Determine matching tolerance
    if len(emg_data) > 1:

        times = sorted(x[0] for x in emg_data)

        intervals = [
            b - a
            for a, b in zip(times, times[1:])
            if b > a
        ]

        tolerance = (
            max(1, int(median(intervals) / 2))
            if intervals
            else DEFAULT_TOLERANCE_US
        )

    else:
        tolerance = DEFAULT_TOLERANCE_US

    # Add IMU samples
    for sensor, data in [
        ("imu1", imu1_data),
        ("imu2", imu2_data)
    ]:

        for sample in sorted(data):

            timestamp = sample[0]
            values = sample[1:]

            # Find nearest existing time row
            times = [r["timestamp_us"] for r in rows]

            index = bisect_left(times, timestamp)

            candidates = []

            if index < len(rows):
                candidates.append(index)

            if index > 0:
                candidates.append(index - 1)

            best = None
            best_diff = float("inf")

            for i in candidates:

                row = rows[i]

                # Do not overwrite an existing sample
                if f"{sensor}_timestamp_us" in row:
                    continue

                diff = abs(
                    row["timestamp_us"] - timestamp
                )

                if diff < best_diff:
                    best = row
                    best_diff = diff

            # Match if close enough
            if best is not None and best_diff <= tolerance:

                row = best

            else:

                # Preserve unmatched sample
                row = {
                    "timestamp_us": timestamp
                }

                rows.insert(index, row)

            row[f"{sensor}_timestamp_us"] = timestamp

            names = ["ax", "ay", "az", "gx", "gy", "gz"]

            for name, value in zip(names, values):
                row[f"{sensor}_{name}"] = value

    # Convert to relative time
    for row in rows:

        row["timestamp_s"] = (
            row["timestamp_us"] - t0
        ) / 1_000_000

    rows.sort(key=lambda r: r["timestamp_us"])

    return rows


# ==========================================
# SAVE CSV
# ==========================================

def save_csv(rows):

    RECORDING_FOLDER.mkdir(
        parents=True,
        exist_ok=True
    )

    timestamp = datetime.now().strftime(
        "%Y-%m-%d_%H-%M-%S_%f"
    )

    filename = (
        RECORDING_FOLDER /
        f"recording_{timestamp}.csv"
    )

    with open(filename, "w", newline="") as file:

        writer = csv.DictWriter(
            file,
            fieldnames=COLUMNS,
            delimiter=";"
        )

        writer.writeheader()
        writer.writerows(rows)

    print(f"\nSaved: {filename}")
    print(f"Total rows: {len(rows)}")


# ==========================================
# MAIN
# ==========================================

if __name__ == "__main__":

    record()

    rows = create_rows()

    if rows:
        save_csv(rows)

    else:
        print("No valid sensor data received.")
