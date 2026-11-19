from __future__ import annotations

import csv
import shutil
from pathlib import Path
import matplotlib.pyplot as plt

# ============================================================
# Paths
# ============================================================

PROJECT_ROOT = Path(__file__).resolve().parents[1]

RECORDINGS_DIR = PROJECT_ROOT / "outputs" / "recordings"
RECONSTRUCTED_DIR = PROJECT_ROOT / "outputs" / "reconstructed"


# ============================================================
# Configuration
# ============================================================

EMG_SAMPLE_RATE_HZ = 4000
EMG_PERIOD_NS = int(1_000_000_000 / EMG_SAMPLE_RATE_HZ)


# ============================================================
# CSV helpers
# ============================================================

def load_csv(path: Path) -> list[dict]:
    with path.open("r", newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def save_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        print(f"No rows to save: {path}")
        return

    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=rows[0].keys(),
        )

        writer.writeheader()
        writer.writerows(rows)


# ============================================================
# Reconstruction
# ============================================================

def reconstruct_emg_timestamps(
    emg_rows: list[dict],
    imu_rows: list[dict],
) -> list[dict]:

    if not emg_rows:
        raise ValueError("EMG file is empty.")

    if not imu_rows:
        raise ValueError("IMU file is empty.")

    # --------------------------------------------------------
    # Use IMU as time reference.
    #
    # Both IMUs are stored in the same imu.csv.
    # Find the earliest synchronized IMU timestamp.
    # --------------------------------------------------------

    imu_times = [
        int(row["synced_host_monotonic_ns"])
        for row in imu_rows
        if row.get("synced_host_monotonic_ns")
    ]

    if not imu_times:
        raise ValueError(
            "IMU file contains no synced_host_monotonic_ns."
        )

    imu_start_ns = min(imu_times)

    # --------------------------------------------------------
    # Preserve original EMG synchronized timestamp and rebuild
    # the EMG timeline from the IMU start.
    # --------------------------------------------------------

    reconstructed = []

    for index, row in enumerate(emg_rows):

        new_row = dict(row)

        original_time = int(
            row["synced_host_monotonic_ns"]
        )

        corrected_time = (
            imu_start_ns
            + index * EMG_PERIOD_NS
        )

        new_row[
            "original_synced_host_monotonic_ns"
        ] = original_time

        new_row[
            "synced_host_monotonic_ns"
        ] = corrected_time

        reconstructed.append(new_row)

    return reconstructed


# ============================================================
# Trial handling
# ============================================================

def reconstruct_trial(ble_dir: Path) -> None:

    emg_path = ble_dir / "emg.csv"
    imu_path = ble_dir / "imu.csv"

    if not emg_path.exists():
        print(f"Missing EMG: {ble_dir}")
        return

    if not imu_path.exists():
        print(f"Missing IMU: {ble_dir}")
        return

    print()
    print(
        f"Reconstructing: "
        f"{ble_dir.parent.parent.name} / "
        f"{ble_dir.parent.name}"
    )

    emg_rows = load_csv(emg_path)
    imu_rows = load_csv(imu_path)

    reconstructed_emg = reconstruct_emg_timestamps(
        emg_rows,
        imu_rows,
    )

    relative_path = ble_dir.relative_to(
        RECORDINGS_DIR
    )

    output_dir = (
        RECONSTRUCTED_DIR
        / relative_path
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    # Save reconstructed EMG
    save_csv(
        output_dir / "emg.csv",
        reconstructed_emg,
    )

    # IMU is already trusted, so copy unchanged
    shutil.copy2(
        imu_path,
        output_dir / "imu.csv",
    )

    # --------------------------------------------------------
    # Diagnostics
    # --------------------------------------------------------

    original_start = int(
        emg_rows[0]["synced_host_monotonic_ns"]
    )

    corrected_start = int(
        reconstructed_emg[0][
            "synced_host_monotonic_ns"
        ]
    )

    original_end = int(
        emg_rows[-1]["synced_host_monotonic_ns"]
    )

    corrected_end = int(
        reconstructed_emg[-1][
            "synced_host_monotonic_ns"
        ]
    )

    start_shift_s = (
        corrected_start - original_start
    ) / 1e9

    end_shift_s = (
        corrected_end - original_end
    ) / 1e9

    duration_s = (
        corrected_end - corrected_start
    ) / 1e9

    print(
        f"EMG samples: {len(emg_rows)}"
    )

    print(
        f"Reconstructed duration: "
        f"{duration_s:.6f} s"
    )

    print(
        f"Start correction: "
        f"{start_shift_s:+.6f} s"
    )

    print(
        f"End correction: "
        f"{end_shift_s:+.6f} s"
    )

    print(
        f"Saved to: {output_dir}"
    )


# ============================================================
# Discovery
# ============================================================

def find_trials() -> list[Path]:

    trials = []

    if not RECORDINGS_DIR.exists():
        return trials

    for emg_file in RECORDINGS_DIR.rglob(
        "ble/emg.csv"
    ):

        ble_dir = emg_file.parent

        if (ble_dir / "imu.csv").exists():
            trials.append(ble_dir)

    return sorted(trials)

def group_trials_by_subject(
    trials: list[Path],
) -> dict[str, list[Path]]:

    subjects: dict[str, list[Path]] = {}

    for trial in trials:
        subject = trial.parent.parent.name

        subjects.setdefault(
            subject,
            [],
        ).append(trial)

    return subjects

def find_reconstructed_trials() -> list[Path]:

    trials = []

    if not RECONSTRUCTED_DIR.exists():
        return trials

    for emg_file in RECONSTRUCTED_DIR.rglob(
        "ble/emg.csv"
    ):
        ble_dir = emg_file.parent

        if (ble_dir / "imu.csv").exists():
            trials.append(ble_dir)

    return sorted(trials)

def plot_reconstructed_trial(
    ble_dir: Path,
) -> None:

    emg_path = ble_dir / "emg.csv"
    imu_path = ble_dir / "imu.csv"

    emg_rows = load_csv(emg_path)
    imu_rows = load_csv(imu_path)

    if not emg_rows or not imu_rows:
        print("Missing reconstructed data.")
        return

    # --------------------------------------------------------
    # Common zero-time
    # --------------------------------------------------------

    start_ns = min(
        int(emg_rows[0]["synced_host_monotonic_ns"]),
        min(
            int(row["synced_host_monotonic_ns"])
            for row in imu_rows
        ),
    )

    # --------------------------------------------------------
    # EMG
    # --------------------------------------------------------

    emg_time = [
        (
            int(row["synced_host_monotonic_ns"])
            - start_ns
        ) / 1e9
        for row in emg_rows
    ]

    emg_value = [
        float(row["value"])
        for row in emg_rows
    ]

    # --------------------------------------------------------
    # IMU split
    # --------------------------------------------------------

    imu1 = [
        row
        for row in imu_rows
        if row["sensor"] == "0"
    ]

    imu2 = [
        row
        for row in imu_rows
        if row["sensor"] == "1"
    ]

    def imu_time(rows):
        return [
            (
                int(row["synced_host_monotonic_ns"])
                - start_ns
            ) / 1e9
            for row in rows
        ]

    imu1_time = imu_time(imu1)
    imu2_time = imu_time(imu2)

    # --------------------------------------------------------
    # Plot
    # --------------------------------------------------------

    fig, axes = plt.subplots(
        3,
        1,
        sharex=True,
        figsize=(12, 8),
    )

    # EMG
    axes[0].plot(
        emg_time,
        emg_value,
    )

    axes[0].set_ylabel("EMG")
    axes[0].set_title(
        f"{ble_dir.parent.parent.name} / "
        f"{ble_dir.parent.name}"
    )
    axes[0].grid(True)

    # IMU1 gyro magnitude-ish overview
    axes[1].plot(
        imu1_time,
        [float(row["gyro_x"]) for row in imu1],
        label="gyro_x",
    )

    axes[1].plot(
        imu1_time,
        [float(row["gyro_y"]) for row in imu1],
        label="gyro_y",
    )

    axes[1].plot(
        imu1_time,
        [float(row["gyro_z"]) for row in imu1],
        label="gyro_z",
    )

    axes[1].set_ylabel("IMU1 [deg/s]")
    axes[1].legend()
    axes[1].grid(True)

    # IMU2
    axes[2].plot(
        imu2_time,
        [float(row["gyro_x"]) for row in imu2],
        label="gyro_x",
    )

    axes[2].plot(
        imu2_time,
        [float(row["gyro_y"]) for row in imu2],
        label="gyro_y",
    )

    axes[2].plot(
        imu2_time,
        [float(row["gyro_z"]) for row in imu2],
        label="gyro_z",
    )

    axes[2].set_ylabel("IMU2 [deg/s]")
    axes[2].set_xlabel("Time [s]")
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plt.show()

# ============================================================
# Main
# ============================================================

def main() -> int:

    trials = find_trials()

    if not trials:
        print("No recordings found.")
        return 1

    subjects = group_trials_by_subject(trials)
    subject_names = sorted(subjects.keys())

    print()
    print("==============================")
    print(" AURORA TIMESTAMP RECONSTRUCTION")
    print("==============================")
    print()

    print(f"Found {len(subject_names)} subjects.")
    print()

    print("1. Reconstruct subject")
    print("2. Plot reconstructed trial")
    print("3. Reconstruct all subjects")
    print("Q. Quit")
    print()

    choice = input(
        "Select option: "
    ).strip().lower()

    # ========================================================
    # Quit
    # ========================================================

    if choice == "q":
        return 0

    # ========================================================
    # 1. Reconstruct one subject
    # ========================================================

    if choice == "1":

        print()
        print("Subjects:")
        print()

        for index, subject in enumerate(
            subject_names,
            start=1,
        ):
            print(
                f"{index}. {subject} "
                f"({len(subjects[subject])} trials)"
            )

        print()

        try:
            subject_index = (
                int(input("Select subject: ")) - 1
            )

            if (
                subject_index < 0
                or subject_index >= len(subject_names)
            ):
                raise ValueError

        except ValueError:
            print("Invalid subject.")
            return 1

        subject = subject_names[
            subject_index
        ]

        print()
        print(
            f"Reconstructing all trials "
            f"for {subject}"
        )

        print(
            f"Trials: "
            f"{len(subjects[subject])}"
        )

        for trial in subjects[subject]:
            reconstruct_trial(trial)

        print()
        print(
            f"{subject} reconstruction complete."
        )

        return 0

    # ========================================================
    # 2. Plot reconstructed trial
    # ========================================================

    if choice == "2":

        reconstructed_trials = (
            find_reconstructed_trials()
        )

        if not reconstructed_trials:
            print(
                "No reconstructed trials found."
            )
            return 0

        reconstructed_subjects = (
            group_trials_by_subject(
                reconstructed_trials
            )
        )

        reconstructed_subject_names = sorted(
            reconstructed_subjects.keys()
        )

        print()
        print("Reconstructed subjects:")
        print()

        for index, subject in enumerate(
            reconstructed_subject_names,
            start=1,
        ):
            print(
                f"{index}. {subject} "
                f"("
                f"{len(reconstructed_subjects[subject])} "
                f"trials)"
            )

        print()

        try:
            subject_index = (
                int(input("Select subject: ")) - 1
            )

            if (
                subject_index < 0
                or subject_index
                >= len(reconstructed_subject_names)
            ):
                raise ValueError

        except ValueError:
            print("Invalid subject.")
            return 1

        subject = reconstructed_subject_names[
            subject_index
        ]

        subject_trials = reconstructed_subjects[
            subject
        ]

        print()
        print(f"{subject} trials:")
        print()

        for index, trial in enumerate(
            subject_trials,
            start=1,
        ):
            print(
                f"{index}. {trial.parent.name}"
            )

        print()

        try:
            trial_index = (
                int(input("Select trial: ")) - 1
            )

            if (
                trial_index < 0
                or trial_index >= len(subject_trials)
            ):
                raise ValueError

        except ValueError:
            print("Invalid trial.")
            return 1

        trial = subject_trials[
            trial_index
        ]

        plot_reconstructed_trial(
            trial
        )

        return 0

    # ========================================================
    # 3. Reconstruct all subjects
    # ========================================================

    if choice == "3":

        for subject in subject_names:

            print()
            print(
                f"===== {subject} ====="
            )

            for trial in subjects[subject]:
                reconstruct_trial(trial)

        print()
        print(
            "All subjects reconstructed."
        )

        return 0

    # ========================================================
    # Invalid option
    # ========================================================

    print("Invalid selection.")
    return 1

if __name__ == "__main__":
    raise SystemExit(main())