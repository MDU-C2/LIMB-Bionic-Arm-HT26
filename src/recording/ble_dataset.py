"""Build LIMB-style labeled windows from decoded BLE notifications."""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
import re
import time

from common import safe_label, utc_now


PACKETS_PER_WINDOW = 10
WINDOW_COUNT = 80
REST_WINDOWS = 20
REST_LABEL = "2"
SAMPLES_PER_WINDOW = {"emg": 400, "imu": 10, "piezo": 100}
IMU_COLUMNS = (
    "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z",
    "temperature", "pitch", "roll",
)


@dataclass
class WindowState:
    expected_sequence: int | None = None
    values: list[object] = field(default_factory=list)


class LabeledBleCapture:
    """Collect 20 rest, 40 movement, and 20 rest windows for one subject."""

    @staticmethod
    def validate_label(value: str) -> str:
        """Allow short movement labels that also work as CSV filenames."""
        label = value.strip()
        if label and re.fullmatch(r"[A-Za-z0-9_-]{1,32}", label) is None:
            raise ValueError("Movement label must use 1-32 letters, digits, _ or -.")
        return label

    def __init__(self, session: Path, label: str, subject: str) -> None:
        self.session = session
        self.label = self.validate_label(label)
        if not self.label:
            raise ValueError("A movement label is required for dataset capture.")
        self.subject = safe_label(subject)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.capture_id = f"{self.label}_{timestamp}"
        self.states: dict[tuple[str, int], WindowState] = {}
        self.windows: dict[str, dict[int, list[tuple[str, list[object]]]]] = {
            name: {} for name in SAMPLES_PER_WINDOW
        }
        self.sequence_gaps = {name: 0 for name in SAMPLES_PER_WINDOW}
        self.invalid_windows = {name: 0 for name in SAMPLES_PER_WINDOW}
        self.last_sequences: dict[str, int] = {}
        self.started = False
        self.last_progress = time.monotonic()
        self.saved_files: list[str] = []

    def start(self) -> None:
        """Discard countdown packets and begin at the next packet boundary."""
        self.states.clear()
        self.last_sequences.clear()
        for channels in self.windows.values():
            channels.clear()
        self.last_progress = time.monotonic()
        self.started = True
        print("[CAPTURE] REST now (first 20 windows, about 2 seconds).")

    @property
    def emg_complete(self) -> bool:
        return len(self.windows["emg"].get(0, [])) >= WINDOW_COUNT

    @property
    def complete(self) -> bool:
        channels_complete = all(
            self.windows[sensor]
            and all(len(data) >= WINDOW_COUNT for data in self.windows[sensor].values())
            for sensor in ("emg", "imu")
        )
        return (
            channels_complete
            and self.sequence_gaps["emg"] == 0
            and self.sequence_gaps["imu"] == 0
            and self.invalid_windows["emg"] == 0
            and self.invalid_windows["imu"] == 0
        )

    @property
    def seconds_since_progress(self) -> float:
        return time.monotonic() - self.last_progress

    def add_packet(self, sensor: str, sequence: int, channels: list[list[object]]) -> None:
        """Accept decoded samples only from complete ten-packet windows."""
        if not self.started or sensor not in SAMPLES_PER_WINDOW:
            return
        previous = self.last_sequences.get(sensor)
        if previous is not None and sequence != previous + 1:
            self.sequence_gaps[sensor] += 1
            for key in tuple(self.states):
                if key[0] == sensor:
                    self.states[key] = WindowState()
        self.last_sequences[sensor] = sequence
        packet_index = sequence % PACKETS_PER_WINDOW
        for channel, values in enumerate(channels):
            saved = self.windows[sensor].setdefault(channel, [])
            if len(saved) >= WINDOW_COUNT:
                continue
            key = (sensor, channel)
            state = self.states.setdefault(key, WindowState())
            if packet_index == 0:
                state.values = list(values)
                state.expected_sequence = sequence + 1
            elif state.expected_sequence == sequence:
                state.values.extend(values)
                state.expected_sequence += 1
            else:
                if state.expected_sequence is not None:
                    self.sequence_gaps[sensor] += 1
                state.values.clear()
                state.expected_sequence = None
                continue

            if packet_index != PACKETS_PER_WINDOW - 1:
                continue
            if len(state.values) == SAMPLES_PER_WINDOW[sensor]:
                saved.append((utc_now(), state.values.copy()))
                if sensor == "emg" and channel == 0:
                    self.last_progress = time.monotonic()
                    self._report_progress(len(saved))
            else:
                self.invalid_windows[sensor] += 1
            state.values.clear()
            state.expected_sequence = None

    def _report_progress(self, count: int) -> None:
        if count == REST_WINDOWS:
            print(f"[CAPTURE] MOVEMENT now: label {self.label} (next 40 windows).")
        elif count == WINDOW_COUNT - REST_WINDOWS:
            print("[CAPTURE] REST now (last 20 windows).")
        elif count == WINDOW_COUNT or count % 10 == 0:
            print(f"[CAPTURE] {count}/{WINDOW_COUNT} EMG windows complete.")

    def save(self) -> None:
        """Save old-style raw CSVs and segment complete EMG captures."""
        raw_root = self.session / "raw_data" / self.subject
        for sensor, channels in self.windows.items():
            for channel, windows in channels.items():
                if not windows:
                    continue
                channel_suffix = "" if channel == 0 else f"_ch{channel + 1}"
                name = self.capture_id + channel_suffix
                directory = raw_root / sensor.upper()
                directory.mkdir(parents=True, exist_ok=True)
                path = directory / f"{name}.csv"
                if sensor == "imu":
                    self._write_imu(path, windows)
                else:
                    self._write_wide(
                        path, windows, "Raw_Label", self.capture_id,
                        SAMPLES_PER_WINDOW[sensor],
                    )
                self.saved_files.append(str(path.relative_to(self.session)))

        if self.complete:
            for channel, windows in self.windows["emg"].items():
                self._write_segments(channel, windows)
        elif self.emg_complete:
            print("[CAPTURE] Some packets or IMU windows are missing; no labeled segments were saved.")

    @staticmethod
    def _write_wide(
        path: Path,
        windows: list[tuple[str, list[object]]],
        label_column: str,
        label: str,
        value_count: int,
    ) -> None:
        with path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow((label_column, "Timestamp", *(f"v{i}" for i in range(value_count))))
            for timestamp, values in windows:
                writer.writerow((label, timestamp, *values))

    def _write_imu(self, path: Path, windows: list[tuple[str, list[object]]]) -> None:
        with path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(("Raw_Label", "Timestamp", *IMU_COLUMNS))
            for timestamp, samples in windows:
                for sample in samples:
                    values = list(sample) if isinstance(sample, (tuple, list)) else [sample]
                    writer.writerow(
                        (self.capture_id, timestamp, *(values + [""] * 9)[:9])
                    )

    def _write_segments(self, channel: int, windows: list[tuple[str, list[object]]]) -> None:
        directory = self.session / "labeled_data" / self.subject / "segmented_emg"
        directory.mkdir(parents=True, exist_ok=True)
        timestamp = self.capture_id[len(self.label) + 1 :]
        channel_suffix = "" if channel == 0 else f"_ch{channel + 1}"
        segments = (
            (REST_LABEL, windows[:REST_WINDOWS], f"{REST_LABEL}_{timestamp}_i{channel_suffix}.csv"),
            (
                self.label,
                windows[REST_WINDOWS : WINDOW_COUNT - REST_WINDOWS],
                f"{self.label}_{timestamp}{channel_suffix}.csv",
            ),
            (REST_LABEL, windows[WINDOW_COUNT - REST_WINDOWS : WINDOW_COUNT],
             f"{REST_LABEL}_{timestamp}_t{channel_suffix}.csv"),
        )
        for label, data, filename in segments:
            path = directory / filename
            self._write_wide(path, data, "Label", label, SAMPLES_PER_WINDOW["emg"])
            self.saved_files.append(str(path.relative_to(self.session)))

    def summary(self) -> dict[str, object]:
        return {
            "capture_id": self.capture_id,
            "movement_label": self.label,
            "rest_label": REST_LABEL,
            "window_count": WINDOW_COUNT,
            "rest_windows_each_end": REST_WINDOWS,
            "complete": self.complete,
            "windows_by_sensor": {
                sensor: {str(channel): len(data) for channel, data in channels.items()}
                for sensor, channels in self.windows.items()
            },
            "sequence_gaps": self.sequence_gaps,
            "invalid_windows": self.invalid_windows,
            "files": self.saved_files,
        }
