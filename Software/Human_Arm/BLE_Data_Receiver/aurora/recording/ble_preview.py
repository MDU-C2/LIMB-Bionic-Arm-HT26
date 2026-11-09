"""Simple real-time preview of decoded AURORA BLE sensor data."""

from __future__ import annotations

import asyncio
import threading
import time
from collections import deque

import matplotlib.pyplot as plt

from aurora.sensors.ble_discovery import find_device


async def run_ble_preview(
    args,
    recorder_type,
    sensor_uuids: dict[str, str],
) -> int:
    """
    Connect to AURORA over BLE and display raw EMG and IMU data.

    BLE acquisition runs in a background thread.
    Matplotlib runs on the main thread.

    No files are saved.
    """

    # =========================================================
    # Shared state
    # =========================================================

    data_lock = threading.Lock()
    stop_event = threading.Event()
    connected_event = threading.Event()

    # Latest EMG samples.
    emg = deque(maxlen=2000)

    # Two IMUs:
    # [IMU][axis][samples]
    imu_accel = [
        [deque(maxlen=300) for _ in range(3)]
        for _ in range(2)
    ]

    imu_gyro = [
        [deque(maxlen=300) for _ in range(3)]
        for _ in range(2)
    ]

    latest_imu: dict[int, list[float]] = {}

    last_sequences: dict[str, int] = {}

    sequence_gaps = {
        name: 0
        for name in sensor_uuids
        if name in ("emg", "imu")
    }

    packet_counts = {
        name: 0
        for name in sensor_uuids
        if name in ("emg", "imu")
    }

    status = {
        "connected": False,
        "error": None,
        "device": None,
    }

    # =========================================================
    # Decoded packet callback
    # =========================================================

    def accept(
        sensor: str,
        sequence: int,
        channels: list[list[object]],
    ) -> None:
        """
        Receive already-decoded sensor packets from BleRecorder.
        """

        if sensor not in ("emg", "imu"):
            return

        with data_lock:

            # -----------------------------------------------
            # Sequence / packet loss check
            # -----------------------------------------------

            previous_sequence = last_sequences.get(sensor)

            if previous_sequence is not None:

                distance = (
                    sequence - previous_sequence
                ) & 0xFFFFFFFF

                if distance != 1:
                    sequence_gaps[sensor] += 1

                # Ignore duplicate or old packets.
                if (
                    distance == 0
                    or distance >= 0x80000000
                ):
                    return

            last_sequences[sensor] = sequence
            packet_counts[sensor] += 1

            # -----------------------------------------------
            # EMG
            # -----------------------------------------------

            if sensor == "emg":

                if channels and channels[0]:

                    for value in channels[0]:
                        emg.append(float(value))

            # -----------------------------------------------
            # IMU
            # -----------------------------------------------

            elif sensor == "imu":

                for imu_index in range(2):

                    if imu_index >= len(channels):
                        continue

                    samples = channels[imu_index]

                    if not samples:
                        continue

                    # Use latest sample from this IMU packet.
                    values = [
                        float(value)
                        for value in samples[-1]
                    ]

                    if len(values) < 6:
                        continue

                    latest_imu[imu_index] = values

                    # Accelerometer XYZ
                    for axis in range(3):

                        imu_accel[imu_index][axis].append(
                            values[axis]
                        )

                    # Gyroscope XYZ
                    for axis in range(3):

                        imu_gyro[imu_index][axis].append(
                            values[axis + 3]
                        )

    # =========================================================
    # BLE worker
    # =========================================================

    async def ble_worker() -> None:

        recorder = None

        try:

            from bleak import BleakClient, BleakScanner

            recorder = recorder_type(
                None,
                accept,
            )

            print()
            print(
                f"Scanning for {args.device!r}..."
            )

            # -----------------------------------------------
            # Find AURORA
            # -----------------------------------------------

            device = (
                args.address
                if args.address
                else await find_device(
                    BleakScanner,
                    args.device,
                    timeout=args.scan_timeout,
                )
            )

            if not device:

                status["error"] = (
                    f"BLE device "
                    f"{args.device!r} was not found."
                )

                print(status["error"])

                stop_event.set()
                return

            status["device"] = device

            print(
                f"Found {args.device}"
            )

            # -----------------------------------------------
            # Connect
            # -----------------------------------------------

            async with BleakClient(device) as client:

                print(
                    "Connected to "
                    f"{getattr(device, 'address', device)}"
                )

                subscribed: list[str] = []

                # -------------------------------------------
                # Subscribe to sensor notifications
                # -------------------------------------------

                for sensor, uuid in sensor_uuids.items():

                    if sensor not in ("emg", "imu"):
                        continue

                    try:

                        await client.start_notify(
                            uuid,
                            lambda _sender,
                            data,
                            name=sensor:
                            recorder.handle(
                                name,
                                data,
                            ),
                        )

                        subscribed.append(sensor)

                        print(
                            f"Subscribed to "
                            f"{sensor.upper()}"
                        )

                    except Exception as error:

                        print(
                            f"{sensor.upper()} "
                            f"unavailable: {error}"
                        )

                if not subscribed:

                    status["error"] = (
                        "No known AURORA sensor "
                        "characteristics were found."
                    )

                    print(status["error"])

                    stop_event.set()
                    return

                status["connected"] = True
                connected_event.set()

                print()
                print(
                    "BLE acquisition running."
                )
                print(
                    "Close the plot window "
                    "or press Ctrl+C to stop."
                )
                print()

                # -------------------------------------------
                # Keep BLE event loop alive
                # -------------------------------------------

                while (
                    client.is_connected
                    and not stop_event.is_set()
                ):

                    await asyncio.sleep(0.05)

                status["connected"] = False

        except Exception as error:

            status["error"] = str(error)

            print(
                f"BLE preview failed: {error}"
            )

            stop_event.set()

        finally:

            if recorder is not None:
                recorder.close()

            connected_event.set()

    # =========================================================
    # BLE thread entry point
    # =========================================================

    def ble_thread_main() -> None:

        asyncio.run(
            ble_worker()
        )

    # =========================================================
    # Start BLE worker thread
    # =========================================================

    ble_thread = threading.Thread(
        target=ble_thread_main,
        daemon=True,
        name="AURORA-BLE",
    )

    ble_thread.start()

    # =========================================================
    # Matplotlib setup
    # =========================================================

    plt.ion()

    fig, axes = plt.subplots(
        5,
        1,
        figsize=(12, 10),
    )

    fig.suptitle(
        "AURORA BLE Sensor Preview"
    )

    # ---------------------------------------------------------
    # EMG
    # ---------------------------------------------------------

    emg_line, = axes[0].plot(
        [],
        [],
    )

    axes[0].set_title("EMG - Raw ADC (Unfiltered)")

    axes[0].set_ylabel("ADC")

    # ---------------------------------------------------------
    # IMU 1 acceleration
    # ---------------------------------------------------------

    imu1_accel_lines = [
        axes[1].plot(
            [],
            [],
            label=axis,
        )[0]
        for axis in ("X", "Y", "Z")
    ]

    axes[1].set_title("IMU 1 - Accelerometer (Unfiltered)")
    axes[1].legend()

    # ---------------------------------------------------------
    # IMU 1 gyro
    # ---------------------------------------------------------

    imu1_gyro_lines = [
        axes[2].plot(
            [],
            [],
            label=axis,
        )[0]
        for axis in ("X", "Y", "Z")
    ]

    axes[2].set_title("IMU 1 - Gyroscope (Unfiltered)")
    axes[2].legend()

    # ---------------------------------------------------------
    # IMU 2 acceleration
    # ---------------------------------------------------------

    imu2_accel_lines = [
        axes[3].plot(
            [],
            [],
            label=axis,
        )[0]
        for axis in ("X", "Y", "Z")
    ]

    axes[3].set_title("IMU 2 - Accelerometer (Unfiltered)")
    axes[3].legend()

    # ---------------------------------------------------------
    # IMU 2 gyro
    # ---------------------------------------------------------

    imu2_gyro_lines = [
        axes[4].plot(
            [],
            [],
            label=axis,
        )[0]
        for axis in ("X", "Y", "Z")
    ]

    axes[4].set_title("IMU 2 - Gyroscope (Unfiltered)")
    axes[4].set_xlabel("Samples")
    axes[4].legend()

    plt.tight_layout()

    # =========================================================
    # Plot update
    # =========================================================

    def update_plot() -> None:

        with data_lock:

            # Make local copies while holding lock.
            emg_values = list(emg)

            accel_values = [
                [
                    list(axis)
                    for axis in sensor
                ]
                for sensor in imu_accel
            ]

            gyro_values = [
                [
                    list(axis)
                    for axis in sensor
                ]
                for sensor in imu_gyro
            ]

        # -----------------------------------------------------
        # EMG
        # -----------------------------------------------------

        if emg_values:

            x = range(
                len(emg_values)
            )

            emg_line.set_data(
                x,
                emg_values,
            )

            axes[0].relim()
            axes[0].autoscale_view()

        # -----------------------------------------------------
        # IMU plots
        # -----------------------------------------------------

        plot_groups = [
            (
                axes[1],
                imu1_accel_lines,
                accel_values[0],
            ),
            (
                axes[2],
                imu1_gyro_lines,
                gyro_values[0],
            ),
            (
                axes[3],
                imu2_accel_lines,
                accel_values[1],
            ),
            (
                axes[4],
                imu2_gyro_lines,
                gyro_values[1],
            ),
        ]

        for axis_object, lines, buffers in plot_groups:

            for line, values in zip(
                lines,
                buffers,
            ):

                line.set_data(
                    range(len(values)),
                    values,
                )

            axis_object.relim()
            axis_object.autoscale_view()

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    # =========================================================
    # Main preview loop
    # =========================================================

    previous_packet_counts = {
        name: 0
        for name in packet_counts
    }

    previous_status_time = time.monotonic()

    next_plot_time = 0.0

    try:

        while (
            plt.fignum_exists(fig.number)
            and not stop_event.is_set()
        ):

            now = time.monotonic()

            # -------------------------------------------------
            # Plot update: ~10 Hz
            # -------------------------------------------------

            if now >= next_plot_time:

                update_plot()

                plt.pause(
                    0.001
                )

                next_plot_time = (
                    now + 0.1
                )

            # -------------------------------------------------
            # Terminal status: once per second
            # -------------------------------------------------

            elapsed = (
                now - previous_status_time
            )

            if elapsed >= 1.0:

                with data_lock:

                    current_counts = dict(
                        packet_counts
                    )

                    current_gaps = dict(
                        sequence_gaps
                    )

                    current_imu = {
                        key: list(value)
                        for key, value
                        in latest_imu.items()
                    }

                status_parts = []

                for sensor in current_counts:

                    packets_since_last = (
                        current_counts[sensor]
                        - previous_packet_counts[sensor]
                    )

                    rate = (
                        packets_since_last
                        / elapsed
                    )

                    status_parts.append(
                        f"{sensor.upper()}: "
                        f"{rate:.0f} pkt/s "
                        f"| gaps "
                        f"{current_gaps[sensor]}"
                    )

                    previous_packet_counts[sensor] = (
                        current_counts[sensor]
                    )

                if status["connected"]:

                    print(
                        " | ".join(
                            status_parts
                        )
                    )

                    # Latest IMU values.
                    for index in sorted(
                        current_imu
                    ):

                        values = (
                            current_imu[index]
                        )

                        print(
                            f"  IMU{index + 1}: "
                            f"A=("
                            f"{values[0]:.2f}, "
                            f"{values[1]:.2f}, "
                            f"{values[2]:.2f}) "
                            f"G=("
                            f"{values[3]:.2f}, "
                            f"{values[4]:.2f}, "
                            f"{values[5]:.2f})"
                        )

                previous_status_time = now

            time.sleep(
                0.005
            )

    except KeyboardInterrupt:

        print()
        print(
            "Stopping BLE preview..."
        )

    finally:

        stop_event.set()

        plt.close(
            "all"
        )

        ble_thread.join(
            timeout=3.0
        )

    # =========================================================
    # Result
    # =========================================================

    if status["error"]:

        print(
            f"Preview stopped: "
            f"{status['error']}"
        )

        return 1

    return 0