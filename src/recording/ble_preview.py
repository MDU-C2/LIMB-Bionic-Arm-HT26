"""Live, file-free display of decoded LIMB BLE notifications."""

from __future__ import annotations

from collections import deque
import asyncio
import math
import time
import tkinter as tk


COLORS = ("#2563eb", "#d97706", "#159669")


def _draw_traces(canvas: tk.Canvas, traces: list[list[float]]) -> None:
    """Draw recent samples with a common auto-scaled vertical axis."""
    canvas.delete("all")
    width = max(canvas.winfo_width(), 2)
    height = max(canvas.winfo_height(), 2)
    canvas.create_line(0, height / 2, width, height / 2, fill="#d6dce5")
    values = [value for trace in traces for value in trace if math.isfinite(value)]
    if not values:
        canvas.create_text(12, height / 2, text="Waiting for samples...", anchor="w")
        return
    low, high = min(values), max(values)
    if high - low < 1e-9:
        low -= 1.0
        high += 1.0
    span = high - low
    for index, trace in enumerate(traces):
        if len(trace) < 2:
            continue
        points: list[float] = []
        for sample, value in enumerate(trace):
            points.extend(
                (10 + sample * (width - 20) / (len(trace) - 1),
                 8 + (high - value) * (height - 16) / span)
            )
        canvas.create_line(*points, fill=COLORS[index % len(COLORS)], width=2)


async def run_ble_preview(args, recorder_type, sensor_uuids: dict[str, str]) -> int:
    """Connect, display live EMG and IMU values, and save nothing."""
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print("bleak is missing. Recreate the simulation environment.")
        return 2

    if args.scan_timeout <= 0:
        print("Scan timeout must be positive.")
        return 2

    try:
        root = tk.Tk()
    except tk.TclError as error:
        print(f"Could not open BLE preview window: {error}")
        return 2
    root.title("AURORA live BLE sensors - NOT RECORDING")
    root.geometry("800x560")
    closing = False

    def close() -> None:
        nonlocal closing
        closing = True

    root.protocol("WM_DELETE_WINDOW", close)
    root.bind("<Escape>", lambda _event: close())
    frame = tk.Frame(root, padx=14, pady=12)
    frame.pack(fill="both", expand=True)
    status = tk.StringVar(value=f"Scanning for {args.device}...")
    tk.Label(
        frame, textvariable=status, font=("Segoe UI", 11, "bold"),
        anchor="w", justify="left", wraplength=760,
    ).pack(fill="x")
    tk.Label(
        frame,
        text="Live preview only. Closing this window stops the connection; no files are saved.",
        anchor="w",
    ).pack(fill="x", pady=(0, 10))
    tk.Label(frame, text="EMG raw ADC samples (last 400 per channel)", anchor="w").pack(fill="x")
    emg_canvas = tk.Canvas(frame, height=135, bg="white", highlightbackground="#ccd3df", highlightthickness=1)
    emg_canvas.pack(fill="x", pady=(3, 3))
    emg_text = tk.StringVar(value="EMG: waiting for packets")
    tk.Label(frame, textvariable=emg_text, anchor="w", font=("Consolas", 10)).pack(fill="x", pady=(0, 10))
    tk.Label(frame, text="IMU acceleration magnitude (recent packets)", anchor="w").pack(fill="x")
    imu_canvas = tk.Canvas(frame, height=105, bg="white", highlightbackground="#ccd3df", highlightthickness=1)
    imu_canvas.pack(fill="x", pady=(3, 3))
    imu_text = tk.StringVar(value="IMU: waiting for packets")
    tk.Label(frame, textvariable=imu_text, anchor="w", justify="left", font=("Consolas", 10)).pack(fill="x", pady=(0, 10))
    piezo_text = tk.StringVar(value="Piezo: waiting for packets")
    tk.Label(frame, textvariable=piezo_text, anchor="w", font=("Consolas", 10)).pack(fill="x")
    tk.Button(frame, text="Close preview", command=close).pack(anchor="e", pady=(10, 0))

    emg = [deque(maxlen=400) for _ in range(2)]
    imu = [deque(maxlen=150) for _ in range(2)]
    latest_imu: dict[int, list[float]] = {}
    latest_piezo: float | None = None
    last_packet = 0.0
    last_sequences: dict[str, int] = {}
    sequence_gaps = {name: 0 for name in sensor_uuids}
    rates = {name: 0.0 for name in sensor_uuids}
    previous_counts = {name: 0 for name in sensor_uuids}
    rate_started = time.monotonic()

    def accept(sensor: str, sequence: int, channels: list[list[object]]) -> None:
        nonlocal latest_piezo, last_packet
        last_packet = time.monotonic()
        previous_sequence = last_sequences.get(sensor)
        if previous_sequence is not None and sequence != (previous_sequence + 1) & 0xFFFFFFFF:
            sequence_gaps[sensor] += 1
        last_sequences[sensor] = sequence
        if sensor == "emg":
            for channel, values in enumerate(channels[:2]):
                emg[channel].extend(float(value) for value in values)
        elif sensor == "imu":
            for channel, samples in enumerate(channels[:2]):
                if samples:
                    values = [float(value) for value in samples[-1]]
                    latest_imu[channel] = values
                    imu[channel].append(math.sqrt(sum(value * value for value in values[:3])))
        elif sensor == "piezo" and channels and channels[0]:
            latest_piezo = sum(float(value) for value in channels[0]) / len(channels[0])

    recorder = recorder_type(None, accept)

    def refresh() -> None:
        try:
            root.update_idletasks()
            root.update()
        except tk.TclError:
            close()

    def draw() -> None:
        nonlocal rate_started
        now = time.monotonic()
        elapsed = now - rate_started
        if elapsed >= 1.0:
            for name in sensor_uuids:
                rates[name] = (recorder.counts[name] - previous_counts[name]) / elapsed
                previous_counts[name] = recorder.counts[name]
            rate_started = now
        _draw_traces(emg_canvas, [list(channel) for channel in emg if channel])
        rms = []
        for index, channel in enumerate(emg):
            if not channel:
                continue
            mean = sum(channel) / len(channel)
            centered_rms = math.sqrt(sum((value - mean) ** 2 for value in channel) / len(channel))
            rms.append(f"ch{index + 1} centered RMS {centered_rms:.1f} ADC")
        emg_text.set("  |  ".join(rms) if rms else "EMG: waiting for packets")
        _draw_traces(imu_canvas, [list(channel) for channel in imu if channel])
        readings = []
        for index, values in sorted(latest_imu.items()):
            if len(values) >= 6:
                readings.append(
                    f"IMU {index + 1}: a=({values[0]:.2f}, {values[1]:.2f}, {values[2]:.2f}) "
                    f"g=({values[3]:.2f}, {values[4]:.2f}, {values[5]:.2f})"
                )
        imu_text.set("\n".join(readings) if readings else "IMU: waiting for packets")
        piezo_text.set(
            f"Piezo mean: {latest_piezo:.1f} ADC" if latest_piezo is not None
            else "Piezo: waiting for packets"
        )
        summary = ", ".join(
            f"{key} {count} ({rates[key]:.0f}/s, gaps {sequence_gaps[key]})"
            for key, count in recorder.counts.items()
        )
        stale = " (no recent packets)" if last_packet and time.monotonic() - last_packet > 2 else ""
        status.set(f"Connected: {summary}{stale}")

    try:
        scan = None if args.address else asyncio.create_task(
            BleakScanner.find_device_by_name(args.device, timeout=args.scan_timeout)
        )
        while scan is not None and not scan.done() and not closing:
            refresh()
            await asyncio.sleep(0.05)
        if closing:
            if scan is not None:
                scan.cancel()
            return 0
        device = args.address or scan.result()
        if not device:
            print(f"BLE device {args.device!r} was not found.")
            status.set("Device not found")
            refresh()
            return 1
        async with BleakClient(device) as client:
            subscribed = []
            for sensor, uuid in sensor_uuids.items():
                try:
                    await client.start_notify(
                        uuid,
                        lambda _sender, data, name=sensor: recorder.handle(name, data),
                    )
                    subscribed.append(sensor)
                except Exception as error:
                    print(f"{sensor.upper()} unavailable: {error}")
            if not subscribed:
                print("No known LIMB sensor characteristics were found.")
                return 1
            print("BLE preview connected. No session folder or recordings will be created.")
            next_draw = 0.0
            while not closing and client.is_connected:
                refresh()
                if time.monotonic() >= next_draw:
                    draw()
                    next_draw = time.monotonic() + 0.12
                await asyncio.sleep(0.03)
            return 0
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print(f"BLE preview failed: {error}")
        return 1
    finally:
        recorder.close()
        try:
            root.destroy()
        except tk.TclError:
            pass
