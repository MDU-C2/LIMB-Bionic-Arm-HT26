"""Live, file-free windows for decoded LIMB BLE notifications."""

from __future__ import annotations

from collections import deque
import asyncio
import math
import queue
import sys
import threading
import time
import tkinter as tk


AXIS_COLORS = ("#2563eb", "#d97706", "#159669")
SENSOR_COLORS = ("#7c3aed", "#db2777")
PREVIEW_SENSORS = ("emg", "imu", "piezo")


def imu_tilt_degrees(values: list[float]) -> tuple[float, float] | None:
    """Estimate gravity-referenced pitch and roll from one accelerometer sample."""
    if len(values) < 3 or not all(math.isfinite(value) for value in values[:3]):
        return None
    accel_x, accel_y, accel_z = values[:3]
    if math.sqrt(accel_x**2 + accel_y**2 + accel_z**2) < 1e-9:
        return None
    pitch = math.degrees(math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)))
    roll = math.degrees(math.atan2(accel_y, accel_z))
    return pitch, roll


def relative_angle(value: float, zero: float) -> float:
    """Return the shortest signed angular difference in degrees."""
    return (value - zero + 180.0) % 360.0 - 180.0


def _draw_traces(canvas: tk.Canvas, traces: list[list[float]]) -> None:
    """Draw recent samples with a common auto-scaled vertical axis."""
    canvas.delete("all")
    width = max(canvas.winfo_width(), 2)
    height = max(canvas.winfo_height(), 2)
    canvas.create_line(0, height / 2, width, height / 2, fill="#d6dce5")
    values = [value for trace in traces for value in trace if math.isfinite(value)]
    if not values:
        canvas.create_text(
            12, height / 2, text="Waiting for samples...", anchor="w", fill="#667085"
        )
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
                (
                    10 + sample * (width - 20) / (len(trace) - 1),
                    8 + (high - value) * (height - 16) / span,
                )
            )
        canvas.create_line(
            *points,
            fill=AXIS_COLORS[index % len(AXIS_COLORS)],
            width=2,
        )


def _draw_orientation(
    canvas: tk.Canvas,
    tilts: dict[int, tuple[float, float]],
) -> None:
    """Draw gravity-referenced arm indicators for available IMUs."""
    canvas.delete("all")
    width = max(canvas.winfo_width(), 2)
    height = max(canvas.winfo_height(), 2)
    center_x = width * 0.44
    center_y = height * 0.46
    radius = max(min(width, height) * 0.27, 25)
    canvas.create_oval(
        center_x - radius,
        center_y - radius,
        center_x + radius,
        center_y + radius,
        outline="#d6dce5",
    )
    canvas.create_line(
        center_x - radius, center_y, center_x + radius, center_y, fill="#e5e7eb"
    )
    canvas.create_line(
        center_x, center_y - radius, center_x, center_y + radius, fill="#e5e7eb"
    )
    if not tilts:
        canvas.create_text(
            center_x, center_y, text="Waiting for IMU...", fill="#667085"
        )
        return

    for index, (sensor, (pitch, roll)) in enumerate(sorted(tilts.items())):
        color = SENSOR_COLORS[index % len(SENSOR_COLORS)]
        roll_rad = math.radians(roll)
        delta_x = radius * math.sin(roll_rad)
        delta_y = -radius * math.cos(roll_rad)
        canvas.create_line(
            center_x - delta_x * 0.65,
            center_y - delta_y * 0.65,
            center_x + delta_x,
            center_y + delta_y,
            fill=color,
            width=7,
            arrow="last",
        )
        pitch_y = 18 + (90.0 - max(-90.0, min(90.0, pitch))) / 180.0 * (
            height - 52
        )
        pitch_x = width - 34 - index * 16
        canvas.create_line(
            pitch_x, 18, pitch_x, height - 34, fill="#d6dce5", width=2
        )
        canvas.create_oval(
            pitch_x - 5,
            pitch_y - 5,
            pitch_x + 5,
            pitch_y + 5,
            fill=color,
            outline="",
        )
        canvas.create_text(
            8,
            height - 22 - index * 17,
            text=f"IMU {sensor + 1}: pitch {pitch:+.1f}°  roll {roll:+.1f}°",
            anchor="w",
            fill=color,
            font=("Consolas", 9, "bold"),
        )


async def run_ble_preview(args, recorder_type, sensor_uuids: dict[str, str]) -> int:
    """Serve separate EMG, IMU, and piezo windows over one BLE connection."""
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
    root.withdraw()

    closing = False
    windows: dict[str, tk.Toplevel] = {}
    views: dict[str, dict[str, object]] = {}
    status = tk.StringVar(master=root, value=f"Scanning for {args.device}...")
    command_queue: queue.Queue[str] = queue.Queue()

    emg = [deque(maxlen=400) for _ in range(2)]
    accel = [[deque(maxlen=180) for _axis in range(3)] for _sensor in range(2)]
    gyro = [[deque(maxlen=180) for _axis in range(3)] for _sensor in range(2)]
    piezo = deque(maxlen=300)
    latest_imu: dict[int, list[float]] = {}
    tilt_zero: dict[int, tuple[float, float]] = {}
    latest_piezo: float | None = None
    last_packet = 0.0
    last_sequences: dict[str, int] = {}
    sequence_gaps = {name: 0 for name in sensor_uuids}
    rates = {name: 0.0 for name in sensor_uuids}
    previous_counts = {name: 0 for name in sensor_uuids}
    rate_started = time.monotonic()

    def close_window(sensor: str) -> None:
        nonlocal closing
        window = windows.pop(sensor, None)
        views.pop(sensor, None)
        if window is not None:
            try:
                window.destroy()
            except tk.TclError:
                pass
        if not windows:
            closing = True

    def base_window(sensor: str, title: str, geometry: str) -> tuple[tk.Toplevel, tk.Frame]:
        window = tk.Toplevel(root)
        window.title(f"AURORA {title} - NOT RECORDING")
        window.geometry(geometry)
        window.minsize(600, 360)
        window.protocol("WM_DELETE_WINDOW", lambda: close_window(sensor))
        window.bind("<Escape>", lambda _event: close_window(sensor))
        frame = tk.Frame(window, padx=14, pady=12)
        frame.pack(fill="both", expand=True)
        tk.Label(
            frame,
            textvariable=status,
            font=("Segoe UI", 11, "bold"),
            anchor="w",
            justify="left",
            wraplength=900,
        ).pack(fill="x")
        tk.Label(
            frame,
            text="Live preview only. No files are saved.",
            anchor="w",
            fg="#667085",
        ).pack(fill="x", pady=(0, 12))
        windows[sensor] = window
        return window, frame

    def zero_orientation() -> None:
        """Use the current gravity direction as the displayed neutral pose."""
        updated = 0
        for sensor, values in latest_imu.items():
            tilt = imu_tilt_degrees(values)
            if tilt is not None:
                tilt_zero[sensor] = tilt
                updated += 1
        view = views.get("imu", {})
        note = view.get("note")
        if isinstance(note, tk.StringVar):
            note.set(
                f"Zeroed {updated} IMU(s). Twist/yaw still appears in the gyro graph."
                if updated
                else "Cannot zero yet: waiting for a valid IMU sample."
            )

    def create_emg_window() -> None:
        _window, frame = base_window("emg", "EMG signal", "900x470")
        tk.Label(
            frame,
            text="Raw EMG ADC samples (last 400 per channel)",
            anchor="w",
        ).pack(fill="x")
        canvas = tk.Canvas(
            frame,
            height=270,
            bg="white",
            highlightbackground="#ccd3df",
            highlightthickness=1,
        )
        canvas.pack(fill="both", expand=True, pady=(4, 6))
        value = tk.StringVar(master=root, value="EMG: waiting for packets")
        tk.Label(
            frame, textvariable=value, anchor="w", font=("Consolas", 10)
        ).pack(fill="x")
        views["emg"] = {"canvas": canvas, "value": value}

    def create_imu_window() -> None:
        _window, frame = base_window("imu", "IMU motion", "1040x680")
        content = tk.Frame(frame)
        content.pack(fill="both", expand=True)
        charts = tk.Frame(content)
        charts.pack(side="left", fill="both", expand=True)
        orientation = tk.Frame(content, padx=(12, 0))
        orientation.pack(side="right", fill="y")

        tk.Label(
            charts,
            text="Acceleration X / Y / Z (blue / orange / green)",
            anchor="w",
        ).pack(fill="x")
        accel_canvas = tk.Canvas(
            charts,
            height=180,
            bg="white",
            highlightbackground="#ccd3df",
            highlightthickness=1,
        )
        accel_canvas.pack(fill="both", expand=True, pady=(4, 10))
        tk.Label(
            charts,
            text="Gyroscope X / Y / Z - rotate or twist the cuff",
            anchor="w",
        ).pack(fill="x")
        gyro_canvas = tk.Canvas(
            charts,
            height=180,
            bg="white",
            highlightbackground="#ccd3df",
            highlightthickness=1,
        )
        gyro_canvas.pack(fill="both", expand=True, pady=(4, 6))

        tk.Label(orientation, text="Arm tilt preview", anchor="w").pack(fill="x")
        orientation_canvas = tk.Canvas(
            orientation,
            width=310,
            height=390,
            bg="white",
            highlightbackground="#ccd3df",
            highlightthickness=1,
        )
        orientation_canvas.pack(fill="both", expand=True, pady=(4, 6))
        value = tk.StringVar(master=root, value="IMU: waiting for packets")
        tk.Label(
            frame,
            textvariable=value,
            anchor="w",
            justify="left",
            font=("Consolas", 10),
        ).pack(fill="x", pady=(4, 8))
        controls = tk.Frame(frame)
        controls.pack(fill="x")
        note = tk.StringVar(
            master=root,
            value="Pitch/roll use gravity. Twist/yaw appears in the gyro graph; no magnetometer is fitted.",
        )
        tk.Label(
            controls, textvariable=note, anchor="w", fg="#667085"
        ).pack(side="left", fill="x", expand=True)
        tk.Button(controls, text="Zero tilt", command=zero_orientation).pack(side="right")
        views["imu"] = {
            "accel": accel_canvas,
            "gyro": gyro_canvas,
            "orientation": orientation_canvas,
            "value": value,
            "note": note,
        }

    def create_piezo_window() -> None:
        _window, frame = base_window(
            "piezo", "piezo contact and vibration", "900x470"
        )
        tk.Label(
            frame,
            text="Raw piezoelectric sensor response (recent ADC samples)",
            anchor="w",
        ).pack(fill="x")
        canvas = tk.Canvas(
            frame,
            height=250,
            bg="white",
            highlightbackground="#ccd3df",
            highlightthickness=1,
        )
        canvas.pack(fill="both", expand=True, pady=(4, 6))
        value = tk.StringVar(master=root, value="Piezo: waiting for packets")
        tk.Label(
            frame, textvariable=value, anchor="w", font=("Consolas", 10)
        ).pack(fill="x")
        tk.Label(
            frame,
            text=(
                "A piezoelectric sensor produces a signal when it is bent, tapped, or vibrated. "
                "Use this graph to check contact and changes, not as a calibrated force measurement."
            ),
            anchor="w",
            justify="left",
            wraplength=850,
            fg="#667085",
        ).pack(fill="x", pady=(8, 0))
        views["piezo"] = {"canvas": canvas, "value": value}

    def show_window(sensor: str) -> None:
        if sensor == "all":
            for name in PREVIEW_SENSORS:
                show_window(name)
            return
        if sensor not in PREVIEW_SENSORS:
            return
        window = windows.get(sensor)
        if window is None:
            {
                "emg": create_emg_window,
                "imu": create_imu_window,
                "piezo": create_piezo_window,
            }[sensor]()
            window = windows[sensor]
        window.deiconify()
        window.lift()
        window.focus_force()

    def read_commands() -> None:
        """Receive requests from the GUI without touching Tk from this thread."""
        for line in sys.stdin:
            command_queue.put(line.strip())

    def drain_commands() -> None:
        try:
            while True:
                command = command_queue.get_nowait().split()
                if len(command) == 2 and command[0].lower() == "show":
                    show_window(command[1].lower())
        except queue.Empty:
            pass

    show_window(getattr(args, "preview_sensor", "all"))
    threading.Thread(target=read_commands, daemon=True).start()

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
                    if len(values) >= 6:
                        for axis in range(3):
                            accel[channel][axis].append(values[axis])
                            gyro[channel][axis].append(values[axis + 3])
        elif sensor == "piezo" and channels and channels[0]:
            samples = [float(value) for value in channels[0]]
            piezo.extend(samples)
            latest_piezo = sum(samples) / len(samples)

    recorder = recorder_type(None, accept)

    def refresh() -> None:
        nonlocal closing
        try:
            drain_commands()
            root.update_idletasks()
            root.update()
        except tk.TclError:
            closing = True

    def draw() -> None:
        nonlocal rate_started
        now = time.monotonic()
        elapsed = now - rate_started
        if elapsed >= 1.0:
            for name in sensor_uuids:
                rates[name] = (recorder.counts[name] - previous_counts[name]) / elapsed
                previous_counts[name] = recorder.counts[name]
            rate_started = now

        emg_view = views.get("emg")
        if emg_view:
            _draw_traces(emg_view["canvas"], [list(channel) for channel in emg if channel])
            rms = []
            for index, channel in enumerate(emg):
                if not channel:
                    continue
                mean = sum(channel) / len(channel)
                centered_rms = math.sqrt(
                    sum((value - mean) ** 2 for value in channel) / len(channel)
                )
                rms.append(f"ch{index + 1} centered RMS {centered_rms:.1f} ADC")
            emg_view["value"].set(
                "  |  ".join(rms) if rms else "EMG: waiting for packets"
            )

        imu_view = views.get("imu")
        if imu_view:
            primary_sensor = min(latest_imu, default=0)
            _draw_traces(
                imu_view["accel"], [list(axis) for axis in accel[primary_sensor]]
            )
            _draw_traces(
                imu_view["gyro"], [list(axis) for axis in gyro[primary_sensor]]
            )
            readings = []
            displayed_tilts: dict[int, tuple[float, float]] = {}
            accel_unit, gyro_unit = getattr(
                recorder, "imu_units", ("device units", "device units")
            )
            for index, values in sorted(latest_imu.items()):
                if len(values) < 6:
                    continue
                tilt = imu_tilt_degrees(values)
                if tilt is not None:
                    zero_pitch, zero_roll = tilt_zero.get(index, (0.0, 0.0))
                    displayed_tilts[index] = (
                        relative_angle(tilt[0], zero_pitch),
                        relative_angle(tilt[1], zero_roll),
                    )
                readings.append(
                    f"IMU {index + 1}: a=({values[0]:.2f}, {values[1]:.2f}, {values[2]:.2f}) "
                    f"{accel_unit}  g=({values[3]:.2f}, {values[4]:.2f}, {values[5]:.2f}) "
                    f"{gyro_unit}"
                )
            _draw_orientation(imu_view["orientation"], displayed_tilts)
            imu_view["value"].set(
                "\n".join(readings) if readings else "IMU: waiting for packets"
            )

        piezo_view = views.get("piezo")
        if piezo_view:
            _draw_traces(piezo_view["canvas"], [list(piezo)])
            piezo_view["value"].set(
                f"Piezo mean: {latest_piezo:.1f} ADC"
                if latest_piezo is not None
                else "Piezo: waiting for packets"
            )

        summary = ", ".join(
            f"{key} {count} ({rates[key]:.0f}/s, gaps {sequence_gaps[key]})"
            for key, count in recorder.counts.items()
        )
        stale = (
            " (no recent packets)"
            if last_packet and time.monotonic() - last_packet > 2
            else ""
        )
        status.set(f"Connected: {summary}{stale}")

    try:
        scan = (
            None
            if args.address
            else asyncio.create_task(
                BleakScanner.find_device_by_name(
                    args.device, timeout=args.scan_timeout
                )
            )
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
            print(
                "BLE preview connected. Sensor windows share one connection; no files are saved."
            )
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
