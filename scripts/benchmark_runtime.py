"""Measure deterministic host processing, recording, physics and UI work."""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import importlib.metadata
import json
import os
import platform
import struct
import sys
import tempfile
import time
from pathlib import Path

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
if os.name == "nt":
    os.environ["PATH"] = (
        str(Path(sys.executable).parent / "Library/bin") + os.pathsep + os.environ.get("PATH", "")
    )


def memory_bytes():
    if os.name != "nt":
        return None

    class Counters(ctypes.Structure):
        _fields_ = [("cb", ctypes.c_ulong), ("page_faults", ctypes.c_ulong)] + [
            (name, ctypes.c_size_t)
            for name in (
                "peak_working_set",
                "working_set",
                "peak_paged_pool",
                "paged_pool",
                "peak_nonpaged_pool",
                "nonpaged_pool",
                "pagefile",
                "peak_pagefile",
            )
        ]

    counters = Counters()
    counters.cb = ctypes.sizeof(counters)
    if ctypes.windll.psapi.GetProcessMemoryInfo(
        ctypes.c_void_p(-1), ctypes.byref(counters), counters.cb
    ):
        return counters.working_set
    return None


def distribution(values):
    import numpy as np

    return {f"p{p}_ms": float(np.percentile(values, p) * 1000) for p in (50, 95, 99)}


def sensor_run(seconds, recording):
    from aurora.sensors.ble_input import BleSensorReader
    from aurora.sensors.ble_protocol import HIGH_RATE_EMG_HEADER, HIGH_RATE_IMU_PACKET

    reader = BleSensorReader(clock=lambda: 1000.0)
    durations = []
    with tempfile.TemporaryDirectory(prefix="aurora_benchmark_") as temporary:
        if recording:
            reader.start_recording(Path(temporary))
        wall, cpu = time.perf_counter(), time.process_time()
        for sequence in range(round(seconds * 100)):
            stamp = 1_000_000 + sequence * 10_000
            imu = HIGH_RATE_IMU_PACKET.pack(
                0x0103, sequence, stamp, stamp + 100, *([0, 0, 1000, 0, 0, 0] * 2)
            )
            emg = HIGH_RATE_EMG_HEADER.pack(0x0100, sequence, stamp, 250, 40, 0)
            emg += struct.pack("<40H", *([2048, 2050, 2046, 2049] * 10))
            for sensor, payload in (("imu", imu), ("emg", emg)):
                start = time.perf_counter()
                reader.notification(sensor, payload)
                durations.append(time.perf_counter() - start)
        callback_wall = time.perf_counter() - wall
        health = reader.stop_recording() if recording else reader.decoder.packet_health()
        return dict(
            callback_wall_s=callback_wall,
            total_wall_s=time.perf_counter() - wall,
            cpu_s=time.process_time() - cpu,
            notification_latency=distribution(durations),
            notifications=len(durations),
            emg_samples=round(seconds * 4000),
            imu_role_samples=round(seconds * 200),
            health=health,
            delivery="deterministic burst; excludes BLE and device acquisition",
        )


def physics_run(steps, gui, motor):
    import pybullet as p

    from aurora.simulation.interactive.arm_control import (
        AuroraArm,
        ManualMotion,
        build_joint_index_map,
        set_elbow,
        set_ready_pose,
        sync_to_pybullet,
    )
    from aurora.simulation.interactive.settings import PHYSICS_SUBSTEPS, SIMULATION_FREQUENCY_HZ
    from aurora.simulation.model.robot_model import load_left_arm

    client = p.connect(p.GUI if gui else p.DIRECT)
    try:
        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(
            fixedTimeStep=1 / SIMULATION_FREQUENCY_HZ,
            numSolverIterations=100,
            numSubSteps=PHYSICS_SUBSTEPS,
        )
        body = load_left_arm(p)
        joints = build_joint_index_map(body, p)
        arm, motion = AuroraArm(), ManualMotion()
        set_ready_pose(arm)
        samples = []
        wall, cpu = time.perf_counter(), time.process_time()
        for index in range(steps):
            start = time.perf_counter()
            keys = {1: (index // 120) % 2 == 0, 2: (index // 120) % 2 != 0}
            delta = motion.step(
                keys, 1, 2, "elbow_x", arm.elbow.angle_x, 1.0, 1 / SIMULATION_FREQUENCY_HZ
            )
            set_elbow(arm, delta, mode="rel")
            sync_to_pybullet(arm, body, joints, client=p, use_motors=motor)
            p.stepSimulation()
            samples.append(time.perf_counter() - start)
        return dict(
            wall_s=time.perf_counter() - wall,
            cpu_s=time.process_time() - cpu,
            steps=steps,
            physics_time_s=steps / SIMULATION_FREQUENCY_HZ,
            stage_latency=distribution(samples),
            solver_iterations=100,
            substeps=PHYSICS_SUBSTEPS,
            frequency_hz=SIMULATION_FREQUENCY_HZ,
            connection="GUI" if gui else "DIRECT",
            motors=motor,
            scope="same arm and settings; excludes task collisions, plots and camera",
        )
    finally:
        p.disconnect(client)


def keyboard_run(steps, displays):
    """Use the complete keyboard loop with a deterministic 60 Hz input clock."""
    from collections import defaultdict
    from unittest.mock import patch

    import pybullet as p
    import pygame

    from aurora.simulation.interactive.runtime import InteractiveSimulator
    from aurora.simulation.interactive.settings import parse_args

    class FrameClock:
        def get_time(self):
            return 1000 / 60

    simulator = InteractiveSimulator(parse_args([]))
    timings = {name: [] for name in ("control_physics", "display", "frame")}
    connect = p.connect
    mode = p.GUI if displays else p.DIRECT
    environment = {} if displays else {"SDL_VIDEODRIVER": "dummy", "SDL_AUDIODRIVER": "dummy"}
    with patch.dict(os.environ, environment):
        try:
            with patch(
                "aurora.simulation.interactive.scene.p.connect",
                side_effect=lambda _mode: connect(mode),
            ):
                simulator.initialize_scene()
            simulator.initialize_arm()
            simulator.initialize_hud()
            simulator.initialize_dashboard()
            simulator.initialize_task()
            simulator.start_live_input()
            if not displays:
                simulator.sensor_window.withdraw()
            simulator.clock = FrameClock()
            wall, cpu = time.perf_counter(), time.process_time()
            for index in range(steps):
                frame_start = time.perf_counter()
                if not simulator.prepare_frame():
                    raise RuntimeError("Keyboard benchmark lost its simulation connection")
                simulator.process_events()
                key = pygame.K_UP if index % 120 < 60 else pygame.K_DOWN
                simulator.keys = defaultdict(bool, {key: True})
                for _ in range(simulator.steps_due):
                    simulator.update_grasp()
                    simulator.update_arm()
                    simulator.advance_physics()
                display_start = time.perf_counter()
                if displays:
                    simulator.update_camera()
                    simulator.draw_hud()
                    simulator.update_dashboard()
                timings["control_physics"].append(display_start - frame_start)
                timings["display"].append(time.perf_counter() - display_start)
                timings["frame"].append(time.perf_counter() - frame_start)
            return dict(
                wall_s=time.perf_counter() - wall,
                cpu_s=time.process_time() - cpu,
                physics_time_s=simulator.simulation_time_s,
                steps=simulator.frame_number,
                stages={name: distribution(values) for name, values in timings.items()},
                scope="Synthetic keyboard, same task/model; fixed 60 Hz steps run without waiting. "
                + (
                    "PyBullet, Pygame and Tk displays enabled."
                    if displays
                    else "DIRECT, dummy Pygame, Tk hidden and display updates disabled."
                ),
            )
        finally:
            simulator.close()


def inference_run(frames):
    from types import SimpleNamespace

    import cv2
    import mediapipe as mp
    import numpy as np

    from aurora.sensors.camera.cup_detection import CupDetector
    from aurora.sensors.camera.hand_tracking import ArmHandTracker
    from aurora.sensors.camera.oak_camera import require_pose_model

    cv2.setNumThreads(1)
    require_pose_model(mp, 1)
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    timings = {
        name: [] for name in ("rgb_conversion", "pose_detection", "hand_detection", "cup_inference")
    }
    load_started = time.perf_counter()
    pose = hands = cups = None
    try:
        pose = mp.solutions.pose.Pose(model_complexity=1)
        hands, cups = ArmHandTracker(mp, cv2), CupDetector(cv2, np)
        load_s = time.perf_counter() - load_started
        wrist, elbow = SimpleNamespace(x=0.5, y=0.5), SimpleNamespace(x=0.5, y=0.8)
        stages = {
            "rgb_conversion": lambda: cv2.cvtColor(image, cv2.COLOR_BGR2RGB),
            "pose_detection": lambda: pose.process(rgb),
            "hand_detection": lambda: hands.process_for_arm(rgb, wrist, elbow),
            "cup_inference": lambda: cups.detect(image),
        }
        cpu = time.process_time()
        for index in range(frames + 3):
            for name, run in stages.items():
                start = time.perf_counter()
                run()
                if index >= 3:
                    timings[name].append(time.perf_counter() - start)
        return dict(
            load_s=load_s,
            cpu_s=time.process_time() - cpu,
            frames=frames,
            stages={name: distribution(values) for name, values in timings.items()},
            cup_status=cups.status,
            scope="synthetic blank RGB; detector cost only, no tracked person or hand; excludes capture",
        )
    finally:
        for tracker in (pose, hands, cups):
            if tracker is not None:
                tracker.close()


def ui_run(browser_files=0):
    import tkinter as tk

    from aurora.gui.app import ProjectGui

    temporary = tempfile.TemporaryDirectory(prefix="aurora_browser_") if browser_files else None
    if temporary is not None:
        for index in range(browser_files):
            (Path(temporary.name) / f"trial_{index:05d}.json").write_text("{}", encoding="utf-8")
    window = tk.Tk()
    window.withdraw()
    start = time.perf_counter()
    app = ProjectGui(window)
    construction = time.perf_counter() - start
    if temporary is not None:
        app.recordings_root.set(temporary.name)
        app._refresh_recordings()
    heartbeat = []
    previous = time.perf_counter()

    def beat():
        nonlocal previous
        now = time.perf_counter()
        heartbeat.append(now - previous)
        previous = now
        window.after(10, beat)

    beat()
    deadline = time.perf_counter() + 20
    try:
        tasks = getattr(app, "background_tasks", None)
        while tasks is not None and tasks.jobs and time.perf_counter() < deadline:
            window.update()
            time.sleep(0.001)
        start = time.perf_counter()
        app._refresh_all()
        refresh = time.perf_counter() - start
        while tasks is not None and tasks.jobs and time.perf_counter() < deadline:
            window.update()
            time.sleep(0.001)
        window.update()
        return dict(
            construction_s=construction,
            refresh_callback_s=refresh,
            heartbeat_intervals=distribution(heartbeat),
            completed=tasks is None or not tasks.jobs,
            recording_files=len(app.recording_files),
            tree_rows=len(app.recordings_tree.get_children()),
            scope=f"withdrawn Tk window; {browser_files} synthetic files"
            if browser_files
            else "withdrawn Tk window, dependency checks and current recording folder",
        )
    finally:
        app.close()
        if temporary is not None:
            temporary.cleanup()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=5)
    parser.add_argument("--steps", type=int, default=600)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--ui", action="store_true")
    parser.add_argument(
        "--browser-files",
        type=int,
        default=0,
        help="Exercise the recording browser with a temporary synthetic folder",
    )
    parser.add_argument("--physics-gui", action="store_true")
    parser.add_argument(
        "--keyboard-gui",
        action="store_true",
        help="Compare the full keyboard loop with displays disabled/enabled",
    )
    parser.add_argument(
        "--inference-only",
        action="store_true",
        help="Run native camera models separately from physics and Tk",
    )
    args = parser.parse_args(argv)
    if not 0 <= args.browser_files <= 10000 or (args.browser_files and not args.ui):
        parser.error("Use --ui and a browser file count from 0 to 10000")
    if not (0 < args.seconds <= 60 and 0 < args.steps <= 10000 and 0 < args.repeats <= 20):
        parser.error("Use seconds 0..60, steps 1..10000 and repeats 1..20")
    result = dict(
        python=sys.version,
        executable=sys.executable,
        platform=platform.platform(),
        processors=os.cpu_count(),
        parameters=vars(args) | {"output": str(args.output)},
        memory_before_bytes=memory_bytes(),
        scenarios={},
        benchmark_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    )
    result["versions"] = {}
    from aurora.paths import PACKAGE_DIRECTORY

    result["source_sha256"] = {
        path.relative_to(PACKAGE_DIRECTORY).as_posix(): hashlib.sha256(
            path.read_bytes()
        ).hexdigest()
        for path in sorted(PACKAGE_DIRECTORY.rglob("*.py"))
    }
    for package in (
        "numpy",
        "scipy",
        "pybullet",
        "pygame",
        "mediapipe",
        "depthai",
        "opencv-contrib-python",
        "onnxruntime",
        "bleak",
        "pyserial",
    ):
        try:
            result["versions"][package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            result["versions"][package] = None
    scenarios = {
        "sensor_core": lambda: sensor_run(args.seconds, False),
        "sensor_recording": lambda: sensor_run(args.seconds, True),
        "physics_direct_preview": lambda: physics_run(args.steps, False, False),
        "physics_direct_motor": lambda: physics_run(args.steps, False, True),
    }
    if args.inference_only:
        scenarios = {"blank_frame_inference": lambda: inference_run(30)}
    if args.physics_gui:
        scenarios["physics_gui_preview"] = lambda: physics_run(args.steps, True, False)
        scenarios["physics_gui_motor"] = lambda: physics_run(args.steps, True, True)
    if args.keyboard_gui:
        scenarios["keyboard_displays_disabled"] = lambda: keyboard_run(args.steps, False)
        scenarios["keyboard_displays_enabled"] = lambda: keyboard_run(args.steps, True)
    if args.ui:
        scenarios["ui"] = lambda: ui_run(args.browser_files)
    for name, run in scenarios.items():
        try:
            result["scenarios"][name] = [run() for _ in range(args.repeats)]
        except (ImportError, RuntimeError) as error:
            result["scenarios"][name] = {"unavailable": str(error)}
        print(f"Measured {name}", flush=True)
    result["memory_after_bytes"] = memory_bytes()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"Saved {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
