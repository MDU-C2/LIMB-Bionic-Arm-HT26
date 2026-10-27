"""Named specialist actions and their typed desktop settings."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

from aurora.paths import RESOURCE_DIRECTORY


@dataclass(frozen=True)
class Setting:
    key: str
    label: str
    flag: str | None
    default: object = ""
    kind: str = "text"
    choices: tuple = ()
    minimum: float | None = None
    maximum: float | None = None


@dataclass(frozen=True)
class Tool:
    title: str
    module: str
    settings: tuple[Setting, ...] = ()
    requirements: tuple[str, ...] = ("numpy", "scipy")
    description: str = ""
    prefix: tuple[str, ...] = ()
    hardware: bool = False
    script: bool = False


DEMO = str(RESOURCE_DIRECTORY / "examples/simulation/demo")
SOURCE = Setting("source", "Recording or trial", "--file", kind="recording")
OUTPUT = Setting("output", "New output folder", "--output", kind="new_directory")
SECONDS = Setting("seconds", "Capture time (seconds)", "--seconds", 8, "float", minimum=0.1)
MODE = Setting(
    "mode",
    "Robot mode",
    "--mode",
    "Direct pose preview",
    "choice",
    ("Direct pose preview", "Motor physics"),
)
SIDE = Setting("side", "Human arm", "--side", "left", "choice", ("left", "right"))
TOOLS = {
    "legacy_ble_preview": Tool(
        "Legacy BLE compatibility preview",
        "aurora.recording.record_ble_sensors",
        (
            Setting("device", "BLE device name", "--device", "AURORA"),
            Setting("address", "Optional known BLE address", "--address", kind="text_optional"),
            Setting(
                "streams",
                "Sensor windows",
                "--sensors",
                ("imu", "emg"),
                "multi",
                ("imu", "emg", "piezo"),
            ),
            Setting(
                "format",
                "Known EMG wire format",
                "--emg-format",
                "auto",
                "choice",
                ("auto", "legacy-fixed", "legacy-prefix"),
            ),
            Setting(
                "timeout", "Scan timeout (seconds)", "--scan-timeout", 12, "float", minimum=0.1
            ),
        ),
        ("bleak",),
        "File-free diagnostic windows for earlier firmware, including piezo. Headerless layouts require known firmware. Use Connect & inspect for the current board.",
        prefix=("--preview",),
        hardware=True,
    ),
    "serial_control": Tool(
        "USB live-control diagnostics",
        "aurora.simulation.interactive",
        (
            Setting("port", "USB streaming port", "--port"),
            Setting("baud", "Baud", "--baud", 115200, "int", minimum=1),
            Setting("subject", "Subject ID", "--subject", "diagnostic"),
            SIDE,
            MODE,
            Setting("depth", "Enable stereo depth", "--depth", True, "bool"),
            Setting(
                "weight",
                "Camera correction (0–1)",
                "--camera-weight",
                0.25,
                "float",
                minimum=0,
                maximum=1,
            ),
            Setting(
                "grip", "Grip source", "--grip-source", "emg", "choice", ("emg", "camera", "auto")
            ),
            Setting(
                "upper_axis",
                "Upper-arm axis (without fitted mounting)",
                "--shoulder-twist-axis",
                "y",
                "choice",
                ("x", "y", "z", "-x", "-y", "-z"),
            ),
            Setting(
                "forearm_axis",
                "Forearm axis (without fitted mounting)",
                "--wrist-twist-axis",
                "y",
                "choice",
                ("x", "y", "z", "-x", "-y", "-z"),
            ),
            Setting(
                "output",
                "Optional detailed telemetry (motor physics only)",
                "--telemetry-out",
                kind="save_jsonl_optional",
            ),
        ),
        ("numpy", "scipy", "serial", "pybullet", "pygame", "depthai", "cv2", "mediapipe"),
        "Requires USB-streaming firmware and the camera. Uses the existing configured-axis diagnostic controller: match the reference, then Align and start USB control. Use BLE for guided functional mounting and deliberate grip calibration. Serial diagnostics do not use learned BLE EMG models. Rate and units depend on the firmware.",
        prefix=("--control", "camera-imu", "--transport", "serial"),
        hardware=True,
    ),
    "plot": Tool(
        "Signal plots and image export",
        "aurora.recording.plot_sensor_recording",
        (
            SOURCE,
            Setting(
                "sensor",
                "Signals",
                "--sensor",
                "all",
                "choice",
                ("all", "emg", "imu1", "imu2", "camera"),
            ),
            Setting("spectrum", "Include EMG spectrum", "--spectrum", False, "bool"),
            Setting("save", "Image export (empty opens a window)", "--save", kind="save_optional"),
        ),
        ("numpy", "scipy", "matplotlib"),
        "Plot acquired samples. Spectrum needs continuous, regularly timed EMG. Image export saves a separate PNG.",
    ),
    "process_emg": Tool(
        "Process saved EMG",
        "aurora.recording.emg_processing",
        (
            SOURCE,
            OUTPUT,
            Setting(
                "lowcut", "Band-pass lower frequency (Hz)", "--lowcut", 20, "float", minimum=0.001
            ),
            Setting(
                "highcut",
                "Band-pass upper frequency (Hz)",
                "--highcut",
                450,
                "float",
                minimum=0.001,
            ),
            Setting("notch", "Mains notch frequency (Hz)", "--notch", 50, "float", minimum=0.001),
            Setting("window", "Feature window (ms)", "--window-ms", 200, "float", minimum=0.001),
            Setting("overlap", "Window overlap (ms)", "--overlap-ms", 50, "float", minimum=0),
            Setting(
                "noise",
                "Zero-crossing/slope noise threshold (ADC)",
                "--noise-threshold",
                5,
                "float",
                minimum=0,
            ),
            Setting("labels", "Optional guided grip cue file", "--labels", kind="file_optional"),
        ),
        description="Saves filtered CSV, six-feature CSV and processing metadata in a new folder. Raw files stay unchanged. Guided cues are operator events; reviewed labels remain separate.",
    ),
    "bundle": Tool(
        "Export recording bundle",
        "aurora.gui.tool_jobs",
        (
            Setting("source", "Recording file or trial folder", "--source", kind="recording"),
            Setting("output", "New ZIP file", "--output", kind="save_zip"),
        ),
        description="Selecting trial.json exports its entire trial folder and metadata. Choose the subject folder to include separate reference sessions. Other selected files export individually. Includes file hashes. Stop recording before export.",
        prefix=("bundle",),
    ),
    "dmp_play": Tool(
        "Play DMP trajectory",
        "aurora.simulation.trajectory_playback",
        (
            Setting("path", "Trajectory folder", "--path", DEMO, "directory"),
            Setting(
                "source",
                "Preferred angles/rollout",
                "--source",
                "clean",
                "choice",
                ("clean", "raw"),
            ),
            Setting("refit", "Fit from angles instead of saved rollout", "--refit", False, "bool"),
            Setting("loop", "Loop playback", "--loop", False, "bool"),
            Setting("headless", "Validate once without a window", "--headless", False, "bool"),
        ),
        ("numpy", "scipy", "pybullet"),
        "Robot trajectory preview with position/speed limits. The inherited angles-only fit uses a one-second movement. This is separate from motor physics.",
    ),
    "dmp_export": Tool(
        "Fit and export DMP",
        "aurora.gui.tool_jobs",
        (
            Setting("source", "Original angles/trajectory folder", "--source", DEMO, "directory"),
            OUTPUT,
            Setting(
                "preferred", "Preferred angles", "--preferred", "clean", "choice", ("clean", "raw")
            ),
            Setting(
                "saved", "Export existing rollout instead of refitting", "--saved", False, "bool"
            ),
        ),
        description="Reuses the existing DMP fitter. Saves unclipped radians in NPZ and CSV, input hashes and a completion manifest in a new folder.",
        prefix=("dmp",),
    ),
    "joints": Tool(
        "Manual robot joint sliders",
        "aurora.simulation.joint_sliders",
        (
            Setting(
                "step",
                "Visual update interval (seconds)",
                "--step-time",
                1 / 120,
                "float",
                minimum=0.001,
            ),
        ),
        requirements=("numpy", "scipy", "pybullet"),
        description="Direct robot-command preview within joint limits. These sliders are not anatomical measurements or motor physics.",
    ),
    "pose_play": Tool(
        "Play recorded camera pose",
        "aurora.simulation.pose_playback",
        (
            Setting("recording", "Camera pose recording", None, kind="file"),
            Setting("fps", "Playback frames per second", "--fps", 20, "float", minimum=0.001),
            Setting("loop", "Loop playback", "--loop", False, "bool"),
            Setting("headless", "Validate without a window", "--headless", False, "bool"),
        ),
        ("numpy", "scipy", "pybullet"),
        "Maps recorded landmarks to robot commands. This preview does not establish tracking accuracy.",
    ),
    "movement_capture": Tool(
        "Capture camera movement profile",
        "aurora.recording.capture_movement",
        (
            Setting("subject", "Subject ID", "--subject-id", "S01"),
            SIDE,
            Setting(
                "output",
                "Movement capture root",
                "--output",
                "outputs/recordings/movement",
                "output_directory",
            ),
        ),
        ("numpy", "scipy", "depthai", "cv2", "mediapipe"),
        "Requires stereo depth. In the camera window, press Space to start and again to save. Each capture gets a new trial ID.",
        hardware=True,
    ),
    "movement_train": Tool(
        "Train movement-profile GRU",
        "aurora.ml.train_gru",
        (
            Setting(
                "data",
                "Movement profile dataset",
                "--data",
                "outputs/recordings/movement",
                "directory",
            ),
            Setting("epochs", "Epochs", "--epochs", 30, "int", minimum=1, maximum=10000),
            Setting("seed", "Random seed", "--seed", 42, "int", minimum=0, maximum=4294967295),
            OUTPUT,
        ),
        ("numpy", "scipy", "torch", "onnx", "matplotlib"),
        "Requires separate trials from each subject for training and validation. Saves a new run and training manifest. The bundled model is preserved.",
    ),
    "movement_results": Tool(
        "Plot movement training results",
        "aurora.ml.training_plot",
        (
            Setting("training", "Training manifest", None, kind="file"),
            Setting("save", "New plot image", "--save", kind="save_png"),
        ),
        ("numpy", "matplotlib"),
    ),
    "emg_train": Tool(
        "Compare EMG models",
        "aurora.ml.train_emg",
        (
            Setting("data", "Reviewed EMG label folder", "--data", kind="directory"),
            Setting("models", "Models", "--models", ("lda",), "multi", ("lda", "ann", "lstm")),
            Setting(
                "features",
                "Feature sets",
                "--features",
                ("basic", "full"),
                "multi",
                ("basic", "full"),
            ),
            Setting("rate", "EMG sample rate (Hz)", "--rate", 4000, "float", minimum=1),
            Setting("window", "Feature window (ms)", "--window-ms", 200, "float", minimum=1),
            Setting("overlap", "Window overlap (ms)", "--overlap-ms", 50, "float", minimum=0),
            Setting(
                "noise", "Feature noise threshold (ADC)", "--noise-threshold", 5, "float", minimum=0
            ),
            Setting(
                "sequence", "Sequence windows", "--sequence-length", 4, "int", minimum=1, maximum=20
            ),
            Setting(
                "calibration",
                "Separate personal calibration trials",
                "--calibration-trials",
                0,
                "int",
                minimum=0,
            ),
            Setting(
                "epochs", "Epochs (neural models)", "--epochs", 30, "int", minimum=1, maximum=10000
            ),
            Setting("seed", "Random seed", "--seed", 42, "int", minimum=0, maximum=4294967295),
            OUTPUT,
        ),
        ("numpy", "scipy", "matplotlib"),
        "Use reviewed intervals from at least two subjects. LDA runs without neural training packages. ANN/LSTM require optional Torch and ONNX. Each run is separate; selection is explicit.",
    ),
    "emg_results": Tool(
        "Plot EMG evaluation results",
        "aurora.ml.emg_results",
        (
            Setting("training", "Training manifest", "--training", kind="file"),
            Setting("save", "New plot file", "--save", kind="save_png"),
            Setting("show", "Show plot window", "--show", True, "bool"),
        ),
        ("numpy", "matplotlib"),
    ),
    "fusion_synthetic": Tool(
        "Compare fusion methods on synthetic motion",
        "aurora.sensors.fusion_comparison",
        (OUTPUT,),
        ("numpy", "scipy", "matplotlib"),
        "An analytic reference checks software behaviour, not physical anatomical accuracy.",
        prefix=("--synthetic",),
    ),
    "fusion_reference": Tool(
        "Compare fusion against an independent reference",
        "aurora.sensors.fusion_comparison",
        (Setting("file", "Aligned reference CSV", "--file", kind="file"), OUTPUT),
        ("numpy", "scipy", "matplotlib"),
        "Needs the CSV's matching JSON describing an independent reference, common clock and calibrated planar frame.",
    ),
    "dynamics": Tool(
        "Inspect robot dynamics",
        "aurora.simulation.model.dynamics",
        (
            Setting("payload", "Hand payload (kg)", "--payload-kg", 0, "float", minimum=0),
            Setting("inspect", "Include all joints and links", "--inspect", True, "bool"),
        ),
        ("numpy", "scipy", "pybullet"),
        "Prints model dynamics to Activity. Payload is a simulated point mass; it does not change the empty-cup collection task.",
    ),
    "ble_check": Tool(
        "Check BLE discovery",
        "check_ble.py",
        (SECONDS, Setting("name", "Advertised name filter", "--name", "AURORA")),
        ("bleak",),
        "Scans devices. Stop other sensor owners first.",
        hardware=True,
        script=True,
    ),
    "usb_check": Tool(
        "Check USB IMU and EMG delivery",
        "check_dual_imu.py",
        (
            Setting("port", "USB port", "--port"),
            Setting("baud", "Baud", "--baud", 115200, "int", minimum=1),
            SECONDS,
            Setting(
                "minimum", "Required samples per channel", "--min-samples", 5, "int", minimum=1
            ),
            Setting("emg", "Require EMG samples", "--require-emg", True, "bool"),
        ),
        ("serial",),
        hardware=True,
        script=True,
    ),
    "camera_check": Tool(
        "Check camera frame delivery",
        "check_oak_camera.py",
        (
            Setting("width", "Width (pixels)", "--width", 640, "int", minimum=1),
            Setting("height", "Height (pixels)", "--height", 480, "int", minimum=1),
            Setting(
                "frames", "Frames to receive (0 = until closed)", "--frames", 120, "int", minimum=0
            ),
            Setting("headless", "Receive without a preview window", "--headless", False, "bool"),
        ),
        ("depthai", "cv2"),
        hardware=True,
        script=True,
    ),
    "emg_check": Tool(
        "Check EMG rest or grip calibration",
        "check_emg_calibration.py",
        (
            Setting("phase", "Capture phase", "--phase", "rest", "choice", ("rest", "grip")),
            Setting("seconds", "Capture time (seconds)", "--seconds", 6, "float", minimum=4),
            Setting("device", "BLE device name", "--device", "AURORA"),
            Setting("output", "New diagnostic JSON", "--out", kind="save_json"),
        ),
        ("numpy", "scipy", "bleak"),
        "Follow the relaxed baseline and automatic GRIP cues in Activity. No terminal input is required.",
        hardware=True,
        script=True,
    ),
    "replay_check": Tool(
        "Check recorded sensor replay",
        "check_live_replay.py",
        (
            Setting("trial", "Recorded trial folder or manifest", None, kind="recording"),
            Setting("output", "New diagnostic folder", "--out", kind="new_directory"),
        ),
        description="Checks replay consistency without sensors. It does not supply independent tracking ground truth.",
        script=True,
    ),
    "live_check": Tool(
        "Profile live sensor control",
        "diagnose_live_control.py",
        (
            SECONDS,
            MODE,
            Setting("depth", "Use stereo depth", "--depth", True, "bool"),
            Setting("device", "BLE device name", "--device", "AURORA"),
            Setting("address", "Optional known BLE address", "--address", kind="text_optional"),
            Setting("output", "New diagnostic JSON", "--out", kind="save_json"),
        ),
        ("numpy", "scipy", "pybullet", "pygame", "bleak", "depthai", "cv2", "mediapipe"),
        "Opens live sensors and the simulator. Operator calibration and a connected device are required.",
        hardware=True,
        script=True,
    ),
    "recorded_profile": Tool(
        "Profile control from a recording",
        "diagnose_live_control.py",
        (
            SECONDS,
            MODE,
            Setting("trial", "Recorded trial", "--replay", kind="recording"),
            Setting("output", "New diagnostic JSON", "--out", kind="save_json"),
        ),
        ("numpy", "scipy", "pybullet", "pygame"),
        script=True,
    ),
    "benchmark": Tool(
        "Benchmark offline processing and display",
        "benchmark_runtime.py",
        (
            Setting(
                "seconds",
                "Synthetic sensor seconds",
                "--seconds",
                5,
                "float",
                minimum=0.1,
                maximum=60,
            ),
            Setting("steps", "Physics steps", "--steps", 600, "int", minimum=1, maximum=10000),
            Setting("repeats", "Repeats", "--repeats", 3, "int", minimum=1, maximum=20),
            Setting("ui", "Measure main UI responsiveness", "--ui", False, "bool"),
            Setting("physics", "Compare physics GUI with DIRECT", "--physics-gui", False, "bool"),
            Setting(
                "keyboard", "Include keyboard display comparison", "--keyboard-gui", False, "bool"
            ),
            Setting(
                "browser",
                "Synthetic browser files",
                "--browser-files",
                0,
                "int",
                minimum=0,
                maximum=10000,
            ),
            Setting("output", "New benchmark JSON", "--output", kind="save_json"),
        ),
        ("numpy", "scipy", "pybullet", "pygame"),
        "Host processing costs only; excludes device acquisition and tracking accuracy.",
        script=True,
    ),
    "inference_benchmark": Tool(
        "Benchmark camera inference on blank images",
        "benchmark_runtime.py",
        (
            Setting("repeats", "Repeats", "--repeats", 3, "int", minimum=1, maximum=20),
            Setting("output", "New benchmark JSON", "--output", kind="save_json"),
        ),
        ("numpy", "scipy", "cv2", "mediapipe", "onnxruntime"),
        "Synthetic blank images measure model cost, not camera capture or tracking accuracy.",
        prefix=("--inference-only",),
        script=True,
    ),
}


def arguments_for(key: str, values: dict) -> list[str]:
    """Validate a desktop form and build arguments without shell parsing."""
    tool = TOOLS[key]
    result = list(tool.prefix)
    for setting in tool.settings:
        value = values.get(setting.key, setting.default)
        if setting.kind == "bool":
            if value:
                result.append(setting.flag)
            continue
        if setting.kind == "multi":
            value = list(value)
            if not value or not set(value) <= set(setting.choices):
                raise ValueError(f"Choose at least one {setting.label.lower()} option.")
            result.extend((setting.flag, *value))
            continue
        value = str(value).strip()
        if not value and setting.kind.endswith("_optional"):
            continue
        if not value:
            raise ValueError(f"{setting.label} is required.")
        if setting.kind in {"int", "float"}:
            try:
                number = int(value) if setting.kind == "int" else float(value)
            except ValueError as error:
                raise ValueError(f"{setting.label} must be a number.") from error
            if (
                not math.isfinite(number)
                or (setting.minimum is not None and number < setting.minimum)
                or (setting.maximum is not None and number > setting.maximum)
            ):
                raise ValueError(f"{setting.label} is outside its allowed range.")
        elif setting.kind == "choice" and value not in setting.choices:
            raise ValueError(f"Choose a valid {setting.label.lower()}.")
        elif setting.kind in {
            "file",
            "file_optional",
            "recording",
            "directory",
            "new_directory",
            "output_directory",
        } or setting.kind.startswith("save_"):
            path = Path(value).expanduser().resolve()
            if setting.kind in {"file", "file_optional"} and not path.is_file():
                raise ValueError(f"{setting.label}: choose an existing file.")
            if setting.kind == "directory" and not path.is_dir():
                raise ValueError(f"{setting.label}: choose an existing folder.")
            if setting.kind == "recording" and not path.exists():
                raise ValueError(f"{setting.label}: choose an existing recording.")
            if (
                setting.kind == "new_directory" or setting.kind.startswith("save_")
            ) and path.exists():
                raise ValueError(f"{setting.label} already exists. Choose a new output.")
            value = str(path)
        if setting.key == "mode":
            value = {"Direct pose preview": "kinematic", "Motor physics": "dynamic"}[value]
        if setting.flag and value.startswith("-"):
            result.append(f"{setting.flag}={value}")
            continue
        if setting.flag:
            result.append(setting.flag)
        result.append(value)
    if key in {"emg_train", "process_emg"} and float(values.get("overlap", 50)) >= float(
        values.get("window", 200)
    ):
        raise ValueError("Window overlap must be shorter than the window.")
    if key in {"dmp_play", "pose_play"} and values.get("loop") and values.get("headless"):
        raise ValueError("Choose either looping preview or one headless validation.")
    if key == "camera_check" and values.get("headless") and int(values.get("frames", 120)) == 0:
        raise ValueError("Choose a positive frame count for headless validation.")
    if key == "benchmark" and int(values.get("browser", 0)) and not values.get("ui"):
        raise ValueError("Enable UI responsiveness to benchmark the recording browser.")
    if (
        key == "serial_control"
        and values.get("output")
        and values.get("mode", MODE.default) != "Motor physics"
    ):
        raise ValueError("Detailed telemetry requires Motor physics mode.")
    return result
