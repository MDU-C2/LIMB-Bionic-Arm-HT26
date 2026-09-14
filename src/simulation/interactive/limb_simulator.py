"""Interactive LIMB arm task simulation migrated from LIMB-HT25.

The simulator opens a PyBullet task scene, a Pygame control HUD, and a Tkinter
sensor dashboard. Paths are resolved from this file so it works from any clone
location, including Windows repositories whose parent path contains non-ASCII
characters.
"""

from __future__ import annotations

import math
import os

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import pybullet as p
import pybullet_data
import pygame
import tkinter as tk
from kinematics import calculate_ik_angles, is_reachable


SIMULATION_FREQUENCY_HZ = 60
PHYSICS_SUBSTEPS = 4
CONTROL_STEP_DEG = 1.5
TARGET_CONTROL_STEP_DEG = 0.8
TARGET_GRASP_DISTANCE_M = 0.20
TARGET_NORMAL_MASS_KG = 0.1
TARGET_GRASPED_MASS_KG = 0.01
FINGER_ZERO_OFFSET_RAD = -1.4

ARM_MOTOR_FORCE = 200.0
ARM_POSITION_GAIN = 0.05
ARM_VELOCITY_GAIN = 1.0
FINGER_MOTOR_FORCE = 2.0
FINGER_POSITION_GAIN = 0.1
FINGER_VELOCITY_GAIN = 1.0
FINGER_JOINT_TOKENS = ("thumb", "index", "middle", "ring", "pinky")


# --- 1. Arm command model ---------------------------------------------------


def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp a numeric value to an inclusive range."""
    return max(min_value, min(max_value, value))


def deg_to_rad(deg: float) -> float:
    """Convert degrees to radians."""
    return deg * math.pi / 180.0


def rad_to_deg(rad: float) -> float:
    """Convert radians to degrees."""
    return rad * 180.0 / math.pi


class Shoulder:
    """Three-axis shoulder command and its inherited software limits."""

    def __init__(self) -> None:
        """Initialize the shoulder in its neutral pose."""
        self.angle_x = 0.0
        self.angle_y = 0.0
        self.angle_z = 0.0
        # Limits (in degrees)
        self.min_angle_x = -120
        self.max_angle_x = 40
        self.min_angle_y = -85
        self.max_angle_y = 150.0
        self.min_angle_z = -95
        self.max_angle_z = 50


class Elbow:
    """Two-axis elbow command and its inherited software limits."""

    def __init__(self) -> None:
        """Initialize elbow flexion and pronation in their neutral pose."""
        self.angle_x = 0.0  # Flexion
        self.angle_y = 0.0  # Pronation
        # Limits (in degrees)
        self.min_angle_x = -90.0
        self.max_angle_x = 0.0
        self.min_angle_y = -90.0
        self.max_angle_y = 90.0


class Wrist:
    """Two-axis wrist command and its inherited software limits."""

    def __init__(self) -> None:
        """Initialize wrist flexion and deviation in their neutral pose."""
        self.angle_x = 0.0  # Flexion
        self.angle_z = 0.0  # Deviation
        # Limits (in degrees)
        self.min_angle_x = -70.0
        self.max_angle_x = 80.0
        self.min_angle_z = -20.0
        self.max_angle_z = 45.0


class Hand:
    """Shared curl command for the index, middle, ring, and little fingers."""

    def __init__(self) -> None:
        """Initialize the hand in an open pose."""
        self.curl = 0.0  # 0.0 is open; 1.0 is a closed fist.
        self.min_curl = 0.0
        self.max_curl = 1.5  # About 86 degrees, closed fist.


class LimbArm:
    """Operator-requested angles for the complete simulated arm."""

    def __init__(self) -> None:
        """Initialize every joint group in its neutral pose."""
        self.shoulder = Shoulder()
        self.elbow = Elbow()
        self.wrist = Wrist()
        self.hand = Hand()


def set_shoulder(
    arm: LimbArm,
    x: float | None = None,
    y: float | None = None,
    z: float | None = None,
    mode: str = "abs",
) -> None:
    """Set or adjust shoulder angles while applying software limits."""
    if x is not None:
        target = (arm.shoulder.angle_x + x) if mode == "rel" else x
        arm.shoulder.angle_x = clamp(target, arm.shoulder.min_angle_x, arm.shoulder.max_angle_x)
    if y is not None:
        target = (arm.shoulder.angle_y + y) if mode == "rel" else y
        arm.shoulder.angle_y = clamp(target, arm.shoulder.min_angle_y, arm.shoulder.max_angle_y)
    if z is not None:
        target = (arm.shoulder.angle_z + z) if mode == "rel" else z
        arm.shoulder.angle_z = clamp(target, arm.shoulder.min_angle_z, arm.shoulder.max_angle_z)


def set_elbow(
    arm: LimbArm,
    x: float | None = None,
    y: float | None = None,
    mode: str = "abs",
) -> None:
    """Set or adjust elbow angles while applying software limits."""
    if x is not None:
        target = (arm.elbow.angle_x + x) if mode == "rel" else x
        arm.elbow.angle_x = clamp(target, arm.elbow.min_angle_x, arm.elbow.max_angle_x)
    if y is not None:
        target = (arm.elbow.angle_y + y) if mode == "rel" else y
        arm.elbow.angle_y = clamp(target, arm.elbow.min_angle_y, arm.elbow.max_angle_y)


def set_wrist(
    arm: LimbArm,
    x: float | None = None,
    z: float | None = None,
    mode: str = "abs",
) -> None:
    """Set or adjust wrist angles while applying software limits."""
    if x is not None:
        target = (arm.wrist.angle_x + x) if mode == "rel" else x
        arm.wrist.angle_x = clamp(target, arm.wrist.min_angle_x, arm.wrist.max_angle_x)
    if z is not None:
        target = (arm.wrist.angle_z + z) if mode == "rel" else z
        arm.wrist.angle_z = clamp(target, arm.wrist.min_angle_z, arm.wrist.max_angle_z)


def get_angles_deg(arm: LimbArm) -> dict[str, float]:
    """Return the seven requested arm-joint angles in degrees."""
    return {
        "shoulder_x": arm.shoulder.angle_x,
        "shoulder_y": arm.shoulder.angle_y,
        "shoulder_z": arm.shoulder.angle_z,
        "elbow_x": arm.elbow.angle_x,
        "elbow_y": arm.elbow.angle_y,
        "wrist_x": arm.wrist.angle_x,
        "wrist_z": arm.wrist.angle_z,
    }


def get_angles_rad(arm: LimbArm) -> dict[str, float]:
    """Return requested arm and finger joint angles in radians."""
    degs = get_angles_deg(arm)
    angles_rad = {k: deg_to_rad(v) for k, v in degs.items()}
    for finger in ("index", "middle", "ring", "pinky"):
        # Ring and little-finger joint 1 use an inherited URDF zero offset.
        if finger in {"ring", "pinky"}:
            angles_rad[f"{finger}_1"] = arm.hand.curl + FINGER_ZERO_OFFSET_RAD
        else:
            angles_rad[f"{finger}_1"] = arm.hand.curl
        angles_rad[f"{finger}_2"] = arm.hand.curl
        if finger in {"ring", "pinky"}:
            angles_rad[f"{finger}_3"] = arm.hand.curl
    return angles_rad


# --- 2. PyBullet mapping and motor commands --------------------------------


def sync_to_pybullet(
    arm: LimbArm,
    body_id: int,
    joint_name_to_index: dict[str, int],
    client=None,
    use_motors: bool = True,
) -> None:
    """Apply the requested arm pose to mapped PyBullet joints."""
    cli = client if client is not None else p
    if not cli.getConnectionInfo().get("isConnected", 0):
        return

    requested_angles = get_angles_rad(arm)

    for name, angle in requested_angles.items():
        if name not in joint_name_to_index:
            continue
        joint_index = joint_name_to_index[name]
        is_finger = any(token in name for token in FINGER_JOINT_TOKENS)
        force = FINGER_MOTOR_FORCE if is_finger else ARM_MOTOR_FORCE
        p_gain = FINGER_POSITION_GAIN if is_finger else ARM_POSITION_GAIN
        v_gain = FINGER_VELOCITY_GAIN if is_finger else ARM_VELOCITY_GAIN

        if use_motors:
            cli.setJointMotorControl2(
                bodyIndex=body_id,
                jointIndex=joint_index,
                controlMode=cli.POSITION_CONTROL,
                targetPosition=angle,
                positionGain=p_gain,
                velocityGain=v_gain,
                force=force,
            )
        else:
            cli.resetJointState(
                bodyUniqueId=body_id,
                jointIndex=joint_index,
                targetValue=angle,
            )

URDF_TO_LOGICAL_JOINT = {
    "jRightShoulder_rotx": "shoulder_x",
    "jRightShoulder_roty": "shoulder_y",
    "jRightShoulder_rotz": "shoulder_z",
    "jRightElbow_roty": "elbow_x",
    "jRightElbow_rotz": "elbow_y",
    "jRightWrist_rotx": "wrist_x",
    "jRightWrist_rotz": "wrist_z",
    "index_joint_1": "index_1",
    "index_joint_2": "index_2",
    "middle_joint_1": "middle_1",
    "middle_joint_2": "middle_2",
    "ring_joint_1": "ring_1",
    "ring_joint_2": "ring_2",
    "ring_joint_3": "ring_3",
    "pinky_joint_1": "pinky_1",
    "pinky_joint_2": "pinky_2",
    "pinky_joint_3": "pinky_3",
}
ARM_JOINT_NAMES = frozenset(
    {"shoulder_x", "shoulder_y", "shoulder_z", "elbow_x", "elbow_y", "wrist_x", "wrist_z"}
)


def build_joint_index_map(robot_id: int, client) -> dict[str, int]:
    """Map logical controller names to their indices in the right-arm URDF."""
    joint_map: dict[str, int] = {}
    for joint_index in range(client.getNumJoints(robot_id)):
        joint_info = client.getJointInfo(robot_id, joint_index)
        urdf_name = joint_info[1].decode("utf-8")
        logical_name = URDF_TO_LOGICAL_JOINT.get(urdf_name)
        if logical_name:
            joint_map[logical_name] = joint_index

    print("--- Joint map (controller -> PyBullet) ---")
    logical_to_urdf = {logical: urdf for urdf, logical in URDF_TO_LOGICAL_JOINT.items()}
    for logical_name, joint_index in joint_map.items():
        print(f"  {logical_name} -> joint {joint_index} ({logical_to_urdf[logical_name]})")

    arm_joint_count = len(ARM_JOINT_NAMES.intersection(joint_map))
    if arm_joint_count != len(ARM_JOINT_NAMES):
        print(f"Warning: {arm_joint_count}/7 arm joints found. Check the URDF names.")
    return joint_map


def build_link_name_index_map(robot_id: int, client) -> dict[str, int]:
    """Map every URDF link name to its PyBullet link index."""
    base_name = client.getBodyInfo(robot_id)[0].decode("utf-8")
    link_map = {base_name: -1}
    for joint_index in range(client.getNumJoints(robot_id)):
        joint_info = client.getJointInfo(robot_id, joint_index)
        child_link_name = joint_info[12].decode("utf-8")
        link_map[child_link_name] = joint_index
    return link_map


def resolve_link_index(
    link_indices_by_name: dict[str, int],
    preferred_names: tuple[str, ...],
    name_fragment: str,
) -> tuple[str, int]:
    """Resolve a display link by preferred name, then by a name fragment."""
    for name in preferred_names:
        if name in link_indices_by_name:
            return name, link_indices_by_name[name]
    for name, link_index in link_indices_by_name.items():
        if name_fragment in name.lower():
            return name, link_index
    raise KeyError(f"No link found for '{name_fragment}'")


# --- 3. Runtime paths and PyBullet setup -----------------------------------
simulator_dir = os.path.abspath(os.path.dirname(__file__))
model_dir = os.path.abspath(os.path.join(simulator_dir, "..", "sim"))
urdf_path = os.path.join(model_dir, "arm", "right_arm.urdf")

print(f"Simulator directory: {simulator_dir}")
print(f"URDF path: {urdf_path}")

try:
    physics_client = p.connect(p.GUI)
except p.error as error:
    raise SystemExit(f"Could not start PyBullet: {error}") from error

if physics_client < 0:
    raise SystemExit("Could not connect to the PyBullet GUI.")

# Match the final LIMB25 workspace layout shown in the project screenshots.
# These previews are useful when camera data is added and can also be toggled
# from PyBullet's View menu.
for preview in (
    p.COV_ENABLE_RGB_BUFFER_PREVIEW,
    p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
    p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
):
    p.configureDebugVisualizer(preview, 1)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.setPhysicsEngineParameter(
    fixedTimeStep=1.0 / SIMULATION_FREQUENCY_HZ,
    numSolverIterations=100,
    numSubSteps=PHYSICS_SUBSTEPS,
)
p.loadURDF("plane.urdf")
p.loadURDF("table/table.urdf", [0, 0.8, -0.2], useFixedBase=True)

# Keep the target fixed so repeatable arm-control experiments start from the
# same position. The fallback cube preserves the scene if PyBullet lacks its
# optional dinnerware assets.
target_start_position = [0.2, 0.5, 0.5]
target_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
try:
    target_body = p.loadURDF(
        "dinnerware/cup_small.urdf",
        target_start_position,
        target_start_orientation,
        useFixedBase=True,
    )
    print(f"Cup loaded (ID: {target_body}) at {target_start_position}")
except p.error:
    print("Cup not found, creating a red cube (with collision).")
    half_extents = [0.03, 0.03, 0.05]
    collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=half_extents,
    )
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=[1, 0.2, 0.2, 1],
    )
    target_body = p.createMultiBody(
        baseMass=TARGET_NORMAL_MASS_KG,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=target_start_position,
    )
    print(f"Replacement cube (ID: {target_body}) placed at {target_start_position}")
p.resetDebugVisualizerCamera(
    cameraDistance=1.0,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.5]
)



# --- 4. Load the arm and configure contact -------------------------------
try:
    print("Attempting to load robot...")
    base_orientation = p.getQuaternionFromEuler([0, 0, 0])

    # PyBullet on Windows cannot open a non-ASCII absolute path reliably. Load
    # the repository-relative path while this directory is the working folder.
    previous_directory = os.getcwd()
    os.chdir(model_dir)
    try:
        p.setAdditionalSearchPath(".")
        robot_body = p.loadURDF(
            "arm/right_arm.urdf",
            [0, 0, 0.7],
            base_orientation,
            useFixedBase=True,
        )
    finally:
        os.chdir(previous_directory)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

    print("\nRobot loaded successfully!")

    p.changeDynamics(
        target_body,
        -1,
        lateralFriction=2.5,
        spinningFriction=0.1,
        rollingFriction=0.1,
    )
    for joint_index in range(p.getNumJoints(robot_body)):
        p.changeDynamics(
            robot_body,
            joint_index,
            lateralFriction=2.5,
            spinningFriction=0.1,
            rollingFriction=0.001,
        )

except p.error as error:
    print("\n--- ERROR LOADING ROBOT ---")
    print("Check that right_arm.urdf and its STL files are in src/simulation/sim/arm.")
    print(f"PyBullet error: {error}")
    p.disconnect()
    raise SystemExit(1) from error

# --- 5. Controls and sensor windows ----------------------------------------
print("Initializing 'brain'...")

arm = LimbArm()
joint_indices = build_joint_index_map(robot_body, p)
link_indices_by_name = build_link_name_index_map(robot_body, p)

try:
    hand_link_name, hand_link_index = resolve_link_index(
        link_indices_by_name,
        ("right_hand", "RightHand"),
        "hand",
    )
    print(f"Link 'Hand' found: {hand_link_name} (Index: {hand_link_index})")

    forearm_link_name, forearm_link_index = resolve_link_index(
        link_indices_by_name,
        ("right_forearm", "RightForeArm"),
        "forearm",
    )
    print(f"Link 'Forearm' (wrist) found: {forearm_link_name} (Index: {forearm_link_index})")

    upperarm_link_name, upperarm_link_index = resolve_link_index(
        link_indices_by_name,
        ("right_upper_arm", "RightUpperArm"),
        "upper",
    )
    print(f"Link 'Upper Arm' (elbow) found: {upperarm_link_name} (Index: {upperarm_link_index})")
except KeyError as error:
    print(f"ERROR: {error}. Link positions will be unavailable.")
    print("Links found:", list(link_indices_by_name.keys()))
    hand_link_index = forearm_link_index = upperarm_link_index = -1

# Build the sensor dashboard before Pygame takes keyboard focus.
print("Initializing sensor window (Tkinter)...")
sensor_window = tk.Tk()
sensor_window.title("LIMB Sensor Dashboard")
sensor_window.geometry("450x450+50+50") # Size and position (X, Y)
sensor_window.attributes('-topmost', True) # Keep on top of PyBullet

# StringVar objects let the simulation update values without recreating labels.
sensor_vars = {}
tk_font = ("Consolas", 11)
tk_font_bold = ("Consolas", 12, "bold")

# Joint-angle and motor-torque readings.
joint_frame = tk.Frame(sensor_window, padx=10, pady=10)
joint_frame.pack(fill='x')

tk.Label(joint_frame, text="--- JOINT SENSORS ---", font=tk_font_bold).pack(anchor='w')

# Short display labels map to the controller's logical joint names.
joint_names_map = {
    "Sh_x": "shoulder_x",
    "Sh_y": "shoulder_y",
    "Sh_z": "shoulder_z",
    "Elb_x": "elbow_x",
    "Elb_y": "elbow_y",
    "Wr_x": "wrist_x",
    "Wr_z": "wrist_z"
}

row = tk.Frame(joint_frame)
tk.Label(row, text="", width=6, font=("Consolas", 11, "bold")).pack(side=tk.LEFT)
tk.Label(
    row,
    text="Angle",
    width=12,
    anchor="w",
    font=("Consolas", 11, "underline"),
).pack(side=tk.LEFT)
tk.Label(
    row,
    text="Torque",
    width=12,
    anchor="w",
    font=("Consolas", 11, "underline"),
).pack(side=tk.LEFT)
row.pack(anchor='w')

for name, key in joint_names_map.items():
    sensor_vars[f"{key}_angle"] = tk.StringVar(value="--.- deg")
    sensor_vars[f"{key}_torque"] = tk.StringVar(value="--.- Nm")

    row = tk.Frame(joint_frame)
    tk.Label(row, text=f"{name}:", width=6, font=tk_font_bold).pack(side=tk.LEFT)
    tk.Label(
        row,
        textvariable=sensor_vars[f"{key}_angle"],
        width=12,
        anchor="w",
        font=tk_font,
    ).pack(side=tk.LEFT)
    tk.Label(
        row,
        textvariable=sensor_vars[f"{key}_torque"],
        width=10,
        anchor="w",
        font=tk_font,
    ).pack(side=tk.LEFT)
    row.pack(anchor='w')

# Hand orientation represented as a simulated IMU.
imu_frame = tk.Frame(sensor_window, padx=10, pady=10)
imu_frame.pack(fill='x')

tk.Label(imu_frame, text="--- IMU SENSOR (Hand) ---", font=tk_font_bold).pack(anchor='w')
sensor_vars["imu_roll"] = tk.StringVar(value="Roll:  --.-")
sensor_vars["imu_pitch"] = tk.StringVar(value="Pitch: --.-")
sensor_vars["imu_yaw"] = tk.StringVar(value="Yaw:   --.-")

tk.Label(imu_frame, textvariable=sensor_vars["imu_roll"], font=tk_font).pack(anchor='w')
tk.Label(imu_frame, textvariable=sensor_vars["imu_pitch"], font=tk_font).pack(anchor='w')
tk.Label(imu_frame, textvariable=sensor_vars["imu_yaw"], font=tk_font).pack(anchor='w')

# Values that approximate future EMG and pressure integrations.
bio_frame = tk.Frame(sensor_window, padx=10, pady=10)
bio_frame.pack(fill='x')

tk.Label(bio_frame, text="--- BIO-SENSORS (Simulated) ---", font=tk_font_bold).pack(anchor='w')
sensor_vars["emg_shoulder"] = tk.StringVar(value="EMG Shoulder: --.-")
sensor_vars["pressure_hand"] = tk.StringVar(value="Hand Pressure: N/A")

tk.Label(bio_frame, textvariable=sensor_vars["emg_shoulder"], font=tk_font).pack(anchor='w')
tk.Label(bio_frame, textvariable=sensor_vars["pressure_hand"], font=tk_font).pack(anchor='w')


# The Pygame HUD receives keyboard input and shows the detailed arm state.
print("Initializing Pygame interface...")
pygame.init()
screen = pygame.display.set_mode((700, 250))
pygame.display.set_caption("Arm Control (LIMB) - [ESC] to quit")
font = pygame.font.SysFont("Consolas", 16)
clock = pygame.time.Clock()
step_deg = CONTROL_STEP_DEG

print("\n--- Keyboard Controls ---")
print("Shoulder (Y/X/Z): Up/Down (Y) | Left/Right (X) | C/V (Z)")
print("Elbow: Z/S (flexion) | A/E (pronation)")
print("Wrist (flex/dev): Q/D (flexion) | W/X (deviation)")
print("Modifiers: Shift (fast), Ctrl (slow)")

# --- 6. Simulation loop ----------------------------------------------------
running = True
hud_visible = True
grasp_constraint = None
reach_line_id = -1
target_text_id = -1

camera_mode = "orbit"
default_cam_yaw = 45
default_cam_pitch = -30
default_cam_target = [0, 0, 0.5]

# Seed display values before the first frame. This also makes an immediate F or
# Tab key press safe before the first sensor-read pass has completed.
default_position = (0.0, 0.0, 0.0)
default_orientation = (0.0, 0.0, 0.0)
hand_position = default_position
wrist_position = default_position
elbow_position = default_position
imu_euler_rad = default_orientation
measured_angles_deg = {name: 0.0 for name in joint_indices}
torques = {name: 0.0 for name in joint_indices}


while running and p.isConnected():

    # Read the world positions before controls use them.
    try:
        target_position, _ = p.getBasePositionAndOrientation(target_body)
        robot_position, _ = p.getBasePositionAndOrientation(robot_body)
    except p.error:
        running = False
        break

    target_x_relative = target_position[0] - robot_position[0]
    target_y_relative = target_position[1] - robot_position[1]
    target_z_relative = target_position[2] - robot_position[2]
    target_relative_position = (target_x_relative, target_y_relative, target_z_relative)

    # Process window and camera events.
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_t:
                hud_visible = not hud_visible

            if event.key == pygame.K_TAB:
                if camera_mode == "orbit":
                    camera_mode = "shoulder"
                    print(">>> VIEW: Shoulder (Embedded) ACTIVATED.")
                else:
                    camera_mode = "orbit"
                    p.resetDebugVisualizerCamera(
                        cameraDistance=1.0,
                        cameraYaw=default_cam_yaw,
                        cameraPitch=default_cam_pitch,
                        cameraTargetPosition=default_cam_target,
                    )

    keys = pygame.key.get_pressed()
    if keys[pygame.K_ESCAPE]:
        running = False

    # Space resets the arm, target, and any active grasp.
    if keys[pygame.K_SPACE]:
        set_shoulder(arm, x=0, y=0, z=0, mode="abs")
        set_elbow(arm, x=0, y=0, mode="abs")
        set_wrist(arm, x=0, z=0, mode="abs")
        if p.isConnected():
            for joint_index in joint_indices.values():
                p.resetJointState(
                    bodyUniqueId=robot_body,
                    jointIndex=joint_index,
                    targetValue=0.0,
                )

        p.resetBasePositionAndOrientation(
            target_body,
            target_start_position,
            target_start_orientation,
        )
        p.resetBaseVelocity(target_body, [0, 0, 0], [0, 0, 0])

        if grasp_constraint is not None:
            p.removeConstraint(grasp_constraint)
            grasp_constraint = None
            p.changeDynamics(target_body, -1, mass=TARGET_NORMAL_MASS_KG)
            p.setCollisionFilterGroupMask(
                target_body,
                -1,
                collisionFilterGroup=1,
                collisionFilterMask=1,
            )
            print(">>> RESET (arm and target)")

    # H steers the shoulder, elbow, and wrist toward the target.
    if keys[pygame.K_h]:
        try:
            relative_x, relative_y, relative_z = target_relative_position
            horizontal_distance = math.hypot(relative_x, relative_y)
            target_shoulder_y, target_elbow_x = calculate_ik_angles(
                horizontal_distance,
                relative_z,
            )
            world_azimuth = math.atan2(relative_y, relative_x)
            target_shoulder_z = math.degrees(world_azimuth - math.pi / 2.0)
            while target_shoulder_z > 180:
                target_shoulder_z -= 360
            while target_shoulder_z <= -180:
                target_shoulder_z += 360

            speed = TARGET_CONTROL_STEP_DEG
            shoulder_y_difference = target_shoulder_y - arm.shoulder.angle_y
            elbow_difference = target_elbow_x - arm.elbow.angle_x
            shoulder_z_difference = target_shoulder_z - arm.shoulder.angle_z
            if shoulder_z_difference > 180:
                shoulder_z_difference -= 360
            elif shoulder_z_difference < -180:
                shoulder_z_difference += 360

            shoulder_x_difference = -arm.shoulder.angle_x
            wrist_x_difference = -arm.wrist.angle_x
            set_shoulder(
                arm,
                x=arm.shoulder.angle_x
                + clamp(shoulder_x_difference, -speed, speed),
                y=arm.shoulder.angle_y
                + clamp(shoulder_y_difference, -speed, speed),
                z=arm.shoulder.angle_z
                + clamp(shoulder_z_difference, -speed, speed),
                mode="abs",
            )
            set_elbow(
                arm,
                x=arm.elbow.angle_x + clamp(elbow_difference, -speed, speed),
                mode="abs",
            )
            set_wrist(
                arm,
                x=arm.wrist.angle_x + clamp(wrist_x_difference, -speed, speed),
                mode="abs",
            )
        except ValueError as error:
            print(f"Target cannot be reached: {error}")

    # F curls the fingers and grasps a nearby target; G releases it.
    delta_hand = (0.05 if keys[pygame.K_f] else 0) + (-0.05 if keys[pygame.K_g] else 0)
    if delta_hand:
        new_value = arm.hand.curl + delta_hand
        arm.hand.curl = clamp(new_value, arm.hand.min_curl, arm.hand.max_curl)

    if keys[pygame.K_f] and grasp_constraint is None and hand_link_index != -1:
        dx = target_position[0] - hand_position[0]
        dy = target_position[1] - hand_position[1]
        dz = target_position[2] - hand_position[2]
        contact_dist = math.sqrt(dx**2 + dy**2 + dz**2)

        if contact_dist < TARGET_GRASP_DISTANCE_M:
            print(">>> GRIP ACTIVATED")
            hand_state = p.getLinkState(robot_body, hand_link_index)
            hand_pos_world = hand_state[0]
            hand_orn_world = hand_state[1]
            target_position_world, target_orientation_world = (
                p.getBasePositionAndOrientation(target_body)
            )
            inverse_hand_position, inverse_hand_orientation = p.invertTransform(
                hand_pos_world,
                hand_orn_world,
            )
            target_position_in_hand, target_orientation_in_hand = p.multiplyTransforms(
                inverse_hand_position,
                inverse_hand_orientation,
                target_position_world,
                target_orientation_world,
            )

            # Reduce target mass and disable its collisions while constrained.
            p.changeDynamics(target_body, -1, mass=TARGET_GRASPED_MASS_KG)
            p.setCollisionFilterGroupMask(
                target_body,
                -1,
                collisionFilterGroup=0,
                collisionFilterMask=0,
            )

            grasp_constraint = p.createConstraint(
                parentBodyUniqueId=robot_body,
                parentLinkIndex=hand_link_index,
                childBodyUniqueId=target_body,
                childLinkIndex=-1,
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=target_position_in_hand,
                parentFrameOrientation=target_orientation_in_hand,
                childFramePosition=[0, 0, 0],
                childFrameOrientation=[0, 0, 0, 1],
            )
            p.changeConstraint(grasp_constraint, maxForce=200)

    if keys[pygame.K_g] and grasp_constraint is not None:
        print(">>> RELEASE")
        p.removeConstraint(grasp_constraint)
        grasp_constraint = None

        p.changeDynamics(target_body, -1, mass=TARGET_NORMAL_MASS_KG)
        p.setCollisionFilterGroupMask(
            target_body,
            -1,
            collisionFilterGroup=1,
            collisionFilterMask=1,
        )
        p.resetBaseVelocity(target_body, linearVelocity=[0, 0, -0.2])



    # Speed
    speed_mult = 1.0
    if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
        speed_mult = 4.0
    if keys[pygame.K_LCTRL] or keys[pygame.K_RCTRL]:
        speed_mult = 0.25
    step = step_deg * speed_mult

    # Shoulder
    delta_should_y = (step if keys[pygame.K_UP] else 0) + (-step if keys[pygame.K_DOWN] else 0)
    delta_should_x = (-step if keys[pygame.K_RIGHT] else 0) + (step if keys[pygame.K_LEFT] else 0)
    delta_should_z = (step if keys[pygame.K_c] else 0) + (-step if keys[pygame.K_v] else 0)
    if delta_should_x or delta_should_y or delta_should_z:
        set_shoulder(arm, x=delta_should_x, y=delta_should_y, z=delta_should_z, mode="rel")

    # Elbow (flexion and pronation)
    delta_elbow_x = (step if keys[pygame.K_z] else 0) + (-step if keys[pygame.K_s] else 0)
    delta_elbow_y = (step if keys[pygame.K_a] else 0) + (-step if keys[pygame.K_e] else 0)
    if delta_elbow_x or delta_elbow_y:
        set_elbow(arm, x=delta_elbow_x, y=delta_elbow_y, mode="rel")

    # Wrist
    delta_wrist_x = (step if keys[pygame.K_q] else 0) + (-step if keys[pygame.K_d] else 0)
    delta_wrist_z = (step if keys[pygame.K_w] else 0) + (-step if keys[pygame.K_x] else 0)
    if delta_wrist_x or delta_wrist_z:
        set_wrist(arm, x=delta_wrist_x, z=delta_wrist_z, mode="rel")

    # --- B. Synchronization and Physics ---
    if not p.isConnected():
        running = False
        continue

    sync_to_pybullet(
        arm=arm,
        body_id=robot_body,
        joint_name_to_index=joint_indices,
        client=p,
        use_motors=True
    )

    # Shoulder mode tracks the arm's rotation from a close camera position.
    if camera_mode == "shoulder":
        base_position, _ = p.getBasePositionAndOrientation(robot_body)
        camera_target = [
            base_position[0] - 0.05,
            base_position[1] + 0.05,
            base_position[2] + 0.05,
        ]
        camera_yaw = measured_angles_deg.get("shoulder_z", 0.0)
        camera_pitch = -15 - measured_angles_deg.get("shoulder_y", 0.0)
        p.resetDebugVisualizerCamera(
            cameraDistance=0.2,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
            cameraTargetPosition=camera_target,
        )

    p.stepSimulation()

    # Read the simulated sensors after stepping physics.
    torques = {}
    measured_angles_deg = {}
    default_torque = 0.0

    try:
        for name, index in joint_indices.items():
            state = p.getJointState(robot_body, index)
            measured_angles_deg[name] = rad_to_deg(state[0])
            torques[name] = state[3]
    except p.error:
        for name in joint_indices:
            torques[name] = default_torque
            measured_angles_deg[name] = 0.0

    try:
        if hand_link_index != -1:
            hand_state = p.getLinkState(robot_body, hand_link_index)
            hand_position = hand_state[0]
            imu_quaternion = hand_state[1]
            imu_euler_rad = p.getEulerFromQuaternion(imu_quaternion)
        else:
            hand_position = default_position
            imu_euler_rad = default_orientation

        wrist_position = (
            p.getLinkState(robot_body, forearm_link_index)[0]
            if forearm_link_index != -1
            else default_position
        )
        elbow_position = (
            p.getLinkState(robot_body, upperarm_link_index)[0]
            if upperarm_link_index != -1
            else default_position
        )
    except p.error:
        hand_position = wrist_position = elbow_position = default_position
        imu_euler_rad = default_orientation

    # Draw the Pygame control and sensor HUD.
    screen.fill((18, 18, 18))

    text_lines = [
        (
            "Shoulder (x,y,z): "
            f"{measured_angles_deg.get('shoulder_x', 0):.1f}, "
            f"{measured_angles_deg.get('shoulder_y', 0):.1f}, "
            f"{measured_angles_deg.get('shoulder_z', 0):.1f} deg"
        ),
        (
            "Elbow (x,y):      "
            f"{measured_angles_deg.get('elbow_x', 0):.1f}, "
            f"{measured_angles_deg.get('elbow_y', 0):.1f} deg"
        ),
        (
            "Wrist (x,z):      "
            f"{measured_angles_deg.get('wrist_x', 0):.1f}, "
            f"{measured_angles_deg.get('wrist_z', 0):.1f} deg"
        ),
        "---",
        (
            "Torque Sh(x,y,z): "
            f"{torques.get('shoulder_x', 0):.2f}, "
            f"{torques.get('shoulder_y', 0):.2f}, "
            f"{torques.get('shoulder_z', 0):.2f} Nm"
        ),
        (
            "Torque Elb(x,y):  "
            f"{torques.get('elbow_x', 0):.2f}, "
            f"{torques.get('elbow_y', 0):.2f} | Wr(x,z): "
            f"{torques.get('wrist_x', 0):.2f}, "
            f"{torques.get('wrist_z', 0):.2f} Nm"
        ),
        "---",
        f"Elbow position: x={elbow_position[0]:.3f} y={elbow_position[1]:.3f} z={elbow_position[2]:.3f}",
        f"Wrist position: x={wrist_position[0]:.3f} y={wrist_position[1]:.3f} z={wrist_position[2]:.3f}",
        f"Hand position:  x={hand_position[0]:.3f} y={hand_position[1]:.3f} z={hand_position[2]:.3f}",
        "---",
        "Keys: Arrows+C/V | Elbow Z/S,A/E | Wrist Q/D,W/X | Space reset | ESC quit",
    ]
    y = 10
    for line in text_lines:
        surf = font.render(line, True, (220, 220, 220))
        screen.blit(surf, (10, y))
        y += 18

    target_is_reachable, reachability_message = is_reachable(target_relative_position)
    if target_is_reachable:
        status_color = (0, 255, 0)
        status_text = f"REACHABLE: {reachability_message}"
    else:
        status_color = (255, 0, 0)
        status_text = f"UNREACHABLE: {reachability_message}"

    surf_status = font.render(status_text, True, status_color)
    screen.blit(surf_status, (10, y + 10))

    # Draw a short-lived reach line and distance label in PyBullet.
    if hud_visible and hand_link_index != -1:
        dx = target_position[0] - hand_position[0]
        dy = target_position[1] - hand_position[1]
        dz = target_position[2] - hand_position[2]
        hand_target_distance = math.sqrt(dx**2 + dy**2 + dz**2)

        reach_line_id = p.addUserDebugLine(
            hand_position,
            target_position,
            lineColorRGB=[1, 0, 0],
            lineWidth=2,
            replaceItemUniqueId=reach_line_id,
        )
        target_text_id = p.addUserDebugText(
            f"TARGET: CUP_ALPHA [DIST: {hand_target_distance:.2f}m]",
            [target_position[0], target_position[1], target_position[2] + 0.15],
            textColorRGB=[1, 1, 0],
            textSize=1.2,
            replaceItemUniqueId=target_text_id,
        )
    elif reach_line_id >= 0:
        p.removeUserDebugItem(reach_line_id)
        p.removeUserDebugItem(target_text_id)
        reach_line_id = -1
        target_text_id = -1

    pygame.display.flip()

    # Mirror sensor readings in the separate Tkinter dashboard.
    try:
        for key in joint_names_map.values():
            sensor_vars[f"{key}_angle"].set(f"{measured_angles_deg.get(key, 0):.1f} deg")
            sensor_vars[f"{key}_torque"].set(f"{torques.get(key, 0):.2f} Nm")

        # Update IMU
        sensor_vars["imu_roll"].set(f"Roll:  {rad_to_deg(imu_euler_rad[0]):.1f}")
        sensor_vars["imu_pitch"].set(f"Pitch: {rad_to_deg(imu_euler_rad[1]):.1f}")
        sensor_vars["imu_yaw"].set(f"Yaw:   {rad_to_deg(imu_euler_rad[2]):.1f}")

        simulated_emg = (
            abs(torques.get("shoulder_x", 0))
            + abs(torques.get("shoulder_y", 0))
            + abs(torques.get("shoulder_z", 0))
        ) / 3
        sensor_vars["emg_shoulder"].set(
            f"EMG Shoulder: {simulated_emg:.2f} (Sim-Torque)"
        )
        pressure_status = (
            "CONTACT (simulated)"
            if grasp_constraint is not None
            else "N/A (No Object)"
        )
        sensor_vars["pressure_hand"].set(f"Hand Pressure: {pressure_status}")

        sensor_window.update()

    except tk.TclError:
        print("Sensor window closed.")
        running = False

    clock.tick(SIMULATION_FREQUENCY_HZ)


# --- 7. Shutdown -----------------------------------------------------------
print("Simulation finished.")
try:
    sensor_window.destroy()
except tk.TclError:
    pass

pygame.quit()
if p.isConnected():
    p.disconnect()
