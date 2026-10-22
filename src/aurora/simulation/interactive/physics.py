# Majd, Christian
"""Simulation steps, measured state, and torque telemetry."""

from __future__ import annotations

import json
import math

import pybullet as p

from aurora.simulation.interactive.arm_control import (
    LOGICAL_TO_URDF_JOINT,
    get_angles_rad,
    rad_to_deg,
    sync_to_pybullet,
)
from aurora.simulation.interactive.settings import (
    HAND_GRIP_OFFSET_M,
    SIMULATION_FREQUENCY_HZ,
    TARGET_NORMAL_MASS_KG,
)
from aurora.simulation.model.controller_params import ARM_ACTUATOR_INFO, ARM_MOTOR_FORCE_NM


class PhysicsMixin:
    """Advance physics and save the selected telemetry."""

    def advance_physics(self) -> bool:
        """Step the arm, measure pose/torque, and follow a held cup."""
        if not p.isConnected():
            self.running = False
            return False

        if self.target_collision_pending and not p.getClosestPoints(
            self.robot_body, self.target_body, distance=0.002
        ):
            self.set_target_robot_collision(True)
            self.target_collision_pending = False

        if self.dynamic_mode:
            sync_to_pybullet(
                arm=self.arm,
                body_id=self.robot_body,
                joint_name_to_index=self.joint_indices,
                client=p,
                use_motors=True,
            )
            physics_steps = max(
                1,
                min(6, round(self.loop_dt_s * SIMULATION_FREQUENCY_HZ)),
            )
            for _ in range(physics_steps):
                p.stepSimulation()
        else:
            # Direct preview keeps the light finger links visually stable.
            p.stepSimulation()
            sync_to_pybullet(
                arm=self.arm,
                body_id=self.robot_body,
                joint_name_to_index=self.joint_indices,
                client=p,
                use_motors=False,
            )
            p.performCollisionDetection()

        self.measured_angles_deg = {}

        try:
            for name, index in self.joint_indices.items():
                state = p.getJointState(self.robot_body, index)
                self.measured_angles_deg[name] = rad_to_deg(state[0])
                self.measured_motor_torque[name] = state[3] if self.dynamic_mode else 0.0
        except p.error:
            for name in self.joint_indices:
                self.measured_angles_deg[name] = 0.0
                self.measured_motor_torque[name] = 0.0

        self.frame_number += 1
        if self.dynamic_mode:
            self.torque_graph.append(self.measured_motor_torque)
        if self.frame_number % 6 == 0:
            self.torque_graph.redraw(self.dynamic_mode)
        if self.dynamic_mode and self.frame_number % 12 == 0:
            commanded_q, _, _ = self.dynamics.joint_state()
            column_for_joint = {joint: column for column, joint in enumerate(self.dynamics.joints)}
            for name, angle in get_angles_rad(self.arm).items():
                joint = self.joint_indices.get(name)
                if joint in column_for_joint:
                    commanded_q[column_for_joint[joint]] = angle
            self.physics_snapshot = self.dynamics.snapshot(
                commanded_positions=commanded_q,
                payload_kg=TARGET_NORMAL_MASS_KG if self.grasp_constraint is not None else 0.0,
                time_s=self.frame_number / SIMULATION_FREQUENCY_HZ,
            )
            if self.telemetry_file is not None:
                self.telemetry_file.write(json.dumps(self.physics_snapshot) + "\n")
                self.telemetry_file.flush()
            if self.torque_csv_writer is not None:
                snapshot_columns = {
                    name: column for column, name in enumerate(self.physics_snapshot["joint_names"])
                }
                for logical_name in self.joint_names_map.values():
                    urdf_name = LOGICAL_TO_URDF_JOINT[logical_name]
                    column = snapshot_columns[urdf_name]
                    actuator = ARM_ACTUATOR_INFO[logical_name]
                    self.torque_csv_writer.writerow(
                        {
                            "time_s": f"{self.physics_snapshot['time_s']:.6f}",
                            "logical_joint": logical_name,
                            "movement": actuator["movement"],
                            "urdf_joint": urdf_name,
                            "motor_model": actuator["model"],
                            "mapping_source": actuator["mapping"],
                            "published_motor_rating": actuator["published_rating"],
                            "angle_deg": f"{math.degrees(self.physics_snapshot['q_rad'][column]):.6f}",
                            "velocity_deg_s": (
                                f"{math.degrees(self.physics_snapshot['qd_rad_s'][column]):.6f}"
                            ),
                            "applied_motor_torque_Nm": (
                                f"{self.physics_snapshot['motor_torque_Nm'][column]:.6f}"
                            ),
                            "gravity_torque_Nm": (
                                f"{self.physics_snapshot['gravity_torque_Nm'][column]:.6f}"
                            ),
                            "inverse_dynamics_torque_Nm": (
                                f"{self.physics_snapshot['inverse_dynamics_torque_Nm'][column]:.6f}"
                            ),
                            "simulation_effort_limit_Nm": f"{ARM_MOTOR_FORCE_NM[logical_name]:.6f}",
                            "payload_kg": f"{self.physics_snapshot['payload_kg']:.6f}",
                        }
                    )
                self.torque_csv_file.flush()

        try:
            if self.hand_link_index != -1:
                hand_state = p.getLinkState(self.robot_body, self.hand_link_index)
                self.hand_position, _ = p.multiplyTransforms(
                    hand_state[4],
                    hand_state[5],
                    HAND_GRIP_OFFSET_M,
                    (0.0, 0.0, 0.0, 1.0),
                )
                imu_quaternion = hand_state[5]
                self.imu_euler_rad = p.getEulerFromQuaternion(imu_quaternion)
            else:
                self.hand_position = self.default_position
                self.imu_euler_rad = self.default_orientation

        except p.error:
            self.hand_position = self.default_position
            self.imu_euler_rad = self.default_orientation

        if (
            not self.dynamic_mode
            and self.grasp_transform is not None
            and self.hand_link_index != -1
        ):
            hand_state = p.getLinkState(self.robot_body, self.hand_link_index)
            self.target_position, target_orientation = p.multiplyTransforms(
                hand_state[4], hand_state[5], *self.grasp_transform
            )
            p.resetBasePositionAndOrientation(
                self.target_body, self.target_position, target_orientation
            )
        return True
