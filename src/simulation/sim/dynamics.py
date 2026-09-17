"""Reusable PyBullet kinematics and rigid-body dynamics for the right LIMB arm.

The five arm joints are a subset of the full model, which also has articulated
finger joints. Arrays here always follow PyBullet's full movable-joint order.
The optional payload is a point mass at the grip center, not a CAD model.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np
import pybullet as p

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim.robot_model import (
    RIGHT_ARM_JOINTS, RIGHT_GRIP_OFFSET_M, RIGHT_HAND_LINK,
    inspect_model, link_name_index, load_right_arm,
    moving_joint_indices,
)


class ArmDynamics:
    """Query the loaded URDF without changing its joints or motor commands."""

    def __init__(self, body_id: int, client=p, gravity=(0.0, 0.0, -9.81)):
        self.client = client
        self.body_id = body_id
        self.gravity = np.asarray(gravity, dtype=float)
        if self.gravity.shape != (3,) or not np.all(np.isfinite(self.gravity)):
            raise ValueError("gravity must contain three finite components")
        client.setGravity(*self.gravity)
        self.joints = moving_joint_indices(body_id, client)
        self.names = tuple(
            client.getJointInfo(body_id, index)[1].decode("utf-8")
            for index in self.joints
        )
        if not all(name in self.names for name in RIGHT_ARM_JOINTS):
            raise ValueError("Loaded model is missing a right-arm joint")
        self.hand_link = link_name_index(body_id, client)[RIGHT_HAND_LINK]

    @property
    def dof(self) -> int:
        return len(self.joints)

    def _vector(self, values, name: str) -> np.ndarray:
        result = np.asarray(values, dtype=float)
        if result.shape != (self.dof,) or not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must contain {self.dof} finite values")
        return result

    def joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        states = self.client.getJointStates(self.body_id, self.joints)
        return (
            np.array([state[0] for state in states], dtype=float),
            np.array([state[1] for state in states], dtype=float),
            np.array([state[3] for state in states], dtype=float),
        )

    def _grip_in_jacobian_frame(self):
        state = self.client.getLinkState(self.body_id, self.hand_link)
        # calculateJacobian's zero point is the link origin. Its local offset
        # is rotated by the link's inertial orientation, so undo that rotation
        # without subtracting the inertial translation.
        inverse = self.client.invertTransform((0.0, 0.0, 0.0), state[3])
        local_point, _ = self.client.multiplyTransforms(
            *inverse, RIGHT_GRIP_OFFSET_M, (0.0, 0.0, 0.0, 1.0)
        )
        return local_point

    def jacobian(self, q) -> tuple[np.ndarray, np.ndarray]:
        """World-frame linear and angular Jacobians at the grip center."""
        q = self._vector(q, "q")
        zeros = [0.0] * self.dof
        linear, angular = self.client.calculateJacobian(
            self.body_id, self.hand_link, self._grip_in_jacobian_frame(),
            q.tolist(), zeros, zeros,
        )
        return np.asarray(linear, dtype=float), np.asarray(angular, dtype=float)

    def mass_matrix(self, q, payload_kg: float = 0.0) -> np.ndarray:
        q = self._vector(q, "q")
        self._check_payload(payload_kg)
        matrix = np.asarray(self.client.calculateMassMatrix(self.body_id, q.tolist()), dtype=float)
        if payload_kg:
            linear, _ = self.jacobian(q)
            matrix += payload_kg * (linear.T @ linear)
        return matrix

    @staticmethod
    def _check_payload(payload_kg: float) -> None:
        if not np.isfinite(payload_kg) or payload_kg < 0:
            raise ValueError("payload_kg must be finite and nonnegative")

    def inverse_dynamics(self, q, qd, qdd, payload_kg: float = 0.0) -> np.ndarray:
        q = self._vector(q, "q")
        qd = self._vector(qd, "qd")
        qdd = self._vector(qdd, "qdd")
        self._check_payload(payload_kg)
        torque = np.asarray(self.client.calculateInverseDynamics(
            self.body_id, q.tolist(), qd.tolist(), qdd.tolist()
        ), dtype=float)
        if payload_kg:
            linear, _ = self.jacobian(q)
            if np.any(qd):
                delta = 1e-5 / max(1.0, float(np.linalg.norm(qd)))
                linear_next, _ = self.jacobian(q + delta * qd)
                jdot_qd = ((linear_next - linear) / delta) @ qd
            else:
                jdot_qd = np.zeros(3)
            torque += payload_kg * linear.T @ (linear @ qdd + jdot_qd - self.gravity)
        return torque

    def terms(self, q, qd, qdd, payload_kg: float = 0.0) -> dict[str, np.ndarray]:
        q = self._vector(q, "q")
        qd = self._vector(qd, "qd")
        qdd = self._vector(qdd, "qdd")
        zeros = np.zeros(self.dof)
        gravity = self.inverse_dynamics(q, zeros, zeros, payload_kg)
        velocity = self.inverse_dynamics(q, qd, zeros, payload_kg) - gravity
        mass = self.mass_matrix(q, payload_kg)
        inverse = self.inverse_dynamics(q, qd, qdd, payload_kg)
        return {
            "mass_matrix": mass,
            "gravity_torque": gravity,
            "velocity_torque": velocity,
            "inverse_dynamics_torque": inverse,
            "reconstructed_torque": mass @ qdd + velocity + gravity,
        }

    def end_effector_state(self) -> dict[str, list[float]]:
        state = self.client.getLinkState(
            self.body_id, self.hand_link, computeLinkVelocity=1,
            computeForwardKinematics=1,
        )
        position, orientation = self.client.multiplyTransforms(
            state[4], state[5], RIGHT_GRIP_OFFSET_M, (0.0, 0.0, 0.0, 1.0)
        )
        offset = np.asarray(position) - np.asarray(state[0])
        velocity = np.asarray(state[6]) + np.cross(np.asarray(state[7]), offset)
        return {"position_m": list(position), "orientation_xyzw": list(orientation),
                "velocity_m_s": velocity.tolist()}

    def snapshot(self, commanded_positions=None, commanded_torques=None,
                 payload_kg: float = 0.0, time_s: float = 0.0) -> dict[str, object]:
        """Return structured state and diagnostics for a GUI or JSON logger."""
        q, qd, measured_motor_torque = self.joint_state()
        zeros = np.zeros(self.dof)
        terms = self.terms(q, qd, zeros, payload_kg)
        linear, angular = self.jacobian(q)
        return {
            "time_s": float(time_s), "joint_names": list(self.names),
            "arm_joint_names": list(RIGHT_ARM_JOINTS),
            "q_rad": q.tolist(), "qd_rad_s": qd.tolist(),
            "commanded_q_rad": None if commanded_positions is None else self._vector(
                commanded_positions, "commanded_positions").tolist(),
            "commanded_torque_Nm": None if commanded_torques is None else self._vector(
                commanded_torques, "commanded_torques").tolist(),
            "motor_torque_Nm": measured_motor_torque.tolist(),
            "end_effector": self.end_effector_state(),
            "jacobian_linear": linear.tolist(), "jacobian_angular": angular.tolist(),
            "mass_matrix_kg_m2": terms["mass_matrix"].tolist(),
            "gravity_torque_Nm": terms["gravity_torque"].tolist(),
            "velocity_torque_Nm": terms["velocity_torque"].tolist(),
            "inverse_dynamics_torque_Nm": terms["inverse_dynamics_torque"].tolist(),
            "payload_kg": payload_kg,
        }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inspect", action="store_true", help="Include every joint and link in JSON output")
    parser.add_argument("--payload-kg", type=float, default=0.0,
                        help="Point mass carried at the hand grip center")
    args = parser.parse_args()
    connection = p.connect(p.DIRECT)
    if connection < 0:
        raise RuntimeError("Could not connect to PyBullet")
    try:
        p.setGravity(0, 0, -9.81)
        robot = load_right_arm(p)
        dynamics = ArmDynamics(robot, p)
        result = dynamics.snapshot(payload_kg=args.payload_kg)
        if args.inspect:
            result["model"] = inspect_model(robot, p)
        print(json.dumps(result, indent=2))
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
