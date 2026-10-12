# Christian, Majd
"""Validate the migrated URDF against PyBullet's geometry and dynamics."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np
import pybullet as p

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "simulation"))

from sim.dynamics import ArmDynamics
from sim.robot_model import (
    RIGHT_GRIP_OFFSET_M,
    load_right_arm,
)


class SimulationPhysicsCase(unittest.TestCase):
    """Shared headless arm setup and pose helpers."""

    def setUp(self) -> None:
        self.client_id = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81)
        self.robot = load_right_arm(p)
        self.dynamics = ArmDynamics(self.robot, p)

    def tearDown(self) -> None:
        p.disconnect(self.client_id)

    def pose(self, shoulder_y=0.35, elbow=-0.4) -> np.ndarray:
        q = np.zeros(self.dynamics.dof)
        q[:5] = [-0.30, shoulder_y, 0.10, elbow, 0.50]
        return q

    def reset_pose(self, q) -> None:
        for joint, angle in zip(self.dynamics.joints, q, strict=False):
            p.resetJointState(self.robot, joint, float(angle))

    def tip_state(self):
        state = p.getLinkState(self.robot, self.dynamics.hand_link, computeForwardKinematics=1)
        return p.multiplyTransforms(state[4], state[5], RIGHT_GRIP_OFFSET_M, (0, 0, 0, 1))
