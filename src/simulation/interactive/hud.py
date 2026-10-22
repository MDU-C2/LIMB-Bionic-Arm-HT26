# Majd, Christian
"""Pygame controls display and target guide."""

from __future__ import annotations

import math
import os

import pybullet as p
import pygame

from .arm_control import (
    ARM_JOINT_NAMES,
    hardware_value,
)
from .kinematics import is_reachable
from .settings import (
    MAX_GRASP_CURL,
)


class HudMixin:
    """Draw the Pygame controller and scene guide."""

    def initialize_hud(self) -> None:
        """Open the keyboard controller window."""
        print("Initializing Pygame interface...")
        os.environ.setdefault("SDL_VIDEO_WINDOW_POS", "540,35")
        pygame.init()
        self.screen = pygame.display.set_mode((780, 370))
        pygame.display.set_caption("LIMB Arm Controller - click here to drive")
        self.font_title = pygame.font.SysFont("Segoe UI", 22, bold=True)
        self.font_heading = pygame.font.SysFont("Segoe UI", 15, bold=True)
        self.font = pygame.font.SysFont("Segoe UI", 14)
        self.font_small = pygame.font.SysFont("Segoe UI", 12)
        self.clock = pygame.time.Clock()
        print("\n--- Keyboard Controls ---")
        print("Shoulder: W/S and A/D | Upper arm: Q/E")
        print("Elbow: Up/Down | Wrist: Left/Right | Fingers: F/G")
        print(
            "Space interact | H auto reach | R pause/reset | L/K next setup step | C camera | Esc quit"
        )

    def draw_text(self, text: str, position, color, text_font=None) -> None:
        text_font = self.font if text_font is None else text_font
        self.screen.blit(text_font.render(text, True, color), position)

    def draw_panel(self, rect, heading: str) -> None:
        pygame.draw.rect(self.screen, (29, 36, 49), rect, border_radius=8)
        pygame.draw.rect(self.screen, (55, 68, 88), rect, width=1, border_radius=8)
        self.draw_text(heading, (rect.x + 14, rect.y + 10), (242, 246, 252), self.font_heading)

    def draw_control_row(self, y: int, keys_text: str, label: str, value: str, x: int = 26) -> None:
        key_rect = pygame.Rect(x, y, 76, 24)
        pygame.draw.rect(self.screen, (50, 104, 216), key_rect, border_radius=5)
        key_surface = self.font_small.render(keys_text, True, (255, 255, 255))
        self.screen.blit(key_surface, key_surface.get_rect(center=key_rect.center))
        self.draw_text(label, (x + 87, y + 3), (207, 216, 230), self.font_small)
        self.draw_text(value, (x + 246, y + 3), (121, 214, 168), self.font_small)

    def draw_hud(self) -> None:
        """Show controls, target reachability, and task notices."""
        self.hardware_angles_deg = {
            name: hardware_value(name, self.measured_angles_deg.get(name, 0.0))
            for name in ARM_JOINT_NAMES
        }

        dx = self.target_position[0] - self.hand_position[0]
        dy = self.target_position[1] - self.hand_position[1]
        dz = self.target_position[2] - self.hand_position[2]
        self.hand_target_distance = math.sqrt(dx**2 + dy**2 + dz**2)
        target_is_reachable, reachability_message = is_reachable(self.target_relative_position)
        if target_is_reachable:
            status_color = (121, 214, 168)
            status_text = f"Target reachable - hand is {self.hand_target_distance:.2f} m away"
        else:
            status_color = (255, 119, 119)
            status_text = reachability_message.title()

        self.screen.fill((15, 21, 31))
        self.draw_text("LIMB Arm Controller", (16, 10), (245, 248, 252), self.font_title)
        control_label = "LIVE CAMERA + IMU" if self.args.control == "camera-imu" else "KEYBOARD"
        self.draw_text(
            f"{self.args.mode.title()} mode | {control_label}",
            (17, 39),
            (151, 164, 183),
            self.font_small,
        )
        camera_label = f"CAMERA  {self.camera_mode.upper()}"
        camera_badge = pygame.Rect(608, 14, 154, 28)
        pygame.draw.rect(self.screen, (38, 55, 79), camera_badge, border_radius=14)
        camera_surface = self.font_small.render(camera_label, True, (177, 207, 255))
        self.screen.blit(camera_surface, camera_surface.get_rect(center=camera_badge.center))

        self.draw_panel(pygame.Rect(16, 64, 366, 220), "Move the arm")
        self.draw_control_row(
            98,
            "W / S",
            "Shoulder up / down",
            f"{self.hardware_angles_deg['shoulder_y']:.1f} deg",
        )
        self.draw_control_row(
            133,
            "A / D",
            "Shoulder left / right",
            f"{self.hardware_angles_deg['shoulder_z']:.1f} deg",
        )
        self.draw_control_row(
            168,
            "Q / E",
            "Upper-arm rotation",
            f"{self.hardware_angles_deg['shoulder_x']:.1f} deg",
        )
        self.draw_control_row(
            203,
            "UP / DOWN",
            "Elbow bend / extend",
            f"{self.hardware_angles_deg['elbow_x']:.1f} deg",
        )
        self.draw_control_row(
            238,
            "LEFT / RIGHT",
            "Wrist rotation",
            f"{self.hardware_angles_deg['wrist_rotation']:.1f} deg",
        )

        self.draw_panel(pygame.Rect(398, 64, 366, 220), "Actions and view")
        action_x = 408
        self.draw_control_row(
            98,
            "F / G",
            "Close / open fingers",
            f"{self.arm.hand.curl / MAX_GRASP_CURL * 100:.0f}%",
            action_x,
        )
        self.draw_control_row(
            133,
            "SPACE",
            "Grab / release",
            "holding"
            if self.grasp_transform is not None
            else self.auto_grasp_phase
            if self.auto_grasp_pending
            else "ready",
            action_x,
        )
        self.draw_control_row(
            168,
            "H",
            "Safe approach",
            "stopped" if self.h_contact_stop else "reach",
            action_x,
        )
        self.draw_control_row(203, "C / 1-3", "Change camera", self.camera_mode, action_x)
        self.draw_control_row(
            238,
            "R / L" if self.live_sensor_input is not None else "R",
            "Align / start live" if self.live_sensor_input is not None else "Reset scene",
            "active" if self.live_feedback_active and self.live_sensor_input is not None else "",
            action_x,
        )

        pygame.draw.rect(self.screen, (24, 31, 43), pygame.Rect(16, 296, 748, 58), border_radius=8)
        self.draw_text(status_text, (30, 305), status_color, self.font)
        current_notice = self.notice_text if pygame.time.get_ticks() < self.notice_until_ms else ""
        if not pygame.key.get_focused():
            current_notice = "Click here for controls; task keys also work in the 3D scene"
        elif not current_notice:
            current_notice = (
                "R pause/reset | L/K next setup step | P previews | I sensors | Esc quit"
                if self.live_sensor_input is not None
                else "Ctrl precise | T guide | P previews | I sensors | Esc quit"
            )
        self.draw_text(current_notice, (30, 331), (164, 178, 198), self.font_small)

        if self.hud_visible and self.hand_link_index != -1:
            self.reach_line_id = p.addUserDebugLine(
                self.hand_position,
                self.target_position,
                lineColorRGB=[1, 0, 0],
                lineWidth=2,
                replaceItemUniqueId=self.reach_line_id,
            )
            self.target_text_id = p.addUserDebugText(
                f"TARGET: CUP_ALPHA [DIST: {self.hand_target_distance:.2f}m]",
                [self.target_position[0], self.target_position[1], self.target_position[2] + 0.15],
                textColorRGB=[1, 1, 0],
                textSize=1.2,
                replaceItemUniqueId=self.target_text_id,
            )
        elif self.reach_line_id >= 0:
            p.removeUserDebugItem(self.reach_line_id)
            p.removeUserDebugItem(self.target_text_id)
            self.reach_line_id = -1
            self.target_text_id = -1

        pygame.display.flip()
