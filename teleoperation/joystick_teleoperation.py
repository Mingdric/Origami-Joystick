#!/usr/bin/env python3
"""Joystick teleoperation of a Franka robot with contact-triggered haptics.

Handle data arrives over UDP 127.0.0.1:9090 and is classified into twelve motion
directions. The bridge sends velocity commands to the robot controller on UDP
127.0.0.1:9999 and receives force/contact telemetry on UDP port 10000.

A contact rising edge blocks handle commands and sends zero velocity. After
the pre-block interval, the bridge requests an ESP32 pulse, waits for the pulse
interval, and requires continuous valid neutral input before restoring control.
ESP32 firmware implements the physical valve pulse.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import sys
import time
from pathlib import Path

from modal_classifier import ModalClassifier


LABEL_TO_UNIT = {
    "X+":  ( 1.0,  0.0,  0.0,  0.0,  0.0,  0.0),
    "X-":  (-1.0,  0.0,  0.0,  0.0,  0.0,  0.0),
    "Y+":  ( 0.0,  1.0,  0.0,  0.0,  0.0,  0.0),
    "Y-":  ( 0.0, -1.0,  0.0,  0.0,  0.0,  0.0),
    "Z+":  ( 0.0,  0.0,  1.0,  0.0,  0.0,  0.0),
    "Z-":  ( 0.0,  0.0, -1.0,  0.0,  0.0,  0.0),
    "RX+": ( 0.0,  0.0,  0.0,  1.0,  0.0,  0.0),
    "RX-": ( 0.0,  0.0,  0.0, -1.0,  0.0,  0.0),
    "RY+": ( 0.0,  0.0,  0.0,  0.0,  1.0,  0.0),
    "RY-": ( 0.0,  0.0,  0.0,  0.0, -1.0,  0.0),
    "RZ+": ( 0.0,  0.0,  0.0,  0.0,  0.0,  1.0),
    "RZ-": ( 0.0,  0.0,  0.0,  0.0,  0.0, -1.0),
    "IDLE": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
}

PHASE_TEXT = {
    0: "CALIBRATING",
    1: "IDLE",
    2: "ACCEL",
    3: "FREE",
    4: "CONTACT",
    5: "RELEASE",
}

HAPTIC_NORMAL = "NORMAL"
HAPTIC_PRE_BLOCK = "PRE_BLOCK"
HAPTIC_PULSE = "PULSE_1S"
HAPTIC_WAIT_NEUTRAL = "WAIT_NEUTRAL"


class JoystickTeleoperation:
    def __init__(
        self,
        *,
        profile_path: str,
        hand_port: int = 9090,
        franka_ip: str = "127.0.0.1",
        franka_port: int = 9999,
        telemetry_port: int = 10000,
        esp32_ip: str = "192.168.4.1",
        esp32_port: int = 5005,
        linear_speed: float = 0.030,
        angular_speed: float = 0.20,
        rate_hz: float = 100.0,
        hand_timeout: float = 0.5,
        pre_block_sec: float = 0.05,
        pulse_sec: float = 1.0,
        neutral_confirm_sec: float = 0.30,
        dashboard_hz: float = 4.0,
    ):
        self.classifier = ModalClassifier.from_path(profile_path)

        self.franka_target = (franka_ip, franka_port)
        self.esp32_target = (esp32_ip, esp32_port)

        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.rate_hz = rate_hz
        self.hand_timeout = hand_timeout
        self.pre_block_sec = pre_block_sec
        self.pulse_sec = pulse_sec
        self.neutral_confirm_sec = neutral_confirm_sec
        self.dashboard_interval = 1.0 / max(0.2, dashboard_hz)

        # Limit the rate of change of commanded velocities.
        self.a_linear = 0.15
        self.a_angular = 0.80

        self.hand_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.hand_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.hand_sock.bind(("127.0.0.1", hand_port))
        self.hand_sock.setblocking(False)

        self.telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.telemetry_sock.bind(("0.0.0.0", telemetry_port))
        self.telemetry_sock.setblocking(False)

        self.franka_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.esp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.esp_sock.setblocking(False)

        self.seq = 0
        self.cmd = [0.0] * 6

        self.handle_state = "IDLE"
        self.last_hand_packet_wall = 0.0
        self.classifier_ready = False

        self.haptic_state = HAPTIC_NORMAL
        self.haptic_state_started = 0.0
        self.neutral_since = None
        self.input_block = False

        # Accept one contact rising edge per feedback cycle while NORMAL.
        self.prev_contact_flag = False

        self.esp_online = False
        self.last_esp_reply = "NO REPLY"
        self.last_esp_reply_wall = 0.0
        self.last_ping_wall = 0.0

        self.last_telemetry_wall = 0.0
        self.telemetry = {
            "t": 0.0,
            "Fx": 0.0, "Fy": 0.0, "Fz": 0.0,
            "Mx": 0.0, "My": 0.0, "Mz": 0.0,
            "Fnorm": 0.0,
            "vx": 0.0, "vy": 0.0, "vz": 0.0, "speed": 0.0,
            "cmd_vx": 0.0, "cmd_vy": 0.0, "cmd_vz": 0.0,
            "v_parallel": 0.0,
            "a_parallel": 0.0,
            "F_parallel": 0.0,
            "dF_parallel": 0.0,
            "F_free_baseline": 0.0,
            "F_residual": 0.0,
            "dF_residual": 0.0,
            "phase": 0,
            "contact": 0,
            "release": 0,
            "franka_contact": 0,
            "command_active": 0,
            "motion_stable": 0,
        }

        self.running = True
        self.last_dashboard_wall = 0.0

    @staticmethod
    def _approach(current: float, target: float, max_step: float) -> float:
        if target > current:
            return min(current + max_step, target)
        return max(current - max_step, target)

    def _hand_alive(self) -> bool:
        return (time.monotonic() - self.last_hand_packet_wall) <= self.hand_timeout

    def _telemetry_alive(self) -> bool:
        return (time.monotonic() - self.last_telemetry_wall) <= 0.30

    def _read_hand_packets(self) -> None:
        while True:
            try:
                data, _ = self.hand_sock.recvfrom(65535)
            except BlockingIOError:
                break
            except OSError:
                break

            try:
                pkt = json.loads(data.decode("utf-8"))
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue

            self.last_hand_packet_wall = time.monotonic()

            state = self.classifier.update(pkt)

            if self.classifier.just_calibrated:
                self.classifier_ready = True
                print("\n[HANDLE] neutral calibration complete / classifier READY")

            if state != self.handle_state:
                old = self.handle_state
                self.handle_state = state
                print(f"\n[HANDLE] {old} -> {state}")

        if not self._hand_alive() and self.handle_state != "IDLE":
            old = self.handle_state
            self.handle_state = "IDLE"
            print(f"\n[HANDLE] stream timeout: {old} -> IDLE")

    @staticmethod
    def _parse_telemetry_packet(data: bytes):
        try:
            parts = data.decode("ascii", errors="ignore").strip().split()
            if len(parts) != 28:
                return None

            vals = [float(x) for x in parts[:22]]
            flags = [int(x) for x in parts[22:28]]
        except (ValueError, IndexError):
            return None

        return {
            "t": vals[0],
            "Fx": vals[1], "Fy": vals[2], "Fz": vals[3],
            "Mx": vals[4], "My": vals[5], "Mz": vals[6],
            "Fnorm": vals[7],
            "vx": vals[8], "vy": vals[9], "vz": vals[10],
            "speed": vals[11],
            "cmd_vx": vals[12], "cmd_vy": vals[13], "cmd_vz": vals[14],
            "v_parallel": vals[15],
            "a_parallel": vals[16],
            "F_parallel": vals[17],
            "dF_parallel": vals[18],
            "F_free_baseline": vals[19],
            "F_residual": vals[20],
            "dF_residual": vals[21],
            "phase": flags[0],
            "contact": flags[1],
            "release": flags[2],
            "franka_contact": flags[3],
            "command_active": flags[4],
            "motion_stable": flags[5],
        }

    def _read_telemetry(self) -> bool:
        """Drain telemetry; return True if CONTACT 0->1 was observed."""
        contact_rising = False
        newest = None

        while True:
            try:
                data, _ = self.telemetry_sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break

            tel = self._parse_telemetry_packet(data)
            if tel is None:
                continue

            contact_now = bool(tel["contact"])
            if contact_now and not self.prev_contact_flag:
                contact_rising = True
            self.prev_contact_flag = contact_now
            newest = tel

        if newest is not None:
            self.telemetry = newest
            self.last_telemetry_wall = time.monotonic()

        return contact_rising

    def _send_franka(self, active: bool, quit_flag: bool = False) -> None:
        self.seq += 1
        msg = (
            f"{self.cmd[0]:+.6f} {self.cmd[1]:+.6f} {self.cmd[2]:+.6f} "
            f"{self.cmd[3]:+.6f} {self.cmd[4]:+.6f} {self.cmd[5]:+.6f} "
            f"{1 if active else 0} {1 if quit_flag else 0} {self.seq}\n"
        )
        self.franka_sock.sendto(msg.encode("ascii"), self.franka_target)

    def _send_zero_now(self) -> None:
        self.cmd = [0.0] * 6
        self._send_franka(False, False)

    def _send_esp(self, command: str) -> None:
        try:
            self.esp_sock.sendto(command.encode("ascii"), self.esp32_target)
            print(f"\n[ESP32] TX -> {command}")
        except OSError as exc:
            print(f"\n[ESP32] UDP send failed: {exc}")

    def _read_esp_replies(self) -> None:
        while True:
            try:
                data, addr = self.esp_sock.recvfrom(1024)
            except BlockingIOError:
                break
            except OSError:
                break

            text = data.decode("utf-8", errors="replace").strip()
            self.last_esp_reply = text
            self.last_esp_reply_wall = time.monotonic()
            self.esp_online = True
            if text != "PONG":
                print(f"\n[ESP32] RX <- {text} ({addr[0]}:{addr[1]})")

    def _maybe_ping_esp(self, now: float) -> None:
        # Keep PING traffic out of the haptic event window.
        if self.haptic_state != HAPTIC_NORMAL:
            return

        if now - self.last_ping_wall >= 2.0:
            self.last_ping_wall = now
            self._send_esp("PING")

        if now - self.last_esp_reply_wall > 5.0:
            self.esp_online = False

    def _start_haptic_event(self, now: float) -> None:
        if self.haptic_state != HAPTIC_NORMAL:
            return

        # Feedback sequence:
        # 1) block handle input
        # 2) send zero command to Franka
        # 3) wait PRE_BLOCK_SEC with repeated zero commands
        # 4) send PULSE
        self.input_block = True
        self.haptic_state = HAPTIC_PRE_BLOCK
        self.haptic_state_started = now
        self.neutral_since = None

        self._send_zero_now()
        print("\n[HAPTIC] CONTACT rising edge")
        print("[HAPTIC] 1/4 INPUT BLOCKED")
        print("[HAPTIC] 2/4 Franka command ZERO")

    def _update_haptic_state(self, now: float) -> None:
        if self.haptic_state == HAPTIC_NORMAL:
            self.input_block = False
            return

        self.input_block = True

        if self.haptic_state == HAPTIC_PRE_BLOCK:
            if now - self.haptic_state_started >= self.pre_block_sec:
                self._send_esp("PULSE")
                self.haptic_state = HAPTIC_PULSE
                self.haptic_state_started = now
                print("[HAPTIC] 3/4 PULSE sent; ESP32 vacuum HIGH for 1.0 s")

        elif self.haptic_state == HAPTIC_PULSE:
            if now - self.haptic_state_started >= self.pulse_sec:
                self.haptic_state = HAPTIC_WAIT_NEUTRAL
                self.haptic_state_started = now
                self.neutral_since = None
                print("[HAPTIC] ESP32 1.0 s pulse elapsed; valve should be LOW")
                print("[HAPTIC] 4/4 waiting handle continuous IDLE...")

        elif self.haptic_state == HAPTIC_WAIT_NEUTRAL:
            if (
                self._hand_alive()
                and self.classifier_ready
                and self.handle_state == "IDLE"
            ):
                if self.neutral_since is None:
                    self.neutral_since = now
                elif now - self.neutral_since >= self.neutral_confirm_sec:
                    self.haptic_state = HAPTIC_NORMAL
                    self.input_block = False
                    self.neutral_since = None
                    self.cmd = [0.0] * 6
                    print("[HAPTIC] handle IDLE confirmed 300 ms")
                    print("[HAPTIC] INPUT RESTORED")
            else:
                self.neutral_since = None

    def _target_from_handle(self):
        # Block handle commands throughout the haptic feedback cycle.
        if self.input_block:
            return [0.0] * 6, False

        if (
            not self.classifier_ready
            or not self._hand_alive()
        ):
            return [0.0] * 6, False

        unit = LABEL_TO_UNIT.get(
            self.handle_state,
            LABEL_TO_UNIT["IDLE"],
        )

        target = [
            unit[0] * self.linear_speed,
            unit[1] * self.linear_speed,
            unit[2] * self.linear_speed,
            unit[3] * self.angular_speed,
            unit[4] * self.angular_speed,
            unit[5] * self.angular_speed,
        ]

        return target, self.handle_state != "IDLE"

    def _update_command(self, target, active: bool, dt: float) -> None:
        if not active:
            self.cmd = [0.0] * 6
            return

        lin_step = self.a_linear * dt
        ang_step = self.a_angular * dt

        for i in range(3):
            self.cmd[i] = self._approach(self.cmd[i], target[i], lin_step)
        for i in range(3, 6):
            self.cmd[i] = self._approach(self.cmd[i], target[i], ang_step)

    def _print_dashboard(self, now: float) -> None:
        if now - self.last_dashboard_wall < self.dashboard_interval:
            return
        self.last_dashboard_wall = now

        tel = self.telemetry
        tel_ok = self._telemetry_alive()
        hand_ok = self._hand_alive()
        phase = PHASE_TEXT.get(tel["phase"], "?") if tel_ok else "TELEMETRY_LOST"

        if self.classifier_ready:
            classifier_text = "READY"
        else:
            cur, total = self.classifier.calibration_progress
            classifier_text = f"CAL {cur}/{total}"

        if self.haptic_state == HAPTIC_WAIT_NEUTRAL and self.neutral_since is not None:
            neutral_ms = (now - self.neutral_since) * 1000.0
        else:
            neutral_ms = 0.0

        # Refresh the terminal status display.
        print("\033[H\033[J", end="")
        print("==============================================================")
        print(" Joystick -> Franka | Contact haptics")
        print("==============================================================")
        print(
            f"HANDLE   state={self.handle_state:<5} "
            f"stream={'OK' if hand_ok else 'LOST':<4} "
            f"classifier={classifier_text}"
        )
        print(
            f"HAPTIC   state={self.haptic_state:<12} "
            f"INPUT_BLOCK={int(self.input_block)} "
            f"neutral={neutral_ms:5.0f} ms"
        )
        print(
            f"ESP32    {'ONLINE' if self.esp_online else 'NO ACK':<7} "
            f"target={self.esp32_target[0]}:{self.esp32_target[1]} "
            f"last={self.last_esp_reply}"
        )
        print(
            f"FRANKA   telemetry={'OK' if tel_ok else 'LOST':<4} "
            f"phase={phase:<15} "
            f"CONTACT={tel['contact'] if tel_ok else '-'} "
            f"RELEASE={tel['release'] if tel_ok else '-'}"
        )
        print("--------------------------------------------------------------")
        print(
            f"FORCE    Fx={tel['Fx']:+7.2f}  Fy={tel['Fy']:+7.2f}  "
            f"Fz={tel['Fz']:+7.2f} N"
        )
        print(
            f"DETECT   Fproj={tel['F_parallel']:+6.2f} N  "
            f"Ffree={tel['F_free_baseline']:+6.2f} N  "
            f"Fres={tel['F_residual']:5.2f} N  "
            f"dFres/dt={tel['dF_residual']:+7.1f} N/s"
        )
        print(
            f"MOTION   vpar={tel['v_parallel']:+.4f} m/s  "
            f"apar={tel['a_parallel']:+.3f} m/s^2  "
            f"|v|={tel['speed']:.4f} m/s"
        )
        print(
            f"COMMAND  vx={self.cmd[0]:+.3f} vy={self.cmd[1]:+.3f} "
            f"vz={self.cmd[2]:+.3f}  "
            f"wx={self.cmd[3]:+.3f} wy={self.cmd[4]:+.3f} "
            f"wz={self.cmd[5]:+.3f}"
        )
        print("--------------------------------------------------------------")
        print("Ctrl+C: safe shutdown")
        print("All 12 directions are controlled by the Kresling handle")
        print("Automatic contact-triggered haptic feedback: ENABLED")

    def _safe_shutdown(self) -> None:
        print("\n[SAFE] shutting down...")
        self.input_block = True
        self.cmd = [0.0] * 6

        # Request valve-off before closing the feedback connection.
        try:
            self._send_esp("0")
        except Exception:
            pass

        for _ in range(10):
            try:
                self._send_franka(False, True)
            except OSError:
                pass
            time.sleep(0.01)

        for sock in (
            self.hand_sock,
            self.telemetry_sock,
            self.franka_sock,
            self.esp_sock,
        ):
            try:
                sock.close()
            except OSError:
                pass

        print("[SAFE] zero/quit sent; sockets closed")

    def run(self) -> None:
        print(f"[HANDLE] inference UDP <- 127.0.0.1:{self.hand_sock.getsockname()[1]}")
        print(f"[FRANKA] command UDP -> {self.franka_target[0]}:{self.franka_target[1]}")
        print(f"[FRANKA] telemetry UDP <- 0.0.0.0:{self.telemetry_sock.getsockname()[1]}")
        print(f"[ESP32] UDP -> {self.esp32_target[0]}:{self.esp32_target[1]}")
        print("[HANDLE] Keep controller neutral for the first 50 classifier frames.")
        print("[HAPTIC] BLOCK -> ZERO -> 50 ms -> PULSE 1 s -> IDLE 300 ms -> RESTORE")
        print("[INFO] Terminal status display is active.")
        print("[CONTACT] software contact-triggered input blocking / ESP32 PULSE is ENABLED.")
        print("[HANDLE] All 12 motion directions use the joystick.")

        self._send_esp("PING")

        last = time.monotonic()
        loop_period = 1.0 / self.rate_hz

        try:
            while self.running:
                loop_start = time.monotonic()
                dt = min(max(loop_start - last, 1e-4), 0.05)
                last = loop_start

                self._read_hand_packets()
                contact_rising = self._read_telemetry()
                self._read_esp_replies()

                # Block motion before advancing the feedback state machine.
                # Each contact rising edge can start one cycle while NORMAL.
                if contact_rising:
                    self._start_haptic_event(loop_start)

                self._update_haptic_state(loop_start)
                self._maybe_ping_esp(loop_start)

                target, active = self._target_from_handle()
                self._update_command(target, active, dt)

                # During haptics this continuously transmits zero with active=0.
                self._send_franka(active and not self.input_block, False)

                self._print_dashboard(loop_start)

                elapsed = time.monotonic() - loop_start
                sleep_time = loop_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n[SAFE] Ctrl+C received")
        finally:
            self._safe_shutdown()


def main():
    script_dir = Path(__file__).resolve().parent
    default_profile = script_dir / "controller_12mode_v2.json"

    parser = argparse.ArgumentParser(
        description="Joystick teleoperation of Franka with automatic contact haptics"
    )
    parser.add_argument("--profile", default=str(default_profile))
    parser.add_argument("--hand-port", type=int, default=9090)
    parser.add_argument("--franka-ip", default="127.0.0.1")
    parser.add_argument("--franka-port", type=int, default=9999)
    parser.add_argument("--telemetry-port", type=int, default=10000)
    parser.add_argument("--esp32-ip", default="192.168.4.1")
    parser.add_argument("--esp32-port", type=int, default=5005)
    parser.add_argument("--linear-speed", type=float, default=0.030)
    parser.add_argument("--angular-speed", type=float, default=0.20)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--hand-timeout", type=float, default=0.5)
    parser.add_argument("--pre-block-sec", type=float, default=0.05)
    parser.add_argument("--pulse-sec", type=float, default=1.0)
    parser.add_argument("--neutral-confirm-sec", type=float, default=0.30)
    parser.add_argument("--dashboard-hz", type=float, default=4.0)
    args = parser.parse_args()

    app = JoystickTeleoperation(
        profile_path=args.profile,
        hand_port=args.hand_port,
        franka_ip=args.franka_ip,
        franka_port=args.franka_port,
        telemetry_port=args.telemetry_port,
        esp32_ip=args.esp32_ip,
        esp32_port=args.esp32_port,
        linear_speed=args.linear_speed,
        angular_speed=args.angular_speed,
        rate_hz=args.rate,
        hand_timeout=args.hand_timeout,
        pre_block_sec=args.pre_block_sec,
        pulse_sec=args.pulse_sec,
        neutral_confirm_sec=args.neutral_confirm_sec,
        dashboard_hz=args.dashboard_hz,
    )

    app.run()


if __name__ == "__main__":
    main()
