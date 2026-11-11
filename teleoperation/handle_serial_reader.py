from __future__ import annotations

import re
import sys
import time
from collections import deque
from enum import Enum, auto
from threading import Lock, Thread
from typing import Callable, Deque, Optional

from .imu_to_rotation import IMUOrientationQuat, IMUOrientationRPY, ThreeIMUPacket


VALVE_MAX_FREQUENCY_HZ = 20


class InputMode(Enum):
    RPY = auto()
    QUATERNION = auto()


RPY_LINE_RE = re.compile(
    r"(\w+):\s*Roll=([-\d.]+),\s*Pitch=([-\d.]+),\s*Yaw=([-\d.]+)"
)

QUAT_LINE_RE = re.compile(
    r"(\w+):\s*q0=([-\d.]+),\s*q1=([-\d.]+),\s*q2=([-\d.]+),\s*q3=([-\d.]+)"
)

PRESSURE_LINE_RE = re.compile(
    r"Pressure:\s*raw=(\d+),\s*voltage=(\d+)\s*mV"
)


def parse_rpy_line(line: str) -> tuple[str, IMUOrientationRPY] | tuple[None, None]:
    m = RPY_LINE_RE.search(line)
    if m:
        return m.group(1), IMUOrientationRPY(
            roll_deg=float(m.group(2)),
            pitch_deg=float(m.group(3)),
            yaw_deg=float(m.group(4)),
        )
    return None, None


def parse_quat_line(line: str) -> tuple[str, IMUOrientationQuat] | tuple[None, None]:
    m = QUAT_LINE_RE.search(line)
    if m:
        return m.group(1), IMUOrientationQuat(
            w=float(m.group(2)),  # q0 -> w (scalar)
            x=float(m.group(3)),  # q1 -> x
            y=float(m.group(4)),  # q2 -> y
            z=float(m.group(5)),  # q3 -> z
        )
    return None, None


def parse_pressure_line(line: str) -> tuple[int, int] | tuple[None, None]:
    m = PRESSURE_LINE_RE.search(line)
    if m:
        return int(m.group(1)), int(m.group(2))
    return None, None


def parse_uart_message(
    message: str, mode: InputMode = InputMode.RPY
) -> ThreeIMUPacket | None:
    readings: dict[str, IMUOrientationRPY | IMUOrientationQuat] = {}
    pressure_raw: int | None = None
    pressure_mv: int | None = None
    parser = parse_rpy_line if mode == InputMode.RPY else parse_quat_line
    for line in message.strip().split("\n"):
        loc, reading = parser(line)
        if loc and reading:
            readings[loc] = reading
            continue
        raw, mv = parse_pressure_line(line)
        if raw is not None and mv is not None:
            pressure_raw = raw
            pressure_mv = mv
    if {"Top", "Mid", "Low"}.issubset(readings.keys()):
        return ThreeIMUPacket(
            top=readings["Top"],
            mid=readings["Mid"],
            low=readings["Low"],
            pressure_raw=pressure_raw,
            pressure_mv=pressure_mv,
        )
    return None


class STM32SerialReader:
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        buffer_size: int = 200,
        timeout: float = 0.1,
        input_mode: InputMode | str = InputMode.RPY,
        trigger_rate: int = 20,
        serial_debug: bool = False,
    ):
        if isinstance(input_mode, str):
            input_mode = InputMode[input_mode.upper()]
        self._input_mode = input_mode
        import serial

        self.serial = serial.Serial(port, baudrate, timeout=timeout)
        self._buffer: Deque[ThreeIMUPacket] = deque(maxlen=buffer_size)
        self._lock = Lock()
        self._write_lock = Lock()
        self._running = False
        self._read_thread: Thread | None = None
        self._trigger_thread: Thread | None = None
        self._msg_buf = ""
        self._trigger_interval = 1.0 / trigger_rate if trigger_rate > 0 else 0.0
        self._serial_debug = serial_debug
        self._raw_callback: Callable[[ThreeIMUPacket, int | None], None] | None = None

    def start(self) -> None:
        self._running = True
        self._read_thread = Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()
        if self._trigger_interval > 0.0:
            self._trigger_thread = Thread(target=self._trigger_loop, daemon=True)
            self._trigger_thread.start()

    def stop(self) -> None:
        self._running = False
        if self._read_thread:
            self._read_thread.join(timeout=2.0)
        if self._trigger_thread:
            self._trigger_thread.join(timeout=2.0)
        if self.serial.is_open:
            self.serial.close()

    def _trigger_loop(self) -> None:
        next_time = time.perf_counter()
        while self._running:
            now = time.perf_counter()
            if now >= next_time:
                try:
                    with self._write_lock:
                        self.serial.write(b"\r\n")
                except Exception:
                    pass
                next_time += self._trigger_interval
                if now > next_time:
                    next_time = now + self._trigger_interval
            time.sleep(0.0005)

    def _read_loop(self) -> None:
        while self._running:
            try:
                if self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting)
                    text = data.decode("utf-8", errors="ignore")
                    if self._serial_debug:
                        print(text, end="", file=sys.stderr)
                    self._msg_buf += text
                    if "----" in self._msg_buf:
                        parts = self._msg_buf.split("----")
                        for i in range(len(parts) - 1):
                            msg = parts[i] + "----"
                            pkt = parse_uart_message(msg, mode=self._input_mode)
                            if pkt:
                                with self._lock:
                                    self._buffer.append(pkt)
                                if self._raw_callback:
                                    self._raw_callback(pkt, getattr(self, "_last_valve_command", None))
                        self._msg_buf = parts[-1]
            except Exception:
                time.sleep(0.01)

    def get_latest(self) -> ThreeIMUPacket | None:
        with self._lock:
            return self._buffer[-1] if self._buffer else None

    def get_history(
        self, n: int | None = None
    ) -> list[ThreeIMUPacket]:
        with self._lock:
            items = list(self._buffer)
        return items if n is None else items[-n:]

    def flush(self) -> None:
        with self._lock:
            self._buffer.clear()

    def set_raw_callback(
        self, callback: Callable[[ThreeIMUPacket, int | None], None] | None
    ) -> None:
        """Register a fast callback ``callback(packet, valve_freq)`` fired for
        every packet parsed by the read thread.

        Runs in the serial reader's background thread, so it keeps up with
        the full trigger rate regardless of downstream inference speed.
        """
        self._raw_callback = callback

    def set_valve_command(self, command: int) -> None:
        command = max(0, min(VALVE_MAX_FREQUENCY_HZ, int(command)))
        self._last_valve_command = command
        with self._write_lock:
            self.serial.write(f"{command}\n".encode("ascii"))

    @property
    def last_valve_command(self) -> int | None:
        return getattr(self, "_last_valve_command", None)

    @property
    def buffer_size(self) -> int:
        return len(self._buffer)

    @property
    def is_running(self) -> bool:
        return self._running
