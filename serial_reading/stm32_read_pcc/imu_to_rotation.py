from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class IMUOrientationRPY:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass
class IMUOrientationQuat:
    w: float
    x: float
    y: float
    z: float


@dataclass
class ThreeIMUPacket:
    top: IMUOrientationRPY | IMUOrientationQuat
    mid: IMUOrientationRPY | IMUOrientationQuat
    low: IMUOrientationRPY | IMUOrientationQuat
    pressure_raw: int | None = None
    pressure_mv: int | None = None
    pressure_raw_x1000: int | None = None
    pressure_uv: int | None = None


def _rotz(angle_deg: float) -> np.ndarray:
    a = np.deg2rad(angle_deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_to_rotation_matrix(
    roll_deg: float, pitch_deg: float, yaw_deg: float
) -> np.ndarray:
    r = np.deg2rad(roll_deg)
    p = np.deg2rad(pitch_deg)
    y = np.deg2rad(yaw_deg)

    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)

    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])

    return Rz @ Ry @ Rx


def quat_to_rotation_matrix(q: IMUOrientationQuat) -> np.ndarray:
    return quat_to_rotation_matrix_values((q.w, q.x, q.y, q.z))


def quat_to_rotation_matrix_values(q: tuple[float, float, float, float] | np.ndarray) -> np.ndarray:
    w, x, y, z = q
    norm = np.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-12:
        return np.eye(3)
    w /= norm
    x /= norm
    y /= norm
    z /= norm

    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
    )


def rotation_matrix_to_quat_values(R: np.ndarray) -> tuple[float, float, float, float]:
    R = np.asarray(R, dtype=np.float64)
    trace = float(np.trace(R))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = np.array([w, x, y, z], dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    q /= norm
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def blend_rotation_matrix(previous: np.ndarray, current: np.ndarray, alpha: float) -> np.ndarray:
    alpha = float(np.clip(alpha, 0.0, 1.0))
    q_prev = np.array(rotation_matrix_to_quat_values(previous), dtype=np.float64)
    q_curr = np.array(rotation_matrix_to_quat_values(current), dtype=np.float64)
    if float(np.dot(q_prev, q_curr)) < 0.0:
        q_curr = -q_curr
    q = (1.0 - alpha) * q_prev + alpha * q_curr
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        q = q_curr
    else:
        q /= norm
    return quat_to_rotation_matrix_values(q)


def build_platform_rotations_from_rpy(
    packet: ThreeIMUPacket, prerotate_mid_deg: float = 0.0
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    R0 = rpy_to_rotation_matrix(packet.low.roll_deg, packet.low.pitch_deg, packet.low.yaw_deg)   # type: ignore
    R1 = rpy_to_rotation_matrix(packet.mid.roll_deg, packet.mid.pitch_deg, packet.mid.yaw_deg)   # type: ignore
    R2 = rpy_to_rotation_matrix(packet.top.roll_deg, packet.top.pitch_deg, packet.top.yaw_deg)   # type: ignore
    if prerotate_mid_deg:
        R1 = R1 @ _rotz(prerotate_mid_deg)
    return R0, R1, R2


def build_platform_rotations_from_quat(
    packet: ThreeIMUPacket, prerotate_mid_deg: float = 0.0
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    R0 = quat_to_rotation_matrix(packet.low)   # type: ignore
    R1 = quat_to_rotation_matrix(packet.mid)   # type: ignore
    R2 = quat_to_rotation_matrix(packet.top)   # type: ignore
    if prerotate_mid_deg:
        R1 = R1 @ _rotz(prerotate_mid_deg)
    return R0, R1, R2
