from __future__ import annotations

import numpy as np


class FixedPCCBaseline:
    """Stage 1 fixed-r,b PCC baseline implemented in pure Python."""

    def __init__(
        self,
        r: float | list[float] | tuple[float, float] = (15e-3, 15e-3),
        b: float | list[float] | tuple[float, float] = (20e-3, 20e-3),
        eps_theta: float = 1e-5,
    ) -> None:
        self.r = _expand_pair(r, "r")
        self.b = _expand_pair(b, "b")
        self.eps_theta = float(eps_theta)
        if self.eps_theta <= 0:
            raise ValueError("eps_theta must be positive.")

    def predict(
        self,
        R01: np.ndarray,
        R12: np.ndarray,
        *,
        keep_phi_continuity: bool = True,
        initial_phi: np.ndarray | list[float] | tuple[float, float] | None = None,
    ) -> dict[str, np.ndarray]:
        """Predict PCC positions from relative segment rotations.

        Parameters
        ----------
        R01:
            Relative rotation from bottom to mid, shape 3x3 or Nx3x3.
        R12:
            Relative rotation from mid to top, shape 3x3 or Nx3x3.
        keep_phi_continuity:
            If true, near-straight frames reuse the previous frame phi in batch order.
        initial_phi:
            Optional [phi_segment_1, phi_segment_2] used for the first frame.
        """

        R01_stack, was_single = as_rotation_stack(R01, "R01")
        R12_stack, was_single_12 = as_rotation_stack(R12, "R12")
        if was_single != was_single_12 or R01_stack.shape[0] != R12_stack.shape[0]:
            raise ValueError("R01 and R12 must both be single-frame or have the same batch size.")

        sample_count = R01_stack.shape[0]
        p01 = np.zeros((sample_count, 3), dtype=np.float64)
        p02 = np.zeros((sample_count, 3), dtype=np.float64)
        p12 = np.zeros((sample_count, 3), dtype=np.float64)
        T01 = np.zeros((sample_count, 4, 4), dtype=np.float64)
        T02 = np.zeros((sample_count, 4, 4), dtype=np.float64)
        phi = np.zeros((sample_count, 2), dtype=np.float64)
        theta = np.zeros((sample_count, 2), dtype=np.float64)
        psi = np.zeros((sample_count, 2), dtype=np.float64)
        h = np.zeros((sample_count, 2), dtype=np.float64)

        phi_prev = np.zeros(2, dtype=np.float64) if initial_phi is None else np.asarray(initial_phi, dtype=np.float64)
        if phi_prev.shape != (2,):
            raise ValueError(f"initial_phi must have shape (2,), got {phi_prev.shape}.")

        for idx in range(sample_count):
            prev_1 = phi_prev[0] if keep_phi_continuity else 0.0
            prev_2 = phi_prev[1] if keep_phi_continuity else 0.0

            seg_1 = _extract_segment_params(
                R01_stack[idx],
                r=self.r[0],
                b=self.b[0],
                eps_theta=self.eps_theta,
                phi_prev=prev_1,
            )
            T01_i = _constant_curve_transform(seg_1, self.eps_theta)
            seg_2 = _extract_segment_params(
                R12_stack[idx],
                r=self.r[1],
                b=self.b[1],
                eps_theta=self.eps_theta,
                phi_prev=prev_2,
            )
            T12_i = _constant_curve_transform(seg_2, self.eps_theta)
            T02_i = T01_i @ T12_i

            T01[idx] = T01_i
            T02[idx] = T02_i
            p01[idx] = T01_i[:3, 3]
            p02[idx] = T02_i[:3, 3]
            p12[idx] = R01_stack[idx].T @ (p02[idx] - p01[idx])
            phi[idx] = [seg_1["phi"], seg_2["phi"]]
            theta[idx] = [seg_1["theta"], seg_2["theta"]]
            psi[idx] = [seg_1["psi"], seg_2["psi"]]
            h[idx] = [seg_1["h"], seg_2["h"]]

            if keep_phi_continuity:
                phi_prev = phi[idx].copy()

        result = {
            "p01": p01,
            "p12": p12,
            "p02": p02,
            "T01": T01,
            "T02": T02,
            "phi": phi,
            "theta": theta,
            "psi": psi,
            "h": h,
            "final_phi": phi_prev.copy(),
        }
        return _squeeze_result(result) if was_single else result


def as_rotation_stack(R: np.ndarray, name: str = "R") -> tuple[np.ndarray, bool]:
    array = np.asarray(R, dtype=np.float64)
    if array.shape == (3, 3):
        _validate_finite(array, name)
        return array[None, :, :], True
    if array.ndim == 3 and array.shape[1:] == (3, 3):
        _validate_finite(array, name)
        return array, False
    raise ValueError(f"{name} must have shape (3,3) or (N,3,3), got {array.shape}.")


def rot6d(R: np.ndarray) -> np.ndarray:
    stack, was_single = as_rotation_stack(R, "R")
    features = np.concatenate([stack[:, :, 0], stack[:, :, 1]], axis=1)
    return features[0] if was_single else features


def _extract_segment_params(
    R_relative: np.ndarray,
    *,
    r: float,
    b: float,
    eps_theta: float,
    phi_prev: float,
) -> dict[str, np.ndarray | float | bool]:
    z_base = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    z_top = R_relative[:, 2]
    cos_theta = float(np.clip(np.dot(z_base, z_top), -1.0, 1.0))
    theta = float(np.arccos(cos_theta))
    is_straight = theta < eps_theta

    if is_straight:
        phi = float(phi_prev)
        bend_axis = np.array([np.cos(phi), np.sin(phi), 0.0], dtype=np.float64)
        R_bend = np.eye(3)
        psi = float(np.arctan2(R_relative[1, 0], R_relative[0, 0]))
        R_clean = _rotz(psi)
    else:
        bend_axis = np.cross(z_base, z_top)
        bend_axis_norm = np.linalg.norm(bend_axis)
        if bend_axis_norm <= np.finfo(float).eps:
            bend_axis = np.array([np.cos(phi_prev), np.sin(phi_prev), 0.0], dtype=np.float64)
        else:
            bend_axis = bend_axis / bend_axis_norm
        phi = float(np.arctan2(bend_axis[1], bend_axis[0]))
        R_bend = _axis_angle_to_rotm(bend_axis, theta)
        R_twist = R_bend.T @ R_relative
        psi = float(np.arctan2(R_twist[1, 0], R_twist[0, 0]))
        R_clean = R_bend @ _rotz(psi)

    h_squared = b**2 + 2.0 * r**2 * (np.cos(psi) - 1.0)
    h = float(np.sqrt(max(h_squared, 0.0)))
    return {
        "R_clean": _project_to_so3(R_clean),
        "R_bend": R_bend,
        "bend_axis": bend_axis,
        "phi": phi,
        "theta": theta,
        "psi": psi,
        "h": h,
        "is_straight": is_straight,
    }


def _constant_curve_transform(segment: dict[str, np.ndarray | float | bool], eps_theta: float) -> np.ndarray:
    theta = float(segment["theta"])
    h = float(segment["h"])
    phi = float(segment["phi"])
    if theta < eps_theta:
        translation = np.array([0.0, 0.0, h], dtype=np.float64)
    else:
        rho = h / theta
        translation = np.array(
            [
                rho * (1.0 - np.cos(theta)) * np.cos(phi),
                rho * (1.0 - np.cos(theta)) * np.sin(phi),
                rho * np.sin(theta),
            ],
            dtype=np.float64,
        )

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(segment["R_clean"], dtype=np.float64)
    T[:3, 3] = translation
    return T


def _axis_angle_to_rotm(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c = np.cos(angle)
    s = np.sin(angle)
    one_minus_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
            [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
            [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
        ],
        dtype=np.float64,
    )


def _rotz(angle: float) -> np.ndarray:
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _project_to_so3(R: np.ndarray) -> np.ndarray:
    u, _, vh = np.linalg.svd(R)
    correction = np.diag([1.0, 1.0, np.linalg.det(u @ vh)])
    return u @ correction @ vh


def _expand_pair(value: float | list[float] | tuple[float, float], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64).reshape(-1)
    if array.size == 1:
        return np.array([array[0], array[0]], dtype=np.float64)
    if array.size == 2:
        return array.astype(np.float64)
    raise ValueError(f"{name} must be a scalar or two-element sequence.")


def _validate_finite(array: np.ndarray, name: str) -> None:
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains NaN or Inf.")


def _squeeze_result(result: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    squeezed: dict[str, np.ndarray] = {}
    for key, value in result.items():
        if isinstance(value, np.ndarray) and value.ndim >= 1 and value.shape[0] == 1 and key != "final_phi":
            squeezed[key] = value[0]
        else:
            squeezed[key] = value
    return squeezed
