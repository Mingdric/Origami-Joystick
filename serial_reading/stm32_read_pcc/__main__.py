"""Read STM32 orientations and reconstruct two segments using fixed PCC."""
from __future__ import annotations

import argparse
import json
import time

import numpy as np

from .imu_to_rotation import (
    blend_rotation_matrix,
    build_platform_rotations_from_quat,
    build_platform_rotations_from_rpy,
)
from .pcc_baseline import FixedPCCBaseline
from .serial_reader import InputMode, STM32SerialReader


class PCCReconstructor:
    def __init__(self, mode=InputMode.RPY, prerotate_mid_deg=0.0, alpha=0.25):
        if not 0 <= alpha <= 1:
            raise ValueError("alpha must be between 0 and 1")
        self.mode = mode
        self.prerotate_mid_deg = prerotate_mid_deg
        self.alpha = alpha
        self.baseline = FixedPCCBaseline()
        self.previous = None
        self.phi = None

    def process(self, packet):
        convert = (build_platform_rotations_from_quat if self.mode == InputMode.QUATERNION
                   else build_platform_rotations_from_rpy)
        rotations = convert(packet, prerotate_mid_deg=self.prerotate_mid_deg)
        if self.previous is not None and 0 < self.alpha < 1:
            rotations = tuple(blend_rotation_matrix(a, b, self.alpha)
                              for a, b in zip(self.previous, rotations))
        self.previous = rotations
        R0, R1, R2 = rotations
        R01, R12 = R0.T @ R1, R1.T @ R2
        result = self.baseline.predict(R01, R12, initial_phi=self.phi)
        self.phi = result["final_phi"].copy()
        return {
            "time_unix_s": time.time(),
            "position_unit": "m",
            "p_mid": result["p01"].tolist(),
            "p_top": result["p02"].tolist(),
            "p12": result["p12"].tolist(),
            "R01": R01.tolist(), "R12": R12.tolist(),
            "T01": result["T01"].tolist(), "T02": result["T02"].tolist(),
            "phi_rad": result["phi"].tolist(), "theta_rad": result["theta"].tolist(),
            "psi_rad": result["psi"].tolist(), "h_m": result["h"].tolist(),
            "pressure_raw": packet.pressure_raw, "pressure_mv": packet.pressure_mv,
            "pressure_raw_x1000": packet.pressure_raw_x1000, "pressure_uv": packet.pressure_uv,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rate", type=int, default=20, help="Trigger rate in Hz; 0 disables triggers")
    parser.add_argument("--quaternion", action="store_true", help="Expect q0,q1,q2,q3 instead of RPY")
    parser.add_argument("--prerotate-mid", type=float, default=0.0, metavar="DEG")
    parser.add_argument("--alpha", type=float, default=0.25, help="Rotation filter alpha; 0 disables")
    parser.add_argument("--frames", type=int, help="Stop after this many packets")
    parser.add_argument("--serial-debug", action="store_true")
    args = parser.parse_args()
    if args.rate < 0 or (args.frames is not None and args.frames < 1):
        parser.error("rate must be nonnegative and frames must be positive")
    if not 0 <= args.alpha <= 1:
        parser.error("alpha must be between 0 and 1")
    mode = InputMode.QUATERNION if args.quaternion else InputMode.RPY
    reconstructor = PCCReconstructor(mode, args.prerotate_mid, args.alpha)
    reader = STM32SerialReader(args.port, args.baud, input_mode=mode,
                               trigger_rate=args.rate, serial_debug=args.serial_debug)
    count = 0
    last_packet = None
    try:
        reader.start()
        while args.frames is None or count < args.frames:
            packet = reader.get_latest()
            if packet is None or packet is last_packet:
                time.sleep(0.005)
                continue
            last_packet = packet
            print(json.dumps(reconstructor.process(packet), allow_nan=False), flush=True)
            count += 1
    except KeyboardInterrupt:
        pass
    except BrokenPipeError:
        return
    finally:
        reader.stop()


if __name__ == "__main__":
    main()
