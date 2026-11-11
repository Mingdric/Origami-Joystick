# Origami Joystick

Source code for computer-based serial data acquisition, piecewise constant
curvature (PCC) reconstruction, and Franka teleoperation using a Kresling
origami joystick.

[中文说明](README.zh-CN.md)

## Repository contents

| Directory | Description |
| --- | --- |
| [`serial_reading/`](serial_reading/README.md) | STM32 serial data acquisition, IMU orientation processing, and two-segment PCC reconstruction |
| [`teleoperation/`](teleoperation/README.md) | Handle data acquisition, Franka motion control, and teleoperation with contact-triggered haptic feedback |

```text
Origami-Joystick/
├── README.md
├── README.zh-CN.md
├── serial_reading/
│   ├── README.md
│   ├── requirements.txt
│   ├── examples/
│   │   └── synthetic_rpy_packet.txt
│   └── stm32_read_pcc/
│       ├── __init__.py
│       ├── __main__.py
│       ├── serial_reader.py
│       ├── imu_to_rotation.py
│       └── pcc_baseline.py
└── teleoperation/
    ├── README.md
    ├── handle_serial_reader.py
    ├── franka_robot_controller.cpp
    └── joystick_teleoperation.py
```

## Serial data acquisition and reconstruction

The serial-reading module acquires orientations from the bottom, middle, and top
IMUs and optional pressure readings from an STM32 device. It supports RPY and
quaternion input, converts orientations to rotation matrices, and reconstructs
the middle and top platform positions using a two-segment PCC model. Results
are written as JSON Lines, with positions in metres and angles in radians.

Installation, serial message examples, and command-line options are described
in the [serial-reading README](serial_reading/README.md).

## Teleoperation

The teleoperation directory contains three source files:

- [`handle_serial_reader.py`](teleoperation/handle_serial_reader.py): serial acquisition and
  parsing of IMU orientation and pressure data.
- [`franka_robot_controller.cpp`](teleoperation/franka_robot_controller.cpp): Franka
  motion control, estimated external-force processing, contact detection, and
  telemetry logging.
- [`joystick_teleoperation.py`](teleoperation/joystick_teleoperation.py):
  direction classification, robot velocity commands, and contact-triggered ESP32
  haptic feedback.

The bridge maps twelve handle directions to translational and rotational velocity
commands. Contact-triggered feedback coordinates input blocking, an ESP32 valve
pulse command, and recovery after the handle returns to neutral.

File roles, control flow, and communication interfaces are described in the
[teleoperation README](teleoperation/README.md).
