# Teleoperation / 遥操作

This directory contains source code for Kresling handle data acquisition,
Franka robot control, and teleoperation with contact-triggered haptic feedback.

本目录包含 Kresling 手柄读取、Franka 机械臂控制，以及连接手柄与机械臂的遥操作代码。

## Files / 文件说明

| File / 文件 | Function / 功能 |
| --- | --- |
| [`handle_serial_reader.py`](handle_serial_reader.py) | STM32 serial data acquisition / 手柄串口读取 |
| [`franka_robot_controller.cpp`](franka_robot_controller.cpp) | Franka motion control and force telemetry / 机械臂控制与力遥测 |
| [`joystick_teleoperation.py`](joystick_teleoperation.py) | Handle-to-robot commands and haptic feedback / 遥操作指令与触觉反馈 |

## `handle_serial_reader.py`

Acquires serial messages from the STM32 and parses the Low, Mid, and Top IMU
orientations and optional pressure readings. It supports RPY angles in degrees
and scalar-first quaternions `(w, x, y, z)`. Packets are separated by `----`.
Background threads handle serial acquisition and sampling triggers; received
packets are buffered for downstream processing. The reader also provides raw
logging callbacks and serial valve-command writes.

通过串口接收 STM32 数据，解析底部、中部和顶部三组 IMU 姿态及可选压力值。支持角度制欧拉角
和标量在前的四元数，以 `----` 分隔数据包。后台线程负责读取和采样触发，缓存的数据供后续
处理使用；程序还提供原始记录回调和串口阀门命令接口。

## `franka_robot_controller.cpp`

Receives six-component velocity commands over UDP and controls the Franka robot
through libfranka. Translation commands use robot-base axes; rotation commands
use the current end-effector/TCP axes. A damped least-squares mapping converts
Cartesian velocity commands to joint velocities. The controller incorporates
joint and workspace limits, velocity and acceleration limits, and a command
watchdog.

The program processes estimated external force/torque, detects contact and
release states, publishes UDP telemetry, and records CSV force logs. Home-pose
functions record a joint configuration and return the robot to that configuration
before teleoperation.

接收 UDP 六维速度指令，通过 libfranka 控制机械臂。平移使用基坐标轴，旋转使用当前末端/TCP
坐标轴；通过阻尼最小二乘映射计算关节速度，并包含关节、工作空间、速度、加速度和指令超时
处理。程序处理估计外力与力矩，检测接触和释放状态，发送遥测并保存 CSV 日志，同时提供
原点记录和启动回原点功能。

## `joystick_teleoperation.py`

Receives handle data over UDP, obtains direction labels through `ModalClassifier`,
and maps `X±`, `Y±`, `Z±`, `RX±`, `RY±`, and `RZ±` to robot velocity commands.
All twelve motion directions use the handle. Defaults are 0.030 m/s for
translation, 0.20 rad/s for rotation, and a 100 Hz command loop. Input is gated
by calibration status, stream freshness, and the haptic state.

The program also receives force/contact telemetry from the C++ controller,
coordinates ESP32 valve commands, and displays handle, robot, and feedback states
in the terminal.

通过 UDP 接收手柄数据，经 `ModalClassifier` 获取动作类别，把十二个方向映射为机械臂速度
指令。十二个方向全部来自手柄；默认平移速度为 0.030 m/s、角速度为 0.20 rad/s，指令循环
为 100 Hz。程序根据校准状态、数据时效和反馈状态管理输入，并接收机械臂力与接触遥测，
协调 ESP32 阀门命令，在终端显示运行状态。

## Contact-triggered haptic feedback / 接触触觉反馈

```text
New contact event / 新接触事件
  -> block handle input and send zero velocity / 锁定输入并发送零速度
  -> wait at least 50 ms / 等待至少 50 ms
  -> send ESP32 PULSE / 发送脉冲命令
  -> wait 1 s / 等待 1 s
  -> calibrated, valid handle remains IDLE for 300 ms / 有效手柄回中持续 300 ms
  -> restore control / 恢复控制
```

A new contact edge starts one feedback cycle. The program waits for continuous
neutral handle input before restoring control. The ESP32 firmware executes the
valve pulse.

新的接触事件触发一次反馈流程，程序在手柄连续回中后恢复控制，阀门脉冲由 ESP32 固件执行。

## Communication interfaces / 通信接口

| Interface / 接口 | Default / 默认值 |
| --- | --- |
| Handle data into Python bridge / 手柄数据输入 | `127.0.0.1:9090` UDP |
| Python bridge to C++ controller / 机械臂速度指令 | `127.0.0.1:9999` UDP |
| C++ force/contact telemetry / 力与接触遥测 | UDP `10000` |
| ESP32 valve commands / 阀门命令 | `192.168.4.1:5005` UDP |

```text
Handle acquisition and processing / 手柄采集与处理
  -> UDP handle data / 手柄数据
  -> joystick_teleoperation.py
  -> UDP velocity commands / 速度指令
  -> franka_robot_controller.cpp -> Franka

Franka contact telemetry / 接触遥测
  -> Python bridge / 遥操作程序
  -> ESP32 valve command / 阀门命令
```
