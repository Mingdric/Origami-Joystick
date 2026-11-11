# Origami Joystick

本仓库提供 Kresling 折纸手柄的电脑端串口采集、分段常曲率（PCC）重建和 Franka 遥操作代码。
英文介绍见 [README.md](README.md)。

## 仓库内容

| 目录 | 内容 |
| --- | --- |
| [serial_reading/](serial_reading/README.md) | STM32 串口读取、IMU 姿态处理和两段 PCC 位置重建 |
| [teleoperation/](teleoperation/README.md) | 手柄数据采集、机械臂运动控制及接触触觉反馈 |

## 串口采集与位置重建

`serial_reading/` 读取底部、中部和顶部三组 IMU 姿态及可选压力数据，支持欧拉角和四元数输入。
程序将姿态转换为旋转矩阵，利用两段 PCC 模型计算中间平台和顶部平台位置，并输出 JSON Lines。
位置单位为米，角度单位为弧度。

Windows PowerShell 使用示例，从仓库根目录执行：

```powershell
cd serial_reading
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m stm32_read_pcc COM3 --frames 100
```

将 `COM3` 换成实际串口。读取四元数数据时加上 `--quaternion`。
数据格式、参数和 Linux 使用方法见 [串口模块说明](serial_reading/README.md)。

## 遥操作代码

`teleoperation/` 包含三个源码文件和一份说明文档：

- `handle_serial_reader.py`：采集并解析手柄的 IMU 姿态和压力数据。
- `franka_robot_controller.cpp`：接收速度指令、控制 Franka，处理外力和接触状态并记录日志。
- `joystick_teleoperation.py`：识别手柄动作方向，生成机械臂速度指令，管理自动触觉反馈。
- `README.md`：说明三个文件的功能、控制流程和通信接口。

手柄十二方向对应三个平移轴和三个旋转轴的正负方向。检测到新的接触事件后，反馈流程锁定
手柄输入并发送零速度，等待 50 ms 后发送 ESP32 脉冲命令；等待 1 s 后，手柄有效回中并
持续中立 300 ms，恢复控制。阀门脉冲动作由 ESP32 固件执行。

详细内容见 [遥操作说明](teleoperation/README.md)。
