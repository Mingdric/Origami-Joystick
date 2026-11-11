# STM32 serial acquisition and PCC reconstruction

Computer-side Python code for reading orientation messages from three IMUs
(Low, Mid, Top), parsing optional pressure data, and reconstructing two segments
using a fixed PCC model.

## Requirements and installation

Use Python 3.10 or later with NumPy and pyserial. Run all commands from
`serial_reading/`.

Windows PowerShell:

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m stm32_read_pcc COM3 --frames 100
```

Linux:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m stm32_read_pcc /dev/ttyACM0 --frames 100
```

Replace the port with the connected STM32 device. To inspect options without
opening a serial port, run `python -m stm32_read_pcc --help` in an environment
with the dependencies installed. Use module execution (`-m`) as shown above.

## Input protocol

A complete packet requires the case-sensitive labels `Low`, `Mid`, and `Top`.
The serial reader splits packets at `----`. RPY values are in degrees:

```text
Low: Roll=0.0, Pitch=0.0, Yaw=0.0
Mid: Roll=0.0, Pitch=0.0, Yaw=0.0
Top: Roll=0.0, Pitch=0.0, Yaw=0.0
Pressure: raw=1000, voltage=800 mV, raw_x1000=1000000, voltage_uV=800000
----
```

The pressure line is optional; its `raw_x1000` and `voltage_uV` fields are also
optional. A synthetic example is
provided in [`examples/synthetic_rpy_packet.txt`](examples/synthetic_rpy_packet.txt).

With `--quaternion`, each orientation line instead uses scalar-first `(w,x,y,z)`:

```text
Low: q0=1.0, q1=0.0, q2=0.0, q3=0.0
Mid: q0=1.0, q1=0.0, q2=0.0, q3=0.0
Top: q0=1.0, q1=0.0, q2=0.0, q3=0.0
----
```

## Common options

| Option | Default | Meaning |
| --- | --- | --- |
| `--baud` | `115200` | Serial baud rate |
| `--rate` | `20` | CRLF request/trigger writes per second; `0` disables triggers |
| `--quaternion` | Off | Parse quaternions instead of RPY |
| `--prerotate-mid` | `0.0` | Right-multiply the middle rotation by a local Z rotation in degrees |
| `--alpha` | `0.25` | Quaternion rotation smoothing; `0` or `1` bypasses smoothing |
| `--frames` | Unbounded | Stop after this many processed packets |
| `--serial-debug` | Off | Print incoming serial text to standard error |

The reader processes the latest available packet. `--frames` sets the number
of processed packets before stopping. Press Ctrl+C to stop manually.

## Output

Each processed packet produces one JSON object on standard output (JSON Lines).
For example, in a Linux shell:

```bash
.venv/bin/python -m stm32_read_pcc /dev/ttyACM0 --frames 100 > capture.jsonl
```

Positions (`p_mid`, `p_top`, `p12`, `h_m`) are in **metres**. `p_mid` and `p_top`
are expressed relative to the bottom platform; `p12` is relative to the middle
platform. Angles (`phi_rad`, `theta_rad`, `psi_rad`) are in radians.
The output also includes `time_unix_s`, relative rotations `R01`/`R12`, transforms
`T01`/`T02`, and optional pressure fields. The timestamp is assigned by the host
when processing the packet.

The fixed PCC defaults are `r=(0.015, 0.015)` m and `b=(0.020, 0.020)` m in
[`stm32_read_pcc/pcc_baseline.py`](stm32_read_pcc/pcc_baseline.py).
The geometry parameters can be set when constructing `FixedPCCBaseline`
through Python.

## Source map

- `serial_reader.py`: framing, RPY/quaternion/pressure parsing, serial threads.
- `imu_to_rotation.py`: packet types, orientation conversion, rotation smoothing.
- `pcc_baseline.py`: two-segment PCC reconstruction.
- `__main__.py`: command-line entry point and JSON Lines output.
