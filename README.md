# Sanity receiver

ESP32-S3 firmware that receives drone commands over Wi-Fi UDP and forwards them
to a flight controller using SBUS. A host broadcasts one datagram containing
commands for multiple devices; each ESP32 selects its own command using `MY_ID`.

The firmware uses ESP-IDF and FreeRTOS.

## What is in this repository?

| Path | Purpose |
| --- | --- |
| `main/combined.c` | Application entry point, Wi-Fi connection, UDP decoding and SBUS output tasks |
| `main/config.h` | Device identity, Wi-Fi and IP settings, UART pins and communication rates |
| `main/wdg_timer.h` | Watchdog used by the command receive and SBUS tasks |
| `main/CMakeLists.txt` | ESP-IDF application component definition |
| `CMakeLists.txt` | ESP-IDF project definition |
| `sdkconfig` | Supplied ESP32-S3 build configuration |
| `scripts/send_drone_commands.py` | Standalone UDP command example using Python's standard library |
| `scripts/swmc_broadcast.py` | Older broadcast experiment; requires NumPy and sends armed commands |
| `scripts/udp_telemetry.py` | Older plotting utility for debug telemetry on UDP port 10250; requires Matplotlib |
| `scripts/batch_flash.sh` | Interactive helper for building and flashing several devices with different IDs |

At startup, the application connects to the configured access point, starts a
UDP listener, and starts an SBUS sender. The listener decodes commands and updates
the channel values. The SBUS task sends those values on UART2 at 100 Hz.

## Install ESP-IDF

The supplied `sdkconfig` was generated with **ESP-IDF 5.3.0** for **ESP32-S3**.
Use ESP-IDF v5.3 for this codebase. Newer SDKs may require source changes; a build
with ESP-IDF 6.2 currently fails on the legacy `ESP_IF_WIFI_STA` constant.

Follow Espressif's [ESP32-S3 getting-started guide](https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32s3/get-started/index.html)
for prerequisites and Windows, Linux or macOS installation options.

For Linux/macOS, after installing the prerequisites listed in the
[toolchain setup guide](https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32s3/get-started/linux-macos-setup.html),
a typical installation is:

```sh
mkdir -p ~/esp
cd ~/esp
git clone -b v5.3 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
. ./export.sh
```

Activate the environment in each new terminal before building:

```sh
. ~/esp/esp-idf/export.sh
idf.py --version
```

The standalone sender only needs Python 3; it does not require the ESP-IDF
environment, ROS 2, NumPy or other Python packages.

## Configure a device

Edit `main/config.h` before building:

- `WIFI_SSID` and `WIFI_PASS`: access-point credentials.
- `MY_ID`: device ID, starting at 1.
- `STATIC_IP_ADDR`, `STATIC_GATEWAY` and `STATIC_NETMASK`: network configuration.
- `UDP_PORT`: command port, default 10240.
- `UART_TX_PIN`, `UART_RX_PIN` and `SBUS_INVERT`: flight-controller connection.

Each ESP32 must have a unique ID and static IP. For example, device 1 can use
`MY_ID = 1` and `10.0.0.101`, while device 2 uses `MY_ID = 2` and `10.0.0.102`.
The default gateway is `10.0.0.1` and the netmask is `255.255.255.0`.

The listener currently binds to `10.0.0.255` explicitly in
`udp_listener_task()` in `main/combined.c`. If using another subnet, update this
address as well as `main/config.h` and the sender's `--ip` option.

SBUS is configured for UART2, TX GPIO 43 and RX GPIO 44, at 100000 baud with
8 data bits, even parity, 2 stop bits and inversion enabled. Check these settings
against the connected board and flight controller.

## Build and flash

Connect the ESP32-S3 over USB. From the repository root, with ESP-IDF activated:

```sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Replace `/dev/ttyACM0` with the actual serial port. Common alternatives are
`/dev/ttyUSB0` on Linux, `/dev/cu.*` on macOS, or `COM3` on Windows.
Exit the serial monitor with Ctrl+].

For several devices, configure and flash each one separately. The existing
`scripts/batch_flash.sh` can help, but first adjust its serial-port setting.
It changes `MY_ID` only: it does not assign a unique static IP to each device.

## Example: send a command without ROS 2

For a simple bench setup, configure one ESP32 with `MY_ID = 1` on the default
`10.0.0.0/24` network, build and flash it, and connect the sending computer to the
same network. Remove propellers before testing commands with a flight controller.

First inspect a packet without transmitting:

```sh
python3 scripts/send_drone_commands.py --num-agents 1 --drone-id 1
```

Then send neutral, disarmed commands at 50 Hz for five seconds:

```sh
python3 scripts/send_drone_commands.py --num-agents 1 --drone-id 1 --rate 50 --duration 5 --send
```

The sender defaults to `10.0.0.255:10240`. To change a command, supply its channel
values; for example, the following changes roll to 1600 while keeping the device
disarmed and thrust at its minimum:

```sh
python3 scripts/send_drone_commands.py --num-agents 1 --drone-id 1 --roll 1600 --thrust 1000 --flags 0 --send
```

The command fields are:

| Option | Meaning | Default |
| --- | --- | --- |
| `--roll`, `--pitch`, `--yaw` | Integer channel values in 1000..2000 | 1500 (neutral) |
| `--thrust` | Integer channel value in 1000..2000 | 1000 (minimum) |
| `--flags` | AUX bitfield; bit 0 sets AUX1 high to arm | 0 (disarmed) |
| `--num-agents` | Highest configured device ID | 10 |
| `--drone-id` | Device whose command is changed | 1 |
| `--ip`, `--port` | UDP destination | `10.0.0.255`, 10240 |
| `--rate`, `--duration` | Datagrams per second and sending duration in seconds | 100, 2 |
| `--send` | Enable transmission; omitted means dry run | Off |

These are channel values, not angles or physical thrust. `--flags 1` arms the
selected device. Every other device receives neutral, disarmed commands, so this
example should be used separately from an active flight broadcaster. At the end
or on Ctrl-C, the sender transmits ten additional disarm datagrams. UDP does not
provide a delivery acknowledgement.

For multiple devices, set `--num-agents` to the highest `MY_ID` and select the
command to change with `--drone-id`. The current receiver reads command slot
`MY_ID + 1`; the example reserves slots 0 and 1 and includes enough slots for
that indexing convention.

Run `python3 scripts/send_drone_commands.py --help` for all options.

## Relationship to the ROS 2 broadcaster

The standalone example uses the same big-endian UDP packet format as
`ros2_sanity_ws/src/udp_broadcaster/udp_broadcaster/broadcast_rpyt.py`.
That node subscribes to drone control commands and converts them to channel
values before broadcasting. This example takes channel values directly and
reproduces the packet construction using `socket` and `struct`.

Each packet contains routing and stream headers, a nine-byte command per slot
(flags, roll, pitch, yaw and thrust), and a trailing byte. The existing sender's
`construct_swmc_packet()` and the firmware's `decode_packet()` show both sides
of the format. The standalone sender has been checked against the ROS packet
serializer and with a local UDP listener; hardware behavior needs to be checked
on the actual setup.
