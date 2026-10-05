#!/usr/bin/env python3
"""Send the ROS 2 broadcaster's wire format using only the Python standard library."""

import argparse
import math
import socket
import struct
import time


NOMINAL = dict(flags=0, roll=1500, pitch=1500, yaw=1500, thrust=1000)


def construct_swmc_packet(payloads, timestamp_us=None):
    """Match broadcast_rpyt.py, including its legacy header values and trailer."""
    if timestamp_us is None:
        timestamp_us = time.time_ns() // 1000
    header = struct.pack(
        ">BHBHQBBBBBHI", 0, 0, 0, 0, timestamp_us, 0, 0, 5, 255, 0, 0, 777
    )
    commands = b"".join(
        struct.pack(
            ">BHHHH", p["flags"], p["roll"], p["pitch"], p["yaw"], p["thrust"]
        )
        for p in payloads
    )
    return header + commands + b"\x00"


def make_payloads(num_agents, drone_id, command):
    # The current firmware reads payload_array[MY_ID + 1]. Slots 0 and 1 are padding.
    payloads = [NOMINAL.copy() for _ in range(num_agents + 2)]
    payloads[drone_id + 1] = command.copy()
    return payloads


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="10.0.0.255")
    parser.add_argument("--port", type=int, default=10240)
    parser.add_argument("--num-agents", type=int, default=10, help="highest device MY_ID")
    parser.add_argument("--drone-id", type=int, default=1)
    for field in ("roll", "pitch", "yaw", "thrust"):
        parser.add_argument("--" + field, type=int, default=NOMINAL[field])
    parser.add_argument("--flags", type=lambda s: int(s, 0), default=0,
                        help="AUX bitfield: bit 0 arms; default 0 disarms")
    parser.add_argument("--rate", type=float, default=100, help="datagrams per second")
    parser.add_argument("--duration", type=float, default=2, help="seconds")
    parser.add_argument("--send", action="store_true", help="transmit; otherwise print one packet")
    args = parser.parse_args()
    if not 1 <= args.num_agents <= 222:
        parser.error("num-agents must be 1..222 to fit the receiver's UDP buffer")
    if not 1 <= args.drone_id <= args.num_agents:
        parser.error("drone-id must be 1..num-agents")
    if not 1 <= args.port <= 65535:
        parser.error("port must be 1..65535")
    if not 0 <= args.flags <= 255:
        parser.error("flags must be 0..255")
    for field in ("roll", "pitch", "yaw", "thrust"):
        if not 1000 <= getattr(args, field) <= 2000:
            parser.error(field + " must be 1000..2000")
    if any(not math.isfinite(v) or v <= 0 for v in (args.rate, args.duration)):
        parser.error("rate and duration must be finite and positive")

    command = {field: getattr(args, field) for field in NOMINAL}
    payloads = make_payloads(args.num_agents, args.drone_id, command)
    packet = construct_swmc_packet(payloads)
    target = (args.ip, args.port)
    print(f"Target {target}; MY_ID={args.drone_id} uses slot {args.drone_id + 1}")
    print(f"{len(payloads)} payloads, {len(packet)} bytes; command={command}")
    if not args.send:
        print("Dry run (add --send to transmit): " + packet.hex())
        return

    interval = 1 / args.rate
    sent = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        deadline = time.monotonic()
        end = deadline + args.duration
        try:
            while time.monotonic() < end:
                sock.sendto(construct_swmc_packet(payloads), target)
                sent += 1
                deadline += interval
                time.sleep(max(0, min(deadline, end) - time.monotonic()))
        except KeyboardInterrupt:
            print("Interrupted")
        finally:
            # As in the ROS node's disarm(): ten datagrams with every device disarmed.
            disarmed = [NOMINAL.copy() for _ in payloads]
            for _ in range(10):
                sock.sendto(construct_swmc_packet(disarmed), target)
                time.sleep(interval)
    print(f"Sent {sent} command datagrams and 10 final disarm datagrams")


if __name__ == "__main__":
    main()
