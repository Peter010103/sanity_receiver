import socket
import time
import struct

import numpy as np
from typing import List, TypedDict

# Configuration
ESP_PORT = 10240  # Port number on which your ESP device is listening
BUFFER_SIZE = 4096  # Buffer size for receiving data
BROADCAST_IP = '10.0.0.255'  # SWMC Broadcast IP address

num_agents = 25
packet_rate = 50


class PayloadDict(TypedDict):
    key1: np.uint8
    key2: np.uint16
    key3: np.uint16
    key4: np.uint16
    key5: np.uint16


def swmc_packet(payload: List[PayloadDict]):
    msg_format = (
        '>' + 'BHBHQB' +  # routing header (15 bytes)
        'B' + 'BB' +  # header count and types (1+2 bytes)
        'BHL' +  # stream header (7 bytes)
        ('BHHHH' * len(payload)) +  # payload array
        'B'  # extraneous byte
    )

    routing_header = [0, 0, 0, 0, 0, 0]
    header_count = [0]
    header_types = [5, 255]
    stream_header = [0, 0, 777]
    payload = [value for d in payload for value in d.values()]
    extraneous_byte = [0]

    return struct.pack(msg_format, *routing_header, *header_count,
                       *header_types, *stream_header, *payload,
                       *extraneous_byte)


def send_packet(packet):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.sendto(packet, (BROADCAST_IP, ESP_PORT))
    sock.close()


def send_ctrl(ctrl_command):
    payload = [ctrl_command for _ in range(num_agents)]
    packet = swmc_packet(payload)
    send_packet(packet)
    time.sleep(1 / packet_rate)


if __name__ == "__main__":
    start_time = time.time()

    while time.time() - start_time < 10:
        ctrl_command = {
            'flags': int('00000001', 2),
            'roll': 1500,
            'pitch': 1500,
            'yaw': 1500,
            'thrust': 1000,
        }

        send_ctrl(ctrl_command)

    for _ in range(10):
        ctrl_command = {
            'flags': int('00000000', 2),
            'roll': 1500,
            'pitch': 1500,
            'yaw': 1500,
            'thrust': 1000,
        }

        send_ctrl(ctrl_command)
