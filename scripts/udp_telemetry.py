import socket
import struct
import time

import matplotlib.pyplot as plt

# Configuration
TELEM_PORT = 10250  # Port number on which your ESP device is listening
BUFFER_SIZE = 1024  # Buffer size for receiving data

packet_rate_desired = 50
collection_time = 30


def decode_stat_telem(data):
    # Telem packets are consisted of 1 byte uuid, followed by 2 byte time intervals
    msg_format = '<' + 'B' + 'H' * int((len(data) - 1) // 2)
    telem = struct.unpack(msg_format, data)

    uuid = telem[0]
    timestamps = telem[1:]

    return uuid, timestamps


def telem_listener(host='0.0.0.0', port=TELEM_PORT):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (host, port)
    sock.bind(server_address)

    print(f"Listening for UDP packets on {host}:{port}")

    start_time = time.time()
    data_dict = {}

    while True:
        # Calculate elapsed time
        elapsed_time = time.time() - start_time

        # Break the loop if the collection time has passed
        if elapsed_time >= collection_time:
            break

        # Set a timeout for the socket to periodically check the elapsed time
        sock.settimeout(1 / 500)

        try:
            # Wait for a packet
            data, address = sock.recvfrom(BUFFER_SIZE)
            uuid, timestamps = decode_stat_telem(data)

            # Add or update the dictionary with the received data
            if uuid != 0 and timestamps:
                if uuid not in data_dict:
                    data_dict[uuid] = []
                data_dict[uuid].extend(timestamps)

        except socket.timeout:
            # Handle socket timeout to periodically check the elapsed time
            continue

    return data_dict


if __name__ == "__main__":
    data_dict = telem_listener(port=TELEM_PORT)
    print(f'Received {len(data_dict)} telem packets')

    if len(data_dict) != 0:
        plt.figure()
        plt.title(f'Packet interval times')
        plt.axhline(y=(1 / packet_rate_desired) * 1000,
                    color='k',
                    linestyle='--',
                    lw=0.8,
                    label='desired')

        keys = list(data_dict.keys())
        keys.sort()
        sorted_dict = {k: data_dict[k] for k in keys}

        for key, value in sorted_dict.items():
            plt.plot(value[1:], lw=0.8, label=f'sanity{str(key).zfill(2)}')

        plt.ylabel('Time (ms)')
        plt.xlabel('Packet number')

        plt.grid(ls=':')
        plt.legend(fontsize=9)

        plt.tight_layout()
        plt.show()
