#!/usr/bin/env python3
"""Forward M20 RoboSense MSOP multicast packets between robot networks.

This is a temporary foreground diagnostic tool for NOS. It sniffs raw IPv4 UDP
packets on the lidar-side interface and re-sends their UDP payload as multicast
from the business-side interface. It avoids binding to 6691/6692, which may
already be occupied by the official rslidar process.
"""

import argparse
import socket
import struct
import time


ETH_P_IP = 0x0800
ETH_P_8021Q = 0x8100
IPPROTO_UDP = 17


def ip_bytes_to_str(data):
    return socket.inet_ntoa(data)


def parse_packet(frame):
    if len(frame) < 14:
        return None

    ether_type = struct.unpack("!H", frame[12:14])[0]
    offset = 14
    while ether_type == ETH_P_8021Q:
        if len(frame) < offset + 4:
            return None
        ether_type = struct.unpack("!H", frame[offset + 2:offset + 4])[0]
        offset += 4

    if ether_type != ETH_P_IP or len(frame) < offset + 20:
        return None

    ip0 = frame[offset]
    version = ip0 >> 4
    ihl = (ip0 & 0x0F) * 4
    if version != 4 or len(frame) < offset + ihl + 8:
        return None

    proto = frame[offset + 9]
    if proto != IPPROTO_UDP:
        return None

    total_len = struct.unpack("!H", frame[offset + 2:offset + 4])[0]
    src_ip = ip_bytes_to_str(frame[offset + 12:offset + 16])
    dst_ip = ip_bytes_to_str(frame[offset + 16:offset + 20])

    udp_offset = offset + ihl
    src_port, dst_port, udp_len, _ = struct.unpack("!HHHH", frame[udp_offset:udp_offset + 8])
    payload_offset = udp_offset + 8
    payload_len = min(udp_len - 8, total_len - ihl - 8)
    if payload_len < 0 or len(frame) < payload_offset + payload_len:
        return None

    return src_ip, src_port, dst_ip, dst_port, frame[payload_offset:payload_offset + payload_len]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--recv-iface", default="eth0")
    parser.add_argument("--send-iface-ip", default="10.21.31.106")
    parser.add_argument("--groups", nargs="+", default=["224.10.10.201", "224.10.10.202"])
    parser.add_argument("--ports", nargs="+", type=int, default=[6691, 6692])
    parser.add_argument("--stats-sec", type=float, default=1.0)
    args = parser.parse_args()

    groups = set(args.groups)
    ports = set(args.ports)

    recv_sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETH_P_IP))
    recv_sock.bind((args.recv_iface, 0))
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)

    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    send_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(args.send_iface_ip))
    send_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    send_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)

    print(
        f"Forwarding {sorted(groups)} ports {sorted(ports)} "
        f"from {args.recv_iface} to {args.send_iface_ip}"
    )

    count = 0
    bytes_total = 0
    last = time.monotonic()
    while True:
        frame = recv_sock.recv(65535)
        parsed = parse_packet(frame)
        if parsed is None:
            continue

        src_ip, src_port, dst_ip, dst_port, payload = parsed
        if dst_ip not in groups or dst_port not in ports:
            continue

        send_sock.sendto(payload, (dst_ip, dst_port))
        count += 1
        bytes_total += len(payload)

        now = time.monotonic()
        if now - last >= args.stats_sec:
            print(
                f"forwarded={count} bytes={bytes_total} "
                f"last={src_ip}:{src_port}->{dst_ip}:{dst_port} len={len(payload)}"
            )
            count = 0
            bytes_total = 0
            last = now


if __name__ == "__main__":
    main()
