"""Giả lập ESP32 + STM32 ngay trên máy tính để thử giao diện khi chưa có phần cứng.

Chạy 2 cửa sổ dòng lệnh:
    python sim_esp32.py
    python app.py --esp-ip 127.0.0.1

Giả lập đúng giao thức thật: beacon, HELLO, PING/PONG, khung AA 55 ... 500 khung/s.
"""
from __future__ import annotations

import math
import random
import socket
import struct
import time

from protocol import BEACON, CTRL_TS, UDP_PORT_ESP, UDP_PORT_PC, build_frame


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", UDP_PORT_ESP))
    sock.setblocking(False)
    client = None
    tick = 0
    t0 = time.perf_counter()
    last_beacon = 0.0
    rpm1 = rpm2 = 0.0
    print(f"Gia lap ESP32 tren 127.0.0.1:{UDP_PORT_ESP} - Ctrl+C de thoat")

    while True:
        now = time.perf_counter()
        if now - last_beacon >= 1.0:
            last_beacon = now
            sock.sendto(BEACON, ("127.0.0.1", UDP_PORT_PC))

        try:
            while True:
                data, addr = sock.recvfrom(1024)
                if data.startswith(b"HELLO"):
                    if client != addr:
                        print("PC ket noi:", addr)
                    client = addr
                elif data.startswith(b"PING"):
                    sock.sendto(b"PONG" + data[4:], addr)
        except (BlockingIOError, ConnectionResetError):
            pass

        # sinh khung đúng nhịp 2 ms
        while (tick + 1) * CTRL_TS <= now - t0:
            tick += 1
            t = tick * CTRL_TS
            angle = 2.5 * math.sin(2 * math.pi * 0.8 * t) + 0.6 * math.sin(2 * math.pi * 3.1 * t) \
                + random.gauss(0, 0.05)
            ref = 23.0 * math.radians(angle) * 9.549 * 3.0
            rpm1 += 0.15 * (ref - rpm1) + random.gauss(0, 1.5)
            rpm2 += 0.12 * (ref - rpm2) + random.gauss(0, 1.5)
            d1, d2 = int(0.8 * ref * 100), int(0.8 * ref * 100)
            payload = struct.pack("<IffffhhBB", tick, angle, ref, rpm1, rpm2,
                                  max(-9500, min(9500, d1)), max(-9500, min(9500, d2)),
                                  0x01 | 0x08 | (3 << 4), 0)
            if client:
                sock.sendto(build_frame(0x01, payload), client)
        time.sleep(0.0005)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
