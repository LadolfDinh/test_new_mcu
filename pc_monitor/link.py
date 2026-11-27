"""Kết nối UDP với ESP32 (chạy trong luồng riêng để không bao giờ chờ giao diện).

- Nghe cổng 4211: beacon "ROBOT_ESP32" (tự tìm IP ESP32), dữ liệu, PONG.
- Mỗi 1 s gửi "HELLO" (để ESP32 biết gửi dữ liệu về đâu) và "PING" (đo trễ WiFi).
- Khung dữ liệu nhận được đưa vào hàng đợi `frames`; giao diện lấy ra khi vẽ.
"""
from __future__ import annotations

import collections
import socket
import struct
import threading
import time

from protocol import BEACON, SOF, TELEM_DIV, TYPE_PARAMS, UDP_PORT_ESP, UDP_PORT_PC, parse_beacon, parse_frames


class UdpLink(threading.Thread):
    def __init__(self, esp_ip: str | None = None):
        super().__init__(daemon=True)
        self.manual_ip = esp_ip
        self.esp_ip: str | None = esp_ip
        self.frames: collections.deque = collections.deque(maxlen=200_000)

        # thống kê (giao diện chỉ đọc)
        self.n_frames = 0
        self.n_bad = 0
        self.n_lost = 0
        self.rtt_ms: float | None = None
        self.esp_info: dict = {}          # tu beacon: rst (ly do reset), up (giay da chay)
        self.last_rx = 0.0
        self._last_tick: int | None = None
        self.params: dict | None = None   # khung tham số (PID) mới nhất từ STM32
        self.n_params = 0                 # đếm số khung tham số đã nhận (giao diện so để biết có khung mới)

        self._running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind(("", UDP_PORT_PC))
        self.sock.settimeout(0.1)

    # ---- gửi ----
    def _send(self, data: bytes) -> None:
        if self.esp_ip:
            try:
                self.sock.sendto(data, (self.esp_ip, UDP_PORT_ESP))
            except OSError:
                pass

    def send_to_stm32(self, frame: bytes) -> None:
        """Gửi 1 khung (AA 55 ...) xuống STM32 qua ESP32 - dành cho tính năng sau này."""
        self._send(frame)

    # ---- luồng chính ----
    def run(self) -> None:
        last_hello = 0.0
        while self._running:
            now = time.monotonic()
            if self.esp_ip and now - last_hello >= 1.0:
                last_hello = now
                self._send(b"HELLO")
                self._send(b"PING" + struct.pack("<q", time.perf_counter_ns()))
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if not self._running:
                    break
                continue

            if data[:2] == SOF:
                self.last_rx = time.monotonic()
                frames, bad = parse_frames(data)
                self.n_bad += bad
                for f in frames:
                    if f.get("type") == TYPE_PARAMS:
                        self.params = f
                        self.n_params += 1
                        continue
                    self._count_lost(f["tick"])
                    self.frames.append(f)
                self.n_frames += len(frames)
            elif data.startswith(b"PONG") and len(data) >= 12:
                t0 = struct.unpack_from("<q", data, 4)[0]
                self.rtt_ms = (time.perf_counter_ns() - t0) / 1e6
            elif data.startswith(BEACON):
                self.esp_info = parse_beacon(data)
                if self.manual_ip is None and self.esp_ip != addr[0]:
                    self.esp_ip = addr[0]
                    last_hello = 0.0          # gửi HELLO ngay vòng sau

    def _count_lost(self, tick: int) -> None:
        if self._last_tick is not None:
            step = tick - self._last_tick
            if step > TELEM_DIV:
                self.n_lost += step // TELEM_DIV - 1
        self._last_tick = tick

    def reset_stats(self) -> None:
        self.n_frames = self.n_bad = self.n_lost = 0
        self._last_tick = None

    def stop(self) -> None:
        self._running = False
        try:
            self.sock.close()
        except OSError:
            pass
