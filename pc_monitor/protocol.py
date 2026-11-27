"""Giao thức khung dữ liệu STM32 -> ESP32 -> PC.

PHẢI khớp với `TelemFrame_t` trong STM32 (Core/Src/main.c).

Khung (little-endian):
    AA 55 | type (1) | len (1) | payload (len byte) | crc8 (1)
    crc8: đa thức 0x07, giá trị đầu 0x00, tính từ 'type' đến hết payload.

Thêm trường / loại khung mới:
    1. STM32: thêm trường vào cuối TelemFrame_t (hoặc tạo type mới).
    2. Ở đây: sửa FRAME_TYPES (định dạng struct + tên trường).
    ESP32 chỉ chuyển nguyên khung nên KHÔNG cần nạp lại.
"""
from __future__ import annotations

import struct

SOF = b"\xAA\x55"
UDP_PORT_ESP = 4210      # ESP32 nghe HELLO / PING / lệnh
UDP_PORT_PC = 4211       # PC nghe beacon + dữ liệu
BEACON = b"ROBOT_ESP32"

CTRL_TS = 0.002          # chu kỳ ngắt TIM6 của STM32 (s): tick * CTRL_TS = thời gian
TELEM_DIV = 1            # PHẢI khớp TELEM_DIV trong STM32 (dùng để đếm khung bị mất)

# type -> (định dạng struct của payload, tên các trường)
FRAME_TYPES: dict[int, tuple[str, tuple[str, ...]]] = {
    0x01: ("<IffffhhBB",
           ("tick", "angle", "rpm_ref", "rpm1", "rpm2",
            "duty1", "duty2", "status", "reserved")),   # reserved = số lệnh PC mà STM32 đã nhận (đếm 0..255)
    # PID đang dùng trên STM32 (ParamsFrame_t). Gửi khi khởi động, khi PC hỏi, và sau mỗi lần PC đổi PID (= xác nhận).
    0x02: ("<7fBB",
           ("bal_kp", "bal_ki", "bal_kd", "bal_sp", "mot_kp", "mot_ki", "mot_kff", "cmd_cnt", "reserved")),
}
TYPE_STATE = 0x01
TYPE_PARAMS = 0x02
# hệ số đổi đơn vị sau khi giải mã
SCALE: dict[str, float] = {"duty1": 0.01, "duty2": 0.01}   # %*100 -> %

MODE_NAMES = {0: "Dừng", 1: "Chạy hở", 2: "PID tốc độ", 3: "Cân bằng"}
FAULT_NAMES = {0: "OK", 1: "MẤT MPU", 2: "ĐỔ", 3: "?"}


def _crc8_table() -> list[int]:
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        table.append(c)
    return table


_CRC8 = _crc8_table()


def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c = _CRC8[c ^ b]
    return c


def parse_frames(buf: bytes) -> tuple[list[dict], int]:
    """Tách mọi khung hợp lệ trong 1 gói UDP. Trả về (danh sách khung, số khung lỗi CRC)."""
    out: list[dict] = []
    bad = 0
    i, n = 0, len(buf)
    while i + 5 <= n:
        if buf[i] != 0xAA or buf[i + 1] != 0x55:
            i += 1
            continue
        ftype, plen = buf[i + 2], buf[i + 3]
        end = i + 4 + plen + 1
        if end > n:
            break
        if crc8(buf[i + 2:i + 4 + plen]) != buf[end - 1]:
            bad += 1
            i += 1
            continue
        spec = FRAME_TYPES.get(ftype)
        if spec and struct.calcsize(spec[0]) == plen:
            d = dict(zip(spec[1], struct.unpack_from(spec[0], buf, i + 4)))
            for k, s in SCALE.items():
                if k in d:
                    d[k] *= s
            d["type"] = ftype
            out.append(d)
        i = end
    return out, bad


def build_frame(ftype: int, payload: bytes) -> bytes:
    """Đóng gói 1 khung (dùng cho trình giả lập, và cho lệnh PC -> STM32 sau này)."""
    body = bytes([ftype, len(payload)]) + payload
    return SOF + body + bytes([crc8(body)])


# ---- lệnh PC -> STM32 (ESP32 chuyển nguyên gói AA 55 xuống UART) ----
CMD_SET_RPM = 0x10       # payload float32 rpm - PHẢI khớp CMD_TYPE_SET_RPM trong main.c
RPM_REF_MAX = 100.0      # PHẢI khớp RPM_REF_MAX trong main.c


def build_set_rpm(rpm: float) -> bytes:
    """Lệnh đặt tốc độ chung 2 bánh (STM32 dùng khi bật PC_RPM_TEST)."""
    return build_frame(CMD_SET_RPM, struct.pack("<f", float(rpm)))


CMD_SET_BAL = 0x11       # payload 4 x float32: kp, ki, kd, góc đặt (độ) - PID góc. PHẢI khớp main.c
CMD_GET_PARAMS = 0x12    # payload rỗng: STM32 gửi lại khung 0x02


def build_set_bal(kp: float, ki: float, kd: float, sp_deg: float) -> bytes:
    """Đặt PID góc (cân bằng) + góc đặt. STM32 kẹp: Kp<=500, Ki<=10000, Kd<=50, |góc đặt|<=15."""
    return build_frame(CMD_SET_BAL, struct.pack("<4f", float(kp), float(ki), float(kd), float(sp_deg)))


def build_get_params() -> bytes:
    return build_frame(CMD_GET_PARAMS, b"")


CMD_SET_SP = 0x13        # payload float32: góc nghiêng đặt (độ), 0 = đứng thẳng. PHẢI khớp CMD_TYPE_SET_SP trong main.c
SP_MAX_DEG = 3.0         # phạm vi thanh kéo trên PC: ±3° (STM32 vẫn nhận tới ±15°)


def build_set_sp(deg: float) -> bytes:
    """Chỉ đổi góc nghiêng đặt, giữ nguyên PID."""
    return build_frame(CMD_SET_SP, struct.pack("<f", float(deg)))


def parse_beacon(data: bytes) -> dict:
    """Beacon ESP32: b"ROBOT_ESP32;rst=BROWNOUT;up=123" -> {"rst": "BROWNOUT", "up": "123"}."""
    info = {}
    for part in data.decode("ascii", errors="ignore").split(";")[1:]:
        if "=" in part:
            k, v = part.split("=", 1)
            info[k] = v
    return info


RESET_NAMES = {"POWERON": "bật nguồn", "BROWNOUT": "SỤT ÁP NGUỒN", "EXT_PIN": "nút reset",
               "SOFTWARE": "phần mềm", "PANIC": "LỖI CHƯƠNG TRÌNH", "TASK_WDT": "watchdog",
               "INT_WDT": "watchdog", "WDT": "watchdog"}


def decode_status(s: int) -> dict:
    return {
        "armed": bool(s & 0x01),
        "fault": (s >> 1) & 0x03,
        "auto_arm": bool(s & 0x08),
        "mode": (s >> 4) & 0x03,
    }
