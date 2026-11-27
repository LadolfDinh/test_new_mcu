---
name: robot-protocol-change
description: Checklist to add or change a telemetry field, frame type or PC->STM32 command in the self-balancing robot so STM32 main.c, pc_monitor/protocol.py, sim_esp32.py, app.py and the ESP32 bridge stay in sync. Use whenever the user wants to send a new value from the STM32 to the PC, plot/log a new signal, add a new command or button that talks to the robot, or change a PID/setpoint message - e.g. "gửi thêm biến lên PC", "thêm lệnh từ PC xuống STM32", "vẽ thêm đường", "thêm trường telemetry", "đổi khung dữ liệu".
---

# Đổi giao thức STM32 ↔ ESP32 ↔ PC

Khung chung: `AA 55 | type | len | payload | CRC-8` (poly 0x07, init 0, CRC từ `type` đến hết payload),
little-endian, struct `packed`. ESP32 (`esp32_bridge.ino`) chỉ kiểm CRC rồi chuyển nguyên khung nên
**không phải nạp lại ESP32**, trừ khi payload vượt `MAX_PAYLOAD` (64 byte) trong `esp32_bridge.ino`.

## A. Thêm trường vào khung trạng thái 0x01 (STM32 → PC)

1. `testNewBrain/Core/Src/main.c`
   - Thêm trường **vào cuối** `TelemFrame_t` (trước `reserved`/`crc` nếu muốn giữ `reserved` ở cuối – nhưng
     phải sửa cả hai phía cùng thứ tự). Chỉ dùng kiểu cố định `uint8_t/int16_t/uint32_t/float`.
   - Sửa con số trong `TelemFrame_size_check` (đang là 31) = kích thước mới của cả khung.
   - Gán giá trị trong hàm gửi telemetry (chỗ `f->reserved = ...`). Mọi thứ nằm trong khối `USER CODE`.
2. `pc_monitor/protocol.py` – sửa `FRAME_TYPES[0x01]`: thêm ký tự struct (`f`, `h`, `B`, `I`…) và tên trường
   **cùng thứ tự** với C. Kiểm: `struct.calcsize(fmt) == sizeof(TelemFrame_t) - 5`.
3. `pc_monitor/sim_esp32.py` – sửa `struct.pack("<IffffhhBB", …)` cho khớp định dạng mới (nếu không, giả lập
   gửi khung sai độ dài và app báo lỗi/không có dữ liệu).
4. `pc_monitor/app.py` – muốn vẽ: thêm vào `PLOTS`; muốn ghi CSV: thêm vào `CSV_FIELDS` và dòng `writerow`.
5. Nếu `tuner.py` cần trường mới thì đọc qua dict khung (`f["ten_truong"]`).

## B. Thêm lệnh PC → STM32

1. `main.c`: thêm `#define CMD_TYPE_xxx 0x1?U` cạnh các lệnh 0x10–0x13, thêm `case` trong `Cmd_Handle`
   (kiểm `fr[1]` = độ dài payload, đọc bằng `Cmd_GetF32`, **kẹp giá trị** bằng `Clampf` với giới hạn an toàn,
   lỗi thì `g_cmd_bad++`). Muốn PC biết lệnh đã tới: đặt `s_params_pending = 1U` (STM32 gửi lại khung 0x02).
2. `protocol.py`: thêm `CMD_xxx = 0x1?` (ghi chú "PHẢI khớp main.c") và hàm `build_xxx()` dùng `build_frame`.
3. `app.py`: gọi `link.send_to_stm32(protocol.build_xxx(...))` từ nút / ô nhập.
4. Không dùng lại mã type đã có: 0x01, 0x02 (STM32 → PC); 0x10–0x13 (PC → STM32).

## C. Kiểm tra (không cần phần cứng)

```bash
cd pc_monitor
# kích thước payload từng loại khung (cộng 5 = sizeof struct C)
python -c "import struct, protocol as p; [print(hex(t), struct.calcsize(f)) for t,(f,_) in p.FRAME_TYPES.items()]"
# đóng gói rồi giải lại 1 khung 0x01 toàn số 0: phải ra 1 khung, đủ tên trường
python -c "import struct, protocol as p; f=p.FRAME_TYPES[1][0]; fr,_=p.parse_frames(p.build_frame(1, bytes(struct.calcsize(f)))); print(fr)"
python selftest.py      # giả lập 6 s: 'loi CRC' phải = 0; 'khung' > 0 (xem ghi chú dưới)
```
Ghi chú: tới 27/11/2026 `selftest.py` trên máy này báo `khung: 0`, `esp_ip: None` ngay cả trước khi sửa
(app không nhận beacon của `sim_esp32.py`; nghi tường lửa Windows chặn UDP cho python). Khi đó dựa vào hai lệnh
`python -c` ở trên để kiểm định dạng khung, và báo người dùng thử thật bằng `sim_esp32.py` + `app.py --esp-ip 127.0.0.1`.
Rồi dùng skill `robot-verify-no-hw` nếu đã sửa logic điều khiển.

## D. Báo cho người dùng
- Firmware STM32 phải build + nạp lại bằng Keil (Claude không build được Keil ở đây).
- Nói rõ ESP32 có cần nạp lại không (chỉ khi payload > 64 byte hoặc đổi baud/cổng).
- Liệt kê các file đã sửa và kích thước khung mới.
