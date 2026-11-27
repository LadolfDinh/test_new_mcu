# Xe hai bánh tự cân bằng – STM32F407 + ESP32 + PC

Đồ án cơ điện tử: xe hai bánh tự cân bằng (con lắc ngược). STM32 chạy vòng điều khiển,
ESP32 làm cầu nối WiFi, máy tính theo dõi thời gian thực và chỉnh PID từ xa.
Có thêm mô phỏng MuJoCo chạy **chính code C của firmware** và mô hình toán con lắc ngược.

```
 MPU6050 ─I2C1─┐                    USART1 921600              WiFi UDP
 Encoder x2 ───┤  STM32F407VET  ◄──────────────────►  ESP32  ◄───────────►  PC: pc_monitor/app.py
 L298N + 2 ĐC ─┘  (testNewBrain)   PA9/PA10 ↔ GPIO16/17  (esp32_bridge)  4210/4211   (PyQt6 + pyqtgraph)
                                   khung AA 55 | type | len | payload | CRC-8
```

## Phần cứng (kit)

| Khối | Linh kiện | Ghi chú |
|---|---|---|
| Vi điều khiển chính | Kit STM32F407VET6 (168 MHz) | CubeMX 6.18, Keil MDK-ARM (ARMCLANG V6.24), nạp qua SWD (PA13/PA14) |
| Cầu nối WiFi | ESP32 DevKit | Arduino IDE, board "ESP32 Dev Module", lõi *esp32 by Espressif* 3.x |
| Cảm biến góc | MPU6050 | I2C1 400 kHz, lọc Kalman gyro + gia tốc |
| Động cơ | 2 động cơ DC có encoder (11 xung/vòng, hộp số 1:45) | đọc cả 2 sườn kênh A qua EXTI |
| Driver | L298N | PWM TIM1 10 kHz |

### Bảng chân STM32

| Chức năng | Chân | Chức năng | Chân |
|---|---|---|---|
| USART1 TX → ESP32 GPIO16 (RX2) | PA9 | USART1 RX ← ESP32 GPIO17 (TX2) | PA10 |
| MPU6050 SCL / SDA | PB6 / PB7 | Nút K1 / LED D2 | PA0 / PA1 |
| Encoder 1 A / B | PE2 / PE3 | Encoder 2 A / B | PE4 / PE5 |
| ĐC1 PWM (TIM1_CH1) | PE9 | ĐC2 PWM (TIM1_CH2) | PE11 |
| ĐC1 IN1 / IN2 | PE7 / PE8 | ĐC2 IN3 / IN4 | PE10 / PE12 |

Nhớ nối chung GND giữa STM32 và ESP32.

## Cấu trúc thư mục

| Thư mục | Nội dung | Tài liệu |
|---|---|---|
| [testNewBrain/](testNewBrain/) | Firmware STM32 (CubeMX + Keil). Code chính: `Core/Src/main.c` | – |
| [esp32/esp32_bridge/](esp32/esp32_bridge/) | **Firmware ESP32 đang dùng**: UART ↔ WiFi UDP, tự tìm PC bằng beacon | đầu file `.ino` |
| [esp32/esp_stm_pc/](esp32/esp_stm_pc/) | Bản ESP32 cũ (gửi chuỗi text "goc,toc_do") – chỉ để tham khảo | đầu file `.ino` |
| [pc_monitor/](pc_monitor/) | App PC: vẽ đồ thị, ghi CSV, chỉnh PID góc, tự chỉnh PID, giả lập ESP32 | [README](pc_monitor/README.md) |
| [sim_mujoco/](sim_mujoco/) | Mô phỏng MuJoCo, biên dịch các khối `USER CODE` của `main.c` thành DLL | [README](sim_mujoco/README.md) |
| [inverted_pendulum_model/](inverted_pendulum_model/) | Mô hình toán con lắc ngược (numpy, sympy, MATLAB) | [README](inverted_pendulum_model/README.md) |

## Firmware STM32 – tóm tắt

- Tất cả chạy trong ngắt **TIM6 mỗi 1 ms** (`CTRL_TS`): đọc MPU6050 → Kalman → PID góc → tốc độ đặt → 2 PID tốc độ động cơ → PWM.
- Chế độ thử (`TEST_*` trong `main.c`): 0 dừng · 1 duty cố định · 2 hai PID động cơ · 3 cân bằng.
- Telemetry khung `0x01` gửi bằng DMA 500 khung/s (`TELEM_DIV 2`); trường `tick` tính theo đơn vị 2 ms.
- Lệnh từ PC (DMA vòng, đọc mỗi 1 ms): `0x10` đặt rpm · `0x11` đặt PID góc + góc đặt · `0x12` hỏi tham số (trả khung `0x02`) · `0x13` đặt góc nghiêng.

## Bắt đầu nhanh

### 0. Lấy code
```bash
git clone https://github.com/LadolfDinh/test_new_mcu.git
```

### 1. STM32
1. Mở `testNewBrain/MDK-ARM/testNewBrain.uvprojx` bằng Keil µVision 5 → Build (F7) → Download (F8).
2. Đổi cấu hình chân/ngoại vi: mở `testNewBrain/testNewBrain.ioc` bằng STM32CubeMX rồi *Generate Code*.
   **Chỉ viết code giữa các cặp `/* USER CODE BEGIN */ … /* USER CODE END */`**, nếu không CubeMX sẽ xoá.
3. Thư mục `Drivers/CMSIS/{DSP,NN,DAP,RTOS,RTOS2,Core_A}` không có trong repo (project không dùng);
   CubeMX sẽ tự chép lại khi Generate.

### 2. ESP32
1. Copy `esp32/esp32_bridge/secrets.example.h` thành `secrets.h` (cùng thư mục), điền tên và mật khẩu WiFi.
   `secrets.h` nằm trong `.gitignore`, không bị đẩy lên GitHub.
2. Arduino IDE → board *ESP32 Dev Module* → nạp `esp32_bridge.ino`. Serial Monitor 115200 in ra IP của ESP32.

### 3. PC
```bash
cd pc_monitor
run.bat                       # hoặc: pip install -r requirements.txt && python app.py
python app.py --esp-ip <IP>   # nếu không tự tìm được ESP32
```
Lần đầu Windows hỏi tường lửa cho `python.exe` → **Allow** (mạng Private).

### 4. Thử không cần phần cứng
```bash
cd pc_monitor && python sim_esp32.py          # cửa sổ 1: giả lập ESP32 + STM32
cd pc_monitor && python app.py --esp-ip 127.0.0.1
cd sim_mujoco && pip install mujoco matplotlib numpy pillow ziglang && python run_sim.py
cd inverted_pendulum_model && python pendulum_model.py
```

## Những chỗ phải khớp nhau khi sửa

| Đổi gì | Sửa đồng thời |
|---|---|
| Thêm trường telemetry | `TelemFrame_t` trong `main.c` **và** `FRAME_TYPES` trong `pc_monitor/protocol.py` **và** `struct.pack` trong `pc_monitor/sim_esp32.py` (ESP32 không cần nạp lại) |
| Thêm lệnh PC → STM32 | `CMD_TYPE_*` + `Cmd_Handle` trong `main.c` **và** hàm `build_*` trong `protocol.py` |
| Đổi baud UART | CubeMX (USART1) **và** `STM32_BAUD` trong `esp32_bridge.ino` |
| Đổi chu kỳ TIM6 | CubeMX **và** `CTRL_TS` trong `main.c`; giữ `tick` theo đơn vị 2 ms hoặc sửa `CTRL_TS`/`TELEM_DIV` trong `protocol.py` |
| Đổi timer PWM | `main.c` **và** `read_cubemx()` trong `sim_mujoco/fw_build.py` |

## Vấn đề đã biết (27/11/2026)

- `sim_mujoco/fw_build.py` vẫn đọc cấu hình **TIM8**, trong khi firmware đã chuyển sang TIM1 hai kênh →
  `check_vs_c.py` / biên dịch lại DLL báo lỗi `TypeError` ở `read_cubemx`. Cần sửa `fw_build.py` theo TIM1 2 kênh.
- `pc_monitor/selftest.py` báo `khung: 0`, `esp_ip: None`: app không nhận beacon của `sim_esp32.py`
  (nghi tường lửa Windows chặn UDP cho python). Chạy tay `sim_esp32.py` + `app.py --esp-ip 127.0.0.1` để thử.
- `pendulum_model.py` in tiếng Việt lỗi `UnicodeEncodeError` trên console không phải UTF-8 → chạy với
  `set PYTHONIOENCODING=utf-8` (PowerShell: `$env:PYTHONIOENCODING="utf-8"`).

## Làm việc với Claude Code

- [CLAUDE.md](CLAUDE.md): hướng dẫn cho Claude về cấu trúc, quy tắc và các điểm phải khớp.
- Skill của dự án trong [.claude/skills/](.claude/skills/):
  - `robot-protocol-change` – thêm trường telemetry / lệnh mới đồng bộ STM32 ↔ PC ↔ mô phỏng.
  - `robot-verify-no-hw` – kiểm tra thay đổi khi không có phần cứng (giả lập ESP32, MuJoCo, mô hình toán).
- Việc đọc nhiều (cả repo, log CSV dài) nên dùng skill `local-ai-delegate` để nhờ Codex/Copilot đọc giúp.
