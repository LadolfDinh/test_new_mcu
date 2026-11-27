# CLAUDE.md – Xe hai bánh tự cân bằng (STM32F407 + ESP32 + PC)

Trả lời người dùng bằng **tiếng Việt**. Tài liệu trong repo viết tiếng Việt; comment trong code C/ino viết
tiếng Việt **không dấu** (giữ đúng kiểu hiện có).

## Bản đồ repo
- `testNewBrain/` – firmware STM32F407VET, CubeMX 6.18 + Keil MDK-ARM (ARMCLANG 6). **Gần như toàn bộ logic
  nằm trong `Core/Src/main.c`** (PID, Kalman, encoder, telemetry, lệnh PC). Keil project: `MDK-ARM/testNewBrain.uvprojx`.
- `esp32/esp32_bridge/` – firmware ESP32 đang dùng (Arduino, ESP32 Dev Module, core 3.x). Chỉ kiểm CRC và chuyển
  nguyên khung `AA 55` giữa UART2 (921600) và UDP (ESP nghe 4210, PC nghe 4211). `esp_stm_pc/` là bản cũ.
- `pc_monitor/` – app PyQt6 + pyqtgraph: `app.py` (GUI), `link.py` (UDP), `protocol.py` (định dạng khung),
  `tuner.py` (tự chỉnh PID), `sim_esp32.py` (giả lập ESP32+STM32), `selftest.py`.
- `sim_mujoco/` – MuJoCo; `fw_build.py` trích các khối `USER CODE` của `main.c`, thay HAL bằng
  `firmware/hal_stub.h`, biên dịch thành DLL bằng gcc hoặc `python -m ziglang cc`.
- `inverted_pendulum_model/` – mô hình toán cart–pendulum (numpy/sympy/MATLAB).

## Không đọc / không sửa
- `testNewBrain/Drivers/` (HAL + CMSIS của ST), `MDK-ARM/RTE/`, `startup_stm32f407xx.s`: code nhà cung cấp.
- Thư mục `_backup*`, `testNewBrain/Claude outputs/`, `*_truoc_*`: bản sao cũ, đã bị `.gitignore`.
- `pc_monitor/logs/`, `sim_mujoco/results/`, `sim_mujoco/build/`: dữ liệu sinh ra. CSV log rất dài – muốn phân tích
  thì dùng skill `local-ai-delegate` (Codex đọc, trả tóm tắt) thay vì tự đọc.

## Quy tắc firmware STM32
- Chỉ sửa code giữa `/* USER CODE BEGIN x */ … /* USER CODE END x */` trong file CubeMX sinh ra; phần ngoài
  bị xoá khi Generate. Đổi chân/ngoại vi → sửa `testNewBrain.ioc` bằng CubeMX, không sửa tay `MX_*_Init`.
- Mô phỏng phụ thuộc đúng tên khối USER CODE (PTD, PD, PV, 0, 2, 3, 4) và các hàm `MX_TIMx_Init` – đổi tên/di chuyển
  khối thì sửa `sim_mujoco/fw_build.py`.
- Vòng điều khiển chạy trong ngắt TIM6 **1 ms** (`CTRL_TS 0.001f`). Telemetry gửi mỗi `TELEM_DIV`=2 ngắt
  (500 khung/s) và `tick = s_tick_count / 2` (đơn vị 2 ms) – vì vậy `protocol.py` có `CTRL_TS = 0.002`, `TELEM_DIV = 1`.
  Đổi một bên thì phải đổi bên kia.
- Không có toolchain Keil trong PATH của Claude: **không build được firmware ở đây**. Sau khi sửa `main.c`, nói rõ
  người dùng cần build/nạp bằng Keil (F7/F8) và kiểm tra bằng mô phỏng nếu được (skill `robot-verify-no-hw`).
- Giới hạn an toàn (`BAL_*_MAX`, `RPM_REF_MAX`, `DUTY_MAX_PCT`) là bảo vệ phần cứng – không nới mà không hỏi.

## Giao thức (STM32 ↔ ESP32 ↔ PC)
Khung: `AA 55 | type | len | payload | CRC-8` (đa thức 0x07, init 0, tính từ `type` đến hết payload), little-endian.
- STM32 → PC: `0x01` trạng thái (`TelemFrame_t`), `0x02` tham số PID (`ParamsFrame_t`).
- PC → STM32: `0x10` rpm, `0x11` PID góc + góc đặt, `0x12` hỏi tham số, `0x13` góc đặt.
Thêm/đổi trường hay lệnh → làm theo skill `robot-protocol-change` (phải sửa đồng thời `main.c`, `protocol.py`,
`sim_esp32.py`; ESP32 không cần nạp lại).

## Bí mật
WiFi SSID/mật khẩu nằm trong `esp32/*/secrets.h` (đã `.gitignore`; mẫu: `secrets.example.h`). Không bao giờ
chép mật khẩu vào `.ino`, README, commit hay prompt gửi AI khác.

## Lệnh hay dùng
```bash
cd pc_monitor && python app.py [--esp-ip <IP>]          # app PC (run.bat tự cài thư viện)
cd pc_monitor && python sim_esp32.py                    # giả lập ESP32+STM32 ở 127.0.0.1
cd sim_mujoco && python run_sim.py | test_speed_pid.py | check_vs_c.py
cd inverted_pendulum_model && PYTHONIOENCODING=utf-8 python pendulum_model.py
```
Console Windows của máy này không phải UTF-8: chạy script in tiếng Việt với `PYTHONIOENCODING=utf-8`.

## Vấn đề đang mở
- `sim_mujoco/fw_build.py::read_cubemx` vẫn đòi `htim8`, firmware đã chuyển sang TIM1 CH1/CH2 → build DLL lỗi.
