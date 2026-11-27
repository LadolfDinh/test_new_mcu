# Mô phỏng MuJoCo – xe 2 bánh tự cân bằng, chạy **code C thật** của `main.c`

`fw_build.py` lấy nguyên các khối `USER CODE` (PTD, PD, PV, 0, 2, 3, 4) trong `testNewBrain/Core/Src/main.c`
rồi biên dịch thành thư viện chạy trên PC, thay HAL bằng `firmware/hal_stub.h`.
**Không có PID nào chép tay.** Sửa `main.c` thì lần chạy sau tự biên dịch lại.

## Thời gian lấy mẫu và PID rời rạc khớp với code như thế nào
| Thành phần | Trên xe | Trong mô phỏng |
|---|---|---|
| Chu kỳ điều khiển | TIM6: PSC 83, ARR 1999 → 2 ms | đọc PSC/ARR từ `MX_TIM6_Init`, gọi `HAL_TIM_PeriodElapsedCallback(&htim6)` mỗi 2 ms |
| PID góc, 2 PID động cơ | `PID_UpdateRate`, `PID_Update` với `Ts = CTRL_TS` | **chính các hàm đó** (đã biên dịch) |
| MPU6050 | I2C đọc 14 byte, DLPF 44 Hz, lấy mẫu 1 kHz | ghi thanh ghi 0x3B–0x48 theo cấu hình code đã ghi vào 0x19/0x1A/0x1B/0x1C |
| Kalman, `dt` | `DWT->CYCCNT` | `DWT->CYCCNT` chạy theo thời gian mô phỏng (168 MHz) |
| Encoder | EXTI cả 2 sườn kênh A, đọc B | sinh xung A/B 11 xung/vòng × 1:45 trên đúng chân trong `main.h`, mỗi sườn A gọi `HAL_GPIO_EXTI_Callback` |
| Đo tốc độ | cửa sổ 5 mẫu (10 ms) theo DWT, lọc thêm với hệ số 0.5 | `Encoder_GetRpm` thật |
| L298N | TIM1/TIM8 PWM 10 kHz, IN1–IN4 | đọc CCR/ARR và mức chân IN, áp lên động cơ sau 0.5 ms (thời gian chạy ngắt) |
| Khởi động | `main()`: `PID_Init`, hiệu chuẩn 2×500 mẫu, `Auto_Arm_Poll` | chạy nguyên khối USER CODE 2 và vòng `while(1)` |

Kiểm tra: `python check_vs_c.py` so bản Python (`controller.py`) với hàm C. Sai lệch nhỏ hơn 4e-4 (do float32 so với float64).

## Cài đặt (Windows)
```
pip install mujoco matplotlib numpy pillow
```
- Thư viện `build/fw_sim.dll` đã dựng sẵn cho bản `main.c` hiện tại.
- Khi **sửa `main.c`**, cần một trình biên dịch C để tự biên dịch lại: `pip install ziglang` (không phải cài MinGW), hoặc có `gcc` trong PATH.
- Không có trình biên dịch thì script tự dùng bản Python `controller.py` (đã kiểm tra khớp, nhưng không cập nhật theo `main.c`).

## Chạy
| Lệnh | Việc |
|---|---|
| `python test_speed_pid.py` | **Chế độ 2** (`TEST_SPEED_PID`): nhấc bánh, 2 PID động cơ bám `g_test_rpm` = 40 → 80 → 20 → −40 → 0 → `results/speed_pid_10kHz.png` |
| `python run_sim.py` | **Chế độ 3** (cân bằng): PID góc → tốc độ đặt → 2 PID động cơ. So sánh *MPU6050 + Kalman* và *góc lý tưởng* → `results/run_*.png` |
| `python run_sim.py --mass 1.0 --com 0.15 --offset 0.5 --kp 20` | đổi thân xe, trọng tâm (m), lệch góc đặt (°, ghi vào `g_bal_setpoint_deg`), PID (ghi vào struct như cửa sổ Watch) |
| `python sweep.py` | quét khối lượng × trọng tâm → `results/sweep.png` |
| `python viewer.py` | xem 3D. Phím **P**/**O** đẩy xe tới/lui, **R** thả lại |
| `python render_video.py` | xuất `results/so_sanh.mp4` (cần ffmpeg) |

## Giả định (chưa đo, sửa trong `params.py`)
| Đại lượng | Giá trị |
|---|---|
| Thân xe (trừ bánh) | 0.8 kg, trọng tâm 12 cm trên trục bánh (quét 0.4–1.5 kg, 4–22 cm) |
| Động cơ | R 12 Ω, Ke 0.0185 V·s/rad, L 0.3 mH, 1:45, hiệu suất 0.7, ma sát 0.04 N·m → không tải khoảng 103 rpm |
| L298N | sụt áp 2 V, khi tắt xả qua diode → vùng chết khoảng 20 % ở 10 kHz |
| MPU6050 | cao 8 cm trên trục, bias gyro 1.5 °/s, nhiễu và rung khi động cơ chạy |

**Nên đo:** cho chạy chế độ 1 (`g_test_duty` 10, 20, … 90 %) và ghi lại rpm. Đường duty → rpm này là thứ quyết định mô hình động cơ có đúng hay không.

## Kết quả với thông số hiện tại trong `main.c`
- **PID động cơ (chế độ 2, bánh nhấc):** lên 90 % sau 16–26 ms, vọt lố 24–35 %, ổn định ±5 % sau 0.18–0.26 s.
  Vọt lố vì feedforward 0.99 %/rpm giả định duty tỉ lệ thuận với tốc độ, trong khi mô hình có vùng chết (40 rpm chỉ cần 27 % duty).
  Đặt 0 rpm thì khâu I giữ khoảng 15 % duty mà bánh vẫn đứng yên.
- **Cân bằng (chế độ 3):** với MPU6050 + Kalman, xe đổ ở cả 48/48 tổ hợp, phần lớn sau 0.8–1 s (trọng tâm 4 cm thì trụ được 2–8 s).
  Với góc lý tưởng thì đứng được, nhưng lệch góc đặt 0.5° hoặc bị đẩy 0.5 N là đổ sau 1–3 s, vì chưa có vòng tốc độ xe.
- Code cho chạy **ngay khi hiệu chuẩn xong** (`t_ok` của `Auto_Arm_Poll` bắt đầu từ 0), lúc Kalman còn hệ số lớn.
  Vì vậy góc ước lượng lệch 1–2° ngay vài trăm ms đầu.
