---
name: robot-verify-no-hw
description: How to check a change to the self-balancing robot without the real hardware - runs the MuJoCo simulation that compiles the real STM32 main.c, the C-vs-Python controller check, the ESP32/STM32 simulator for the PC app, and the inverted-pendulum math model, then reports what passed. Use after editing testNewBrain/Core/Src/main.c (PID, Kalman, encoder, PWM, timing), pc_monitor, sim_mujoco or inverted_pendulum_model, or when the user asks "chạy thử", "kiểm tra không cần xe", "mô phỏng lại", "test PID trên máy tính", "xe có đứng được không".
---

# Kiểm tra không cần phần cứng

Claude **không build/nạp được firmware Keil** ở đây; các bước dưới đây là cách duy nhất để thử thay đổi.
Chạy từ gốc repo. Console Windows không phải UTF-8 → đặt `PYTHONIOENCODING=utf-8` cho mọi lệnh.
Mô phỏng chạy lâu (vài chục giây – vài phút): đặt timeout Bash 600000 ms.

## 1. Chọn bước theo file đã sửa

| Đã sửa | Chạy |
|---|---|
| `main.c` – PID, Kalman, encoder, PWM, chu kỳ | 2 → 3 (và 4 nếu đổi PID động cơ) |
| `main.c` – telemetry / lệnh PC | skill `robot-protocol-change`, rồi 5 |
| `pc_monitor/*` | 5 |
| `sim_mujoco/*` | 2 → 3 |
| `inverted_pendulum_model/*` | 6 |

## 2. Biên dịch main.c cho PC và so với bản Python
```bash
cd sim_mujoco && PYTHONIOENCODING=utf-8 python check_vs_c.py
```
- Cần `gcc` trong PATH hoặc `pip install ziglang`. Sai lệch C ↔ Python phải < 4e-4.
- **Lỗi đã biết:** `TypeError ... r['htim8']` trong `fw_build.py::read_cubemx` – firmware đã bỏ TIM8 (2 động cơ
  dùng TIM1 CH1/CH2). Đến khi `fw_build.py` được sửa, bước 2–4 sẽ dừng ở đây: báo người dùng, đừng tự che lỗi
  bằng cách lùi `main.c`.

## 3. Cân bằng (chế độ 3)
```bash
cd sim_mujoco && PYTHONIOENCODING=utf-8 python run_sim.py            # thêm --kp/--mass/--com/--offset nếu cần
```
Đọc ảnh/CSV mới nhất trong `sim_mujoco/results/run_*.png|csv`: xe có đứng được không, góc dao động bao nhiêu độ,
duty có chạm kịch không. CSV dài → nhờ skill `local-ai-delegate` tóm tắt thay vì tự đọc.

## 4. PID tốc độ động cơ (chế độ 2, bánh nhấc)
```bash
cd sim_mujoco && PYTHONIOENCODING=utf-8 python test_speed_pid.py      # -> results/speed_pid_*.png
```
Báo thời gian lên, vọt lố, thời gian ổn định (so với README của sim_mujoco).

## 5. App PC với giả lập ESP32 + STM32
```bash
cd pc_monitor && PYTHONIOENCODING=utf-8 python selftest.py           # 6 s, lưu logs/test_giaodien.png
```
Đọc dòng `khung / mat / loi CRC`. Lỗi đã biết (27/11/2026): `khung: 0`, `esp_ip: None` (app không nhận beacon
của giả lập, nghi tường lửa) – khi đó chỉ khẳng định được là app khởi động không lỗi; nhờ người dùng chạy tay
`python sim_esp32.py` + `python app.py --esp-ip 127.0.0.1`. Xem ảnh `logs/test_giaodien.png` bằng Read.

## 6. Mô hình toán
```bash
cd inverted_pendulum_model && PYTHONIOENCODING=utf-8 python pendulum_model.py
```
(`pendulum_symbolic.py` cần `pip install sympy`.)

## Báo cáo
Liệt kê bước đã chạy → kết quả (đạt / lỗi + dòng lỗi chính) → bước không chạy được và lý do. Luôn nhắc:
kết quả mô phỏng dùng thông số giả định trong `sim_mujoco/params.py`; cần thử trên xe thật sau khi nạp bằng Keil.
