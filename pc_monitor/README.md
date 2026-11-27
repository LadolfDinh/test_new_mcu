# pc_monitor – Theo dõi robot tự cân bằng thời gian thực

```
STM32 (USART1 921600, DMA, 500 khung/s) --UART--> ESP32 (esp32/esp32_bridge) --WiFi UDP--> app.py (máy tính)
```

## Chạy
- Nháy đúp `run.bat` (tự cài thư viện nếu thiếu), hoặc `python app.py`.
- Máy tính và ESP32 phải cùng mạng WiFi. App tự tìm ESP32 qua beacon. Nếu không tìm thấy: `python app.py --esp-ip <IP ESP32>` (IP in ra Serial Monitor của ESP32, 115200).
- **Lần đầu chạy, Windows hỏi tường lửa cho python.exe → chọn Cho phép (Allow)**, tích cả mạng Private. Nếu bỏ qua thì app không nhận được gói nào.

## Giao diện
- Biểu đồ 1: tốc độ đặt + tốc độ thực 2 động cơ (rpm). Bấm vào chú thích để ẩn/hiện một đường.
- Biểu đồ 2: góc nghiêng đã lọc (độ) – đúng giá trị đưa vào PID góc.
- Dòng trạng thái: số khung/s, số khung mất, lỗi CRC, trễ WiFi khứ hồi (RTT), chế độ, lỗi, duty.
- Nút: Tạm dừng, Xoá, Ghi CSV (lưu vào `logs/`), ô chọn độ dài cửa sổ thời gian.
- Trục thời gian lấy theo `tick` của STM32 (mỗi tick 2 ms), nên đúng thời điểm lấy mẫu, không bị lệch theo độ trễ WiFi.

## Thử không cần phần cứng
- `python sim_esp32.py` rồi `python app.py --esp-ip 127.0.0.1`, hoặc
- `python selftest.py`: chạy giả lập 6 s, lưu ảnh `logs/test_giaodien.png`.
- Ở ESP32 có `SIMULATE_STM32 1` để thử WiFi + app khi chưa nối STM32.

## Thêm tính năng
| Muốn | Sửa ở đâu |
|---|---|
| Thêm đường vào biểu đồ có sẵn / thêm biểu đồ | `PLOTS` trong `app.py` |
| Gửi thêm số liệu từ STM32 | thêm trường cuối `TelemFrame_t` (STM32 `main.c`) + sửa `FRAME_TYPES` trong `protocol.py`. **ESP32 không cần nạp lại.** |
| Gửi lệnh từ PC xuống STM32 (chỉnh PID…) | `link.send_to_stm32(protocol.build_frame(type, payload))` – ESP32 đã chuyển sẵn xuống UART; STM32 cần viết thêm phần nhận |

## Độ trễ
- STM32: gửi ngay cuối mỗi chu kỳ điều khiển bằng DMA (31 byte ≈ 0,34 ms), không chặn vòng điều khiển.
- ESP32: UART báo ngay sau 1 ký tự im lặng, chuyển UDP tức thì; tắt modem-sleep WiFi (nguyên nhân trễ 100 ms phổ biến nhất).
- PC: luồng nhận riêng, vẽ 60 khung hình/giây.
- Dùng UDP: mất gói thì bỏ qua (đếm ở "mất"), không chờ gửi lại như TCP.

## Chỉnh PID góc từ PC (tab "PID góc", 26/11/2026)
- STM32 phải ở chế độ cân bằng (`PC_RPM_TEST 0` trong `main.c`). App tự hỏi PID đang chạy (lệnh `0x12`), STM32 trả khung `0x02`.
- **Góc nghiêng đặt** (27/11): thanh kéo DỌC ở cột ngoài cùng bên phải cửa sổ, ±3°, bước 0.05° (phím mũi tên 0.05°, PageUp/Down 0.5°), nút +0.05° / −0.05° / Về 0°. **Lò xo** (ô "Lò xo", mặc định bật): thả chuột khỏi thanh kéo thì góc tự trôi mượt về 0° (mỗi 30 ms còn 85 %, từ 3° về 0 khoảng 0.5 s); nhấn giữ lại thì dừng. Nút ± và phím vẫn giữ nguyên góc; bỏ chọn "Lò xo" để giữ góc khi kéo. Gửi lệnh `0x13` (chỉ đổi góc, giữ PID). Thanh kéo luôn hiện góc thật trên STM32 (STM32 khởi động = 0°); kéo xong mà mất gói thì tự gửi lại trong 2 s. Bị khoá khi đang tự chỉnh.
- **Chỉnh tay:** sửa Kp / Ki / Kd rồi bấm "Gửi xuống STM32" (lệnh `0x11`, kèm góc đặt của thanh kéo; tự gửi lại đến khi STM32 xác nhận). Giá trị mất khi STM32 reset: chép vào `main.c`.
- **Tự chỉnh** (`tuner.py`): dựng xe cho tự chạy, bấm "Bắt đầu tự chỉnh". Mỗi bộ PID: chờ 1.5 s rồi đo 4 s; thử tăng/giảm từng hệ số
  Kp → Kd → Ki (±20 %, giảm dần còn 5 %), giữ bộ nào làm xe ít rung hơn. Thường 30–45 bộ, khoảng 4–5 phút.
  - Chi phí J = σ góc (°) + 0.02·σ tốc độ bánh (rpm) + 5·tỉ lệ duty kịch. Hệ số đổi trong `TuneConfig` (`tuner.py`).
  - Xe lệch > 20° hoặc đổ: tự gửi lại bộ tốt nhất, chờ xe được dựng lên và đứng yên 2 s rồi thử tiếp; không thử vượt giá trị vừa làm đổ.
    Hỏng 3 lần liên tiếp thì dừng.
  - "Dừng" / đóng app: áp dụng bộ tốt nhất. Kết quả từng lần thử: `logs/tune_*.csv`.
  - "Chép #define": copy 3 dòng `BAL_KP/KI/KD` để dán vào `main.c`.
