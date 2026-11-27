"""Giao diện theo dõi robot tự cân bằng - vẽ thời gian thực dữ liệu từ STM32 (qua ESP32, WiFi UDP).

Chạy:   python app.py                 (tự tìm ESP32 qua beacon)
        python app.py --esp-ip 192.168.1.50
        python app.py --window 20     (hiển thị 20 s gần nhất)

Thêm đường / biểu đồ mới: sửa danh sách PLOTS bên dưới (key = tên trường trong protocol.FRAME_TYPES).
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from link import UdpLink
from protocol import (CTRL_TS, FAULT_NAMES, MODE_NAMES, RESET_NAMES, RPM_REF_MAX, SP_MAX_DEG, build_get_params,
                      build_set_bal, build_set_rpm, build_set_sp, decode_status)
from tuner import AutoTuner, metrics

# ======================= CẤU HÌNH BIỂU ĐỒ =======================
PLOTS = [
    {
        "title": "Tốc độ bánh xe",
        "unit": "rpm",
        "curves": [
            {"key": "rpm_ref", "name": "Tốc độ đặt", "color": "#ffd400", "z": 10},   # z lớn = vẽ đè lên trên
            {"key": "rpm1", "name": "Động cơ 1 (encoder)", "color": "#3da5ff"},
            {"key": "rpm2", "name": "Động cơ 2 (encoder)", "color": "#ff6b3d"},
        ],
    },
    {
        "title": "Góc nghiêng đã lọc (đưa vào PID)",
        "unit": "độ",
        "curves": [
            {"key": "angle", "name": "Góc nghiêng", "color": "#4cd964"},
        ],
    },
]
HISTORY_S = 120          # giữ tối đa 120 s dữ liệu trong bộ nhớ
REFRESH_MS = 16          # ~60 khung hình/giây
CMD_PERIOD_MS = 50       # gửi tốc độ đặt 20 lần/s (STM32 tự về 0 nếu 0.5 s không nhận được lệnh)
SP_STEP_DEG = 0.05       # bước thanh kéo góc nghiêng đặt (độ)
SPRING_MS = 30           # lò xo: chu kỳ kéo về 0 (ms)
SPRING_K = 0.85          # lò xo: mỗi chu kỳ còn 85 % -> từ 3° về 0° trong ~0.5 s, mượt, không giật
CSV_FIELDS = ["pc_time", "tick", "t_s", "angle", "rpm_ref", "rpm1", "rpm2", "duty1", "duty2", "status"]


class Ring:
    """Bộ đệm vòng numpy, luôn trả về mảng liên tục (mỗi mẫu ghi 2 nơi) -> vẽ không phải copy."""

    def __init__(self, cap: int):
        self.cap = cap
        self.data = np.zeros(2 * cap)
        self.pos = 0
        self.count = 0

    def extend(self, values) -> None:
        v = np.asarray(values, dtype=float)[-self.cap:]
        k = len(v)
        if k == 0:
            return
        p = self.pos
        first = min(k, self.cap - p)
        self.data[p:p + first] = v[:first]
        self.data[p + self.cap:p + self.cap + first] = v[:first]
        rest = k - first
        if rest:
            self.data[:rest] = v[first:]
            self.data[self.cap:self.cap + rest] = v[first:]
        self.pos = (p + k) % self.cap
        self.count = min(self.cap, self.count + k)

    def view(self) -> np.ndarray:
        end = self.pos + self.cap
        return self.data[end - self.count:end]

    def clear(self) -> None:
        self.pos = self.count = 0


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, link: UdpLink, window_s: float):
        super().__init__()
        self.link = link
        self.setWindowTitle("Robot tự cân bằng - theo dõi thời gian thực")
        self.resize(1200, 760)

        cap = int(HISTORY_S / CTRL_TS)
        keys = {c["key"] for p in PLOTS for c in p["curves"]}
        self.buf = {k: Ring(cap) for k in keys | {"t"}}
        self.last = None          # khung mới nhất
        self.last_tick = None
        self.paused = False
        self.csv_file = None
        self.csv_writer = None
        self._rate_n, self._rate_t, self.rate = 0, time.monotonic(), 0.0

        # ---- thanh điều khiển ----
        bar = QtWidgets.QHBoxLayout()
        self.lbl_status = QtWidgets.QLabel("Đang tìm ESP32...")
        self.lbl_status.setStyleSheet("font-family: Consolas, monospace; font-size: 12px;")
        bar.addWidget(self.lbl_status, 1)
        bar.addWidget(QtWidgets.QLabel("Cửa sổ (s):"))
        self.spin_win = QtWidgets.QDoubleSpinBox()
        self.spin_win.setRange(1, HISTORY_S)
        self.spin_win.setValue(window_s)
        self.spin_win.setSingleStep(1)
        bar.addWidget(self.spin_win)
        self.btn_pause = QtWidgets.QPushButton("Tạm dừng")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(self._on_pause)
        bar.addWidget(self.btn_pause)
        btn_clear = QtWidgets.QPushButton("Xoá")
        btn_clear.clicked.connect(self._on_clear)
        bar.addWidget(btn_clear)
        self.btn_rec = QtWidgets.QPushButton("Ghi CSV")
        self.btn_rec.setCheckable(True)
        self.btn_rec.toggled.connect(self._on_record)
        bar.addWidget(self.btn_rec)

        # ---- biểu đồ ----
        pg.setConfigOptions(antialias=False, background="#111418", foreground="#d0d4d8")
        self.glw = pg.GraphicsLayoutWidget()
        self.curves: list[tuple[str, pg.PlotDataItem]] = []
        first_plot = None
        for i, cfg in enumerate(PLOTS):
            p = self.glw.addPlot(row=i, col=0, title=cfg["title"])
            p.showGrid(x=True, y=True, alpha=0.25)
            p.setLabel("left", cfg["unit"])
            p.setLabel("bottom", "thời gian (s, 0 = hiện tại)")
            p.addLegend(offset=(10, 10))
            p.enableAutoRange("y", True)
            p.setMouseEnabled(x=False, y=True)
            if first_plot is None:
                first_plot = p
            else:
                p.setXLink(first_plot)
            for c in cfg["curves"]:
                item = p.plot(pen=pg.mkPen(c["color"], width=2), name=c["name"])
                item.setClipToView(True)
                item.setDownsampling(auto=True, method="peak")
                item.setSkipFiniteCheck(True)
                item.setZValue(c.get("z", 0))
                self.curves.append((c["key"], item))
        self.first_plot = first_plot

        # ---- cần gạt tốc độ đặt: PC -> ESP32 -> STM32 ----
        panel = QtWidgets.QGroupBox("Tốc độ đặt\n(PC → STM32)")
        pv = QtWidgets.QVBoxLayout(panel)
        self.chk_send = QtWidgets.QCheckBox("Gửi lệnh")
        self.chk_send.setChecked(True)
        pv.addWidget(self.chk_send)
        self.lbl_rpm = QtWidgets.QLabel("+0 rpm")
        self.lbl_rpm.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.lbl_rpm.setStyleSheet("font-size: 16px; font-weight: bold;")
        pv.addWidget(self.lbl_rpm)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Vertical)
        self.slider.setRange(-int(RPM_REF_MAX), int(RPM_REF_MAX))
        self.slider.setValue(0)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(10)
        self.slider.setTickInterval(10)
        self.slider.setTickPosition(QtWidgets.QSlider.TickPosition.TicksBothSides)
        self.slider.setMinimumHeight(300)
        self.slider.valueChanged.connect(self._on_slider)
        pv.addWidget(self.slider, 1, QtCore.Qt.AlignmentFlag.AlignHCenter)
        btn_zero = QtWidgets.QPushButton("Về 0")
        btn_zero.clicked.connect(lambda: self.slider.setValue(0))
        pv.addWidget(btn_zero)
        self.lbl_cmd = QtWidgets.QLabel("gửi   --\nnhận  --")
        self.lbl_cmd.setStyleSheet("font-family: Consolas, monospace; font-size: 11px;")
        self.lbl_cmd.setToolTip("Đếm vòng 0..255. 'nhận' = số lệnh STM32 đã nhận (gửi ngược về trong telemetry).")
        pv.addWidget(self.lbl_cmd)
        self.n_cmd_sent = 0
        self.cmd_timer = QtCore.QTimer(self)
        self.cmd_timer.timeout.connect(self._send_cmd)
        self.cmd_timer.start(CMD_PERIOD_MS)

        # ---- PID góc: chỉnh tay + tự chỉnh (tuner.py) ----
        self.tuner = AutoTuner(self._send_bal, self._tune_log)
        self._seen_params = 0
        self._params_loaded = False
        self._t_get = 0.0
        self._sp_target: float | None = None   # góc đặt vừa kéo, chờ STM32 xác nhận (None = thanh kéo theo STM32)
        self._t_sp_sent = 0.0
        self._t_sp_change = 0.0
        self._pid_target: dict | None = None   # bộ PID gửi tay, chờ STM32 xác nhận
        self._t_pid_sent = self._t_pid_change = 0.0
        tabs = QtWidgets.QTabWidget()
        tabs.addTab(panel, "Tốc độ")
        tabs.addTab(self._build_pid_tab(), "PID góc")
        tabs.setCurrentIndex(1)
        tabs.setMaximumWidth(420)

        central = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(central)
        lay.addLayout(bar)
        body = QtWidgets.QHBoxLayout()
        body.addWidget(self.glw, 1)
        body.addWidget(tabs)
        body.addWidget(self._build_sp_panel())
        lay.addLayout(body, 1)
        self.setCentralWidget(central)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(REFRESH_MS)
        self._status_t = 0.0

    # ---------------- nút bấm ----------------
    def _on_pause(self, on: bool) -> None:
        self.paused = on
        self.btn_pause.setText("Tiếp tục" if on else "Tạm dừng")

    def _on_clear(self) -> None:
        for r in self.buf.values():
            r.clear()
        self.link.reset_stats()
        self.last_tick = None

    def _on_record(self, on: bool) -> None:
        if on:
            os.makedirs("logs", exist_ok=True)
            name = time.strftime("logs/robot_%Y%m%d_%H%M%S.csv")
            self.csv_file = open(name, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(CSV_FIELDS)
            self.btn_rec.setText("Dừng ghi")
            self.btn_rec.setToolTip(os.path.abspath(name))
        else:
            if self.csv_file:
                self.csv_file.close()
            self.csv_file = self.csv_writer = None
            self.btn_rec.setText("Ghi CSV")

    # ---------------- PID góc ----------------
    def _build_sp_panel(self) -> QtWidgets.QGroupBox:
        """Thanh kéo DỌC góc nghiêng đặt, cột ngoài cùng bên phải (lệnh 0x13: chỉ đổi góc, giữ nguyên PID)."""
        self.grp_sp = QtWidgets.QGroupBox("Góc đặt")
        self.grp_sp.setToolTip("Góc nghiêng đặt cho PID góc. 0° = xe đứng thẳng.\n"
                               "Phím mũi tên: 0.05°, PageUp/PageDown hoặc bấm vào rãnh: 0.5°.")
        self.grp_sp.setFixedWidth(112)
        gl = QtWidgets.QVBoxLayout(self.grp_sp)
        self.lbl_sp = QtWidgets.QLabel("+0.00°")
        self.lbl_sp.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.lbl_sp.setStyleSheet("font-size: 16px; font-weight: bold;")
        gl.addWidget(self.lbl_sp)

        def nudge(d: int):
            return lambda _=False: self.sl_sp.setValue(0 if d == 0 else self.sl_sp.value() + d)
        b_up = QtWidgets.QPushButton(f"+{SP_STEP_DEG:g}°")
        b_up.clicked.connect(nudge(+1))
        gl.addWidget(b_up)
        n = int(round(SP_MAX_DEG / SP_STEP_DEG))
        self.sl_sp = QtWidgets.QSlider(QtCore.Qt.Orientation.Vertical)
        self.sl_sp.setRange(-n, n)
        self.sl_sp.setValue(0)
        self.sl_sp.setSingleStep(1)                     # 0.05°
        self.sl_sp.setPageStep(10)                      # 0.5°
        self.sl_sp.setTickInterval(10)                  # vạch mỗi 0.5°
        self.sl_sp.setTickPosition(QtWidgets.QSlider.TickPosition.TicksBothSides)
        self.sl_sp.setMinimumHeight(300)
        self.sl_sp.valueChanged.connect(self._on_sp_slider)
        gl.addWidget(self.sl_sp, 1, QtCore.Qt.AlignmentFlag.AlignHCenter)
        b_dn = QtWidgets.QPushButton(f"-{SP_STEP_DEG:g}°")
        b_dn.clicked.connect(nudge(-1))
        gl.addWidget(b_dn)
        b0 = QtWidgets.QPushButton("Về 0°")
        b0.clicked.connect(nudge(0))
        gl.addWidget(b0)
        self.chk_spring = QtWidgets.QCheckBox("Lò xo")
        self.chk_spring.setChecked(True)
        self.chk_spring.setToolTip("Thả chuột khỏi thanh kéo -> tự trả mượt về 0°.\n"
                                   "Bỏ chọn (hoặc dùng nút ±) để giữ nguyên một góc.")
        gl.addWidget(self.chk_spring)
        self.spring_timer = QtCore.QTimer(self)
        self.spring_timer.setInterval(SPRING_MS)
        self.spring_timer.timeout.connect(self._spring_step)
        self.sl_sp.installEventFilter(self)              # bắt nhấn / thả chuột trên thanh kéo
        rng = QtWidgets.QLabel(f"±{SP_MAX_DEG:g}°")
        rng.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        rng.setStyleSheet("color: gray; font-size: 11px;")
        gl.addWidget(rng)
        return self.grp_sp

    def _build_pid_tab(self) -> QtWidgets.QWidget:
        mono = "font-family: Consolas, monospace; font-size: 11px;"
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)

        self.lbl_params = QtWidgets.QLabel("Trên STM32: (chưa nhận - đang hỏi...)")
        self.lbl_params.setWordWrap(True)
        self.lbl_params.setStyleSheet(mono)
        v.addWidget(self.lbl_params)

        def spin(lo, hi, dec, step):
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setDecimals(dec)
            s.setSingleStep(step)
            return s
        self.sp_kp, self.sp_ki = spin(0, 500, 3, 1.0), spin(0, 10000, 2, 10.0)
        self.sp_kd = spin(0, 50, 4, 0.01)
        form = QtWidgets.QFormLayout()
        form.addRow("Kp", self.sp_kp)
        form.addRow("Ki", self.sp_ki)
        form.addRow("Kd", self.sp_kd)
        v.addLayout(form)
        row = QtWidgets.QHBoxLayout()
        self.btn_send_pid = QtWidgets.QPushButton("Gửi xuống STM32")
        self.btn_send_pid.clicked.connect(self._on_send_pid)
        btn_read = QtWidgets.QPushButton("Đọc lại")
        btn_read.clicked.connect(lambda: self.link.send_to_stm32(build_get_params()))
        row.addWidget(self.btn_send_pid)
        row.addWidget(btn_read)
        v.addLayout(row)
        self.lbl_osc = QtWidgets.QLabel("Dao động 2 s gần nhất: --")
        self.lbl_osc.setWordWrap(True)
        self.lbl_osc.setStyleSheet(mono)
        v.addWidget(self.lbl_osc)

        box = QtWidgets.QGroupBox("Tự chỉnh PID góc")
        bl = QtWidgets.QVBoxLayout(box)
        f2 = QtWidgets.QFormLayout()
        self.sp_eval = spin(2, 15, 1, 0.5)
        self.sp_eval.setValue(self.tuner.cfg.eval_s)
        self.sp_step = spin(5, 50, 0, 5)
        self.sp_step.setValue(self.tuner.cfg.step0 * 100)
        f2.addRow("Đo mỗi bộ (s)", self.sp_eval)
        f2.addRow("Bước đầu (%)", self.sp_step)
        bl.addLayout(f2)
        r2 = QtWidgets.QHBoxLayout()
        self.btn_tune = QtWidgets.QPushButton("Bắt đầu tự chỉnh")
        self.btn_tune.clicked.connect(self._on_tune_start)
        self.btn_tune_stop = QtWidgets.QPushButton("Dừng (giữ bộ tốt nhất)")
        self.btn_tune_stop.clicked.connect(lambda: self.tuner.stop())
        r2.addWidget(self.btn_tune)
        r2.addWidget(self.btn_tune_stop)
        bl.addLayout(r2)
        self.lbl_tune = QtWidgets.QLabel("Chưa chạy")
        self.lbl_tune.setWordWrap(True)
        self.lbl_tune.setStyleSheet(mono + " color: #ffd400;")
        bl.addWidget(self.lbl_tune)
        self.txt_tune = QtWidgets.QPlainTextEdit()
        self.txt_tune.setReadOnly(True)
        self.txt_tune.setMaximumBlockCount(1000)
        self.txt_tune.setStyleSheet(mono)
        bl.addWidget(self.txt_tune, 1)
        self.btn_copy_def = QtWidgets.QPushButton("Chép #define bộ tốt nhất (dán vào main.c)")
        self.btn_copy_def.clicked.connect(self._copy_define)
        bl.addWidget(self.btn_copy_def)
        v.addWidget(box, 1)
        return w

    def _send_bal(self, g: dict) -> None:
        self.link.send_to_stm32(build_set_bal(g["kp"], g["ki"], g["kd"], g["sp"]))

    def _on_send_pid(self) -> None:
        g = {"kp": self.sp_kp.value(), "ki": self.sp_ki.value(), "kd": self.sp_kd.value(), "sp": self._sp_value()}
        self._send_bal(g)
        self._pid_target = g
        self._t_pid_sent = self._t_pid_change = time.monotonic()
        self._tune_log(f"gửi tay: {AutoTuner.fmt(g)}  góc đặt {g['sp']:+.2f}°")

    def _on_params(self, p: dict) -> None:
        self.lbl_params.setText(
            f"Trên STM32: Kp {p['bal_kp']:.4g} | Ki {p['bal_ki']:.4g} | Kd {p['bal_kd']:.4g} | góc đặt {p['bal_sp']:+.2f}°\n"
            f"PID động cơ: Kp {p['mot_kp']:.3g} | Ki {p['mot_ki']:.3g} | FF {p['mot_kff']:.3g}")
        if not self._params_loaded or self.tuner.active:     # ô nhập luôn hiện bộ đang chạy khi tự chỉnh
            self.sp_kp.setValue(p["bal_kp"])
            self.sp_ki.setValue(p["bal_ki"])
            self.sp_kd.setValue(p["bal_kd"])
            self._params_loaded = True
        if self._pid_target is not None and AutoTuner._match(p, self._pid_target):
            self._pid_target = None
            self._tune_log("STM32 đã nhận PID gửi tay")
        if self._sp_target is not None and abs(p["bal_sp"] - self._sp_target) <= 0.005:
            self._sp_target = None                            # STM32 đã nhận góc vừa kéo
        if self._sp_target is None and not self.sl_sp.isSliderDown():
            self._set_sp_silent(p["bal_sp"])                  # thanh kéo luôn hiện góc đặt thật (kể cả sau khi STM32 reset về 0)
        self.tuner.on_params(p)

    def _on_tune_start(self) -> None:
        p = self.link.params
        if p is None:
            self._tune_log("chưa nhận được PID từ STM32")
            return
        if self.last is not None:
            s = decode_status(self.last["status"])
            if s["mode"] != 3:
                self._tune_log("STM32 không ở chế độ cân bằng (mode 3): đặt PC_RPM_TEST 0 trong main.c")
                return
            if not s["armed"]:
                self._tune_log("động cơ đang tắt: dựng xe đứng thẳng cho xe tự chạy rồi bấm lại")
                return
        self.tuner.cfg.eval_s = self.sp_eval.value()
        self.tuner.cfg.step0 = self.sp_step.value() / 100.0
        sp = self._sp_value() if self._sp_target is not None else p["bal_sp"]
        self.tuner.start({"kp": p["bal_kp"], "ki": p["bal_ki"], "kd": p["bal_kd"], "sp": sp})

    def _sp_value(self) -> float:
        return round(self.sl_sp.value() * SP_STEP_DEG, 3)

    def _on_sp_slider(self, v: int) -> None:
        self.lbl_sp.setText(f"{v * SP_STEP_DEG:+.2f}°")
        self._sp_target = self._sp_value()
        self._t_sp_change = time.monotonic()
        self._send_sp()

    def eventFilter(self, obj, ev) -> bool:
        """Lò xo: nhấn chuột vào thanh kéo -> dừng kéo về; thả chuột -> bắt đầu kéo về 0."""
        if obj is getattr(self, "sl_sp", None):
            t = ev.type()
            if t == QtCore.QEvent.Type.MouseButtonPress:
                self.spring_timer.stop()
            elif t == QtCore.QEvent.Type.MouseButtonRelease and self.chk_spring.isChecked():
                self.spring_timer.start()
        return super().eventFilter(obj, ev)

    def _spring_step(self) -> None:
        v = self.sl_sp.value()
        if v == 0 or self.sl_sp.isSliderDown() or not self.grp_sp.isEnabled():
            self.spring_timer.stop()
            return
        self.sl_sp.setValue(int(v * SPRING_K))           # int() cắt về phía 0 -> chắc chắn tiến dần về 0

    def _set_sp_silent(self, deg: float) -> None:
        """Đặt thanh kéo theo STM32 mà không gửi lệnh."""
        self.sl_sp.blockSignals(True)
        self.sl_sp.setValue(int(round(deg / SP_STEP_DEG)))
        self.sl_sp.blockSignals(False)
        self.lbl_sp.setText(f"{deg:+.2f}°")

    def _send_sp(self) -> None:
        if self.link.esp_ip and self._sp_target is not None:
            self.link.send_to_stm32(build_set_sp(self._sp_target))
            self._t_sp_sent = time.monotonic()

    def _tune_log(self, msg: str) -> None:
        self.txt_tune.appendPlainText(time.strftime("%H:%M:%S ") + msg)

    def _copy_define(self) -> None:
        txt = self.tuner.define_text()
        if txt:
            QtWidgets.QApplication.clipboard().setText(txt)
            self._tune_log("đã chép vào clipboard:\n" + txt.rstrip())

    def _update_pid_panel(self) -> None:
        n = int(2.0 / CTRL_TS)
        if self.buf["t"].count > 50:
            m = metrics(self.buf["t"].view()[-n:], self.buf["angle"].view()[-n:],
                        self.buf["rpm1"].view()[-n:], self.buf["rpm2"].view()[-n:])
            if m:
                self.lbl_osc.setText(f"Dao động 2 s gần nhất: góc ±{m['std_angle']:.2f}° (TB {m['mean_angle']:+.2f}°), "
                                     f"~{m['freq_hz']:.1f} Hz | bánh ±{m['std_speed']:.1f} rpm (TB {m['mean_speed']:+.1f})")
        self.lbl_tune.setText(self.tuner.status_text())
        act = self.tuner.active
        self.btn_tune.setEnabled(not act and self.link.params is not None)
        self.btn_tune_stop.setEnabled(act)
        self.btn_send_pid.setEnabled(not act and bool(self.link.esp_ip))
        self.btn_copy_def.setEnabled(self.tuner.best is not None)
        self.grp_sp.setEnabled(not act)                       # đang tự chỉnh: giữ góc đặt cố định
        now = time.monotonic()
        p = self.link.params
        if self._pid_target is not None and not act and now - self._t_pid_sent > 0.3:
            if now - self._t_pid_change < 2.0:                # PID gửi tay chưa được xác nhận -> gửi lại
                self._send_bal(self._pid_target)
                self._t_pid_sent = now
            else:
                self._pid_target = None
                self._tune_log("LỖI: STM32 không xác nhận PID gửi tay (mất kết nối?) - bấm gửi lại")
        if self._sp_target is not None and p is not None and not act and now - self._t_sp_sent > 0.3:
            if now - self._t_sp_change < 2.0:                 # chưa được xác nhận (mất gói) -> gửi lại
                self._send_sp()
            else:                                             # 2 s vẫn chưa nhận -> thanh kéo trả về giá trị trên STM32
                self._sp_target = None
                self._set_sp_silent(p["bal_sp"])
                self._tune_log("góc đặt chưa được STM32 xác nhận -> thanh kéo trả về giá trị trên STM32")
        if self.link.esp_ip and self.link.params is None and now - self._t_get > 1.0:   # hỏi PID đến khi có
            self._t_get = now
            self.link.send_to_stm32(build_get_params())

    # ---------------- cần gạt tốc độ ----------------
    def _on_slider(self, v: int) -> None:
        self.lbl_rpm.setText(f"{v:+d} rpm")
        self._send_cmd()                      # gửi ngay khi kéo, không chờ chu kỳ

    def _send_cmd(self) -> None:
        if not self.chk_send.isChecked() or not self.link.esp_ip:
            return
        self.link.send_to_stm32(build_set_rpm(self.slider.value()))
        self.n_cmd_sent += 1

    # ---------------- vòng vẽ ----------------
    def _refresh(self) -> None:
        q = self.link.frames
        batch = []
        while q:
            batch.append(q.popleft())

        if self.link.n_params != self._seen_params:          # khung PID mới từ STM32 (cũng là xác nhận)
            self._seen_params = self.link.n_params
            self._on_params(self.link.params)
        if self.tuner.active:
            if batch:
                self.tuner.feed(batch)
            self.tuner.tick()

        if batch:
            # STM32 khởi động lại -> tick quay về nhỏ: xoá dữ liệu cũ
            if self.last_tick is not None and batch[0]["tick"] + 10 < self.last_tick:
                self._on_clear()
            self.last_tick = batch[-1]["tick"]
            self.last = batch[-1]
            self._rate_n += len(batch)

            self.buf["t"].extend([f["tick"] * CTRL_TS for f in batch])
            for k, r in self.buf.items():
                if k != "t":
                    r.extend([f[k] for f in batch])

            if self.csv_writer:
                now = time.time()
                for f in batch:
                    self.csv_writer.writerow([f"{now:.3f}", f["tick"], f"{f['tick'] * CTRL_TS:.3f}",
                                              f"{f['angle']:.4f}", f"{f['rpm_ref']:.3f}", f"{f['rpm1']:.3f}",
                                              f"{f['rpm2']:.3f}", f"{f['duty1']:.2f}", f"{f['duty2']:.2f}",
                                              f["status"]])

        if not self.paused and self.buf["t"].count:
            t = self.buf["t"].view()
            x = t - t[-1]
            win = self.spin_win.value()
            for key, item in self.curves:
                item.setData(x, self.buf[key].view())
            self.first_plot.setXRange(-win, 0, padding=0)

        now = time.monotonic()
        if now - self._status_t >= 0.25:
            self._status_t = now
            self._update_status(now)

    def _update_status(self, now: float) -> None:
        self._update_pid_panel()
        dt = now - self._rate_t
        if dt >= 1.0:
            self.rate = self._rate_n / dt
            self._rate_n, self._rate_t = 0, now
        rx = self.last["reserved"] if self.last else None
        self.lbl_cmd.setText(f"gửi  {self.n_cmd_sent % 256:4d}\nnhận {rx if rx is not None else '--':>4}")
        L = self.link
        if not L.esp_ip:
            self.lbl_status.setText("Đang tìm ESP32 (chờ beacon)... kiểm tra WiFi / tường lửa Windows")
            return
        alive = (time.monotonic() - L.last_rx) < 1.0
        rtt = f"{L.rtt_ms:.1f} ms" if L.rtt_ms is not None else "--"
        txt = (f"ESP32 {L.esp_ip} | {'NHẬN' if alive else 'MẤT TÍN HIỆU'} {self.rate:5.0f} khung/s | "
               f"mất {L.n_lost} | lỗi CRC {L.n_bad} | trễ WiFi (RTT) {rtt}")
        if L.esp_info.get("rst"):
            rst = L.esp_info["rst"]
            txt += f" | ESP32 chạy {L.esp_info.get('up', '?')} s, reset do: {RESET_NAMES.get(rst, rst)}"
        if self.last:
            s = decode_status(self.last["status"])
            txt += (f"\n{MODE_NAMES.get(s['mode'], '?')} | {'ĐANG CHẠY' if s['armed'] else 'dừng'} | "
                    f"lỗi: {FAULT_NAMES.get(s['fault'], '?')} | góc {self.last['angle']:+6.2f}° | "
                    f"đặt {self.last['rpm_ref']:+6.1f} | rpm1 {self.last['rpm1']:+6.1f} | "
                    f"rpm2 {self.last['rpm2']:+6.1f} | duty {self.last['duty1']:+5.1f}% / {self.last['duty2']:+5.1f}%")
        self.lbl_status.setText(txt)

    def closeEvent(self, ev) -> None:
        if self.tuner.active and self.tuner.best:              # đang tự chỉnh: trả về bộ tốt nhất trước khi thoát
            for _ in range(3):
                self._send_bal(self.tuner.best)
        self.cmd_timer.stop()
        self.slider.setValue(0)               # gửi lệnh 0 rpm trước khi đóng
        self.timer.stop()
        self._on_record(False)
        self.link.stop()
        super().closeEvent(ev)


def main() -> None:
    ap = argparse.ArgumentParser(description="Theo dõi robot tự cân bằng")
    ap.add_argument("--esp-ip", help="IP của ESP32 (bỏ trống = tự tìm qua beacon)")
    ap.add_argument("--window", type=float, default=10.0, help="độ dài trục thời gian hiển thị (s)")
    args = ap.parse_args()

    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    app = QtWidgets.QApplication(sys.argv)
    link = UdpLink(args.esp_ip)
    link.start()
    w = MainWindow(link, args.window)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
