"""Tự chỉnh PID góc (cân bằng) trực tuyến: PC đo dao động từ telemetry, thử bộ PID mới, gửi xuống STM32.

Thuật toán: tìm theo từng thông số, bước nhân (kiểu Hooke-Jeeves). Không cần mô hình xe.
  1. Đo chi phí J của bộ PID hiện tại: bỏ settle_s giây đầu sau khi đổi PID, rồi đo eval_s giây.
  2. Lần lượt Kp -> Kd -> Ki: thử tăng x(1+bước); không tốt hơn thì thử giảm /(1+bước).
     J giảm ít nhất `improve` (tương đối) thì nhận làm bộ tốt nhất và thử tiếp cùng chiều.
  3. Hết 1 vòng không cải thiện -> bước giảm 1/2; bước < step_min -> xong, áp dụng bộ tốt nhất.
     Đầu mỗi vòng đo lại bộ tốt nhất (pin yếu dần, sàn khác...) để so sánh công bằng.

  J = w_angle * σ(góc) [độ] + w_speed * σ(tốc độ bánh trung bình) [rpm] + w_sat * (tỉ lệ thời gian |duty| >= 95 %)
  σ = độ lệch chuẩn trong cửa sổ đo = mức rung lắc (xe trôi chậm không làm J tăng).

An toàn:
  - Mỗi bộ PID gửi đi phải được STM32 xác nhận (khung 0x02 trả về đúng giá trị); mất gói thì gửi lại.
  - |góc - góc đặt| > abort_deg, động cơ bị ngắt hoặc báo lỗi -> bỏ bộ đang thử (J = vô cùng), gửi lại ngay
    bộ tốt nhất, chờ xe đứng ổn định rearm_s giây (dựng xe lên nếu đổ) rồi thử tiếp.
    fall_stop lần liên tiếp -> dừng hẳn.
  - Mọi giá trị nằm trong `bounds` (STM32 còn kẹp thêm một lớp nữa).

Module không phụ thuộc Qt: app.py gọi feed() (khung trạng thái), on_params() (khung 0x02), tick() (định kỳ).
"""
from __future__ import annotations

import csv
import math
import os
import time
from dataclasses import dataclass, field

import numpy as np

from protocol import CTRL_TS, decode_status

PARAMS = ("kp", "ki", "kd")


@dataclass
class TuneConfig:
    settle_s: float = 1.5        # bỏ qua sau khi đổi PID (s, theo đồng hồ STM32)
    eval_s: float = 4.0          # thời gian đo mỗi bộ (s)
    step0: float = 0.20          # bước đầu: ±20 %
    step_min: float = 0.05       # dừng khi bước < 5 %
    improve: float = 0.03        # phải tốt hơn >= 3 % mới nhận (chống nhiễu đo)
    max_trials: int = 60
    w_angle: float = 1.0         # trọng số σ góc (độ)
    w_speed: float = 0.02        # trọng số σ tốc độ bánh (rpm): 50 rpm ~ 1 độ
    w_sat: float = 5.0           # phạt duty kịch
    abort_deg: float = 20.0      # lệch quá góc này -> bỏ bộ đang thử trước khi xe đổ (STM32 tự ngắt ở 30)
    rearm_s: float = 2.0         # xe phải đứng ổn định bấy lâu mới thử tiếp
    fall_stop: int = 3           # số lần hỏng liên tiếp thì dừng
    order: tuple = ("kp", "kd", "ki")
    bounds: dict = field(default_factory=lambda: {"kp": (2.0, 200.0), "ki": (0.0, 3000.0), "kd": (0.0, 5.0)})
    floors: dict = field(default_factory=lambda: {"kp": 2.0, "ki": 5.0, "kd": 0.005})  # đang 0 mà tăng -> lên giá trị này
    ack_timeout_s: float = 0.4   # chờ STM32 xác nhận (s, đồng hồ PC)
    ack_retries: int = 12


def metrics(t, angle, rpm1, rpm2, duty1=None, duty2=None) -> dict | None:
    """Chỉ số dao động trong 1 cửa sổ. t: giây (theo tick STM32)."""
    a = np.asarray(angle, dtype=float)
    if len(a) < 20:
        return None
    t = np.asarray(t, dtype=float)
    w = 0.5 * (np.asarray(rpm1, dtype=float) + np.asarray(rpm2, dtype=float))
    T = max(float(t[-1] - t[0]), CTRL_TS)
    # tần số dao động: đếm số lần cắt qua giá trị trung bình (sau khi làm mượt 20 ms để bỏ nhiễu)
    k = 10
    sm = np.convolve(a - a.mean(), np.ones(k) / k, mode="valid") if len(a) > 2 * k else a - a.mean()
    s = np.signbit(sm)
    zc = int(np.count_nonzero(s[1:] != s[:-1]))
    m = {"std_angle": float(a.std()), "mean_angle": float(a.mean()),
         "std_speed": float(w.std()), "mean_speed": float(w.mean()),
         "freq_hz": zc / (2.0 * T), "n": len(a), "T": T, "sat": 0.0}
    if duty1 is not None and duty2 is not None:
        d = np.maximum(np.abs(np.asarray(duty1, dtype=float)), np.abs(np.asarray(duty2, dtype=float)))
        m["sat"] = float(np.mean(d >= 95.0))
    return m


class AutoTuner:
    def __init__(self, send, log, cfg: TuneConfig | None = None, csv_dir: str | None = "logs", clock=time.monotonic):
        self.send = send             # send(dict kp, ki, kd, sp)
        self.log = log               # log(str)
        self.cfg = cfg or TuneConfig()
        self.csv_dir = csv_dir
        self.clock = clock
        self.state = "idle"          # idle | ack | settle | eval | rearm | done
        self.best: dict | None = None
        self.best_J = math.inf
        self.trials: list[dict] = []
        self.t_last = 0.0
        self._csv = None
        self._csvw = None

    # ------------------------------------------------------------------ điều khiển
    @property
    def active(self) -> bool:
        return self.state not in ("idle", "done")

    def start(self, gains: dict) -> None:
        """gains: kp, ki, kd, sp - bộ đang chạy trên STM32 (đọc từ khung 0x02)."""
        c = self.cfg
        self.best = {k: float(gains[k]) for k in ("kp", "ki", "kd", "sp")}
        for k in PARAMS:                                   # đưa về trong giới hạn tìm kiếm
            lo, hi = c.bounds[k]
            self.best[k] = min(max(self.best[k], lo), hi)
        self.best_J = math.inf
        self.step = c.step0
        self.pi, self.di = 0, 0
        self.pref = {k: +1 for k in c.order}
        self.no_go = {k: [-math.inf, math.inf] for k in c.order}   # vùng đã làm xe hỏng: không thử vượt qua nữa
        self.cur_dirs = [self.pref[c.order[0]], -self.pref[c.order[0]]]
        self.pass_improved = False
        self.falls = 0
        self.n_trial = 0
        self.trials = []
        self._final = False
        self._open_csv()
        self.log(f"BẮT ĐẦU: {self.fmt(self.best)} | đo {c.eval_s:g} s/bộ, bước {c.step0:.0%}")
        self._try(dict(self.best), "bộ ban đầu", "base")

    def stop(self, reason: str = "dừng theo yêu cầu") -> None:
        if not self.active or self.best is None:
            return
        self.log(f"{reason} -> áp dụng bộ tốt nhất")
        self._try(dict(self.best), "áp dụng bộ tốt nhất", "final")

    # ------------------------------------------------------------------ dữ liệu vào
    def on_params(self, p: dict) -> None:
        """Khung 0x02 từ STM32: xác nhận bộ PID vừa gửi."""
        if self.state != "ack" or not self._match(p, self.cand):
            return
        if self.kind == "final":
            self.state = "done"
            self._close_csv()
            self.log(f"XONG. Bộ tốt nhất: {self.fmt(self.best)}  J={self.best_J:.3f}")
        elif self.kind == "restore":
            self.state, self.t_ok = "rearm", None
        else:
            self.state, self.t0 = "settle", None

    def tick(self) -> None:
        """Gọi định kỳ (vài chục ms): gửi lại khi chưa được xác nhận."""
        if self.state != "ack":
            return
        now = self.clock()
        if now - self.t_sent < self.cfg.ack_timeout_s:
            return
        self.retries += 1
        if self.retries > self.cfg.ack_retries:
            self.log("LỖI: STM32 không xác nhận PID (mất kết nối?) -> dừng tự chỉnh")
            self.state = "idle"
            self._close_csv()
            return
        self.t_sent = now
        self.send(self.cand)

    def feed(self, frames) -> None:
        """Khung trạng thái (type 0x01) theo thứ tự thời gian."""
        c = self.cfg
        for f in frames:
            t = f["tick"] * CTRL_TS
            self.t_last = t
            if self.state in ("idle", "done", "ack"):
                continue
            st = decode_status(f["status"])
            sp = self.cand["sp"]
            dev = abs(f["angle"] - sp)
            if self.state == "rearm":
                if st["armed"] and st["fault"] == 0 and dev < 10.0:
                    if self.t_ok is None:
                        self.t_ok = t
                    elif t - self.t_ok >= c.rearm_s:
                        self.log("xe đã đứng ổn định -> thử tiếp")
                        if self.resume == "cand":
                            self._next()
                        else:
                            self._try(dict(self.best), "đo lại bộ tốt nhất" if self.resume == "rebase" else "bộ ban đầu",
                                      self.resume)
                else:
                    self.t_ok = None
                continue

            if not st["armed"] or st["fault"] != 0:
                self._abort("động cơ bị ngắt / xe đổ")
                continue
            if dev > c.abort_deg:
                self._abort(f"lệch {dev:.1f}° > {c.abort_deg:g}°")
                continue

            if self.state == "settle":
                if self.t0 is None:
                    self.t0 = t
                elif t - self.t0 >= c.settle_s:
                    self.state, self.t_eval0, self.buf = "eval", t, []
            elif self.state == "eval":
                self.buf.append((t, f["angle"], f["rpm1"], f["rpm2"], f["duty1"], f["duty2"]))
                if t - self.t_eval0 >= c.eval_s:
                    self._finish_eval()

    # ------------------------------------------------------------------ hiển thị
    def status_text(self) -> str:
        c = self.cfg
        s = self.state
        if s == "idle":
            return "Chưa chạy"
        if s == "done":
            return f"Xong. Tốt nhất: {self.fmt(self.best)}  J={self.best_J:.3f}"
        head = f"[{self.desc}] {self.fmt(self.cand)}"
        if s == "ack":
            body = f"chờ STM32 xác nhận (gửi lại {self.retries})"
        elif s == "settle":
            body = f"chờ ổn định {0 if self.t0 is None else self.t_last - self.t0:.1f}/{c.settle_s:g} s"
        elif s == "eval":
            body = f"đang đo {self.t_last - self.t_eval0:.1f}/{c.eval_s:g} s"
        else:
            body = "XE ĐỔ / LỆCH NHIỀU: dựng xe lên, chờ đứng yên " + f"{c.rearm_s:g} s"
        best = "" if self.best is None else f"\nTốt nhất: {self.fmt(self.best)}  J={self.best_J:.3f}  | bước {self.step:.0%}"
        return f"{head}\n{body}{best}"

    def define_text(self) -> str:
        g = self.best
        if g is None:
            return ""

        def cf(v: float) -> str:                       # hằng float hợp lệ trong C: 300 -> 300.0f
            s = f"{v:.4g}"
            return (s if ("." in s or "e" in s) else s + ".0") + "f"
        txt = (f"#define BAL_KP                  {cf(g['kp'])}\n"
               f"#define BAL_KI                  {cf(g['ki'])}\n"
               f"#define BAL_KD                  {cf(g['kd'])}\n")
        if abs(g["sp"]) > 1e-6:
            txt += f"#define BAL_SETPOINT_DEG        {cf(g['sp'])}\n"
        return txt

    @staticmethod
    def fmt(g: dict | None) -> str:
        if not g:
            return "--"
        return f"Kp {g['kp']:.4g}  Ki {g['ki']:.4g}  Kd {g['kd']:.4g}"

    # ------------------------------------------------------------------ nội bộ
    def _try(self, cand: dict, desc: str, kind: str) -> None:
        self.cand, self.desc, self.kind = cand, desc, kind
        self.state, self.retries, self.t_sent = "ack", 0, self.clock()
        self.send(cand)

    @staticmethod
    def _match(p: dict, g: dict) -> bool:
        pairs = (("bal_kp", "kp"), ("bal_ki", "ki"), ("bal_kd", "kd"), ("bal_sp", "sp"))
        return all(abs(p[a] - g[b]) <= 1e-4 * max(1.0, abs(g[b])) for a, b in pairs)

    def _move(self, g: dict, name: str, d: int) -> dict | None:
        c = self.cfg
        lo, hi = c.bounds[name]
        fl = c.floors[name]
        v = g[name]
        if d > 0:
            nv = v * (1.0 + self.step) if v >= fl else fl
        else:
            nv = v / (1.0 + self.step)
            if nv < fl:
                nv = lo
        nv = float(f"{min(max(nv, lo), hi):.4g}")
        if abs(nv - v) <= 1e-6 * max(1.0, abs(v)):
            return None
        ng_lo, ng_hi = self.no_go[name]
        if nv >= ng_hi or nv <= ng_lo:                 # đã từng làm xe hỏng ở phía này
            return None
        out = dict(g)
        out[name] = nv
        return out

    def _next(self) -> None:
        c = self.cfg
        while True:
            if self.n_trial >= c.max_trials:
                self.stop(f"đã thử {c.max_trials} bộ")
                return
            if self.di >= len(self.cur_dirs):            # hết chiều cho thông số này -> thông số kế
                self.pi += 1
                self.di = 0
                if self.pi >= len(c.order):                # hết 1 vòng
                    self.pi = 0
                    if not self.pass_improved:
                        self.step /= 2.0
                        if self.step < c.step_min - 1e-9:
                            self.stop("bước đã nhỏ hơn mức tối thiểu")
                            return
                        self.log(f"vòng không cải thiện -> bước còn {self.step:.0%}")
                    self.pass_improved = False
                    n0 = c.order[0]
                    self.cur_dirs = [self.pref[n0], -self.pref[n0]]
                    self._try(dict(self.best), "đo lại bộ tốt nhất", "rebase")
                    return
                n = c.order[self.pi]
                self.cur_dirs = [self.pref[n], -self.pref[n]]
            name = c.order[self.pi]
            d = self.cur_dirs[self.di]
            self.di += 1
            cand = self._move(self.best, name, d)
            if cand is None:
                continue
            self.n_trial += 1
            self.cand_move = (name, d)
            self._try(cand, f"#{self.n_trial} {name.upper()} {'+' if d > 0 else '-'}{self.step:.0%}", "cand")
            return

    def _abort(self, reason: str) -> None:
        self.falls += 1
        self._record(self.cand, None, f"HỎNG: {reason}")
        self.log(f"[{self.desc}] {self.fmt(self.cand)} -> HỎNG ({reason}) -> gửi lại bộ tốt nhất")
        self.resume = self.kind if self.kind in ("base", "rebase") else "cand"
        if self.kind == "cand":                        # nhớ giới hạn: không thử vượt giá trị vừa hỏng
            name, d = self.cand_move
            v = self.cand[name]
            if d > 0:
                self.no_go[name][1] = min(self.no_go[name][1], v)
            else:
                self.no_go[name][0] = max(self.no_go[name][0], v)
            self.pref[name] = -d
            self.log(f"  -> từ giờ không thử {name.upper()} {'>=' if d > 0 else '<='} {v:.4g}")
        if self.falls >= self.cfg.fall_stop:
            self.stop(f"hỏng {self.falls} lần liên tiếp")
            return
        self._try(dict(self.best), "khôi phục bộ tốt nhất", "restore")

    def _finish_eval(self) -> None:
        c = self.cfg
        b = np.array(self.buf)
        need = 0.5 * c.eval_s / CTRL_TS
        if len(b) < need:
            self.log(f"[{self.desc}] thiếu dữ liệu ({len(b)} mẫu, mất WiFi?) -> đo lại")
            self.state, self.t_eval0, self.buf = "eval", self.t_last, []
            return
        m = metrics(b[:, 0], b[:, 1], b[:, 2], b[:, 3], b[:, 4], b[:, 5])
        J = c.w_angle * m["std_angle"] + c.w_speed * m["std_speed"] + c.w_sat * m["sat"]
        m["J"] = J
        self.falls = 0
        info = (f"J={J:.3f} (σgóc {m['std_angle']:.2f}°, σbánh {m['std_speed']:.1f} rpm, "
                f"~{m['freq_hz']:.1f} Hz, kịch {m['sat']:.0%})")
        if self.kind == "base":
            self.best_J = J
            self._record(self.cand, m, "gốc")
            self.log(f"[bộ ban đầu] {self.fmt(self.cand)} {info}")
            self._next()
        elif self.kind == "rebase":
            old = self.best_J
            self.best_J = J if not math.isfinite(old) else 0.5 * (old + J)
            self._record(self.cand, m, "đo lại")
            self.log(f"[đo lại tốt nhất] {info} -> J tham chiếu {self.best_J:.3f}")
            self._next()
        else:
            if J < self.best_J * (1.0 - c.improve):
                name, d = self.cand_move
                self.best, self.best_J = dict(self.cand), J
                self.pass_improved = True
                self.pref[name] = d
                self.cur_dirs, self.di = [d], 0          # tiếp tục cùng chiều
                self._record(self.cand, m, "NHẬN")
                self.log(f"[{self.desc}] {self.fmt(self.cand)} {info} -> NHẬN")
            else:
                self._record(self.cand, m, "bỏ")
                self.log(f"[{self.desc}] {self.fmt(self.cand)} {info} -> bỏ")
            self._next()

    def _record(self, g: dict, m: dict | None, result: str) -> None:
        row = {"trial": self.n_trial, "desc": self.desc, "kp": g["kp"], "ki": g["ki"], "kd": g["kd"], "sp": g["sp"],
               "J": m["J"] if m else math.inf, "result": result}
        if m:
            row.update({k: m[k] for k in ("std_angle", "std_speed", "sat", "freq_hz", "mean_angle", "mean_speed", "n")})
        self.trials.append(row)
        if self._csvw:
            self._csvw.writerow([f"{time.time():.3f}", row["trial"], row["desc"], row["kp"], row["ki"], row["kd"],
                                 row["sp"], f"{row['J']:.4f}", f"{row.get('std_angle', float('nan')):.4f}",
                                 f"{row.get('std_speed', float('nan')):.3f}", f"{row.get('sat', float('nan')):.3f}",
                                 f"{row.get('freq_hz', float('nan')):.2f}", result])
            self._csv.flush()

    def _open_csv(self) -> None:
        self._close_csv()
        if not self.csv_dir:
            return
        os.makedirs(self.csv_dir, exist_ok=True)
        name = os.path.join(self.csv_dir, time.strftime("tune_%Y%m%d_%H%M%S.csv"))
        self._csv = open(name, "w", newline="", encoding="utf-8")
        self._csvw = csv.writer(self._csv)
        self._csvw.writerow(["pc_time", "trial", "desc", "kp", "ki", "kd", "sp", "J", "std_angle_deg",
                             "std_speed_rpm", "sat", "freq_hz", "result"])
        self.log(f"ghi kết quả: {name}")

    def _close_csv(self) -> None:
        if self._csv:
            self._csv.close()
        self._csv = self._csvw = None
