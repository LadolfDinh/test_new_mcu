# -*- coding: utf-8 -*-
"""
Mo phong che do 2 (TEST_SPEED_PID) bang code C that: xe nhac banh khoi dat,
2 PID dong co cung bam g_test_rpm. Doi g_test_rpm theo lich (nhu sua trong Watch).
  python test_speed_pid.py                # PWM theo CubeMX (10 kHz)
  python test_speed_pid.py --pwm 2000     # thu neu doi PSC TIM1/TIM8 ra 2 kHz (chi doi mo hinh L298N)
Ket qua: results/speed_pid_<pwm>.png
"""
import argparse
import os

import numpy as np

import plot_style as ps
from sim_fw import FwSim

ps.setup()
import matplotlib.pyplot as plt

SCHEDULE = [(0.0, 40.0), (1.0, 80.0), (2.0, 20.0), (3.0, -40.0), (4.0, 0.0)]   # (t, g_test_rpm)
T_END = 5.0


def step_metrics(t, y_true, y_meas, t0, t1, r_from, r_to):
    """len 90 %% / vot lo tinh tren toc do THAT (bo nhieu luong tu encoder); dao dong tinh tren g_rpm1."""
    s = (t >= t0) & (t < t1)
    tt, yy, ym = t[s] - t0, y_true[s], y_meas[s]
    dr = r_to - r_from
    if dr == 0:
        return None
    frac = (yy - r_from) / dr
    i10 = np.argmax(frac >= 0.1) if np.any(frac >= 0.1) else None
    i90 = np.argmax(frac >= 0.9) if np.any(frac >= 0.9) else None
    rise = (tt[i90] - tt[i10]) * 1000 if (i10 is not None and i90 is not None) else float('nan')
    t90 = tt[i90] * 1000 if i90 is not None else float('nan')
    over = max(0.0, (np.max(frac) - 1.0) * 100)
    tail = tt >= (t1 - t0) - 0.3
    ripple = np.ptp(ym[tail])
    sse = np.mean(yy[tail]) - r_to
    ts = tt[np.nonzero(np.abs(yy - r_to) > max(0.05 * abs(dr), 3.0))[0][-1]] * 1000 \
        if np.any(np.abs(yy - r_to) > max(0.05 * abs(dr), 3.0)) else 0.0
    return dict(t90=t90, rise=rise, over=over, ripple=ripple, sse=sse, ts=ts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pwm', type=float, help='Hz (mac dinh: theo CubeMX trong main.c)')
    a = ap.parse_args()
    ov = {'PWM_FREQ': a.pwm} if a.pwm else {}
    s = FwSim(ov, mode=2, test_rpm=SCHEDULE[0][1])
    log = s.run(T_END, schedule=[(t, 'g_test_rpm', v) for t, v in SCHEDULE[1:]])
    t = log['t']
    pwm = s.P['PWM_FREQ']

    print(f'PWM {pwm:.0f} Hz, PID dong co Kp {s.fw.pid("g_pid1").kp:g} Ki {s.fw.pid("g_pid1").ki:g} '
          f'Kff {s.fw.pid("g_pid1").kff:g}')
    rows = []
    sch = SCHEDULE + [(T_END, None)]
    prev = 0.0
    for (t0, r), (t1, _) in zip(sch[:-1], sch[1:]):
        m = step_metrics(t, log['w1_true'], log['rpm1'], t0, t1, prev, r)
        if m:
            rows.append((prev, r, m))
            print(f"  {prev:5.0f} -> {r:5.0f} rpm: len 90% sau {m['t90']:5.0f} ms, vot lo {m['over']:4.1f} %, "
                  f"on dinh (+-5%) sau {m['ts']:4.0f} ms, g_rpm1 dao dong {m['ripple']:4.1f} rpm, duty xac lap {np.mean(log['duty1'][(t > t1 - 0.3) & (t < t1)]):5.1f} %")
        prev = r

    os.makedirs('results', exist_ok=True)
    fig, ax = plt.subplots(2, 1, figsize=(10, 6.2), sharex=True, gridspec_kw={'height_ratios': [3, 2]})
    ax[0].plot(t, log['rpm_ref'], color=ps.TEXT2, lw=1.2, ls='--', label='g_rpm_ref (đặt)')
    ax[0].plot(t, log['w1_true'], color=ps.AQUA, lw=1.0, label='tốc độ thật bánh 1')
    ax[0].plot(t, log['rpm1'], color=ps.BLUE, label='g_rpm1 (encoder, code đo)')
    ax[0].plot(t, log['rpm2'], color=ps.ORANGE, lw=1.0, label='g_rpm2')
    ax[0].set_ylabel('rpm')
    ax[0].legend(loc='upper right', ncol=2)
    ax[1].plot(t, log['duty1'], color=ps.BLUE, label='g_duty1')
    ax[1].plot(t, log['duty2'], color=ps.ORANGE, lw=1.0, label='g_duty2')
    ax[1].set_ylabel('duty (%)')
    ax[1].set_xlabel('thời gian từ lúc động cơ được bật (s)')
    ax[1].legend(loc='upper right')
    p1 = s.fw.pid('g_pid1')
    ax[0].set_title(f'Chế độ 2 (TEST_SPEED_PID), bánh nhấc khỏi đất · PID động cơ Kp {p1.kp:g} / '
                    f'Ki {p1.ki:g} / FF {p1.kff:g} · PWM {pwm/1000:g} kHz · code C thật, Ts {s.P["CTRL_TS"]*1000:g} ms',
                    loc='left', fontsize=9)
    fig.tight_layout()
    out = f'results/speed_pid_{pwm/1000:g}kHz.png'
    fig.savefig(out, dpi=130)
    print('saved', out)


if __name__ == '__main__':
    main()
