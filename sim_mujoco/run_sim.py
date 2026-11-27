# -*- coding: utf-8 -*-
"""
Chay 1 lan mo phong va ve do thi: so sanh
  (A) MPU6050 + Kalman nhu xe that  va  (B) goc do ly tuong (khong sai so cam bien).

Vi du:
  python run_sim.py                         # thong so trong params.py
  python run_sim.py --mass 1.0 --com 0.15   # doi khoi luong / trong tam
  python run_sim.py --offset 0.5            # goc dat lech 0.5 do so voi diem can bang that
  python run_sim.py --kp 15 --ki 290 --kd 0.05 --t 10
Ket qua: results/run_<...>.png  (+ .csv)
"""
import argparse
import csv
import os

import numpy as np

import plot_style as ps
from sim import metrics
from sim_fw import make_sim

ps.setup()
import matplotlib.pyplot as plt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mass', type=float, help='kg, than xe tru 2 banh')
    ap.add_argument('--com', type=float, help='m, trong tam tren truc banh')
    ap.add_argument('--theta0', type=float, default=3.0, help='goc luc tha tay (do)')
    ap.add_argument('--offset', type=float, default=0.0, help='lech goc dat (do)')
    ap.add_argument('--kp', type=float)
    ap.add_argument('--ki', type=float)
    ap.add_argument('--kd', type=float)
    ap.add_argument('--pwm', type=float, help='Hz')
    ap.add_argument('--push', type=float, default=1.5, help='luc day than xe (N) trong 0.1 s tai t=4 s')
    ap.add_argument('--t', type=float, default=8.0)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--backend', default='auto', choices=['auto', 'c', 'py'], help='c = chay code C that cua main.c')
    a = ap.parse_args()

    ov = {}
    for k, name in (('mass', 'BODY_MASS'), ('com', 'COM_HEIGHT'), ('kp', 'BAL_KP'), ('ki', 'BAL_KI'),
                    ('kd', 'BAL_KD'), ('pwm', 'PWM_FREQ')):
        if getattr(a, k) is not None:
            ov[name] = getattr(a, k)
    ov['ANGLE_OFFSET_DEG'] = a.offset

    runs = []
    for ideal in (False, True):
        o = dict(ov, IDEAL_IMU=ideal)
        s = make_sim(o, seed=a.seed, theta0_deg=a.theta0, backend=a.backend)
        if a.push:
            s.push = [(4.0, 4.1, a.push)]
        log = s.run(a.t)
        runs.append((ideal, s.P, log, metrics(log, a.t)))
        backend = s.backend
        gains = (s.fw.pid('g_pid_bal').kp, s.fw.pid('g_pid_bal').ki, s.fw.pid('g_pid_bal').kd) if hasattr(s, 'fw') \
            else (s.P['BAL_KP'], s.P['BAL_KI'], s.P['BAL_KD'])

    os.makedirs('results', exist_ok=True)
    P = runs[0][1]
    tag = f"m{P['BODY_MASS']:.2f}_h{P['COM_HEIGHT']:.2f}_kp{gains[0]:g}_ki{gains[1]:g}_kd{gains[2]:.3g}_off{a.offset:g}"
    png = os.path.join('results', f'run_{tag}.png')

    fig, ax = plt.subplots(4, 2, figsize=(11, 8.2), sharex=True)
    titles = ['MPU6050 + Kalman (giống xe thật)', 'Góc đo lý tưởng (không sai số cảm biến)']
    for c, (ideal, P_, log, m) in enumerate(runs):
        t = log['t']
        a0 = ax[0, c]
        a0.plot(t, log['angle_true'], color=ps.BLUE, label='góc thật')
        if not ideal:
            a0.plot(t, log['angle_est'], color=ps.ORANGE, label='góc STM32 ước lượng')
        a0.set_ylabel('góc nghiêng (°)')
        a0.legend(loc='upper left')
        st = 'ĐỨNG ĐƯỢC hết %.0f s' % a.t if m['ok'] else 'ĐỔ sau %.2f s' % m['t_fall']
        a0.set_title(f"{titles[c]}\n{st}", loc='left')
        ax[1, c].plot(t, log['rpm_ref'], color=ps.BLUE, label='tốc độ đặt (PID góc)')
        ax[1, c].plot(t, log['rpm1'], color=ps.ORANGE, label='tốc độ đo (encoder)')
        ax[1, c].set_ylabel('rpm')
        ax[1, c].legend(loc='upper left')
        ax[2, c].plot(t, log['duty1'], color=ps.BLUE)
        ax[2, c].axhline(99, color=ps.TEXT2, lw=0.8, ls='--')
        ax[2, c].axhline(-99, color=ps.TEXT2, lw=0.8, ls='--')
        ax[2, c].set_ylabel('duty động cơ 1 (%)')
        ax[3, c].plot(t, log['x'] * 100, color=ps.BLUE)
        ax[3, c].set_ylabel('xe đi được (cm)')
        ax[3, c].set_xlabel('thời gian (s)')
        if a.push and t[-1] > 4.0:
            for r in range(4):
                ax[r, c].axvspan(4.0, 4.1, color=ps.GRID, lw=0)
    fig.suptitle(f"{backend} · PID góc Kp {gains[0]:g} / Ki {gains[1]:g} / Kd {gains[2]:.3g} · "
                 f"thân xe {P['BODY_MASS']:.2f} kg, trọng tâm {P['COM_HEIGHT']*100:.0f} cm trên trục bánh · "
                 f"thả ở {a.theta0:g}° · lệch góc đặt {a.offset:g}° · t = 0 lúc code bật động cơ",
                 fontsize=9, color=ps.TEXT2, x=0.01, ha='left')
    fig.tight_layout()
    fig.savefig(png, dpi=130)

    with open(png.replace('.png', '.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        log = runs[0][2]
        keys = ['t', 'angle_true', 'angle_est', 'rpm_ref', 'rpm1', 'rpm2', 'duty1', 'duty2', 'x']
        w.writerow(keys)
        for i in range(len(log['t'])):
            w.writerow([f'{log[k][i]:.5g}' for k in keys])

    for ideal, _, _, m in runs:
        name = 'ideal IMU ' if ideal else 'MPU+Kalman'
        if m['ok']:
            print(f"{name}: OK, angle p-p {m['angle_pp']:.2f} deg, drift {m['drift_x']*100:.1f} cm")
        else:
            print(f"{name}: FELL at {m['t_fall']:.2f} s")
    print('saved', png)


if __name__ == '__main__':
    main()
