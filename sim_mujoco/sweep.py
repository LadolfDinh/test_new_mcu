# -*- coding: utf-8 -*-
"""
Quet khoi luong than xe x chieu cao trong tam (chua do duoc) voi PID hien tai.
Moi o = so giay xe dung duoc (toi da T_END). 3 truong hop:
  1) MPU6050 + Kalman nhu xe that
  2) goc do ly tuong
  3) goc do ly tuong nhung goc dat lech 0.5 do (trong tam khong dung ngay tren truc)
Ket qua: results/sweep.png + results/sweep.csv
"""
import csv
import os
import sys
from multiprocessing import Pool

import numpy as np

import plot_style as ps
from sim import metrics
from sim_fw import make_sim
import fw_build

ps.setup()
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

MASSES = [0.4, 0.6, 0.8, 1.0, 1.2, 1.5]
HEIGHTS = [0.04, 0.06, 0.08, 0.10, 0.12, 0.15, 0.18, 0.22]
T_END = 10.0
CASES = [
    ('MPU6050 + Kalman (giống xe thật)', {}),
    ('Góc đo lý tưởng', {'IDEAL_IMU': True}),
    ('Góc đo lý tưởng, góc đặt lệch 0.5°', {'IDEAL_IMU': True, 'ANGLE_OFFSET_DEG': 0.5}),
]
EXTRA = {}   # vd: {'BAL_KP': 20} de quet voi PID khac


def one(job):
    ci, m, h = job
    ov = dict(CASES[ci][1], BODY_MASS=m, COM_HEIGHT=h, **EXTRA)
    s = make_sim(ov, seed=1)
    log = s.run(T_END)
    r = metrics(log, T_END)
    return ci, m, h, (T_END if r['ok'] else r['t_fall']), r.get('angle_pp', float('nan'))


def main():
    try:
        fw_build.build(quiet=True)        # bien dich 1 lan truoc khi chia tien trinh
    except Exception as e:
        print('[sweep] khong bien dich duoc code C -> dung ban Python:', str(e).splitlines()[0])
    jobs = [(ci, m, h) for ci in range(len(CASES)) for m in MASSES for h in HEIGHTS]
    with Pool() as p:
        res = p.map(one, jobs)
    os.makedirs('results', exist_ok=True)
    with open('results/sweep.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['case', 'body_mass_kg', 'com_height_m', 'time_standing_s', 'angle_pp_deg'])
        for ci, m, h, ts, pp in res:
            w.writerow([CASES[ci][0], m, h, round(ts, 3), round(pp, 3)])

    grid = np.zeros((len(CASES), len(HEIGHTS), len(MASSES)))
    for ci, m, h, ts, _ in res:
        grid[ci, HEIGHTS.index(h), MASSES.index(m)] = ts

    bounds = [0, 1, 2, 3, 5, 7, 9.99, 10.01]
    cmap = ListedColormap(ps.SEQ_BLUE)
    norm = BoundaryNorm(bounds, cmap.N)
    fig, axs = plt.subplots(1, len(CASES), figsize=(4.0 * len(CASES), 4.4), sharey=True)
    for ci, axx in enumerate(axs):
        axx.imshow(grid[ci], cmap=cmap, norm=norm, origin='lower', aspect='auto')
        axx.grid(False)
        for i in range(len(HEIGHTS)):
            for j in range(len(MASSES)):
                v = grid[ci, i, j]
                txt = 'đứng' if v >= T_END else f'{v:.1f}'
                axx.text(j, i, txt, ha='center', va='center', fontsize=7.5,
                         color='white' if v >= 5 else ps.TEXT)
        axx.set_xticks(range(len(MASSES)), [f'{m:g}' for m in MASSES])
        axx.set_yticks(range(len(HEIGHTS)), [f'{h*100:g}' for h in HEIGHTS])
        axx.set_xlabel('khối lượng thân xe (kg)')
        axx.set_title(CASES[ci][0], loc='left', fontsize=9)
    axs[0].set_ylabel('trọng tâm trên trục bánh (cm)')
    backend = make_sim({}).backend
    fig.suptitle(f'PID hiện tại ({backend}): số giây xe đứng được (tối đa {T_END:g} s, thả ở 3°)',
                 x=0.01, ha='left', color=ps.TEXT, fontsize=10, fontweight='bold')
    fig.tight_layout()
    fig.savefig('results/sweep.png', dpi=130)
    for ci in range(len(CASES)):
        ok = int((grid[ci] >= T_END).sum())
        print(f'case {ci+1}: standing {ok}/{grid[ci].size}, median time {np.median(grid[ci]):.2f} s')
    print('saved results/sweep.png')


if __name__ == '__main__':
    main()
