# -*- coding: utf-8 -*-
"""
Kiem tra ban chep Python (controller.py) cho ra CUNG KET QUA voi ham C that trong main.c:
PID_Update (PID dong co), PID_UpdateRate (PID goc), Kalman_Update. Chay 20 000 buoc voi
dau vao ngau nhien, in sai lech lon nhat. Can gcc.
"""
import ctypes
import numpy as np

import params
import fw_build
from fw_build import Firmware, PID_t, Kalman_t
from controller import PID, Kalman


def main():
    mc, mh = fw_build.find_main_c()
    D = fw_build.read_defines(mc)
    fw = Firmware(mc, mh)
    L = fw.lib
    for f in ('sim_pid_update', 'sim_pid_update_rate', 'sim_kalman_update'):
        getattr(L, f).restype = ctypes.c_float
    L.sim_pid_update.argtypes = [ctypes.POINTER(PID_t)] + [ctypes.c_float] * 3
    L.sim_pid_update_rate.argtypes = [ctypes.POINTER(PID_t)] + [ctypes.c_float] * 4
    L.sim_kalman_update.argtypes = [ctypes.POINTER(Kalman_t)] + [ctypes.c_float] * 3
    L.sim_kalman_init.argtypes = [ctypes.POINTER(Kalman_t)] + [ctypes.c_float] * 3
    ts = D['CTRL_TS']
    rng = np.random.default_rng(0)
    N = 20000

    # --- PID dong co (PID_Update) ---
    c = PID_t(D['MOT_KP'], D['MOT_KI'], D['MOT_KD'], D['MOT_KFF'], D['MOT_D_ALPHA'], D['MOT_KAW'],
              -D['DUTY_MAX_PCT'], D['DUTY_MAX_PCT'], 0, 0, 0, 0)
    p = PID(D['MOT_KP'], D['MOT_KI'], D['MOT_KD'], D['MOT_KFF'], D['MOT_D_ALPHA'], D['MOT_KAW'], D['DUTY_MAX_PCT'])
    ref = np.cumsum(rng.normal(0, 3, N))
    meas = ref + rng.normal(0, 15, N)
    err = [abs(L.sim_pid_update(ctypes.byref(c), r - m, D['MOT_KFF'] * r, ts) -
               p.update(r - m, D['MOT_KFF'] * r, ts)) for r, m in zip(ref, meas)]
    print('PID dong co  (PID_Update)    : sai lech lon nhat %.2e %%duty' % max(err))

    # --- PID goc (PID_UpdateRate) ---
    c = PID_t(D['BAL_KP'], D['BAL_KI'], D['BAL_KD'], 0, D['BAL_D_ALPHA'], D['BAL_KAW'],
              -D['BAL_SPEED_LIMIT'], D['BAL_SPEED_LIMIT'], 0, 0, 0, 0)
    p = PID(D['BAL_KP'], D['BAL_KI'], D['BAL_KD'], 0.0, D['BAL_D_ALPHA'], D['BAL_KAW'], D['BAL_SPEED_LIMIT'])
    e = np.cumsum(rng.normal(0, 0.002, N))
    r = rng.normal(0, 0.3, N)
    err = [abs(L.sim_pid_update_rate(ctypes.byref(c), a, 0.0, ts, b) - p.update_rate(a, 0.0, ts, b))
           for a, b in zip(e, r)]
    print('PID goc      (PID_UpdateRate): sai lech lon nhat %.2e rad/s' % max(err))

    # --- Kalman ---
    P = params.as_dict()
    P.update({k: D[k] for k in D if k in P})
    ck = Kalman_t()
    L.sim_kalman_init(ctypes.byref(ck), 2.0, 1.5, 0.03)
    pk = Kalman(P, 2.0, 1.5, 0.03)
    ang = np.cumsum(rng.normal(0, 0.05, N))
    gy = np.diff(np.r_[0, ang]) / ts + 1.5 + rng.normal(0, 0.3, N)
    acc = ang + rng.normal(0, 0.5, N)
    acc[5000:5030] += 40        # thu cong loai mau (KF_GATE)
    err = [abs(L.sim_kalman_update(ctypes.byref(ck), g, a, ts) - pk.update(g, a, ts)) for g, a in zip(gy, acc)]
    print('Kalman       (Kalman_Update) : sai lech lon nhat %.2e do' % max(err))
    print('(C tinh bang float 32 bit, Python 64 bit -> sai lech ~1e-5 la do lam tron)')


if __name__ == '__main__':
    main()
