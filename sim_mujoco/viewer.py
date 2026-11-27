# -*- coding: utf-8 -*-
"""
Xem 3D truc tiep (chay tren PC co man hinh):  python viewer.py [--ideal] [--mass 1.0 --com 0.15]

Phim tat (bam trong cua so MuJoCo):
  P : day than xe ve truoc (1 N trong 0.1 s)
  O : day ve sau
  R : tha lai tu dau
Ngoai ra: giu Ctrl + keo chuot phai len than xe de tu day bang tay.
"""
import argparse
import time

import mujoco
import mujoco.viewer

from sim_fw import make_sim as Sim

ap = argparse.ArgumentParser()
ap.add_argument('--ideal', action='store_true', help='dung goc ly tuong thay cho MPU6050 + Kalman')
ap.add_argument('--mass', type=float)
ap.add_argument('--com', type=float)
ap.add_argument('--offset', type=float, default=0.0)
ap.add_argument('--theta0', type=float, default=3.0)
a = ap.parse_args()

ov = {'IDEAL_IMU': a.ideal, 'ANGLE_OFFSET_DEG': a.offset}
if a.mass is not None:
    ov['BODY_MASS'] = a.mass
if a.com is not None:
    ov['COM_HEIGHT'] = a.com

state = {'sim': Sim(ov, theta0_deg=a.theta0), 'reset': False}


def key_cb(key):
    s = state['sim']
    if key == ord('P'):
        s.push.append((s.t_rel, s.t_rel + 0.1, 1.0))
    elif key == ord('O'):
        s.push.append((s.t_rel, s.t_rel + 0.1, -1.0))
    elif key == ord('R'):
        state['reset'] = True


s = state['sim']
with mujoco.viewer.launch_passive(s.model, s.data, key_callback=key_cb) as v:
    v.cam.distance, v.cam.elevation, v.cam.azimuth = 0.9, -12, 90
    t_wall0, t_sim0 = time.time(), s.t
    last_print = 0.0
    while v.is_running():
        if state['reset']:
            state['reset'] = False
            old = state['sim']
            new = Sim(ov, theta0_deg=a.theta0)
            mujoco.mj_copyData(old.data, old.model, new.data)   # viewer giu nguyen MjData cu
            new.model, new.data = old.model, old.data
            state['sim'] = new
            t_wall0, t_sim0 = time.time(), new.t
            last_print = -1.0
        s = state['sim']
        if not s.fell:
            s.step_ctrl()
        v.cam.lookat[:] = [s.data.qpos[0], 0, 0.1]
        v.sync()
        if s.t - last_print > 0.5:
            last_print = s.t
            st = 'FELL' if s.fell else 'ok'
            print(f"t={s.t:5.1f}s  angle={s.true_pitch_deg():6.2f}  est={s.ctrl.angle_deg:6.2f}  "
                  f"rpm_ref={s.ctrl.rpm_ref:6.1f}  duty={s.ctrl.duty1:6.1f}  {st}")
        # chay dung thoi gian thuc
        lag = (s.t - t_sim0) - (time.time() - t_wall0)
        if lag > 0:
            time.sleep(lag)
