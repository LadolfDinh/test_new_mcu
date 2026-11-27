# -*- coding: utf-8 -*-
"""
Xuat video so sanh 2 truong hop chay song song (can ffmpeg):
  trai : MPU6050 + Kalman (giong xe that)
  phai : goc do ly tuong, bi day nhe 1 N luc t = 3 s
  python render_video.py   -> results/so_sanh.mp4
"""
import os
import subprocess

import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont

from sim_fw import make_sim as Sim

W, H, FPS, T_END = 480, 360, 30, 6.0


def font(sz):
    for f in ('DejaVuSans.ttf', 'arial.ttf', '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'):
        try:
            return ImageFont.truetype(f, sz)
        except OSError:
            pass
    return ImageFont.load_default()


def run_frames(ov, title, push=None):
    s = Sim(ov)
    if hasattr(s, 'wait_armed'):
        s.wait_armed()                    # hieu chuan xong, code vua bat dong co -> t = 0
    if push:
        s.push = [push]
    r = mujoco.Renderer(s.model, H, W)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.8, -14, 90
    f1, f2 = font(18), font(15)
    frames, next_t, t_fall = [], 0.0, None
    while s.t_rel < T_END:
        if not s.fell:
            s.step_ctrl()
        else:
            if t_fall is None:
                t_fall = s.t_rel
            mujoco.mj_step(s.model, s.data)   # dong co tat: de xe nga tu nhien
            s.t += s.P['SIM_DT']
        if s.t_rel >= next_t:
            next_t += 1.0 / FPS
            cam.lookat[:] = [s.data.qpos[0], 0, 0.09]
            r.update_scene(s.data, cam)
            img = Image.fromarray(r.render())
            d = ImageDraw.Draw(img)
            d.rectangle((0, 0, W, 78), fill=(255, 255, 255))
            d.text((10, 8), title, fill=(25, 25, 25), font=f1)
            st = f'ĐỔ sau {t_fall:.2f} s' if t_fall else f'góc {s.true_pitch_deg():+5.1f}°   t = {s.t_rel:4.1f} s'
            d.text((10, 32), st, fill=(200, 40, 40) if t_fall else (60, 60, 60), font=f2)
            if push and push[0] <= s.t_rel < push[0] + 0.4:
                d.text((10, 54), '← bị đẩy 1 N', fill=(244, 114, 48), font=f2)
            frames.append(np.asarray(img))
    r.close()
    return frames


def main():
    os.makedirs('results', exist_ok=True)
    a = run_frames({}, 'MPU6050 + Kalman (giống xe thật)')
    b = run_frames({'IDEAL_IMU': True}, 'Góc đo lý tưởng', push=(3.0, 3.1, 1.0))
    n = min(len(a), len(b))
    out = 'results/so_sanh.mp4'
    p = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                          '-s', f'{2 * W + 4}x{H}', '-r', str(FPS), '-i', '-', '-c:v', 'libx264',
                          '-preset', 'veryfast', '-pix_fmt', 'yuv420p', out], stdin=subprocess.PIPE)
    sep = np.full((H, 4, 3), 255, np.uint8)
    for i in range(n):
        p.stdin.write(np.concatenate([a[i], sep, b[i]], axis=1).tobytes())
    p.stdin.close()
    p.wait()
    print('saved', out)


if __name__ == '__main__':
    main()
