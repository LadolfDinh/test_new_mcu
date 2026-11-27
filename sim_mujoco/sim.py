# -*- coding: utf-8 -*-
"""
Mo hinh MuJoCo + dong co DC/L298N + MPU6050 + encoder, ghep voi controller.py.
Quy uoc: truc x = phia truoc xe, banh quay quanh truc y.
Goc nghieng > 0 = nghieng ve truoc (giong ANGLE_SIGN = 1 trong main.c).
"""
import math
import numpy as np
import mujoco

import params
from controller import BalanceController


# ============================ MO HINH MJCF ============================
def build_xml(P, lifted=False):
    bx, by, bz = P['BODY_SIZE']
    m = P['BODY_MASS']
    h = P['COM_HEIGHT']
    ixx = m / 12.0 * (by ** 2 + bz ** 2)
    iyy = m / 12.0 * (bx ** 2 + bz ** 2)
    izz = m / 12.0 * (bx ** 2 + by ** 2)
    r, w, mw = P['WHEEL_RADIUS'], P['WHEEL_WIDTH'], P['WHEEL_MASS']
    iw_ax = 0.5 * mw * r * r
    iw_tr = mw * (3 * r * r + w * w) / 12.0
    yw = P['WHEEL_TRACK'] / 2.0
    box_z = max(h, bz / 2.0 - 0.01)   # chi de ve
    by_vis = min(by, P['WHEEL_TRACK'] - P['WHEEL_WIDTH'] - 0.01)   # khong cham vao banh
    # vach ke tren san moi 10 cm (chi de nhin thay xe chay)
    marks = '\n'.join(
        f'    <geom type="box" size="0.003 0.25 0.0005" pos="{x / 10:.1f} 0 0.0005" '
        f'rgba="{".45 .45 .45" if x % 10 == 0 else ".72 .72 .72"} 1" contype="0" conaffinity="0"/>'
        for x in range(-20, 61))
    return f"""
<mujoco model="xe_2_banh">
  <compiler angle="degree"/>
  <option timestep="{P['SIM_DT']}" integrator="implicitfast" gravity="0 0 -9.81"/>
  <visual><global offwidth="960" offheight="540"/><headlight ambient=".4 .4 .4" diffuse=".5 .5 .5"/></visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="1 1 1" rgb2=".86 .89 .93" width="256" height="256"/>
  </asset>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" directional="true" castshadow="false"/>
    <geom name="floor" type="plane" size="10 1.5 0.1" rgba=".9 .9 .9 1" friction="{P['GROUND_FRICTION']} 0.005 0.0001"/>
{marks}
    <body name="chassis" pos="0 0 {0.25 if lifted else r}">
      {'' if lifted else '<freejoint name="root"/>'}
      <inertial pos="{P['COM_X_OFFSET']} 0 {h}" mass="{m}" diaginertia="{ixx} {iyy} {izz}"/>
      <geom name="body_vis" type="box" size="{bx/2} {by_vis/2} {bz/2}" pos="0 0 {box_z}" rgba=".2 .45 .8 .6" mass="0"/>
      <geom name="axle" type="cylinder" size="0.006 {yw}" euler="90 0 0" rgba=".3 .3 .3 1" mass="0" contype="0" conaffinity="0"/>
      <geom name="com_mark" type="sphere" size="0.012" pos="{P['COM_X_OFFSET']} 0 {h}" rgba="1 .2 .2 1" mass="0" contype="0" conaffinity="0"/>
      <site name="imu" pos="0 0 {P['IMU_HEIGHT']}" size="0.008" rgba="0 1 0 1"/>
      <body name="wheel_L" pos="0 {yw} 0">
        <joint name="wheel_L" type="hinge" axis="0 1 0" armature="{P['ROTOR_INERTIA_OUT']}" frictionloss="{P['GEAR_FRICTION']}"/>
        <inertial pos="0 0 0" mass="{mw}" diaginertia="{iw_tr} {iw_ax} {iw_tr}"/>
        <geom type="cylinder" size="{r} {w/2}" euler="90 0 0" rgba=".1 .1 .1 1" mass="0" friction="{P['GROUND_FRICTION']} 0.005 0.0001"/>
        <geom type="box" size="{r*0.8} {w/2+0.001} 0.004" rgba=".9 .8 .1 1" mass="0" contype="0" conaffinity="0"/>
      </body>
      <body name="wheel_R" pos="0 {-yw} 0">
        <joint name="wheel_R" type="hinge" axis="0 1 0" armature="{P['ROTOR_INERTIA_OUT']}" frictionloss="{P['GEAR_FRICTION']}"/>
        <inertial pos="0 0 0" mass="{mw}" diaginertia="{iw_tr} {iw_ax} {iw_tr}"/>
        <geom type="cylinder" size="{r} {w/2}" euler="90 0 0" rgba=".1 .1 .1 1" mass="0" friction="{P['GROUND_FRICTION']} 0.005 0.0001"/>
        <geom type="box" size="{r*0.8} {w/2+0.001} 0.004" rgba=".9 .8 .1 1" mass="0" contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="m1" joint="wheel_L" gear="1" ctrllimited="false"/>
    <motor name="m2" joint="wheel_R" gear="1" ctrllimited="false"/>
  </actuator>
  <sensor>
    <accelerometer name="acc" site="imu"/>
    <gyro name="gyro" site="imu"/>
  </sensor>
</mujoco>
"""


# ============================ DONG CO + L298N ============================
def motor_current_avg(duty_pct, omega_out, P):
    """Dong trung binh 1 chu ky PWM (A), L298N kieu IN co dinh + EN bam xung:
    luc EN tat thi dong xa qua diode ve nguon (fast decay) -> dong co the ve 0
    va dung lai (gian doan) -> tao vung chet, tang theo tan so PWM.
    duty_pct co dau; omega_out: toc do banh so voi than xe (rad/s)."""
    if duty_pct == 0.0:
        return 0.0
    s = 1.0 if duty_pct > 0 else -1.0
    d = min(abs(duty_pct) / 100.0, 1.0)
    E = s * P['MOTOR_KE'] * P['GEAR_RATIO'] * omega_out    # back-EMF theo chieu dang day
    R, L = P['MOTOR_R'], P['MOTOR_L']
    v_on = P['V_BATT'] - P['L298_DROP']
    v_off = -(P['V_BATT'] + P['DIODE_DROP'])
    T = 1.0 / P['PWM_FREQ']
    ton = d * T
    tau = L / R
    a = (v_on - E) / R
    if a <= 0.0:
        return 0.0                      # dong co quay nhanh hon dien ap dat: khong co dong
    ip = a * (1.0 - math.exp(-ton / tau))
    B = (E - v_off) / R
    tf = tau * math.log((ip + B) / B)
    if ton + tf <= T:
        i = (a * ton - B * tf) / T      # che do gian doan
    else:
        i = (d * v_on + (1 - d) * v_off - E) / R   # che do lien tuc
        i = max(i, 0.0)
    return s * i


def motor_torque(duty_pct, omega_out, P):
    i = motor_current_avg(duty_pct, omega_out, P)
    return P['MOTOR_KE'] * P['GEAR_RATIO'] * P['GEAR_EFF'] * i


# ============================ MPU6050 ============================
class Mpu6050:
    """Lay mau 1 kHz, loc thong thap bac 1 (DLPF 44 Hz), luong tu hoa theo LSB,
    co bias gyro + nhieu. Tra ve (acc_deg, gyro_dps) giong Mpu6050_ReadRaw."""

    def __init__(self, P, rng):
        self.P, self.rng = P, rng
        dt = 1.0 / P['MPU_RATE_HZ']
        self.k = 1.0 - math.exp(-2 * math.pi * P['MPU_DLPF_HZ'] * dt)
        self.acc = None
        self.gyr = None
        mr = math.radians(P['MOUNT_ERR_DEG'])
        self.cm, self.sm = math.cos(mr), math.sin(mr)

    def sample(self, acc_site, gyro_site, vib):
        """acc_site, gyro_site: gia tri MuJoCo trong khung site (m/s^2, rad/s)"""
        P, rng = self.P, self.rng
        g = 9.81
        an = P['ACC_NOISE_G'] + (P['VIB_ACC_NOISE_G'] if vib else 0.0)
        gn = P['GYRO_NOISE_DPS'] + (P['VIB_GYRO_NOISE_DPS'] if vib else 0.0)
        ax, ay, az = acc_site / g
        # lech lap dat quanh truc y
        ax, az = self.cm * ax - self.sm * az, self.sm * ax + self.cm * az
        a = np.array([ax, ay, az]) + rng.normal(0, an, 3)
        w = math.degrees(gyro_site[1]) + P['GYRO_BIAS_DPS'] + rng.normal(0, gn)
        if self.acc is None:
            self.acc, self.gyr = a, w
        self.acc = self.acc + self.k * (a - self.acc)
        self.gyr = self.gyr + self.k * (w - self.gyr)

    def read(self):
        P = self.P
        q = lambda v, lsb: round(v * lsb) / lsb
        ax, ay, az = (q(v, P['ACCEL_LSB_PER_G']) for v in self.acc)
        acc_deg = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        return acc_deg, q(self.gyr, P['GYRO_LSB_PER_DPS'])


# ============================ MO PHONG ============================
class Sim:
    backend = 'Python (ban chep)'

    def __init__(self, overrides=None, seed=0, theta0_deg=3.0):
        P = params.as_dict()
        try:                                  # lay thong so dieu khien moi nhat tu main.c (neu co)
            import fw_build
            D = fw_build.read_defines(fw_build.find_main_c()[0])
            P.update({k: v for k, v in D.items() if k in P})
            P['USE_KALMAN'] = bool(P['USE_KALMAN'])
        except Exception:
            pass
        if overrides:
            P.update(overrides)
        self.P = P
        self.rng = np.random.default_rng(seed)
        self.model = mujoco.MjModel.from_xml_string(build_xml(P))
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.jL = m.joint('wheel_L').qposadr[0]
        self.jR = m.joint('wheel_R').qposadr[0]
        self.vL = m.joint('wheel_L').dofadr[0]
        self.vR = m.joint('wheel_R').dofadr[0]
        self.acc_adr = m.sensor('acc').adr[0]
        self.gyr_adr = m.sensor('gyro').adr[0]
        self.body_id = m.body('chassis').id

        # dat goc nghieng ban dau quanh truc banh
        th = math.radians(theta0_deg)
        quat0 = [math.cos(th / 2), 0, math.sin(th / 2), 0]
        self.data.qpos[3:7] = quat0
        # cho banh lun vao dat on dinh (giu than xe dung goc ban dau, nhu dang giu tay)
        for _ in range(400):
            mujoco.mj_step(m, self.data)
            self.data.qpos[0:2] = 0.0
            self.data.qpos[3:7] = quat0
            self.data.qvel[:] = 0.0
            self.data.qpos[self.jL] = self.data.qpos[self.jR] = 0.0
        mujoco.mj_forward(m, self.data)

        # hieu chuan nhu Mpu6050_Calibrate: 500 mau, xe dung yen (giu tay), dong co tat
        self.imu = Mpu6050(P, self.rng)
        acc_s, gyr_s = [], []
        site_R = self.data.site('imu').xmat.reshape(3, 3)
        acc_static = site_R.T @ np.array([0.0, 0.0, 9.81])   # dung yen: gia toc ke chi thay trong luc
        for i in range(P['GYRO_CAL_SAMPLES'] * 2):      # 2 ms / mau, IMU 1 kHz
            self.imu.sample(acc_static, np.zeros(3), vib=False)
            if i % 2 == 1:
                a, g = self.imu.read()
                acc_s.append(a)
                gyr_s.append(g)
        self.ctrl = BalanceController(P, acc_s, gyr_s)

        # giong AUTO_ARM: Kalman chay them AUTO_ARM_HOLD (0.5 s) luc xe con duoc giu thang,
        # dong co chua chay -> roi moi arm (reset 3 PID) va tha tay
        self.ctrl.armed = False
        for i in range(int(P['AUTO_ARM_HOLD_S'] / P['CTRL_TS'])):
            for _ in range(2):
                self.imu.sample(acc_static, np.zeros(3), vib=False)
            a, g = self.imu.read()
            self.ctrl.mpu_update(a, g, P['CTRL_TS'])
            self.ctrl.step(0, 0)
        self.ctrl.arm()

        self.out1 = self.out2 = 0.0          # duty that dang ra chan PWM
        self.pending = None
        self.t = 0.0
        self.k = 0
        self.push = []                       # (t0, t1, Fx) luc day than xe
        self.log = {k: [] for k in ('t', 'angle_true', 'angle_est', 'gyro_est', 'rpm_ref', 'rpm1', 'rpm2',
                                    'duty1', 'duty2', 'x', 'tau1', 'i1', 'armed')}
        self.fell = False

    t_rel = property(lambda s: s.t)

    def true_pitch_deg(self):
        q = self.data.qpos[3:7]
        w, x, y, z = q
        return math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))

    def step_ctrl(self):
        """1 chu ky TIM6 (2 ms) = 4 buoc vat ly."""
        P, d, m = self.P, self.data, self.model
        n_sub = int(round(P['CTRL_TS'] / P['SIM_DT']))
        per_imu = int(round(1.0 / P['MPU_RATE_HZ'] / P['SIM_DT']))

        # dau ngat: doc MPU (gia tri moi nhat trong thanh ghi) + encoder
        if P.get('IDEAL_IMU', False):
            self.ctrl.angle_deg = self.true_pitch_deg() + P['ANGLE_OFFSET_DEG']
            self.ctrl.gyro_dps = math.degrees(d.qvel[4])   # van toc goc quanh y (khung than)
        else:
            acc_deg, gyro_dps = self.imu.read()
            self.ctrl.mpu_update(acc_deg, gyro_dps, P['CTRL_TS'])
        c1 = math.floor(d.qpos[self.jL] * P['COUNTS_PER_REV'] / (2 * math.pi))
        c2 = math.floor(d.qpos[self.jR] * P['COUNTS_PER_REV'] / (2 * math.pi))
        new1, new2 = self.ctrl.step(c1, c2)

        for s in range(n_sub):
            if s == P['ISR_DELAY_STEPS']:
                self.out1, self.out2 = new1, new2
            tau1 = motor_torque(self.out1, d.qvel[self.vL], P)
            tau2 = motor_torque(self.out2, d.qvel[self.vR], P)
            d.ctrl[0], d.ctrl[1] = tau1, tau2
            d.xfrc_applied[self.body_id, :] = 0
            for (t0, t1, fx) in self.push:
                if t0 <= self.t < t1:
                    d.xfrc_applied[self.body_id, 0] = fx
            mujoco.mj_step(m, d)
            self.t += P['SIM_DT']
            self.k += 1
            if self.k % per_imu == 0:
                self.imu.sample(d.sensordata[self.acc_adr:self.acc_adr + 3],
                                d.sensordata[self.gyr_adr:self.gyr_adr + 3],
                                vib=self.ctrl.armed)

        L = self.log
        L['t'].append(self.t)
        L['angle_true'].append(self.true_pitch_deg())
        L['angle_est'].append(self.ctrl.angle_deg)
        L['gyro_est'].append(self.ctrl.gyro_dps)
        L['rpm_ref'].append(self.ctrl.rpm_ref)
        L['rpm1'].append(self.ctrl.rpm1)
        L['rpm2'].append(self.ctrl.rpm2)
        L['duty1'].append(self.ctrl.duty1)
        L['duty2'].append(self.ctrl.duty2)
        L['x'].append(d.qpos[0])
        L['tau1'].append(tau1)
        L['i1'].append(motor_current_avg(self.out1, d.qvel[self.vL], P))
        L['armed'].append(self.ctrl.armed)
        if abs(self.true_pitch_deg()) > 45 or not self.ctrl.armed:
            self.fell = True

    def run(self, t_end, stop_on_fall=True):
        while self.t < t_end - 1e-9:
            self.step_ctrl()
            if self.fell and stop_on_fall:
                break
        return {k: np.array(v) for k, v in self.log.items()}


def metrics(log, t_end, settle_from=None):
    """Tom tat 1 lan chay."""
    t = log['t']
    ok = len(t) > 0 and t[-1] >= t_end - 0.01 and bool(log['armed'][-1])
    res = {'ok': ok, 't_fall': None if ok else float(t[-1]) if len(t) else 0.0}
    if ok:
        s = t >= (settle_from if settle_from is not None else t_end - 2.0)
        res['angle_pp'] = float(np.ptp(log['angle_true'][s]))           # dao dong dinh-dinh (do)
        res['angle_std'] = float(np.std(log['angle_true'][s]))
        res['duty_sat_pct'] = float(np.mean(np.abs(log['duty1']) >= 98.9) * 100)
        res['max_angle'] = float(np.max(np.abs(log['angle_true'])))
        res['drift_x'] = float(log['x'][-1] - log['x'][0])
        res['rpm_pp'] = float(np.ptp(log['rpm1'][s]))
    return res
