# -*- coding: utf-8 -*-
"""
Mo phong voi CODE C THAT cua main.c (bien dich boi fw_build.py).

Phan cung gia lap (moi buoc vat ly 0.5 ms):
  - MPU6050: ghi thanh ghi tho 0x3B..0x48 (int16, big-endian) theo cau hinh ma code da ghi
             vao 0x1A/0x1B/0x1C (DLPF, thang do). Code doc bang HAL_I2C_Mem_Read nhu that.
  - Encoder: sinh xung A/B vuong pha (11 xung/vong truc dong co, hop so 1:45) tren dung chan
             trong main.h; moi suon kenh A goi HAL_GPIO_EXTI_Callback -> Encoder_ISR.
  - TIM6:    goi HAL_TIM_PeriodElapsedCallback dung chu ky tinh tu PSC/ARR cua MX_TIM6_Init.
  - DWT->CYCCNT, HAL_GetTick: chay theo thoi gian mo phong (168 MHz).
  - L298N:   doc CCR/ARR cua TIM1, TIM8 va muc chan IN1..IN4 -> dien ap dong co.
  - Vong while(1): Button_Poll / Auto_Arm_Poll / Led_Update goi moi 0.5 ms.
"""
import math
import re

import numpy as np
import mujoco

import params
import fw_build
from fw_build import Firmware
from sim import build_xml, motor_current_avg, motor_torque, metrics  # noqa: F401

C_U8 = __import__('ctypes').c_uint8
C_F = __import__('ctypes').c_float
DLPF_HZ = {0: 260, 1: 184, 2: 94, 3: 44, 4: 21, 5: 10, 6: 5, 7: 260}
GYRO_LSB = {0: 131.0, 1: 65.5, 2: 32.8, 3: 16.4}
ACC_LSB = {0: 16384.0, 1: 8192.0, 2: 4096.0, 3: 2048.0}
PORTS = {'GPIOA': 0, 'GPIOB': 1, 'GPIOC': 2, 'GPIOD': 3, 'GPIOE': 4}


def read_pins(main_h):
    src = open(main_h, encoding='utf-8', errors='replace').read()
    pins = {}
    for name in ('ENC1_A', 'ENC1_B', 'ENC2_A', 'ENC2_B', 'M1_IN1', 'M1_IN2', 'M2_IN3', 'M2_IN4'):
        pn = int(re.search(r'#define %s_Pin GPIO_PIN_(\d+)' % name, src).group(1))
        port = re.search(r'#define %s_GPIO_Port (GPIO\w)' % name, src).group(1)
        pins[name] = (PORTS[port], 1 << pn)
    return pins


class FwSim:
    """Giao dien giong sim.Sim: run(), log, fell, push, t, true_pitch_deg()."""
    backend = 'code C that (main.c)'

    def __init__(self, overrides=None, seed=0, theta0_deg=3.0, mode=3, test_rpm=40.0,
                 main_c=None, main_h=None):
        if main_c is None:
            main_c, main_h = fw_build.find_main_c()
        P = params.as_dict()
        D = fw_build.read_defines(main_c)
        cm = fw_build.read_cubemx(main_c)
        P['CTRL_TS'] = cm['tick_s']
        P['PWM_FREQ'] = cm['pwm1_hz']
        if overrides:
            P.update(overrides)
        self.P, self.D, self.mode = P, D, mode
        self.rng = np.random.default_rng(seed)
        self.pins = read_pins(main_h)

        lifted = P.get('LIFTED', mode in (1, 2))
        self.model = mujoco.MjModel.from_xml_string(build_xml(P, lifted=lifted))
        self.data = mujoco.MjData(self.model)
        m, d = self.model, self.data
        self.lifted = lifted
        self.jq = [m.joint('wheel_L').qposadr[0], m.joint('wheel_R').qposadr[0]]
        self.jv = [m.joint('wheel_L').dofadr[0], m.joint('wheel_R').dofadr[0]]
        self.acc_adr = m.sensor('acc').adr[0]
        self.gyr_adr = m.sensor('gyro').adr[0]
        self.body_id = m.body('chassis').id

        # dat goc ban dau, cho banh tiep dat on dinh (xe dang duoc giu tay)
        th = math.radians(theta0_deg if not lifted else 0.0)
        self.quat0 = [math.cos(th / 2), 0, math.sin(th / 2), 0]
        if not lifted:
            d.qpos[3:7] = self.quat0
            for _ in range(400):
                mujoco.mj_step(m, d)
                self._hold()
        mujoco.mj_forward(m, d)

        # khung cam bien theo MPU_TILT_AXIS / ANGLE_SIGN (khung than: x truoc, y truc banh, z len)
        ax_sel = int(D.get('MPU_TILT_AXIS', 0))
        sg = 1.0 if D.get('ANGLE_SIGN', 1.0) > 0 else -1.0
        if ax_sel == 0:     # nghieng quanh truc X cam bien: X = y_than, Y = -x_than
            R = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], float)
        else:               # quanh truc Y cam bien: X = x_than, Y = y_than
            R = np.eye(3)
        if sg < 0:
            R = np.diag([-1.0, -1.0, 1.0]) @ R
        mr = math.radians(P['MOUNT_ERR_DEG'])
        Ry = np.array([[math.cos(mr), 0, math.sin(mr)], [0, 1, 0], [-math.sin(mr), 0, math.cos(mr)]])
        self.R_sb = R @ Ry.T           # vector khung than -> khung cam bien
        self.mpu_state = None
        self.next_mpu = 0.0

        # encoder: chieu xung khi banh quay TOI theo cach noi day (khop ENCx_INVERT da chinh tren xe that)
        self.enc_sign = [(-1 if D.get('ENC1_INVERT', 0) else 1), (-1 if D.get('ENC2_INVERT', 0) else 1)]
        self.enc_x4 = 4.0 * D.get('ENC_PPR', 11.0) * D.get('GEAR_RATIO', 45.0) / (2 * math.pi)
        self.enc_n = [0, 0]
        self.mot_inv = [int(D.get('MOTOR1_INVERT', 0)), int(D.get('MOTOR2_INVERT', 0))]

        # ---- "bat nguon" STM32 ----
        self.fw = Firmware(main_c, main_h)
        fw = self.fw
        fw.set('g_test_mode', mode, C_U8)
        fw.set('g_test_rpm', test_rpm)
        self.t = 0.0
        self._set_time()
        fw.set_delay_cb(self._on_delay)
        fw.lib.sim_init()                 # khoi USER CODE 2 cua main(): PID_Init, Mpu6050_Init, hieu chuan...
        fw.set_delay_cb(lambda ms: None)

        # ghi de nhu sua trong cua so Watch
        if 'ANGLE_OFFSET_DEG' in P and P['ANGLE_OFFSET_DEG']:
            fw.set('g_bal_setpoint_deg', -P['ANGLE_OFFSET_DEG'])
        for key, (pid, field) in {'BAL_KP': ('g_pid_bal', 'kp'), 'BAL_KI': ('g_pid_bal', 'ki'),
                                  'BAL_KD': ('g_pid_bal', 'kd'), 'MOT_KP_W': ('g_pid1', 'kp')}.items():
            if overrides and key in overrides:
                setattr(fw.pid(pid), field, overrides[key])
                if pid == 'g_pid1':
                    setattr(fw.pid('g_pid2'), field, overrides[key])

        self.next_tick = self.t
        self.n_sub = 0
        self.pending = None
        self.duty_hw = [0.0, 0.0]
        self.push = []
        self.fell = False
        self.was_armed = False
        self.t_release = None
        self.held = not lifted
        self.log = {k: [] for k in ('t', 'angle_true', 'angle_est', 'gyro_est', 'rpm_ref', 'rpm1', 'rpm2',
                                    'duty1', 'duty2', 'x', 'armed', 'kf_bias', 'w1_true', 'w2_true')}

    # ------------------------------------------------------------------
    def _hold(self):
        d = self.data
        d.qpos[0:2] = 0.0
        d.qpos[3:7] = self.quat0
        d.qvel[:] = 0.0
        d.qpos[self.jq[0]] = d.qpos[self.jq[1]] = 0.0

    def _set_time(self):
        cyc = int(round(self.t * 168e6)) & 0xFFFFFFFF
        self.fw.lib.sim_set_time(int(self.t * 1000.0 + 1e-9) & 0xFFFFFFFF, cyc)

    def _on_delay(self, ms):
        """HAL_Delay trong luc khoi dong/hieu chuan: xe duoc giu yen, chi thoi gian va cam bien chay."""
        for _ in range(int(ms)):
            self.t += 0.001
            self._mpu_sample(static=True)
        self._set_time()

    def true_pitch_deg(self):
        if self.lifted:
            return 0.0
        w, x, y, z = self.data.qpos[3:7]
        return math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))

    # ------------------------------ MPU6050 ------------------------------
    def _mpu_sample(self, static=False):
        P, regs = self.P, self.fw.regs
        cfg = regs[0x1A] & 7
        g_lsb = GYRO_LSB[(regs[0x1B] >> 3) & 3]
        a_lsb = ACC_LSB[(regs[0x1C] >> 3) & 3]
        rate = (8000.0 if cfg in (0, 7) else 1000.0) / (1 + regs[0x19])
        dt = 1.0 / rate
        site_R = self.data.site('imu').xmat.reshape(3, 3)
        if static or self.held:
            acc_b = site_R.T @ np.array([0.0, 0.0, 9.81])
            gyr_b = np.zeros(3)
        else:
            acc_b = np.array(self.data.sensordata[self.acc_adr:self.acc_adr + 3])
            gyr_b = np.array(self.data.sensordata[self.gyr_adr:self.gyr_adr + 3])
        ideal = P.get('IDEAL_IMU', False)
        if ideal and not static:
            acc_b = site_R.T @ np.array([0.0, 0.0, 9.81])      # chi thay trong luc: goc gia toc ke dung
        armed = (not static) and self._armed()
        an = 0.0 if ideal else P['ACC_NOISE_G'] + (P['VIB_ACC_NOISE_G'] if armed else 0.0)
        gn = 0.0 if ideal else P['GYRO_NOISE_DPS'] + (P['VIB_GYRO_NOISE_DPS'] if armed else 0.0)
        a = self.R_sb @ (acc_b / 9.81) + self.rng.normal(0, an, 3)
        w = self.R_sb @ np.degrees(gyr_b) + self.rng.normal(0, gn, 3)
        if not ideal:
            w = w + np.array([P['GYRO_BIAS_DPS'], -0.6 * P['GYRO_BIAS_DPS'], 0.3 * P['GYRO_BIAS_DPS']])
        k = 1.0 if ideal else 1.0 - math.exp(-2 * math.pi * DLPF_HZ[cfg] * dt)
        if self.mpu_state is None:
            self.mpu_state = [a, w]
        self.mpu_state[0] = self.mpu_state[0] + k * (a - self.mpu_state[0])
        self.mpu_state[1] = self.mpu_state[1] + k * (w - self.mpu_state[1])
        raw = [*(self.mpu_state[0] * a_lsb), 0.0, *(self.mpu_state[1] * g_lsb)]
        for i, v in enumerate(raw):
            iv = int(max(-32768, min(32767, round(v))))
            regs[0x3B + 2 * i] = (iv >> 8) & 0xFF
            regs[0x3C + 2 * i] = iv & 0xFF
        return dt

    def _armed(self):
        return self.fw.get('g_armed', C_U8) != 0

    # ------------------------------ encoder ------------------------------
    def _encoders(self):
        fw, pins = self.fw, self.pins
        for i in range(2):
            n_new = math.floor(self.data.qpos[self.jq[i]] * self.enc_x4) * self.enc_sign[i]
            pa, pb = pins['ENC%d_A' % (i + 1)], pins['ENC%d_B' % (i + 1)]
            n = self.enc_n[i]
            step = 1 if n_new > n else -1
            while n != n_new:
                n_old_a = (n % 4) in (1, 2)
                n += step
                st = n % 4                      # thu tu thuan: AB = 00 -> 10 -> 11 -> 01
                a = st in (1, 2)
                b = st in (2, 3)
                fw.lib.sim_set_input(pb[0], pb[1], int(b))
                fw.lib.sim_set_input(pa[0], pa[1], int(a))
                if a != n_old_a:
                    fw.lib.sim_exti(pa[1])      # ngat EXTI suon kenh A
            self.enc_n[i] = n

    # ------------------------------ L298N ------------------------------
    def _read_driver(self):
        ccr1, arr1, ccr8, arr8 = self.fw.pwm()
        odr = {p: self.fw.lib.sim_get_odr(p) for p in set(v[0] for v in self.pins.values())}
        out = []
        for i, (ccr, arr, ia, ib) in enumerate(((ccr1, arr1, 'M1_IN1', 'M1_IN2'), (ccr8, arr8, 'M2_IN3', 'M2_IN4'))):
            a = (odr[self.pins[ia][0]] & self.pins[ia][1]) != 0
            b = (odr[self.pins[ib][0]] & self.pins[ib][1]) != 0
            duty = 100.0 * ccr / (arr + 1)
            fwd, rev = (a, b) if self.mot_inv[i] == 0 else (b, a)   # day noi theo MOTORx_INVERT
            out.append(duty if (fwd and not rev) else (-duty if (rev and not fwd) else 0.0))
        return out

    # ------------------------------ 1 buoc 0.5 ms ------------------------------
    def step(self):
        P, d, m = self.P, self.data, self.model
        if self.pending is not None:
            self.pending[0] -= 1
            if self.pending[0] <= 0:
                self.duty_hw = self.pending[1]
                self.pending = None
        if not self.held:
            for i in range(2):
                d.ctrl[i] = motor_torque(self.duty_hw[i], d.qvel[self.jv[i]], P)
            d.xfrc_applied[self.body_id, :] = 0
            for (t0, t1, fx) in self.push:          # thoi diem tinh tu luc tha tay
                if t0 <= self.t_rel < t1:
                    d.xfrc_applied[self.body_id, 0] = fx
            mujoco.mj_step(m, d)
        else:
            mujoco.mj_forward(m, d)
        self.t += P['SIM_DT']
        self._encoders()
        if self.t >= self.next_mpu - 1e-9:
            self.next_mpu += self._mpu_sample()
        self._set_time()
        self.fw.lib.sim_main_loop()
        if self.t_release is None and self._armed():   # Auto_Arm_Poll vua cho chay -> tha tay
            self.held = False
            self.t_release = self.t
        if self.t >= self.next_tick - 1e-9:
            self.next_tick += P['CTRL_TS']
            self.fw.lib.sim_tim6_irq()           # HAL_TIM_PeriodElapsedCallback(&htim6)
            self.pending = [P['ISR_DELAY_STEPS'], self._read_driver()]
            self._log()

    def _log(self):
        fw, L = self.fw, self.log
        armed = self._armed()
        L['t'].append(self.t - (self.t_release or 0.0))
        L['angle_true'].append(self.true_pitch_deg())
        L['angle_est'].append(fw.get('g_angle_deg'))
        L['gyro_est'].append(fw.get('g_gyro_dps'))
        L['rpm_ref'].append(fw.get('g_rpm_ref'))
        L['rpm1'].append(fw.get('g_rpm1'))
        L['rpm2'].append(fw.get('g_rpm2'))
        L['duty1'].append(fw.get('g_duty1'))
        L['duty2'].append(fw.get('g_duty2'))
        L['x'].append(self.data.qpos[0] if not self.lifted else 0.0)
        L['armed'].append(armed)
        L['kf_bias'].append(fw.get('g_kf_bias_dps'))
        L['w1_true'].append(self.data.qvel[self.jv[0]] * 9.549297)
        L['w2_true'].append(self.data.qvel[self.jv[1]] * 9.549297)
        if armed:
            self.was_armed = True
        if self.mode == 3 and self.was_armed and (not armed or abs(self.true_pitch_deg()) > 45):
            self.fell = True

    def wait_armed(self, t_max=5.0):
        """Giu xe cho toi khi code tu cho chay (AUTO_ARM)."""
        while self.t_release is None and self.t < t_max:
            self.step()
        for k in self.log:
            self.log[k].clear()

    def run(self, t_end, stop_on_fall=True, schedule=None):
        """t_end tinh tu luc tha tay. schedule: [(t, ten_bien, gia_tri)] ghi bien nhu cua so Watch."""
        self.wait_armed()
        t0 = self.t
        sched = sorted(schedule or [])
        while self.t - t0 < t_end - 1e-9:
            while sched and self.t - t0 >= sched[0][0]:
                _, name, val = sched.pop(0)
                self.fw.set(name, val)
            self.step()
            if self.fell and stop_on_fall:
                break
        return {k: np.array(v) for k, v in self.log.items()}

    # tuong thich voi viewer / render_video (1 chu ky dieu khien)
    def step_ctrl(self):
        n = int(round(self.P['CTRL_TS'] / self.P['SIM_DT']))
        for _ in range(n):
            self.step()

    @property
    def ctrl(self):
        return self

    t_rel = property(lambda s: s.t - (s.t_release if s.t_release is not None else s.t))

    # ten giong controller.BalanceController (cho viewer / render_video)
    angle_deg = property(lambda s: s.fw.get('g_angle_deg'))
    rpm_ref = property(lambda s: s.fw.get('g_rpm_ref'))
    duty1 = property(lambda s: s.fw.get('g_duty1'))
    armed = property(lambda s: s._armed())


def make_sim(overrides=None, seed=0, theta0_deg=3.0, backend='auto', **kw):
    """backend: 'c' = code C that (can gcc), 'py' = ban chep Python, 'auto' = C neu duoc."""
    if backend in ('auto', 'c'):
        try:
            return FwSim(overrides, seed=seed, theta0_deg=theta0_deg, **kw)
        except (RuntimeError, FileNotFoundError) as e:
            if backend == 'c':
                raise
            print('[sim] khong chay duoc code C (%s) -> dung ban Python' % str(e).splitlines()[0])
    from sim import Sim
    return Sim(overrides, seed=seed, theta0_deg=theta0_deg)
