# -*- coding: utf-8 -*-
"""
Ban chep Python cua phan dieu khien trong main.c (che do TEST_BALANCE).
Ten ham / bien giu giong code C de de doi chieu.
"""
import math


def clampf(x, lo, hi):
    return hi if x > hi else (lo if x < lo else x)


class PID:
    """PID_t + PID_Update / PID_UpdateRate cua main.c"""

    def __init__(self, kp, ki, kd, kff, d_alpha, kaw, out_limit):
        self.kp, self.ki, self.kd, self.kff = kp, ki, kd, kff
        self.d_alpha, self.kaw = d_alpha, kaw
        self.out_min, self.out_max = -out_limit, out_limit
        self.reset()

    def reset(self):
        self.e_prev = 0.0
        self.uI = 0.0
        self.uD = 0.0
        self.out = 0.0

    def _finish(self, e, ff, uP, uI, uD, ts):
        u_raw = ff + uP + uI + uD
        u = clampf(u_raw, self.out_min, self.out_max)
        uI += self.kaw * ts * (u - u_raw)
        self.e_prev, self.uI, self.uD, self.out = e, uI, uD, u
        return u

    def update(self, e, ff, ts):
        uP = self.kp * e
        uI = self.uI + self.ki * ts * e
        uD_raw = self.kd * (e - self.e_prev) / ts
        uD = self.d_alpha * uD_raw + (1.0 - self.d_alpha) * self.uD
        return self._finish(e, ff, uP, uI, uD, ts)

    def update_rate(self, e, ff, ts, de_dt):
        uP = self.kp * e
        uI = self.uI + self.ki * ts * e
        uD_raw = self.kd * de_dt
        uD = self.d_alpha * uD_raw + (1.0 - self.d_alpha) * self.uD
        return self._finish(e, ff, uP, uI, uD, ts)


class Kalman:
    """Kalman_t + Kalman_SetTuning / Kalman_Init / Kalman_Update cua main.c"""

    def __init__(self, P, angle0, bias0, r_meas):
        self.P = P
        self.set_tuning(r_meas, P['KF_TAU_S'], P['KF_BIAS_TAU_S'])
        self.angle, self.bias, self.rate = angle0, bias0, 0.0
        self.p00, self.p01, self.p10, self.p11 = r_meas, 0.0, 0.0, P.get('KF_P11_INIT', 0.01)
        self.innov, self.k0 = 0.0, 0.0
        self.gate_run = 0
        self.gate_cnt = 0

    def set_tuning(self, r_meas, tau_s, tau_bias_s):
        r_meas = max(r_meas, self.P['KF_R_MIN_DEG2'])
        self.r_meas = r_meas
        self.q_angle = self.P['CTRL_TS'] * r_meas / (tau_s * tau_s)
        self.q_bias = self.q_angle / (tau_bias_s * tau_bias_s)

    def update(self, gyro_dps, acc_deg, dt):
        self.rate = gyro_dps - self.bias
        self.angle += dt * self.rate
        self.p00 += dt * (dt * self.p11 - self.p01 - self.p10 + self.q_angle)
        self.p01 -= dt * self.p11
        self.p10 -= dt * self.p11
        self.p11 += self.q_bias * dt

        y = acc_deg - self.angle
        self.innov = y
        if abs(y) > self.P['KF_GATE_DEG']:
            if self.gate_run < self.P['KF_GATE_MAX_RUN']:
                self.gate_run += 1
                self.gate_cnt += 1
                return self.angle
        self.gate_run = 0

        S = self.p00 + self.r_meas
        K0 = self.p00 / S
        K1 = self.p10 / S
        self.k0 = K0
        self.angle += K0 * y
        self.bias += K1 * y
        p00, p01 = self.p00, self.p01
        self.p00 -= K0 * p00
        self.p01 -= K0 * p01
        self.p10 -= K1 * p00
        self.p11 -= K1 * p01
        return self.angle


class Encoder:
    """Encoder_GetRpm: cua so truot SPEED_WIN mau. dt lay dung 2 ms (khong tre ngat)."""

    def __init__(self, P):
        self.P = P
        self.hist = [0] * P['SPEED_WIN']
        self.idx = 0

    def get_rpm(self, count):
        old = self.hist[self.idx]
        self.hist[self.idx] = count
        self.idx = (self.idx + 1) % self.P['SPEED_WIN']
        dt = self.P['SPEED_WIN'] * self.P['CTRL_TS']
        return (count - old) * 60.0 / (self.P['COUNTS_PER_REV'] * dt)


class BalanceController:
    """Mpu6050_Update (phan loc) + Control_Step (che do TEST_BALANCE) + Motor_Set (phan duty)."""

    def __init__(self, P, cal_acc_samples, cal_gyro_samples):
        self.P = P
        n = len(cal_acc_samples)
        acc_mean = sum(cal_acc_samples) / n
        gyro_mean = sum(cal_gyro_samples) / n
        acc_std = math.sqrt(max(sum(a * a for a in cal_acc_samples) / n - acc_mean ** 2, 0.0))
        self.s_angle = acc_mean
        self.s_gyro_bias = gyro_mean
        self.kf = Kalman(P, acc_mean, gyro_mean, acc_std * acc_std * P['KF_R_MARGIN'])
        self.cal_acc_std = acc_std

        self.pid_bal = PID(P['BAL_KP'], P['BAL_KI'], P['BAL_KD'], 0.0,
                           P['BAL_D_ALPHA'], P['BAL_KAW'], P['BAL_SPEED_LIMIT'])
        self.pid1 = PID(P['MOT_KP'], P['MOT_KI'], P['MOT_KD'], P['MOT_KFF'],
                        P['MOT_D_ALPHA'], P['MOT_KAW'], P['DUTY_MAX_PCT'])
        self.pid2 = PID(P['MOT_KP'], P['MOT_KI'], P['MOT_KD'], P['MOT_KFF'],
                        P['MOT_D_ALPHA'], P['MOT_KAW'], P['DUTY_MAX_PCT'])
        self.enc1, self.enc2 = Encoder(P), Encoder(P)
        self.rpm1 = self.rpm2 = 0.0
        self.armed = True
        self.fault = 0
        self.angle_deg = self.gyro_dps = 0.0
        self.speed_ref_radps = self.rpm_ref = 0.0
        self.duty1 = self.duty2 = 0.0

    def arm(self):
        """g_armed 0 -> 1: Control_Step reset ca 3 PID"""
        for p in (self.pid_bal, self.pid1, self.pid2):
            p.reset()
        self.armed = True
        self.fault = 0

    # ---- Mpu6050_Update (phan sau khi doc duoc acc_deg, gyro_dps tho) ----
    def mpu_update(self, acc_deg, gyro_dps, dt):
        P = self.P
        if P['USE_KALMAN']:
            self.s_angle = self.kf.update(gyro_dps, acc_deg, dt)
            rate = self.kf.rate
        else:
            g = gyro_dps - self.s_gyro_bias
            self.s_angle = P['GYRO_ACC_ALPHA'] * (self.s_angle + g * dt) + (1 - P['GYRO_ACC_ALPHA']) * acc_deg
            rate = g
        self.gyro_dps = rate
        self.angle_deg = self.s_angle + P['ANGLE_OFFSET_DEG']

    # ---- Motor_Set: chi phan tinh duty (co bu vung chet) ----
    def motor_duty(self, duty, deadzone):
        P = self.P
        duty = clampf(duty, -P['DUTY_MAX_PCT'], P['DUTY_MAX_PCT'])
        mag = abs(duty)
        if mag < 0.5:
            return 0.0, 0.0
        mag = deadzone + mag * (P['DUTY_MAX_PCT'] - deadzone) / P['DUTY_MAX_PCT']
        return duty, math.copysign(mag, duty)   # (g_duty ghi log, duty that ra chan PWM)

    # ---- Control_Step ----
    def step(self, enc_count1, enc_count2):
        P = self.P
        a = P['SPEED_LPF_ALPHA']
        self.rpm1 += a * (self.enc1.get_rpm(enc_count1) - self.rpm1)
        self.rpm2 += a * (self.enc2.get_rpm(enc_count2) - self.rpm2)

        if self.armed and abs(self.angle_deg - P['BAL_SETPOINT_DEG']) > P['FALL_ANGLE_DEG']:
            self.fault = 2
            self.armed = False
            for p in (self.pid_bal, self.pid1, self.pid2):
                p.reset()
        if not self.armed:
            self.speed_ref_radps = self.rpm_ref = 0.0
            self.duty1 = self.duty2 = 0.0
            return 0.0, 0.0

        e_rad = (self.angle_deg - P['BAL_SETPOINT_DEG']) / P['RAD_TO_DEG']
        self.speed_ref_radps = self.pid_bal.update_rate(e_rad, 0.0, P['CTRL_TS'],
                                                        self.gyro_dps / P['RAD_TO_DEG'])
        rpm_ref = clampf(self.speed_ref_radps * P['RADPS_TO_RPM'], -P['RPM_REF_MAX'], P['RPM_REF_MAX'])
        d1 = self.pid1.update(rpm_ref - self.rpm1, self.pid1.kff * rpm_ref, P['CTRL_TS'])
        d2 = self.pid2.update(rpm_ref - self.rpm2, self.pid2.kff * rpm_ref, P['CTRL_TS'])
        self.rpm_ref = rpm_ref
        self.duty1, out1 = self.motor_duty(d1, P['MOTOR1_DEADZONE_PCT'])
        self.duty2, out2 = self.motor_duty(d2, P['MOTOR2_DEADZONE_PCT'])
        return out1, out2
