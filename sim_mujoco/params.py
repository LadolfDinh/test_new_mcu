# -*- coding: utf-8 -*-
"""
Tham so mo phong xe 2 banh tu can bang.

- Phan "DIEU KHIEN" chep NGUYEN tu testNewBrain/Core/Src/main.c (ban 25/11/2026).
  Sua code STM32 thi sua ca o day cho khop.
- Phan "CO KHI" la UOC LUONG vi chua do duoc khoi luong / trong tam.
  Dung sweep.py de xem PID con giu duoc xe trong khoang nao.
"""
import math

# ======================= CO KHI (UOC LUONG - sua khi do duoc) =======================
BODY_MASS      = 1.20    # kg  : toan bo xe TRU 2 banh (khung, pin, mach, 2 dong co)
COM_HEIGHT     = 0.05    # m   : trong tam THAN XE tinh tu TRUC BANH (khong phai tu mat dat)
COM_X_OFFSET   = 0.0     # m   : trong tam lech ve truoc (+) so voi truc banh khi xe dung thang
BODY_SIZE      = (0.08, 0.16, 0.22)   # m (truoc-sau, ngang, cao): chi de tinh quan tinh + ve
WHEEL_RADIUS   = 0.0325  # m   : banh 65 mm
WHEEL_WIDTH    = 0.026   # m
WHEEL_MASS     = 0.035   # kg / banh
WHEEL_TRACK    = 0.18    # m   : khoang cach 2 banh
IMU_HEIGHT     = 0.08    # m   : MPU6050 cao hon truc banh bao nhieu
GROUND_FRICTION = 0.98

# ======================= DONG CO JGA25-370 1:45 + L298N (UOC LUONG) =======================
# Tu thong so nha san xuat: 12 V, 130 rpm khong tai, dong ket ~1 A.
# Kiem tra: qua L298N (sut ~2 V) khong tai ~105 rpm -> khop chu thich trong main.c (~11 rad/s)
V_BATT        = 12.0     # V   nguon cap L298N
L298_DROP     = 2.0      # V   sut ap 2 transistor L298N khi dan
DIODE_DROP    = 1.4      # V   2 diode xa khi tat (fast decay)
MOTOR_R       = 12.0     # ohm
MOTOR_L       = 0.3e-3   # H   chon de vung chet ~25 % o 20 kHz (giong ghi chu VQ-Vinh)
MOTOR_KE      = 0.01845  # V.s/rad (= Kt, N.m/A) phia truc dong co
GEAR_RATIO    = 45.0
GEAR_EFF      = 0.80     # hieu suat hop so
GEAR_FRICTION = 0.04     # N.m ma sat Coulomb quy ve truc banh (dong khong tai ~70 mA)
ROTOR_INERTIA_OUT = 1.0e-3  # kg.m^2 quan tinh rotor quy ve truc banh (J_rotor * 45^2)
PWM_FREQ      = 10000.0  # Hz  TIM1/TIM8: PSC 1, ARR 8399 -> 10 kHz

# ======================= MPU6050 =======================
ACCEL_LSB_PER_G   = 8192.0   # +-4 g
GYRO_LSB_PER_DPS  = 65.5     # +-500 do/s
MPU_DLPF_HZ       = 44.0     # CONFIG = 0x03
MPU_RATE_HZ       = 1000.0   # SMPLRT_DIV = 0
ACC_NOISE_G       = 0.004    # nhieu trang gia toc ke luc dong co dung (g, rms tai 1 kHz)
GYRO_NOISE_DPS    = 0.10     # nhieu gyro (do/s rms tai 1 kHz)
VIB_ACC_NOISE_G   = 0.03     # rung them khi dong co chay (g rms) - them vao khi armed
VIB_GYRO_NOISE_DPS = 0.5
GYRO_BIAS_DPS     = 1.5      # offset gyro that (Kalman phai tu uoc luong)
MOUNT_ERR_DEG     = 0.0      # cam bien gan lech so voi than xe (do)

# ======================= DIEU KHIEN (CHEP TU main.c) =======================
CTRL_TS          = 0.002
SPEED_WIN        = 5
SPEED_LPF_ALPHA  = 0.5
ENC_PPR, ENC_DECODE = 11.0, 2.0
COUNTS_PER_REV   = ENC_PPR * ENC_DECODE * GEAR_RATIO      # 990
RADPS_TO_RPM     = 9.54929659
RAD_TO_DEG       = 57.2957795

DUTY_MAX_PCT        = 99.0
MOTOR1_DEADZONE_PCT = 0.0
MOTOR2_DEADZONE_PCT = 0.0

USE_KALMAN     = True
GYRO_ACC_ALPHA = 0.98
KF_TAU_S       = 0.40
KF_BIAS_TAU_S  = 20.0
KF_R_MARGIN    = 4.0
KF_R_MIN_DEG2  = 0.01
KF_GATE_DEG    = 15.0
KF_GATE_MAX_RUN = 25
GYRO_CAL_SAMPLES = 500

FALL_ANGLE_DEG   = 30.0
AUTO_ARM_HOLD_S  = 0.0      # code that: t_ok bat dau = 0 nen xe chay ngay sau hieu chuan neu dang gan thang
ANGLE_OFFSET_DEG = 0.0
BAL_SETPOINT_DEG = 0.0

BAL_KP, BAL_KI, BAL_KD = 15.0, 290.0, 0.05
BAL_D_ALPHA, BAL_KAW   = 1.0, 5.0
BAL_SPEED_LIMIT        = 12.0     # rad/s

MOT_KP, MOT_KI, MOT_KD = 1.0, 18.0, 0.0
MOT_KFF, MOT_D_ALPHA, MOT_KAW = 0.99, 0.3, 10.0
RPM_REF_MAX = 130.0

# ======================= MO PHONG =======================
SIM_DT = 0.0005          # buoc vat ly 0.5 ms (4 buoc / chu ky dieu khien)
ISR_DELAY_STEPS = 1      # duty moi ra L298N tre ~0.5 ms sau dau chu ky (doc I2C ~0.45 ms)


def as_dict():
    return {k: v for k, v in globals().items() if k.isupper()}
