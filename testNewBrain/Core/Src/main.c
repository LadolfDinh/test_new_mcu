/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Xe tu can bang: PID goc (MPU6050) -> toc do dat -> 2 PID toc do
  *                    dong co GA25-370 (encoder EXTI x2), tat ca trong ngat TIM6 1 ms
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Bo dieu khien PID roi rac - cung cau truc voi code xe cu (P + I + D loc + anti-windup) */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float kff;       /* feedforward (chi dung cho PID dong co) */
    float d_alpha;   /* loc thong thap khau D (0..1)          */
    float kaw;       /* he so anti-windup (back-calculation)  */
    float out_min;   /* gioi han dau ra                       */
    float out_max;
    float e_prev;    /* e[k-1]  */
    float uI;        /* uI[k-1] */
    float uD;        /* uD[k-1] */
    float out;       /* u[k] sau bao hoa */
} PID_t;

/* Bo loc Kalman 2 trang thai cho goc nghieng: x = [goc (do), bias gyro (do/s)].
   Mo hinh:  goc[k] = goc[k-1] + (gyro - bias)*dt ;  bias[k] = bias[k-1]
   Do:       goc tu gia toc ke (atan2)
   Khac loc bu o cho: he so tron tu tinh lai moi chu ky theo P, va bias gyro
   duoc UOC LUONG lien tuc -> dung duoc hang so thoi gian dai ma khong bi troi. */
typedef struct
{
    float q_angle;      /* do^2/s   : nhieu qua trinh cua goc (tich phan gyro)  */
    float q_bias;       /* do^2/s^3 : bias gyro troi nhanh co nao               */
    float r_meas;       /* do^2     : phuong sai goc gia toc ke (do luc hieu chuan) */
    float angle;        /* do       : goc da loc (khung cam bien)               */
    float bias;         /* do/s     : bias gyro uoc luong duoc                  */
    float rate;         /* do/s     : gyro da tru bias -> dung cho khau D       */
    float p00, p01, p10, p11;
    float innov;        /* do       : sai lech gia toc ke - du doan             */
    float k0;           /* he so loc hien tai cho goc (0..1)                    */
} Kalman_t;

#define SPEED_WIN               10U      /* cua so truot do toc do: 10 x 1 ms = 10 ms (nhu ban 2 ms) */

/* Encoder 2 kenh A/B, giai ma x2: ngat ngoai ca 2 suon cua kenh A, kenh B chi doc muc de biet chieu */
typedef struct
{
    GPIO_TypeDef     *portA;
    uint16_t          pinA;
    GPIO_TypeDef     *portB;
    uint16_t          pinB;
    int8_t            dir;              /* +1 / -1 : chieu dem */
    volatile uint8_t  a_prev;           /* muc kenh A lan truoc */
    volatile int32_t  count;            /* tong so xung co dau */
    volatile uint32_t err;              /* so lan ngat A ma muc A khong doi (xung nhieu) */
    int32_t           hist[SPEED_WIN];  /* count cua SPEED_WIN lan lay mau gan nhat */
    uint32_t          hist_t[SPEED_WIN];/* thoi diem lay mau tuong ung (chu ky CPU, DWT) */
    uint8_t           hist_idx;
} Encoder_t;

/* Khung telemetry STM32 -> ESP32 -> PC (nhi phan, little-endian, 31 byte).
   Giong het pc_monitor/protocol.py. Muon them truong: them vao cuoi payload,
   tang TELEM_TYPE neu doi y nghia, sua format ben protocol.py (ESP32 khong can sua). */
typedef struct __attribute__((packed))
{
    uint8_t  sof1;          /* 0xAA */
    uint8_t  sof2;          /* 0x55 */
    uint8_t  type;          /* TELEM_TYPE_STATE */
    uint8_t  len;           /* so byte payload (tu tick den reserved) */
    uint32_t tick;          /* thoi gian tu luc khoi dong, DON VI 2 ms (= so ngat 1 ms / 2) -> ESP32/do thi khong phai sua */
    float    angle_deg;     /* goc nghieng da loc, dung trong PID goc */
    float    rpm_ref;       /* toc do dat chung cho 2 dong co */
    float    rpm1;          /* toc do thuc dong co 1 (encoder) */
    float    rpm2;          /* toc do thuc dong co 2 (encoder) */
    int16_t  duty1_x100;    /* duty dong co 1 (% x 100) */
    int16_t  duty2_x100;
    uint8_t  status;        /* bit0 armed, bit1-2 fault, bit3 auto_arm, bit4-5 mode */
    uint8_t  reserved;      /* so lenh tu PC da nhan (dem vong 0..255) */
    uint8_t  crc;           /* CRC-8 (poly 0x07) tu 'type' den 'reserved' */
} TelemFrame_t;

/* Kiem tra luc bien dich: khung phai dung 31 byte (khop pc_monitor/protocol.py "<IffffhhBB").
   Neu them truong ma quen sua ben PC thi loi o day de nhac. */
typedef char TelemFrame_size_check[(sizeof(TelemFrame_t) == 31U) ? 1 : -1];

/* Khung tham so STM32 -> PC (type 0x02, 35 byte): PID dang dung. Gui NGAY SAU khung trang thai
   (chung 1 lan DMA, 66 byte = 0.72 ms) khi khoi dong, khi PC hoi (0x12) va moi lan PC doi PID
   (0x11) -> PC dung lam xac nhan. Khop pc_monitor/protocol.py FRAME_TYPES[0x02] "<7fBB".        */
typedef struct __attribute__((packed))
{
    uint8_t  sof1;          /* 0xAA */
    uint8_t  sof2;          /* 0x55 */
    uint8_t  type;          /* TELEM_TYPE_PARAMS */
    uint8_t  len;           /* 30 */
    float    bal_kp;        /* PID goc dang dung */
    float    bal_ki;
    float    bal_kd;
    float    bal_sp_deg;    /* goc dat (do) */
    float    mot_kp;        /* PID dong co 1 (de xem) */
    float    mot_ki;
    float    mot_kff;
    uint8_t  cmd_cnt;       /* = g_cmd_ok (dem vong 0..255) */
    uint8_t  reserved;
    uint8_t  crc;           /* CRC-8 tu 'type' den 'reserved' */
} ParamsFrame_t;
typedef char ParamsFrame_size_check[(sizeof(ParamsFrame_t) == 35U) ? 1 : -1];

typedef struct __attribute__((packed))
{
    TelemFrame_t  st;
    ParamsFrame_t pa;
} TxBuf_t;                  /* bo dem DMA: khung trang thai (+ khung tham so khi can) */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ================= DONG CO GA25-370 (12V, 1:45, 130 rpm) + ENCODER ================= */
#define ENC_PPR                 11.0f    /* xung / vong truc dong co / 1 kenh */
#define ENC_DECODE              2.0f     /* giai ma x2: ca 2 suon cua kenh A */
#define GEAR_RATIO              45.0f    /* hop so 1:45 */
#define COUNTS_PER_REV          (ENC_PPR * ENC_DECODE * GEAR_RATIO)   /* = 990 xung / vong banh */

/* Chieu quay (da kiem tra o ban test truoc):
   - banh quay NGUOC chieu tien         -> dao MOTORx_INVERT
   - banh quay dung nhung rpm bi AM     -> dao ENCx_INVERT                     */
#define MOTOR1_INVERT           0
#define MOTOR2_INVERT           1
#define ENC1_INVERT             0
#define ENC2_INVERT             1
#define ENC1_DIR                ((ENC1_INVERT != 0) ? -1 : 1)
#define ENC2_DIR                ((ENC2_INVERT != 0) ? -1 : 1)

/* ================= PWM L298N: 2 dong co chung TIM1, 2 kenh =================
   Dong co 1: TIM1_CH1 (PE9), dong co 2: TIM1_CH2 (PE11). 168MHz / 2 / 8400 = 10 kHz.
   2 kenh cung 1 timer van doc lap (moi kenh 1 thanh ghi CCR rieng), chung tan so.
   Doi tan so: sua Prescaler cua TIM1 trong CubeMX (9 -> 2 kHz, 19 -> 1 kHz),
   code tu tinh duty theo ARR nen khong can sua gi them.                                  */
#define DUTY_MAX_PCT            99.9f    /* gioi han duty (%) */
#define MOTOR1_DEADZONE_PCT     55.0f     /* bu vung chet rieng tung dong co: duty nho nhat lam banh */
#define MOTOR2_DEADZONE_PCT     55.0f     /* bat dau quay (do o TEST_OPEN_LOOP, tang dan g_test_duty) */

/* ================= CHU KY: tat ca chay trong ngat TIM6 1 ms ================= */
#define CTRL_TS                 0.001f   /* PHAI khop TIM6: 84MHz / 84 / 1000 = 1 ms */
#define SPEED_LPF_ALPHA         0.29f    /* loc them toc do (1.0 = khong loc); 0.29 o 1 ms = 0.5 o 2 ms */
#define RADPS_TO_RPM            9.54929659f   /* 60 / (2*pi) */

/* ================= MPU6050 ================= */
#define MPU6050_ADDR            (0x68 << 1)   /* AD0 = GND */
#define MPU6050_REG_SMPLRT_DIV  0x19
#define MPU6050_REG_CONFIG      0x1A
#define MPU6050_REG_GYRO_CFG    0x1B
#define MPU6050_REG_ACCEL_CFG   0x1C
#define MPU6050_REG_ACCEL_XOUT  0x3B
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_WHO_AM_I    0x75

#define ACCEL_LSB_PER_G         8192.0f  /* +-4 g */
#define GYRO_LSB_PER_DPS        65.5f    /* +-500 do/s */
#define RAD_TO_DEG              57.2957795f
#define GYRO_ACC_ALPHA          0.99f    /* he so loc bu (chi dung khi USE_KALMAN = 0); 0.99 o 1 ms = 0.98 o 2 ms (tau ~0.1 s) */
#define GYRO_CAL_SAMPLES        500U     /* hieu chuan offset gyro luc khoi dong (~1 s, xe dung yen) */

/* ================= BO LOC GOC: KALMAN ================= */
/* USE_KALMAN 1 = Kalman 2 trang thai ; 0 = quay ve loc bu cu (de so sanh A/B).
   Khong can tinh truoc tham so nao: r_meas do ngay trong luc hieu chuan,
   q_angle / q_bias suy ra tu 2 hang so thoi gian duoi day.                    */
#define USE_KALMAN              1U

/* Hang so thoi gian cua goc: bao lau thi goc gia toc ke keo duoc goc ve.
   Loc bu cu: tau = alpha*dt/(1-alpha) = 0.98*0.002/0.02 = 0.098 s -> rat ngan,
   nen nhieu gia toc ke (gom ca sai so do xe tang toc ngang) di gan het vao goc.
   Kalman uoc luong duoc bias nen dung tau dai duoc. 0.3..0.8 s la vung hop ly. */
#define KF_TAU_S                0.40f

/* Hang so thoi gian uoc luong bias gyro. Phai dai hon KF_TAU_S nhieu lan,
   neu khong bias se "an" ca chuyen dong that. 10..40 s.                        */
#define KF_BIAS_TAU_S           20.0f

/* He so an toan cho r_meas: luc hieu chuan dong co DUNG YEN nen nhieu do duoc
   nho hon thuc te khi chay. Tang len neu g_kf_innov_rms_deg >> sqrt(g_kf.r_meas). */
#define KF_R_MARGIN             4.0f
#define KF_R_MIN_DEG2           0.01f    /* san duoi: sigma >= 0.1 do */

/* Cong loai mau gia toc ke bat thuong (loi I2C, va cham): bo mau neu lech qua
   nguong. Neu lech lau hon KF_GATE_MAX_RUN chu ky thi chap nhan lai (xe nga
   hoac bi be len that -> phai bam theo, khong duoc bo mai).                    */
#define KF_GATE_DEG             15.0f
#define KF_GATE_MAX_RUN         50U      /* 50 x 1 ms = 50 ms */

/* Nguong kiem tra xe DUNG YEN trong luc hieu chuan. Vuot nguong -> hieu chuan
   lai 1 lan, van sai thi bao loi (g_cal_fail) va dung so do vua do duoc.       */
#define CAL_GYRO_STD_MAX_DPS    2.0f
#define CAL_ACC_STD_MAX_DEG     2.0f

#define MPU_TILT_AXIS           0        /* 0: nghieng quanh truc X cam bien (Ay + Gx, giong xe cu)
                                            1: nghieng quanh truc Y cam bien (-Ax + Gy)          */
#define ANGLE_SIGN              1.0f     /* doi -1.0f neu nghieng ve TRUOC ma goc AM */
#define ANGLE_OFFSET_DEG        0.0f     /* bu lech lap dat / trong tam (xe cu: +1.2) */
#define MPU_ERR_MAX             20U      /* 20 lan loi I2C lien tiep (20 ms) -> coi la mat cam bien */
#define MPU_RETRY_TICKS         500U     /* khi mat cam bien: thu doc lai moi 0.5 s (500 x 1 ms) */
#define MPU_I2C_TIMEOUT_MS      1U       /* timeout HAL (1 -> toi da ~2 ms); doc binh thuong mat ~0.45 ms */
#define MPU_DT_MAX_S            0.02f    /* gioi han dt khi tich phan gyro sau 1 lan doc loi (= MPU_ERR_MAX x 1 ms) */
#define TICK_OVERRUN_US         1500U    /* ngat TIM6 cach nhau > 1.5 ms = chuong trinh bi tre */

/* ================= CHE DO (doi g_test_mode trong Watch) ================= */
#define TEST_STOP               0U  /* dong co dung */
#define TEST_OPEN_LOOP          1U  /* 2 dong co chay duty co dinh g_test_duty (%) */
#define TEST_SPEED_PID          2U  /* 2 PID dong co cung bam g_test_rpm -> dung de chinh PID dong co */
#define TEST_BALANCE            3U  /* PID goc -> toc do dat -> 2 PID dong co (can bang) */

#define FALL_ANGLE_DEG          30.0f    /* vuot goc nay -> tat dong co (giong xe cu) */
#define ARM_ANGLE_DEG           10.0f    /* che do can bang: chi bat duoc dong co khi |goc - goc dat| < 10 do */
#define AUTO_ARM                1U       /* 1 = bat nguon la tu chay, khong can nhan K1 */
#define AUTO_ARM_HOLD_MS        500U     /* xe phai gan thang dung lien tuc 0.5 s moi tu chay (ca sau khi nga) */

/* ================= TELEMETRY -> ESP32 (USART1 PA9 TX, 921600 baud, DMA2 Stream7) ================= */
#define TELEM_DIV               2U       /* gui moi 2 ngat TIM6 = 500 khung/s nhu cu (1 -> 1000 khung/s, 31 byte mat 0.34 ms van kip) */
#define TELEM_SOF1              0xAAU
#define TELEM_SOF2              0x55U
#define TELEM_TYPE_STATE        0x01U
#define TELEM_PAYLOAD_LEN       ((uint8_t)(sizeof(TelemFrame_t) - 5U))   /* tru sof1, sof2, type, len, crc */

#define FAULT_NONE              0U
#define FAULT_MPU               1U       /* mat cam bien / loi I2C */
#define FAULT_FALL              2U       /* nghieng qua FALL_ANGLE_DEG */

/* ================= PID GOC (CAN BANG) - lay tu code xe cu ================= */
/* Vao: sai so goc (rad). Ra: toc do banh dat (rad/s). Ts = 1 ms.
   Xe cu (dong co buoc): Kp 23, Ki 630, Kd 0.3, loc D 0.3, Kaw 5, gioi han 40 rad/s. */
#define BAL_KP                  36.0f
#define BAL_KI                  336.0f
#define BAL_KD                  0.0f 
/* Khau D cua PID goc lay truc tiep tu gyro (xem PID_UpdateRate) chu khong vi phan
   goc da loc nua -> tin hieu da sach, khong can loc them -> 1.0f = khong loc.
   Nho vi phan goc o 2 ms rat nhieu nen truoc day phai ep BAL_KD xuong 0.05;
   gio co the tang BAL_KD dan len (0.1 ... 0.3) de tang giam chan.              */
#define BAL_D_ALPHA             1.0f
#define BAL_KAW                 5.0f
#define BAL_SPEED_LIMIT         (RPM_REF_MAX / RADPS_TO_RPM)   /* rad/s (= RPM_REF_MAX, ~10.5 rad/s): PID goc khong doi vuot gioi han dong co */
#define BAL_SETPOINT_DEG        0.0f     /* goc dat mac dinh khi khoi dong: 0 = xe dung thang (PC doi bang thanh keo, lenh 0x13) */

/* ================= PID TOC DO DONG CO (vao rpm, ra %duty) - gia tri khoi dau ================= */
/* Tinh tu mo hinh do thuc nghiem JGA25-370 1:45 (K ~ 1.1 rpm/%duty, tau co ~30..150 ms),
   da mo phong: len 90% trong 35..115 ms, vot lo <= 15 %.                                  */
#define MOT_KP                  1.5f     /* %duty / rpm        */
#define MOT_KI                  20.0f    /* %duty / (rpm.s)    */
#define MOT_KD                  0.0f
#define MOT_KFF                 0.99f     /* %duty / rpm: ~ 100% / toc do toi da do duoc */
#define MOT_D_ALPHA             0.3f
#define MOT_KAW                 10.0f
#define RPM_REF_MAX             100.0f   /* gioi han toc do dat cho 2 PID dong co. Do 26/11 (banh nhac, duty 99%):
                                            dc1 +120 / -115, dc2 +115 / -118 rpm -> 100 de con ~13% du cho PID (tai, pin yeu) */

/* ================= LENH PC -> ESP32 -> STM32 (USART1 RX PA10, DMA2 Stream2 kenh 4, vong tron) ================= */
/* Khung giong telemetry: AA 55 | type | len | payload | CRC-8 (poly 0x07, tu type den het payload).
   DMA nhan cau hinh bang code (KHONG sua CubeMX); ngat TIM6 doc bo dem moi 1 ms (Cmd_Poll).
   PC: pc_monitor/protocol.py build_set_rpm(). ESP32 chuyen nguyen goi AA 55 xuong nen khong can nap lai. */
#define PC_RPM_TEST             0U       /* TAM THOI: 1 = che do 2 (2 PID dong co), toc do dat lay tu can gat tren PC
                                            0 = nhu cu (che do can bang; lenh van duoc dem nhung khong dung)        */
#define CMD_TYPE_SET_RPM        0x10U    /* payload 4 byte: float32 rpm (little-endian) */
#define CMD_TYPE_SET_BAL        0x11U    /* payload 16 byte: float32 kp, ki, kd, goc dat (do) -> PID goc */
#define CMD_TYPE_GET_PARAMS     0x12U    /* payload 0: PC hoi -> STM32 gui khung tham so 0x02 */
#define CMD_TYPE_SET_SP         0x13U    /* payload 4 byte: float32 goc nghieng dat (do), 0 = dung thang; kep +-BAL_SP_MAX_DEG */
#define TELEM_TYPE_PARAMS       0x02U    /* STM32 -> PC: PID dang dung (ParamsFrame_t) */
#define BAL_KP_MAX              500.0f   /* gioi han an toan khi nhan PID goc tu PC */
#define BAL_KI_MAX              10000.0f
#define BAL_KD_MAX              50.0f
#define BAL_SP_MAX_DEG          15.0f
#define CMD_TIMEOUT_MS          500U     /* PC_RPM_TEST: 0.5 s khong co lenh (mat WiFi / tat app) -> toc do dat = 0 */
#define CMD_RX_BUF              256U     /* bo dem vong DMA (921600 baud -> toi da ~92 byte moi 1 ms) */
#define CMD_MAX_PAYLOAD         16U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* ---- Xem / chinh truc tiep trong cua so Watch cua Keil khi Debug ---- */
volatile uint8_t  g_test_mode        = (PC_RPM_TEST != 0U) ? TEST_SPEED_PID : TEST_BALANCE;  /* che do (TEST_xxx) */
volatile float    g_test_duty        = 30.0f;             /* %   - TEST_OPEN_LOOP */
volatile float    g_test_rpm         = (PC_RPM_TEST != 0U) ? 0.0f : 40.0f;  /* rpm - TEST_SPEED_PID (PC_RPM_TEST: tu can gat PC) */
volatile float    g_bal_setpoint_deg = BAL_SETPOINT_DEG;  /* goc dat - chinh de tim diem can bang */
volatile uint8_t  g_armed            = 0U;                /* 1 = dong co dang chay */
volatile uint8_t  g_auto_arm         = AUTO_ARM;          /* 1 = tu chay khi du dieu kien; nhan K1 de dung han */
volatile uint8_t  g_fault            = FAULT_NONE;

volatile float g_angle_deg       = 0.0f;  /* goc nghieng sau loc bu (do), duong = nghieng ve truoc */
volatile float g_angle_acc_deg   = 0.0f;  /* goc chi tinh tu gia toc ke (de so sanh) */
volatile float g_gyro_dps        = 0.0f;  /* van toc goc da tru bias (do/s) */

/* ---- Bo loc Kalman: xem / chinh trong Watch ---- */
Kalman_t          g_kf;                      /* .q_angle .q_bias .r_meas sua duoc khi dang chay */
volatile uint8_t  g_use_kalman   = USE_KALMAN;
volatile float    g_kf_bias_dps  = 0.0f;     /* bias gyro Kalman tim ra (do/s) - phai hoi tu roi nam yen */
volatile float    g_kf_innov_deg = 0.0f;     /* sai lech gia toc ke - du doan (do) */
volatile float    g_kf_innov_rms = 0.0f;     /* RMS truot cua innov (do): so voi sqrt(r_meas) de chinh KF_R_MARGIN */
volatile float    g_kf_k0        = 0.0f;     /* he so loc hien tai (0..1) */
volatile uint32_t g_kf_gate_cnt  = 0U;       /* so mau gia toc ke bi loai - tang lien tuc = nhieu / loi I2C */
volatile float    g_cal_acc_std_deg  = 0.0f; /* do lech chuan goc gia toc ke luc hieu chuan */
volatile float    g_cal_gyro_std_dps = 0.0f; /* do lech chuan gyro luc hieu chuan */
volatile uint8_t  g_cal_fail         = 0U;   /* 1 = hieu chuan luc xe KHONG dung yen -> bias co the sai */

volatile float g_speed_ref_radps = 0.0f;  /* dau ra PID goc: toc do banh dat (rad/s) */
volatile float g_rpm_ref         = 0.0f;  /* toc do dat CHUNG cho ca 2 PID dong co (rpm) */
volatile float g_rpm1            = 0.0f;  /* toc do banh 1 do duoc (rpm) */
volatile float g_rpm2            = 0.0f;  /* toc do banh 2 do duoc (rpm) */
volatile float g_duty1           = 0.0f;  /* duty xuat ra L298N (%) */
volatile float g_duty2           = 0.0f;

volatile uint8_t  g_mpu_whoami    = 0U;   /* MPU6050 that = 0x68 */
volatile uint32_t g_mpu_err_total = 0U;   /* tong so lan doc I2C loi */
volatile uint32_t g_isr_us        = 0U;   /* thoi gian chay ngat TIM6 (us) */
volatile uint32_t g_isr_us_max    = 0U;
volatile uint32_t g_tick_gap_us_max = 0U; /* khoang cach lon nhat giua 2 lan ngat TIM6 (binh thuong ~1000 us) */
volatile uint32_t g_overrun_cnt     = 0U; /* so lan ngat TIM6 bi tre > 1.5 ms (chuong trinh "dung") */
volatile uint32_t g_i2c_recover_cnt = 0U; /* so lan phai giai phong bus I2C bi treo */
volatile uint32_t g_telem_sent      = 0U; /* so khung da gui cho ESP32 */
volatile uint32_t g_telem_drop      = 0U; /* so khung bo qua vi UART con ban (binh thuong = 0) */

PID_t g_pid_bal;   /* PID goc -> toc do (sua g_pid_bal.kp / .ki / .kd trong Watch) */
PID_t g_pid1;      /* PID toc do dong co 1 */
PID_t g_pid2;      /* PID toc do dong co 2 */

Encoder_t g_enc1 = { ENC1_A_GPIO_Port, ENC1_A_Pin, ENC1_B_GPIO_Port, ENC1_B_Pin, ENC1_DIR, 0U, 0, 0U, {0}, {0}, 0U };
Encoder_t g_enc2 = { ENC2_A_GPIO_Port, ENC2_A_Pin, ENC2_B_GPIO_Port, ENC2_B_Pin, ENC2_DIR, 0U, 0, 0U, {0}, {0}, 0U };

static float    s_angle       = 0.0f;   /* trang thai bo loc bu (khung cam bien) */
static float    s_gyro_bias   = 0.0f;   /* offset gyro (do/s) */
static uint8_t  s_mpu_err_cnt = 0U;     /* so lan loi I2C lien tiep */
static uint16_t s_mpu_retry   = 0U;
static uint8_t  s_armed_prev  = 0U;
static uint8_t  s_mode_prev   = 0xFFU;
static uint32_t s_mpu_t_last  = 0U;     /* thoi diem tich phan gyro lan cuoi (chu ky CPU) */
static uint16_t s_kf_gate_run = 0U;     /* so chu ky lien tiep mau gia toc ke bi loai */
static float    s_kf_innov_ms = 0.0f;   /* trung binh truot cua innov^2 */
static uint32_t s_tick_last   = 0U;     /* thoi diem vao ngat TIM6 lan truoc */
static uint32_t s_tick_count  = 0U;     /* so lan ngat TIM6 (chu ky 1 ms) */
static uint8_t  s_telem_div   = 0U;
static TxBuf_t   s_tx;                  /* bo dem DMA: phai ton tai suot luc dang gui */
static uint8_t   s_params_pending = 1U; /* 1 = gui kem khung tham so o lan gui toi (khoi dong: 1) */
/* ---- lenh tu PC (xem trong Watch) ---- */
volatile uint32_t g_cmd_ok         = 0U;    /* so lenh hop le da nhan (gui ve PC o byte 'reserved' cua telemetry) */
volatile uint32_t g_cmd_bad        = 0U;    /* khung sai CRC / sai do dai / type la */
volatile uint32_t g_cmd_rx_restart = 0U;    /* so lan phai bat lai DMA nhan (binh thuong = 0) */
volatile float    g_cmd_rpm        = 0.0f;  /* toc do dat nhan duoc gan nhat (rpm) */
static DMA_HandleTypeDef s_hdma_usart1_rx;
static volatile uint8_t  s_cmd_rx[CMD_RX_BUF];          /* DMA ghi vong tron vao day */
static uint8_t  s_cmd_dma_ok   = 0U;
static uint16_t s_cmd_rd       = 0U;                    /* vi tri da doc trong s_cmd_rx */
static uint8_t  s_cmd_state    = 0U;
static uint8_t  s_cmd_n        = 0U;
static uint8_t  s_cmd_frame[2U + CMD_MAX_PAYLOAD];      /* type, len, payload */
static uint32_t s_cmd_last_ms  = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ====================================================================
   SO DO NOI DAY - DevEBox STM32F4XX_M (STM32F407VET6)
   --------------------------------------------------------------------
   L298N   ENA <- PE9  (TIM1_CH1, PWM 10 kHz)  -> THAO jumper ENA
           IN1 <- PE7
           IN2 <- PE8
           ENB <- PE11 (TIM1_CH2, PWM 10 kHz)  -> THAO jumper ENB  (truoc 27/11: PC6 / TIM8_CH1)
           IN3 <- PE10
           IN4 <- PE12
           GND <-> GND STM32 (bat buoc chung mass)
   Encoder dong co 1: A -> PE2 (EXTI2, ca 2 suon), B -> PE3 (input, chi doc chieu)
   Encoder dong co 2: A -> PE4 (EXTI4, ca 2 suon), B -> PE5 (input, chi doc chieu)
           VCC encoder = 3.3V (chan dang bat pull-up noi)
   MPU6050 SCL -> PB6, SDA -> PB7 (I2C1 400 kHz), VCC 3.3V, AD0 = GND
   Bat nguon -> hieu chuan MPU (~1.3 s, de xe dung yen) -> tu chay khi xe gan thang dung.
   Nut K1 (PA0): dang chay -> dung han; dang dung -> cho phep chay lai.
   LED D2 (PA1): nhay cham = cho, sang = dang chay, nhay nhanh = loi
   ESP32  : PA9 (USART1_TX) -> GPIO16 (RX2), PA10 (USART1_RX) <- GPIO17 (TX2), GND chung
            921600 baud, 1 khung 31 byte moi 2 ms qua DMA (khong chan vong dieu khien)
   --------------------------------------------------------------------
   SO DO DIEU KHIEN (moi 1 ms trong ngat TIM6):
     goc dat --(+)--> [PID goc] --> toc do dat (rad/s -> rpm) --+--> [PID dong co 1] --> L298N --> banh 1
              goc do -|                                         |       ^ rpm1 (encoder 1)
                                                                +--> [PID dong co 2] --> L298N --> banh 2
                                                                        ^ rpm2 (encoder 2)
   Uu tien ngat: SysTick 0 > EXTI encoder 1 > TIM6 2
   ==================================================================== */

static float Clampf(float x, float lo, float hi)
{
    if (x > hi) { return hi; }
    if (x < lo) { return lo; }
    return x;
}

/* ============================ ENCODER (giai ma x2) ============================ */
static uint8_t Encoder_ReadA(const Encoder_t *e)
{
    return ((e->portA->IDR & e->pinA) != 0U) ? 1U : 0U;
}

static uint8_t Encoder_ReadB(const Encoder_t *e)
{
    return ((e->portB->IDR & e->pinB) != 0U) ? 1U : 0U;
}

static void Encoder_Reset(Encoder_t *e)
{
    uint8_t i;

    __disable_irq();
    e->a_prev = Encoder_ReadA(e);
    e->count  = 0;
    e->err    = 0U;
    for (i = 0U; i < SPEED_WIN; i++) { e->hist[i] = 0; e->hist_t[i] = DWT->CYCCNT; }
    e->hist_idx = 0U;
    __enable_irq();
}

/* Goi trong ngat EXTI moi khi kenh A doi muc (ca suon len va suon xuong).
   Chieu thuan (A di truoc B): AB = 00 -> 10 -> 11 -> 01 -> 00
     - suon len A  (00 -> 10): A = 1, B = 0  -> A != B
     - suon xuong A (11 -> 01): A = 0, B = 1  -> A != B
   => sau suon cua A: A != B la quay thuan (+1), A == B la quay nguoc (-1).
   B luon dung yen tai suon cua A (lech pha 90 do) nen doc B luc nay la chac chan. */
static void Encoder_ISR(Encoder_t *e)
{
    uint8_t a = Encoder_ReadA(e);
    uint8_t b = Encoder_ReadB(e);

    if (a == e->a_prev)
    {
        e->err++;          /* xung nhieu ngan hon thoi gian vao ngat: bo qua */
        return;
    }
    e->a_prev = a;
    e->count += (a != b) ? (int32_t)e->dir : -(int32_t)e->dir;
}

/* Goi moi 1 ms. Toc do = so xung trong cua so truot SPEED_WIN lan lay mau (~10 ms).
   Chia cho THOI GIAN THUC giua 2 lan lay mau (do bang DWT), khong gia dinh dung 1 ms:
   neu ngat TIM6 bi tre, toc do khong bi vot len gia (truoc day tre 25 ms -> rpm do x3).
   Do phan giai ~6 rpm (x2).                                                           */
static float Encoder_GetRpm(Encoder_t *e)
{
    uint32_t now   = DWT->CYCCNT;
    int32_t  c     = e->count;                 /* doc 32 bit la nguyen tu tren Cortex-M4 */
    int32_t  old_c = e->hist[e->hist_idx];     /* mau cach day SPEED_WIN lan */
    uint32_t dt    = now - e->hist_t[e->hist_idx];   /* dung ca khi CYCCNT tran so */

    e->hist[e->hist_idx]   = c;
    e->hist_t[e->hist_idx] = now;
    e->hist_idx = (uint8_t)((e->hist_idx + 1U) % SPEED_WIN);

    if (dt == 0U) { return 0.0f; }
    return (float)(c - old_c) * 60.0f * (float)SystemCoreClock / (COUNTS_PER_REV * (float)dt);
}

/* ============================ DONG CO (L298N) ============================ */
/* id = 1 / 2; duty = -100..100 (%), duong = banh quay "tien".
   duty = 0 -> IN = 0 0: dong co tha troi.                                  */
static void Motor_Set(uint8_t id, float duty)
{
    uint32_t           channel  = (id == 1U) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;   /* chung TIM1, moi dong co 1 kenh */
    GPIO_PinState      in_a     = GPIO_PIN_RESET;
    GPIO_PinState      in_b     = GPIO_PIN_RESET;
    uint8_t            invert   = (id == 1U) ? (uint8_t)MOTOR1_INVERT : (uint8_t)MOTOR2_INVERT;
    float              deadzone = (id == 1U) ? MOTOR1_DEADZONE_PCT : MOTOR2_DEADZONE_PCT;
    uint32_t           period   = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
    float              mag;
    uint32_t           ccr;

    duty = Clampf(duty, -DUTY_MAX_PCT, DUTY_MAX_PCT);
    mag  = fabsf(duty);

    if (mag < 0.5f)
    {
        mag  = 0.0f;
        duty = 0.0f;
    }
    else
    {
        /* bu vung chet: anh xa 0..DUTY_MAX -> DEADZONE..DUTY_MAX */
        mag = deadzone + mag * (DUTY_MAX_PCT - deadzone) / DUTY_MAX_PCT;
        if ((duty > 0.0f) != (invert != 0U)) { in_a = GPIO_PIN_SET; }
        else                                  { in_b = GPIO_PIN_SET; }
    }

    ccr = (uint32_t)(mag * (float)period / 100.0f);
    __HAL_TIM_SET_COMPARE(&htim1, channel, ccr);

    if (id == 1U)
    {
        HAL_GPIO_WritePin(M1_IN1_GPIO_Port, M1_IN1_Pin, in_a);
        HAL_GPIO_WritePin(M1_IN2_GPIO_Port, M1_IN2_Pin, in_b);
        g_duty1 = duty;
    }
    else
    {
        HAL_GPIO_WritePin(M2_IN3_GPIO_Port, M2_IN3_Pin, in_a);
        HAL_GPIO_WritePin(M2_IN4_GPIO_Port, M2_IN4_Pin, in_b);
        g_duty2 = duty;
    }
}

static void Motor_StopAll(void)
{
    Motor_Set(1U, 0.0f);
    Motor_Set(2U, 0.0f);
}

/* ============================ PID ============================ */
static void PID_Reset(PID_t *p)
{
    p->e_prev = 0.0f;
    p->uI     = 0.0f;
    p->uD     = 0.0f;
    p->out    = 0.0f;
}

static void PID_Init(PID_t *p, float kp, float ki, float kd, float kff,
                     float d_alpha, float kaw, float out_limit)
{
    p->kp      = kp;
    p->ki      = ki;
    p->kd      = kd;
    p->kff     = kff;
    p->d_alpha = d_alpha;
    p->kaw     = kaw;
    p->out_min = -out_limit;
    p->out_max =  out_limit;
    PID_Reset(p);
}

/* Giong Balance_Update cua xe cu:
   uP = Kp*e ; uI = uI[k-1] + Ki*Ts*e ; uD = loc(Kd*(e - e[k-1])/Ts)
   u  = bao hoa(ff + uP + uI + uD) ; uI += Kaw*Ts*(u - u_raw)  (anti-windup)        */
static float PID_Update(PID_t *p, float e, float ff, float ts)
{
    float uP     = p->kp * e;
    float uI     = p->uI + p->ki * ts * e;
    float uD_raw = p->kd * (e - p->e_prev) / ts;
    float uD     = p->d_alpha * uD_raw + (1.0f - p->d_alpha) * p->uD;
    float u_raw  = ff + uP + uI + uD;
    float u      = Clampf(u_raw, p->out_min, p->out_max);

    uI += p->kaw * ts * (u - u_raw);

    p->e_prev = e;
    p->uI     = uI;
    p->uD     = uD;
    p->out    = u;
    return u;
}

/* Giong PID_Update nhung khau D lay tu dao ham DO TRUC TIEP (de_dt) thay vi
   vi phan sai so. Dung cho PID goc: de/dt = toc do goc tu gyro (rad/s).
   Loi: khong khuech dai nhieu, khong them tre vi phan.                        */
static float PID_UpdateRate(PID_t *p, float e, float ff, float ts, float de_dt)
{
    float uP     = p->kp * e;
    float uI     = p->uI + p->ki * ts * e;
    float uD_raw = p->kd * de_dt;
    float uD     = p->d_alpha * uD_raw + (1.0f - p->d_alpha) * p->uD;
    float u_raw  = ff + uP + uI + uD;
    float u      = Clampf(u_raw, p->out_min, p->out_max);

    uI += p->kaw * ts * (u - u_raw);

    p->e_prev = e;
    p->uI     = uI;
    p->uD     = uD;
    p->out    = u;
    return u;
}

/* ============================ BO LOC KALMAN ============================ */
/* Dat q_angle, q_bias tu 2 hang so thoi gian + r_meas vua do duoc.
   Trang thai xac lap cua bo loc:  tau = sqrt(dt * r / q_angle)
                                   tau_bias = sqrt(q_angle / q_bias)
   -> dao lai thanh 2 cong thuc duoi. Khong phai do thuc nghiem gi them.      */
static void Kalman_SetTuning(Kalman_t *k, float r_meas, float tau_s, float tau_bias_s)
{
    if (r_meas < KF_R_MIN_DEG2) { r_meas = KF_R_MIN_DEG2; }
    k->r_meas  = r_meas;
    k->q_angle = CTRL_TS * r_meas / (tau_s * tau_s);
    k->q_bias  = k->q_angle / (tau_bias_s * tau_bias_s);
}

static void Kalman_Init(Kalman_t *k, float angle0, float bias0, float r_meas)
{
    Kalman_SetTuning(k, r_meas, KF_TAU_S, KF_BIAS_TAU_S);
    k->angle = angle0;
    k->bias  = bias0;
    k->rate  = 0.0f;
    /* vua hieu chuan nen kha tin -> P nho, bo loc khong giat luc khoi dong */
    k->p00 = r_meas;  k->p01 = 0.0f;
    k->p10 = 0.0f;    k->p11 = 0.01f;
    k->innov = 0.0f;
    k->k0    = 0.0f;
}

/* gyro_dps: gyro THO (chua tru bias) ; acc_deg: goc tu gia toc ke ; dt: giay */
static float Kalman_Update(Kalman_t *k, float gyro_dps, float acc_deg, float dt)
{
    float y, S, K0, K1, p00, p01;

    /* --- 1. DU DOAN: tin gyro, do bat dinh P tang len --- */
    k->rate   = gyro_dps - k->bias;
    k->angle += dt * k->rate;

    k->p00 += dt * (dt * k->p11 - k->p01 - k->p10 + k->q_angle);
    k->p01 -= dt * k->p11;
    k->p10 -= dt * k->p11;
    k->p11 += k->q_bias * dt;

    /* --- 2. HIEU CHINH: keo goc ve phia gia toc ke --- */
    y        = acc_deg - k->angle;
    k->innov = y;

    /* trung binh truot innov^2 (hang so thoi gian ~1 s) de chan doan r_meas */
    s_kf_innov_ms += CTRL_TS * (y * y - s_kf_innov_ms);

    if (fabsf(y) > KF_GATE_DEG)
    {
        if (s_kf_gate_run < KF_GATE_MAX_RUN)
        {
            s_kf_gate_run++;
            g_kf_gate_cnt++;
            return k->angle;           /* mau la: bo, chu ky nay chi chay bang gyro */
        }
        /* lech lau qua -> day la that (nga / bi be len), chap nhan va bam lai */
    }
    s_kf_gate_run = 0U;

    S  = k->p00 + k->r_meas;
    K0 = k->p00 / S;
    K1 = k->p10 / S;
    k->k0 = K0;

    k->angle += K0 * y;
    k->bias  += K1 * y;

    p00 = k->p00;
    p01 = k->p01;
    k->p00 -= K0 * p00;
    k->p01 -= K0 * p01;
    k->p10 -= K1 * p00;
    k->p11 -= K1 * p01;

    return k->angle;
}

/* ============================ MPU6050 ============================ */
static HAL_StatusTypeDef Mpu6050_Write(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1U, 10U);
}

/* Doc 14 byte, tra ve goc tu gia toc ke + van toc goc cua truc nghieng
   (khung cam bien, chua tru offset, chua doi dau)                       */
static HAL_StatusTypeDef Mpu6050_ReadRaw(float *acc_deg, float *gyro_dps)
{
    uint8_t d[14];
    float   ax, ay, az;

    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT,
                         I2C_MEMADD_SIZE_8BIT, d, 14U, MPU_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    ax = (float)(int16_t)((d[0] << 8) | d[1]) / ACCEL_LSB_PER_G;
    ay = (float)(int16_t)((d[2] << 8) | d[3]) / ACCEL_LSB_PER_G;
    az = (float)(int16_t)((d[4] << 8) | d[5]) / ACCEL_LSB_PER_G;
    /* d[6], d[7] = nhiet do: khong dung */

#if (MPU_TILT_AXIS == 0)
    *acc_deg  = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
    *gyro_dps = (float)(int16_t)((d[8] << 8) | d[9]) / GYRO_LSB_PER_DPS;
#else
    *acc_deg  = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    *gyro_dps = (float)(int16_t)((d[10] << 8) | d[11]) / GYRO_LSB_PER_DPS;
#endif
    return HAL_OK;
}

static void Delay_us(uint32_t us)
{
    uint32_t t0 = DWT->CYCCNT;
    uint32_t n  = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - t0) < n) { }
}

/* Giai phong bus I2C bi treo (MPU6050 giu SDA = 0 khi bi nhieu / ngat giua chung):
   tam chuyen PB6 (SCL) / PB7 (SDA) thanh GPIO, phat 9 xung SCL + dieu kien STOP,
   roi khoi dong lai I2C1 (HAL_I2C_Init co reset mem SWRST). Mat khoang 110 us.        */
static void I2C1_BusRecover(void)
{
    GPIO_InitTypeDef g = {0};
    uint8_t i;

    g_i2c_recover_cnt++;
    (void)HAL_I2C_DeInit(&hi2c1);

    g.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_Init(GPIOB, &g);

    for (i = 0U; i < 9U; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); Delay_us(5U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   Delay_us(5U);
    }
    /* STOP: SDA len 1 trong khi SCL = 1 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); Delay_us(5U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); Delay_us(5U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   Delay_us(5U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   Delay_us(5U);

    (void)HAL_I2C_Init(&hi2c1);   /* MspInit dua PB6/PB7 ve lai AF I2C */
}

/* Khoi dong MPU6050 - tra ve 1 neu OK */
static uint8_t Mpu6050_Init(void)
{
    uint8_t who = 0U;

    HAL_Delay(100);
    if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 3U, 50U) != HAL_OK) { return 0U; }
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_REG_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &who, 1U, 10U) != HAL_OK)  { return 0U; }
    g_mpu_whoami = who;   /* 0x68 = MPU6050 that; ban clone co the la 0x70/0x72/0x98, van dung duoc */

    if (Mpu6050_Write(MPU6050_REG_PWR_MGMT_1, 0x80U) != HAL_OK) { return 0U; }  /* reset */
    HAL_Delay(100);
    if (Mpu6050_Write(MPU6050_REG_PWR_MGMT_1, 0x01U) != HAL_OK) { return 0U; }  /* thoat sleep, clock = PLL gyro X */
    if (Mpu6050_Write(MPU6050_REG_SMPLRT_DIV, 0x00U) != HAL_OK) { return 0U; }  /* lay mau 1 kHz */
    if (Mpu6050_Write(MPU6050_REG_CONFIG,     0x03U) != HAL_OK) { return 0U; }  /* DLPF ~44 Hz: loc rung dong co */
    if (Mpu6050_Write(MPU6050_REG_GYRO_CFG,   0x08U) != HAL_OK) { return 0U; }  /* +-500 do/s */
    if (Mpu6050_Write(MPU6050_REG_ACCEL_CFG,  0x08U) != HAL_OK) { return 0U; }  /* +-4 g */
    HAL_Delay(50);
    return 1U;
}

/* Hieu chuan offset gyro + goc ban dau. Xe phai DUNG YEN ~1 s sau khi cap nguon */
/* Do 1 luot: tra ve so mau tot, dong thoi tra trung binh + do lech chuan cua
   goc gia toc ke va cua gyro. Do lech chuan cua goc gia toc ke CHINH LA
   thong tin Kalman can (r_meas), nen khong phai do tay gi truoc.             */
static uint32_t Mpu6050_CalPass(float *acc_mean, float *acc_std,
                                float *gyro_mean, float *gyro_std)
{
    float    acc, gyro;
    float    s1a = 0.0f, s2a = 0.0f, s1g = 0.0f, s2g = 0.0f;
    float    inv, va, vg;
    uint32_t n = 0U, i;

    for (i = 0U; i < GYRO_CAL_SAMPLES; i++)
    {
        if (Mpu6050_ReadRaw(&acc, &gyro) == HAL_OK)
        {
            s1a += acc;   s2a += acc  * acc;
            s1g += gyro;  s2g += gyro * gyro;
            n++;
        }
        HAL_Delay(2);
    }
    if (n < 2U) { return n; }

    inv        = 1.0f / (float)n;
    *acc_mean  = s1a * inv;
    *gyro_mean = s1g * inv;
    va         = s2a * inv - (*acc_mean)  * (*acc_mean);
    vg         = s2g * inv - (*gyro_mean) * (*gyro_mean);
    if (va < 0.0f) { va = 0.0f; }
    if (vg < 0.0f) { vg = 0.0f; }
    *acc_std   = sqrtf(va);
    *gyro_std  = sqrtf(vg);
    return n;
}

static void Mpu6050_Calibrate(void)
{
    float    acc_mean = 0.0f, acc_std = 0.0f, gyro_mean = 0.0f, gyro_std = 0.0f;
    uint32_t n;
    uint8_t  pass;

    /* Toi da 2 luot: neu luot dau xe bi dung vao thi do lai.
       Xe KHONG dung yen -> gyro_std lon -> bias do duoc se sai.               */
    for (pass = 0U; pass < 2U; pass++)
    {
        n = Mpu6050_CalPass(&acc_mean, &acc_std, &gyro_mean, &gyro_std);
        if (n < 2U) { continue; }
        if ((gyro_std <= CAL_GYRO_STD_MAX_DPS) && (acc_std <= CAL_ACC_STD_MAX_DEG))
        {
            break;
        }
    }

    g_cal_acc_std_deg  = acc_std;
    g_cal_gyro_std_dps = gyro_std;
    g_cal_fail = ((gyro_std > CAL_GYRO_STD_MAX_DPS) ||
                  (acc_std  > CAL_ACC_STD_MAX_DEG)) ? 1U : 0U;

    if (n >= 2U)
    {
        s_gyro_bias = gyro_mean;
        s_angle     = acc_mean;
    }

    /* r_meas = phuong sai goc gia toc ke do duoc, nhan them he so an toan vi
       luc hieu chuan dong co dang dung yen (chua co rung).                    */
    Kalman_Init(&g_kf, s_angle, s_gyro_bias,
                acc_std * acc_std * KF_R_MARGIN);
    s_kf_innov_ms = g_kf.r_meas;
    s_kf_gate_run = 0U;

    s_mpu_t_last = DWT->CYCCNT;
}

/* Goi moi CTRL_TS trong ngat TIM6: doc cam bien + loc bu. Tra ve 1 neu goc dung duoc.
   Chong treo: HAL_I2C_Mem_Read se cho toi 25 ms neu bus dang BUSY (SDA/SCL bi giu) ->
   kiem tra co BUSY truoc, neu bi giu thi giai phong bus thay vi de HAL cho trong ngat.   */
static uint8_t Mpu6050_Update(void)
{
    float             acc = 0.0f, gyro = 0.0f, dt, rate_dps;
    uint32_t          now;
    HAL_StatusTypeDef st = HAL_ERROR;

    if (s_mpu_err_cnt >= MPU_ERR_MAX)
    {
        /* da mat cam bien: chi thu lai moi 0.5 s de khong chiem thoi gian ngat */
        if (++s_mpu_retry < MPU_RETRY_TICKS) { return 0U; }
        s_mpu_retry = 0U;
    }

    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
    {
        I2C1_BusRecover();                       /* bo qua lan doc nay */
    }
    else
    {
        st = Mpu6050_ReadRaw(&acc, &gyro);
        if ((st != HAL_OK) &&
            ((hi2c1.ErrorCode & (HAL_I2C_ERROR_TIMEOUT | HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO)) != 0U))
        {
            I2C1_BusRecover();
        }
    }

    if (st != HAL_OK)
    {
        g_mpu_err_total++;
        if (s_mpu_err_cnt < 255U) { s_mpu_err_cnt++; }
        return (s_mpu_err_cnt < MPU_ERR_MAX) ? 1U : 0U;   /* loi le te: giu goc cu, lan doc sau tich phan bu */
    }

    s_mpu_err_cnt = 0U;

    /* tich phan gyro theo THOI GIAN THUC tu lan doc thanh cong truoc (khong gia dinh 1 ms) */
    now = DWT->CYCCNT;
    dt  = (float)(now - s_mpu_t_last) / (float)SystemCoreClock;
    s_mpu_t_last = now;
    if (dt > MPU_DT_MAX_S) { dt = MPU_DT_MAX_S; }

    if (g_use_kalman != 0U)
    {
        /* Kalman: gyro THO vao, bo loc tu tru bias no uoc luong duoc */
        s_angle   = Kalman_Update(&g_kf, gyro, acc, dt);
        rate_dps  = g_kf.rate;

        g_kf_bias_dps  = g_kf.bias;
        g_kf_innov_deg = g_kf.innov;
        g_kf_innov_rms = sqrtf(s_kf_innov_ms);
        g_kf_k0        = g_kf.k0;
    }
    else
    {
        /* loc bu cu: bias chot 1 lan luc hieu chuan, he so tron co dinh */
        gyro    -= s_gyro_bias;
        s_angle  = GYRO_ACC_ALPHA * (s_angle + gyro * dt) + (1.0f - GYRO_ACC_ALPHA) * acc;
        rate_dps = gyro;
    }

    g_gyro_dps      = ANGLE_SIGN * rate_dps;
    g_angle_acc_deg = ANGLE_SIGN * acc + ANGLE_OFFSET_DEG;
    g_angle_deg     = ANGLE_SIGN * s_angle + ANGLE_OFFSET_DEG;
    return 1U;
}

/* ============================ DIEU KHIEN (moi 1 ms) ============================ */
static void Control_Step(uint8_t angle_ok)
{
    float   d1 = 0.0f, d2 = 0.0f;
    float   rpm_ref = 0.0f;
    float   e_rad;
    uint8_t mode  = g_test_mode;
    uint8_t armed = g_armed;

    /* 1) toc do 2 banh tu encoder */
    g_rpm1 += SPEED_LPF_ALPHA * (Encoder_GetRpm(&g_enc1) - g_rpm1);
    g_rpm2 += SPEED_LPF_ALPHA * (Encoder_GetRpm(&g_enc2) - g_rpm2);

    /* 2) bao ve o che do can bang: mat cam bien hoac nga -> tat dong co */
    if ((armed != 0U) && (mode == TEST_BALANCE))
    {
        if (angle_ok == 0U)
        {
            g_fault = FAULT_MPU;
            armed   = 0U;
        }
        else if (fabsf(g_angle_deg - g_bal_setpoint_deg) > FALL_ANGLE_DEG)
        {
            g_fault = FAULT_FALL;
            armed   = 0U;
        }
        g_armed = armed;
    }

    /* 3) bat / tat dong co hoac doi che do -> xoa trang thai ca 3 PID */
    if ((armed != s_armed_prev) || (mode != s_mode_prev))
    {
        PID_Reset(&g_pid_bal);
        PID_Reset(&g_pid1);
        PID_Reset(&g_pid2);
    }
    s_armed_prev = armed;
    s_mode_prev  = mode;

    if (armed == 0U)
    {
        g_speed_ref_radps = 0.0f;
        g_rpm_ref         = 0.0f;
        Motor_StopAll();
        return;
    }

    /* 4) toc do dat theo che do */
    switch (mode)
    {
    case TEST_OPEN_LOOP:          /* khong PID */
        d1 = g_test_duty;
        d2 = g_test_duty;
        break;

    case TEST_SPEED_PID:          /* toc do dat co dinh - de chinh PID dong co */
        rpm_ref = g_test_rpm;
        break;

    case TEST_BALANCE:
        /* PID goc (giong xe cu). Dau: e = goc - goc dat, nen nghieng ve truoc (goc > 0)
           -> toc do dat > 0 -> banh chay toi, dua diem tiep dat ve duoi trong tam.   */
        /* goc dat la hang so -> de/dt = toc do goc do boi gyro (rad/s).
           Dung truc tiep gyro lam khau D: sach hon va khong tre so voi
           vi phan goc da loc.                                              */
        e_rad             = (g_angle_deg - g_bal_setpoint_deg) / RAD_TO_DEG;
        g_speed_ref_radps = PID_UpdateRate(&g_pid_bal, e_rad, 0.0f, CTRL_TS,
                                           g_gyro_dps / RAD_TO_DEG);
        rpm_ref           = g_speed_ref_radps * RADPS_TO_RPM;
        break;

    default:                      /* TEST_STOP */
        break;
    }

    /* 5) 2 PID dong co doc lap, cung bam toc do dat */
    if ((mode == TEST_SPEED_PID) || (mode == TEST_BALANCE))
    {
        rpm_ref = Clampf(rpm_ref, -RPM_REF_MAX, RPM_REF_MAX);
        d1 = PID_Update(&g_pid1, rpm_ref - g_rpm1, g_pid1.kff * rpm_ref, CTRL_TS);
        d2 = PID_Update(&g_pid2, rpm_ref - g_rpm2, g_pid2.kff * rpm_ref, CTRL_TS);
    }

    g_rpm_ref = rpm_ref;
    Motor_Set(1U, d1);
    Motor_Set(2U, d2);
}

/* ============================ TELEMETRY -> ESP32 ============================ */
/* CRC-8, da thuc 0x07, gia tri dau 0x00 (giong ESP32 va protocol.py) */
static uint8_t Crc8(const uint8_t *p, uint32_t n)
{
    uint8_t  crc = 0U;
    uint32_t i;
    uint8_t  b;

    for (i = 0U; i < n; i++)
    {
        crc ^= p[i];
        for (b = 0U; b < 8U; b++)
        {
            crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static int16_t DutyToX100(float duty)
{
    return (int16_t)Clampf(duty * 100.0f, -10000.0f, 10000.0f);
}

/* Dong goi khung tham so (PID dang dung) - goi trong Telemetry_Send khi s_params_pending */
static void Params_Fill(ParamsFrame_t *p)
{
    p->sof1       = TELEM_SOF1;
    p->sof2       = TELEM_SOF2;
    p->type       = TELEM_TYPE_PARAMS;
    p->len        = (uint8_t)(sizeof(ParamsFrame_t) - 5U);
    p->bal_kp     = g_pid_bal.kp;
    p->bal_ki     = g_pid_bal.ki;
    p->bal_kd     = g_pid_bal.kd;
    p->bal_sp_deg = g_bal_setpoint_deg;
    p->mot_kp     = g_pid1.kp;
    p->mot_ki     = g_pid1.ki;
    p->mot_kff    = g_pid1.kff;
    p->cmd_cnt    = (uint8_t)g_cmd_ok;
    p->reserved   = 0U;
    p->crc        = Crc8(&p->type, (uint32_t)sizeof(ParamsFrame_t) - 3U);
}

/* Goi cuoi moi ngat TIM6: dong goi so lieu VUA tinh xong va day ra UART bang DMA
   (31 byte @ 921600 baud = 0.34 ms, CPU khong phai cho). Neu lan gui truoc chua xong
   thi bo khung nay (dem g_telem_drop) chu khong cho.                               */
static void Telemetry_Send(void)
{
    TelemFrame_t *f = &s_tx.st;
    uint16_t      n = (uint16_t)sizeof(TelemFrame_t);

    if (++s_telem_div < TELEM_DIV) { return; }
    s_telem_div = 0U;

    if (huart1.gState != HAL_UART_STATE_READY)
    {
        g_telem_drop++;
        return;
    }

    f->sof1       = TELEM_SOF1;
    f->sof2       = TELEM_SOF2;
    f->type       = TELEM_TYPE_STATE;
    f->len        = TELEM_PAYLOAD_LEN;
    f->tick       = s_tick_count / 2U;    /* giu don vi 2 ms nhu truoc */
    f->angle_deg  = g_angle_deg;
    f->rpm_ref    = g_rpm_ref;
    f->rpm1       = g_rpm1;
    f->rpm2       = g_rpm2;
    f->duty1_x100 = DutyToX100(g_duty1);
    f->duty2_x100 = DutyToX100(g_duty2);
    f->status     = (uint8_t)(((g_armed != 0U) ? 0x01U : 0x00U)
                            | ((uint8_t)(g_fault & 0x03U) << 1)
                            | ((g_auto_arm != 0U) ? 0x08U : 0x00U)
                            | ((uint8_t)(g_test_mode & 0x03U) << 4));
    f->reserved   = (uint8_t)g_cmd_ok;   /* so lenh PC da nhan (mod 256) -> PC kiem tra duong xuong */
    f->crc        = Crc8(&f->type, (uint32_t)sizeof(TelemFrame_t) - 3U);

    if (s_params_pending != 0U)
    {
        Params_Fill(&s_tx.pa);
        n = (uint16_t)sizeof(TxBuf_t);
    }

    if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&s_tx, n) == HAL_OK)
    {
        g_telem_sent++;
        if (n == (uint16_t)sizeof(TxBuf_t)) { s_params_pending = 0U; }
    }
    else
    {
        g_telem_drop++;
    }
}

/* ============================ LENH TU PC (USART1 RX + DMA vong tron) ============================ */
/* Goi 1 lan truoc khi bat TIM6. DMA2 Stream2 kenh 4 = USART1_RX, che do vong tron, KHONG dung ngat DMA:
   DMA tu ghi vao s_cmd_rx, ngat TIM6 doc phan moi -> khong mat byte du ngat TIM6 dai ~0.5 ms.     */
static void Cmd_RxStart(void)
{
    if (s_cmd_dma_ok == 0U)
    {
        s_hdma_usart1_rx.Instance                 = DMA2_Stream2;
        s_hdma_usart1_rx.Init.Channel             = DMA_CHANNEL_4;
        s_hdma_usart1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        s_hdma_usart1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        s_hdma_usart1_rx.Init.MemInc              = DMA_MINC_ENABLE;
        s_hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        s_hdma_usart1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        s_hdma_usart1_rx.Init.Mode                = DMA_CIRCULAR;
        s_hdma_usart1_rx.Init.Priority            = DMA_PRIORITY_LOW;
        s_hdma_usart1_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&s_hdma_usart1_rx) != HAL_OK) { return; }
        __HAL_LINKDMA(&huart1, hdmarx, s_hdma_usart1_rx);
        s_cmd_dma_ok = 1U;
    }
    s_cmd_rd    = 0U;
    s_cmd_state = 0U;
    if (HAL_UART_Receive_DMA(&huart1, (uint8_t *)s_cmd_rx, (uint16_t)CMD_RX_BUF) == HAL_OK)
    {
        /* Tat ngat loi UART (nhieu / khung / tran): neu de bat, 1 byte loi la HAL HUY nhan DMA
           va khong tu bat lai. Khung hong da co CRC loai bo.                                   */
        ATOMIC_CLEAR_BIT(huart1.Instance->CR3, USART_CR3_EIE);
    }
}

/* Doc float32 little-endian tu khung lenh; tra 0 neu NaN / vo cuc */
static uint8_t Cmd_GetF32(const uint8_t *src, float *out)
{
    uint32_t u;
    memcpy(&u, src, 4U);
    if ((u & 0x7F800000U) == 0x7F800000U) { return 0U; }
    memcpy(out, &u, 4U);
    return 1U;
}

static void Cmd_Handle(const uint8_t *fr)      /* fr: type, len, payload */
{
    float v, kp, ki, kd, sp;

    switch (fr[0])
    {
    case CMD_TYPE_SET_RPM:
        if ((fr[1] != 4U) || (Cmd_GetF32(&fr[2], &v) == 0U)) { g_cmd_bad++; return; }
        g_cmd_rpm = Clampf(v, -RPM_REF_MAX, RPM_REF_MAX);
        if (PC_RPM_TEST != 0U) { g_test_rpm = g_cmd_rpm; }
        break;

    case CMD_TYPE_SET_BAL:                      /* PID goc tu PC (tu chinh / chinh tay) */
        if ((fr[1] != 16U) ||
            (Cmd_GetF32(&fr[2],  &kp) == 0U) || (Cmd_GetF32(&fr[6],  &ki) == 0U) ||
            (Cmd_GetF32(&fr[10], &kd) == 0U) || (Cmd_GetF32(&fr[14], &sp) == 0U))
        {
            g_cmd_bad++;
            return;
        }
        /* doi he so khong reset PID: khau I luu san gia tri dau ra (uI) nen doi Ki khong bi giat */
        g_pid_bal.kp       = Clampf(kp, 0.0f, BAL_KP_MAX);
        g_pid_bal.ki       = Clampf(ki, 0.0f, BAL_KI_MAX);
        g_pid_bal.kd       = Clampf(kd, 0.0f, BAL_KD_MAX);
        g_bal_setpoint_deg = Clampf(sp, -BAL_SP_MAX_DEG, BAL_SP_MAX_DEG);
        s_params_pending   = 1U;                /* gui lai PID dang dung = xac nhan cho PC */
        break;

    case CMD_TYPE_SET_SP:                       /* goc nghieng dat tu thanh keo tren PC (chi doi goc, giu nguyen PID) */
        if ((fr[1] != 4U) || (Cmd_GetF32(&fr[2], &sp) == 0U)) { g_cmd_bad++; return; }
        g_bal_setpoint_deg = Clampf(sp, -BAL_SP_MAX_DEG, BAL_SP_MAX_DEG);
        s_params_pending   = 1U;                /* gui lai khung tham so = xac nhan */
        break;

    case CMD_TYPE_GET_PARAMS:
        if (fr[1] != 0U) { g_cmd_bad++; return; }
        s_params_pending = 1U;
        break;

    default:
        g_cmd_bad++;
        return;
    }
    g_cmd_ok++;
    s_cmd_last_ms = HAL_GetTick();
}

static void Cmd_ParseByte(uint8_t b)
{
    switch (s_cmd_state)
    {
    case 0U:                                    /* cho AA */
        if (b == TELEM_SOF1) { s_cmd_state = 1U; }
        break;
    case 1U:                                    /* cho 55 */
        if (b == TELEM_SOF2)      { s_cmd_state = 2U; }
        else if (b != TELEM_SOF1) { s_cmd_state = 0U; }
        break;
    case 2U:                                    /* type */
        s_cmd_frame[0] = b;
        s_cmd_state    = 3U;
        break;
    case 3U:                                    /* len */
        if (b > CMD_MAX_PAYLOAD) { g_cmd_bad++; s_cmd_state = 0U; break; }
        s_cmd_frame[1] = b;
        s_cmd_n        = 0U;
        s_cmd_state    = (b != 0U) ? 4U : 5U;
        break;
    case 4U:                                    /* payload */
        s_cmd_frame[2U + s_cmd_n] = b;
        s_cmd_n++;
        if (s_cmd_n >= s_cmd_frame[1]) { s_cmd_state = 5U; }
        break;
    default:                                    /* CRC */
        if (Crc8(s_cmd_frame, 2U + (uint32_t)s_cmd_frame[1]) == b) { Cmd_Handle(s_cmd_frame); }
        else                                                         { g_cmd_bad++; }
        s_cmd_state = 0U;
        break;
    }
}

/* Goi dau moi ngat TIM6: doc cac byte DMA vua ghi, tach khung; het han lenh -> toc do dat = 0 */
static void Cmd_Poll(void)
{
    uint16_t wr;

    if (s_cmd_dma_ok == 0U) { return; }
    if (huart1.RxState != HAL_UART_STATE_BUSY_RX)       /* DMA nhan bi dung (khong mong doi) -> bat lai */
    {
        g_cmd_rx_restart++;
        Cmd_RxStart();
    }

    wr = (uint16_t)(CMD_RX_BUF - __HAL_DMA_GET_COUNTER(&s_hdma_usart1_rx));
    if (wr >= CMD_RX_BUF) { wr = 0U; }
    while (s_cmd_rd != wr)
    {
        Cmd_ParseByte(s_cmd_rx[s_cmd_rd]);
        s_cmd_rd = (uint16_t)((s_cmd_rd + 1U) % CMD_RX_BUF);
    }

    if ((PC_RPM_TEST != 0U) && ((HAL_GetTick() - s_cmd_last_ms) > CMD_TIMEOUT_MS))
    {
        g_test_rpm = 0.0f;
    }
}

/* ============================ NGAT TIM6 (1 ms) ============================ */
static void Control_Tick(void)
{
    uint32_t t0 = DWT->CYCCNT;
    uint32_t gap_us;
    uint8_t  angle_ok;

    /* chan doan: khoang cach thuc giua 2 lan ngat (binh thuong 1000 us) */
    if (s_tick_last != 0U)
    {
        gap_us = (t0 - s_tick_last) / (SystemCoreClock / 1000000U);
        if (gap_us > g_tick_gap_us_max) { g_tick_gap_us_max = gap_us; }
        if (gap_us > TICK_OVERRUN_US)   { g_overrun_cnt++; }
    }
    s_tick_last = t0;

    Cmd_Poll();                       /* lenh tu PC truoc -> toc do dat moi dung ngay chu ky nay */
    angle_ok = Mpu6050_Update();

    Control_Step(angle_ok);
    s_tick_count++;
    Telemetry_Send();

    g_isr_us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000U);
    if (g_isr_us > g_isr_us_max) { g_isr_us_max = g_isr_us; }
}

/* ============================ NUT K1 + LED (vong while) ============================ */
static uint8_t Can_Arm(void)
{
    if (g_test_mode != TEST_BALANCE)  { return 1U; }   /* test dong co khong can MPU */
    if (s_mpu_err_cnt >= MPU_ERR_MAX) { return 0U; }
    return (fabsf(g_angle_deg - g_bal_setpoint_deg) < ARM_ANGLE_DEG) ? 1U : 0U;
}

/* Nhan K1 (chong doi phim 30 ms):
   - dang chay  -> tat dong co va KHONG tu chay lai (dung han)
   - dang dung  -> cho phep chay lai (chay ngay neu du dieu kien)          */
static void Button_Poll(void)
{
    static uint8_t  raw_prev = 0U;
    static uint8_t  stable   = 0U;
    static uint32_t t_change = 0U;
    uint32_t now = HAL_GetTick();
    uint8_t  raw = (HAL_GPIO_ReadPin(BTN_K1_GPIO_Port, BTN_K1_Pin) == GPIO_PIN_SET) ? 1U : 0U;

    if (raw != raw_prev)
    {
        raw_prev = raw;
        t_change = now;
    }
    else if (((now - t_change) > 30U) && (raw != stable))
    {
        stable = raw;
        if (stable == 1U)
        {
            if (g_armed != 0U)
            {
                g_armed    = 0U;
                g_auto_arm = 0U;
            }
            else
            {
                g_auto_arm = 1U;
                if (Can_Arm() != 0U)
                {
                    g_fault = FAULT_NONE;
                    g_armed = 1U;
                }
            }
        }
    }
}

/* Tu chay: khi xe gan thang dung lien tuc AUTO_ARM_HOLD_MS (luc bat nguon va sau khi nga) */
static void Auto_Arm_Poll(void)
{
    static uint32_t t_ok = 0U;
    uint32_t now = HAL_GetTick();

    if ((g_auto_arm == 0U) || (g_armed != 0U) || (Can_Arm() == 0U))
    {
        t_ok = now;
        return;
    }
    if ((now - t_ok) >= AUTO_ARM_HOLD_MS)
    {
        g_fault = FAULT_NONE;
        g_armed = 1U;
    }
}

/* LED D2 active-low: nhay cham = cho, sang lien = dang chay, nhay nhanh = loi */
static void Led_Update(void)
{
    uint32_t      t = HAL_GetTick();
    GPIO_PinState s;

    if (g_fault != FAULT_NONE)  { s = (((t / 100U) % 2U) != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET; }
    else if (g_armed != 0U)     { s = GPIO_PIN_RESET; }
    else                        { s = (((t / 500U) % 2U) != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET; }

    HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, s);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* PWM L298N: dong co 1 = TIM1_CH1 (PE9), dong co 2 = TIM1_CH2 (PE11), duty 0, IN = 0 -> tha troi */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  Motor_StopAll();

  /* Bo dem chu ky DWT: do thoi gian chay ngat (g_isr_us) */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

  /* 3 bo PID: goc -> toc do (rad/s), va 2 PID toc do dong co (rpm -> %duty) */
  PID_Init(&g_pid_bal, BAL_KP, BAL_KI, BAL_KD, 0.0f,    BAL_D_ALPHA, BAL_KAW, BAL_SPEED_LIMIT);
  PID_Init(&g_pid1,    MOT_KP, MOT_KI, MOT_KD, MOT_KFF, MOT_D_ALPHA, MOT_KAW, DUTY_MAX_PCT);
  PID_Init(&g_pid2,    MOT_KP, MOT_KI, MOT_KD, MOT_KFF, MOT_D_ALPHA, MOT_KAW, DUTY_MAX_PCT);
  Encoder_Reset(&g_enc1);
  Encoder_Reset(&g_enc2);

  /* MPU6050: LED sang trong luc hieu chuan (~1.3 s) -> de xe dung yen */
  HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_RESET);
  if (Mpu6050_Init() != 0U)
  {
    Mpu6050_Calibrate();
  }
  else
  {
    s_mpu_err_cnt = MPU_ERR_MAX;   /* khong co cam bien: van test duoc dong co o che do 1, 2 */
    g_fault       = FAULT_MPU;
  }
  HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_SET);

  /* Nhan lenh tu PC (qua ESP32) bang DMA vong tron, doc trong ngat TIM6 */
  Cmd_RxStart();

  /* Ngat TIM6 moi 1 ms: doc goc + PID goc + do toc do + 2 PID dong co */
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Button_Poll();
    Auto_Arm_Poll();
    Led_Update();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M1_IN1_Pin|M1_IN2_Pin|M2_IN3_Pin|M2_IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENC1_A_Pin ENC2_A_Pin */
  GPIO_InitStruct.Pin = ENC1_A_Pin|ENC2_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC1_B_Pin ENC2_B_Pin */
  GPIO_InitStruct.Pin = ENC1_B_Pin|ENC2_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_K1_Pin */
  GPIO_InitStruct.Pin = BTN_K1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_K1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_D2_Pin */
  GPIO_InitStruct.Pin = LED_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_IN1_Pin M1_IN2_Pin M2_IN3_Pin M2_IN4_Pin */
  GPIO_InitStruct.Pin = M1_IN1_Pin|M1_IN2_Pin|M2_IN3_Pin|M2_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Ngat ngoai encoder: moi suon len / xuong cua kenh A (kenh B khong co ngat) */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ENC1_A_Pin)
    {
        Encoder_ISR(&g_enc1);
    }
    else if (GPIO_Pin == ENC2_A_Pin)
    {
        Encoder_ISR(&g_enc2);
    }
}

/* Ngat timer chu ky dieu khien */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        Control_Tick();
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
