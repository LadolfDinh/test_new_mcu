/* HAL gia de bien dich NGUYEN cac khoi USER CODE cua main.c tren PC.
   Moi thanh ghi / ham HAL ma code dung deu duoc thay bang bien do mo phong cap. */
#ifndef HAL_STUB_H
#define HAL_STUB_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef _WIN32
#define SIM_API __declspec(dllexport)
#else
#define SIM_API __attribute__((visibility("default")))
#endif

/* ---------- kieu co ban ---------- */
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef enum { RESET = 0, SET = 1 } FlagStatus;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;

/* ---------- GPIO ---------- */
typedef struct { volatile uint32_t IDR; volatile uint32_t ODR; } GPIO_TypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
extern GPIO_TypeDef sim_gpio[5];
#define GPIOA (&sim_gpio[0])
#define GPIOB (&sim_gpio[1])
#define GPIOC (&sim_gpio[2])
#define GPIOD (&sim_gpio[3])
#define GPIOE (&sim_gpio[4])
#define GPIO_PIN_0  ((uint16_t)0x0001)
#define GPIO_PIN_1  ((uint16_t)0x0002)
#define GPIO_PIN_2  ((uint16_t)0x0004)
#define GPIO_PIN_3  ((uint16_t)0x0008)
#define GPIO_PIN_4  ((uint16_t)0x0010)
#define GPIO_PIN_5  ((uint16_t)0x0020)
#define GPIO_PIN_6  ((uint16_t)0x0040)
#define GPIO_PIN_7  ((uint16_t)0x0080)
#define GPIO_PIN_8  ((uint16_t)0x0100)
#define GPIO_PIN_9  ((uint16_t)0x0200)
#define GPIO_PIN_10 ((uint16_t)0x0400)
#define GPIO_PIN_11 ((uint16_t)0x0800)
#define GPIO_PIN_12 ((uint16_t)0x1000)
#define GPIO_PIN_13 ((uint16_t)0x2000)
#define GPIO_PIN_14 ((uint16_t)0x4000)
#define GPIO_PIN_15 ((uint16_t)0x8000)
#define GPIO_MODE_OUTPUT_OD 0x11U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_HIGH 2U
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s)
{ if (s != GPIO_PIN_RESET) p->ODR |= pin; else p->ODR &= ~(uint32_t)pin; }
static inline GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin)
{ return ((p->IDR & pin) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET; }
static inline void HAL_GPIO_TogglePin(GPIO_TypeDef *p, uint16_t pin) { p->ODR ^= pin; }
static inline void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g) { (void)p; (void)g; }

/* ---------- loi / ngat ---------- */
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}

/* ---------- DWT + clock: do mo phong tang theo thoi gian ---------- */
typedef struct { volatile uint32_t CYCCNT; volatile uint32_t CTRL; } DWT_Type;
typedef struct { volatile uint32_t DEMCR; } CoreDebug_Type;
extern DWT_Type sim_dwt;
extern CoreDebug_Type sim_coredebug;
#define DWT (&sim_dwt)
#define CoreDebug (&sim_coredebug)
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
extern uint32_t SystemCoreClock;

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);

/* ---------- TIM ---------- */
typedef struct { uint32_t id; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; uint32_t ARR; uint32_t CCR1; } TIM_HandleTypeDef;
extern TIM_TypeDef sim_tim[3];
#define TIM1 (&sim_tim[0])
#define TIM6 (&sim_tim[1])
#define TIM8 (&sim_tim[2])
#define TIM_CHANNEL_1 0U
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->ARR)
#define __HAL_TIM_SET_COMPARE(h, ch, v) do { (void)(ch); (h)->CCR1 = (uint32_t)(v); } while (0)
static inline HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t ch) { (void)h; (void)ch; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *h) { (void)h; return HAL_OK; }

/* ---------- I2C: MPU6050 doc tu bo dem do mo phong ghi ---------- */
typedef struct { void *Instance; volatile uint32_t ErrorCode; } I2C_HandleTypeDef;
#define I2C_MEMADD_SIZE_8BIT 1U
#define I2C_FLAG_BUSY 0x00100002U
#define HAL_I2C_ERROR_BERR    0x01U
#define HAL_I2C_ERROR_ARLO    0x02U
#define HAL_I2C_ERROR_TIMEOUT 0x20U
extern volatile uint8_t sim_i2c_busy;
#define __HAL_I2C_GET_FLAG(h, f) ((sim_i2c_busy != 0U) ? SET : RESET)
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t dev, uint16_t reg, uint16_t sz,
                                   uint8_t *buf, uint16_t n, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t dev, uint16_t reg, uint16_t sz,
                                    uint8_t *buf, uint16_t n, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t dev, uint32_t trials, uint32_t timeout);
static inline HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *h) { (void)h; return HAL_OK; }

/* ---------- UART + DMA (telemetry): coi nhu gui xong ngay ---------- */
typedef enum { HAL_UART_STATE_READY = 0x20U, HAL_UART_STATE_BUSY_TX = 0x21U } HAL_UART_StateTypeDef;
typedef struct { volatile HAL_UART_StateTypeDef gState; } UART_HandleTypeDef;
typedef struct { uint32_t dummy; } DMA_HandleTypeDef;
static inline HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ (void)h; (void)p; (void)n; return HAL_OK; }

#endif
