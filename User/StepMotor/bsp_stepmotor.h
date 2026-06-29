#ifndef __BSP_STEPMOTOR_H
#define __BSP_STEPMOTOR_H

#include "stm32f10x.h"

/********************** 电机轴选择宏 ****************************/
/* 双轴步进电机: Motor X 和 Motor Y, 各用独立定时器 */
#define STEPMOTOR_X          TIM2          /* X轴定时器: TIM2 */
#define STEPMOTOR_Y          TIM3          /* Y轴定时器: TIM3 */
#define STEPMOTOR_X_IRQ      TIM2_IRQn
#define STEPMOTOR_Y_IRQ      TIM3_IRQn
#define STEPMOTOR_X_PERIPH   RCC_APB1Periph_TIM2
#define STEPMOTOR_Y_PERIPH   RCC_APB1Periph_TIM3

/********************** GPIO 引脚定义 (Motor X) *********************/
/* ---- STEP / DIR / EN 引脚 ---- */
#define MOTOR_X_STEP_PORT       GPIOA
#define MOTOR_X_STEP_PIN        GPIO_Pin_0      /* PA0 — STEP 脉冲 */
#define MOTOR_X_STEP_CLK        RCC_APB2Periph_GPIOA

#define MOTOR_X_DIR_PORT        GPIOA
#define MOTOR_X_DIR_PIN         GPIO_Pin_1      /* PA1 — 方向控制 */
#define MOTOR_X_DIR_CLK         RCC_APB2Periph_GPIOA

#define MOTOR_X_EN_PORT         GPIOA
#define MOTOR_X_EN_PIN          GPIO_Pin_4      /* PA4 — 使能 (低有效) */
#define MOTOR_X_EN_CLK          RCC_APB2Periph_GPIOA

/********************** GPIO 引脚定义 (Motor Y) *********************/
#define MOTOR_Y_STEP_PORT       GPIOA
#define MOTOR_Y_STEP_PIN        GPIO_Pin_6      /* PA6 — STEP 脉冲 */
#define MOTOR_Y_STEP_CLK        RCC_APB2Periph_GPIOA

#define MOTOR_Y_DIR_PORT        GPIOA
#define MOTOR_Y_DIR_PIN         GPIO_Pin_7      /* PA7 — 方向控制 */
#define MOTOR_Y_DIR_CLK         RCC_APB2Periph_GPIOA

#define MOTOR_Y_EN_PORT         GPIOB
#define MOTOR_Y_EN_PIN          GPIO_Pin_0      /* PB0 — 使能 (低有效) */
#define MOTOR_Y_EN_CLK          RCC_APB2Periph_GPIOB

/********************** 运动参数默认值 ****************************/
/* 42步进电机 1.8° + 256细分 = 51200 microsteps/rev */
#define DEFAULT_MAX_SPEED       3000UL     /* 最大速度 (steps/s)  3kHz, 低速大扭矩 */
#define DEFAULT_ACCEL           3000UL     /* 加速度   (steps/s²) 3k, 温柔起步 */
#define DEFAULT_HOME_SPEED      1000UL     /* 回零速度 (steps/s) */
#define MIN_SPEED               200UL      /* 最小启动速度 (克服静摩擦) */
#define MAX_SPEED               30000UL    /* 绝对最大速度上限     */
#define MAX_ACCEL               50000UL    /* 绝对最大加速度上限   */

/* 定时器时钟 = 72MHz (APB1, PSC=0) */
#define STEP_TIM_CLK            72000000UL

/********************** 电机状态枚举 ****************************/
typedef enum {
    MOTOR_IDLE = 0,      /* 空闲 */
    MOTOR_ACCEL,         /* 加速阶段 */
    MOTOR_CONST,         /* 匀速阶段 */
    MOTOR_DECEL,         /* 减速阶段 */
    MOTOR_HOMING         /* 回零中 */
} MotorState;

/********************** 方向定义 ****************************/
#define DIR_CW   0   /* 顺时针 (正向) */
#define DIR_CCW  1   /* 逆时针 (反向) */

/********************** 步进电机结构体 (前向声明) ******************/
typedef struct stepper_motor_s StepperMotor;

struct stepper_motor_s {
    /* —— 硬件绑定 —— */
    TIM_TypeDef  *TIMx;              /* 定时器基址 */
    GPIO_TypeDef *step_port;         /* STEP 端口 */
    uint16_t      step_pin;          /* STEP 引脚 */
    GPIO_TypeDef *dir_port;          /* DIR 端口 */
    uint16_t      dir_pin;           /* DIR 引脚 */
    GPIO_TypeDef *en_port;           /* EN 端口 */
    uint16_t      en_pin;            /* EN 引脚 */
    uint8_t       axis_id;           /* TMC2208 轴 ID (0=X, 1=Y) */

    /* —— 位置跟踪 (microsteps) —— */
    volatile int32_t  position;      /* 当前绝对位置 */
    volatile int32_t  target;        /* 目标位置     */

    /* —— 速度参数 (steps/s) —— */
    uint32_t          max_speed;     /* 最大速度     */
    uint32_t          accel;         /* 加速度       */
    uint32_t          min_speed;     /* 最小启动速度 */

    /* —— 运动状态 —— */
    volatile MotorState state;       /* 运动阶段     */
    volatile uint8_t    direction;   /* 当前方向     */
    volatile uint8_t    running;     /* 运行标志     */
    volatile uint32_t   current_speed; /* 当前瞬时速度 */

    /* —— 梯形加减速内部变量 —— */
    volatile int32_t    decel_start;   /* 减速起始步号 */
    volatile uint32_t   step_count;    /* 本次移动已走步数 */
    volatile uint32_t   pulse_delay;   /* 当前脉冲间隔 (TIM->ARR) */

    /* —— 回调 —— */
    void (*on_done)(StepperMotor *m);

};

/********************** API 声明 ****************************/
void Stepper_Init(StepperMotor *m,
                  TIM_TypeDef *TIMx,
                  GPIO_TypeDef *step_port, uint16_t step_pin,
                  GPIO_TypeDef *dir_port,  uint16_t dir_pin,
                  GPIO_TypeDef *en_port,   uint16_t en_pin,
                  uint8_t axis_id,
                  uint32_t max_speed, uint32_t accel);

void Stepper_MoveTo(StepperMotor *m, int32_t target);
void Stepper_MoveRel(StepperMotor *m, int32_t rel);
void Stepper_Stop(StepperMotor *m);
void Stepper_SoftStop(StepperMotor *m);
void Stepper_SetMaxSpeed(StepperMotor *m, uint32_t speed);
void Stepper_SetAccel(StepperMotor *m, uint32_t accel);
void Stepper_Enable(StepperMotor *m, uint8_t enable);
int32_t Stepper_GetPosition(StepperMotor *m);
uint8_t Stepper_IsRunning(StepperMotor *m);
void Stepper_SetHome(StepperMotor *m);
void Stepper_SetDoneCallback(StepperMotor *m, void (*cb)(StepperMotor *));
void Stepper_TIM_IRQHandler(StepperMotor *m);

#endif /* __BSP_STEPMOTOR_H */
