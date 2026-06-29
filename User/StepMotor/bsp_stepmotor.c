#include "bsp_stepmotor.h"
#include "..\TMC2208\bsp_tmc2208.h"
#include <stddef.h>

/********************** 短延时 (NOP 循环, 约 200ns @72MHz) **************/
static void nop_delay(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        __NOP();
    }
}

/********************** 定时器启动 ************************************/
static void TIM_StartPulse(TIM_TypeDef *TIMx, uint32_t period)
{
    TIMx->ARR = (uint16_t)period;        /* 设定周期 */
    TIMx->CNT = 0;                       /* 清零计数器 */
    TIM_ClearFlag(TIMx, TIM_FLAG_Update);/* 清除更新标志 */
    TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);  /* 使能更新中断 */
    TIM_Cmd(TIMx, ENABLE);               /* 启动定时器 */
}

/********************** 定时器停止 ************************************/
static void TIM_StopPulse(TIM_TypeDef *TIMx)
{
    TIM_Cmd(TIMx, DISABLE);              /* 停止定时器 */
    TIM_ITConfig(TIMx, TIM_IT_Update, DISABLE); /* 关闭中断 */
}

/********************** 设置方向 (在 STEP 之前) *************************/
static void SetDirection(StepperMotor *m, uint8_t dir)
{
    m->direction = dir;
    if (dir == DIR_CW)
        GPIO_ResetBits(m->dir_port, m->dir_pin);
    else
        GPIO_SetBits(m->dir_port, m->dir_pin);
}

/********************** 运动完成 ************************************/
static void MoveDone(StepperMotor *m)
{
    TIM_StopPulse(m->TIMx);
    GPIO_ResetBits(m->step_port, m->step_pin);   /* STEP 拉低 */
    m->state   = MOTOR_IDLE;
    m->running = 0;
    if (m->on_done) {
        m->on_done(m);
    }
}

/********************** 电机初始化 ************************************/
/*
 * 功能: 绑定硬件引脚、初始化参数、注册到全局列表
 */
void Stepper_Init(StepperMotor *m,
                  TIM_TypeDef *TIMx,
                  GPIO_TypeDef *step_port, uint16_t step_pin,
                  GPIO_TypeDef *dir_port,  uint16_t dir_pin,
                  GPIO_TypeDef *en_port,   uint16_t en_pin,
                  uint8_t axis_id,
                  uint32_t max_speed, uint32_t accel)
{
    m->TIMx        = TIMx;
    m->step_port   = step_port;
    m->step_pin    = step_pin;
    m->dir_port    = dir_port;
    m->dir_pin     = dir_pin;
    m->en_port     = en_port;
    m->en_pin      = en_pin;
    m->axis_id     = axis_id;

    m->position    = 0;
    m->target      = 0;
    m->max_speed   = (max_speed > 0) ? max_speed : DEFAULT_MAX_SPEED;
    m->accel       = (accel > 0)    ? accel     : DEFAULT_ACCEL;
    m->min_speed   = MIN_SPEED;
    m->state       = MOTOR_IDLE;
    m->running     = 0;
    m->direction   = DIR_CW;
    m->pulse_delay = 65535;
    m->step_count  = 0;
    m->on_done     = NULL;

    /* 初始引脚状态: STEP=0, DIR=0, EN=1 (禁用) */
    GPIO_ResetBits(m->step_port, m->step_pin);
    GPIO_ResetBits(m->dir_port,  m->dir_pin);
    GPIO_SetBits(m->en_port,    m->en_pin);
}

/********************** 绝对位置移动 (核心) ****************************/
/*
 * 梯形加减速规划:
 *   - 短距离 (delta <= 2*加速距离): 三角速度曲线 (无匀速段)
 *   - 长距离:                         加速 → 匀速 → 减速
 *
 * 加速距离: N = (Vmax² - Vmin²) / (2 * a)
 */
void Stepper_MoveTo(StepperMotor *m, int32_t target)
{
    if (target == m->position) return;

    int32_t delta = target - m->position;

    /* 设置方向 */
    if (delta > 0) {
        SetDirection(m, DIR_CW);
    } else {
        SetDirection(m, DIR_CCW);
        delta = -delta;
    }

    m->target = target;

    /* 计算加速到 max_speed 所需步数 */
    uint32_t accel_steps = (m->max_speed * m->max_speed
                           - m->min_speed * m->min_speed)
                           / (2 * m->accel);

    if ((uint32_t)delta <= accel_steps * 2) {
        /* 三角曲线: 一半加速, 一半减速 */
        m->decel_start = delta / 2;
    } else {
        /* 梯形曲线: 加速 → 匀速 → 减速 */
        m->decel_start = delta - accel_steps;
    }

    m->state         = MOTOR_ACCEL;
    m->running       = 1;
    m->step_count    = 0;
    m->current_speed = m->min_speed;

    /* 初始脉冲间隔: c0 = TIM_CLK / Vmin */
    uint32_t delay = STEP_TIM_CLK / m->min_speed;
    if (delay > 65535) delay = 65535;
    if (delay < 20)    delay = 20;
    m->pulse_delay = delay;

    /* 使能电机 */
    GPIO_ResetBits(m->en_port, m->en_pin);

    /* 启动定时器产生 STEP 脉冲 */
    TIM_StartPulse(m->TIMx, delay);
}

/********************** 相对移动 ************************************/
void Stepper_MoveRel(StepperMotor *m, int32_t rel)
{
    Stepper_MoveTo(m, m->position + rel);
}

/********************** 紧急停止 ************************************/
void Stepper_Stop(StepperMotor *m)
{
    TIM_StopPulse(m->TIMx);
    GPIO_ResetBits(m->step_port, m->step_pin);
    m->state   = MOTOR_IDLE;
    m->running = 0;
}

/********************** 软停止 (立即减速) ******************************/
void Stepper_SoftStop(StepperMotor *m)
{
    if (!m->running) return;
    m->state       = MOTOR_DECEL;
    m->decel_start = 0;
}

/********************** 参数设置 ************************************/
void Stepper_SetMaxSpeed(StepperMotor *m, uint32_t speed)
{
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    m->max_speed = speed;
}

void Stepper_SetAccel(StepperMotor *m, uint32_t accel)
{
    if (accel > MAX_ACCEL) accel = MAX_ACCEL;
    m->accel = accel;
}

/********************** 使能/禁用 ************************************/
void Stepper_Enable(StepperMotor *m, uint8_t enable)
{
    if (enable) {
        GPIO_ResetBits(m->en_port, m->en_pin);
    } else {
        GPIO_SetBits(m->en_port, m->en_pin);
    }
    TMC2208_Enable(m->axis_id, enable);
}

/********************** 位置/状态查询 *********************************/
int32_t  Stepper_GetPosition(StepperMotor *m) { return m->position; }
uint8_t  Stepper_IsRunning(StepperMotor *m)   { return m->running; }
void     Stepper_SetHome(StepperMotor *m)     { m->position = 0; m->target = 0; }
void     Stepper_SetDoneCallback(StepperMotor *m, void (*cb)(StepperMotor *))
{
    m->on_done = cb;
}

/********************** 定时器中断处理 (★核心★) **************************/
/*
 * 每次 TIM 更新中断 = 一个 STEP 脉冲
 *
 * ISR 执行流程:
 *   1. 清除中断标志
 *   2. STEP 上升沿 → 延时 → STEP 下降沿 (产生一个脉冲)
 *   3. 更新位置计数器
 *   4. 检查是否到达目标
 *   5. 根据加减速曲线计算下一个脉冲的 ARR 值
 *
 * AVR446 加速算法:
 *   c_{n+1} = c_n - (2*c_n)/(4*n+1)      (加速)
 *   c_{n+1} = c_n + (2*c_n)/(4*d+1)      (减速, d=剩余步数)
 *
 * ⚠ 中断上下文运行, 须精简高效
 */
void Stepper_TIM_IRQHandler(StepperMotor *m)
{
    if (!m || !m->running) return;

    /* 1—清除更新中断标志 */
    TIM_ClearITPendingBit(m->TIMx, TIM_IT_Update);

    /* 2—产生 STEP 脉冲 (~200ns 脉宽, TMC2208 要求 >100ns) */
    GPIO_SetBits(m->step_port, m->step_pin);           /* STEP ↑ */
    nop_delay(20);
    GPIO_ResetBits(m->step_port, m->step_pin);         /* STEP ↓ */

    /* 3—更新位置 */
    if (m->direction == DIR_CW)
        m->position++;
    else
        m->position--;
    m->step_count++;

    /* 4—到达目标? */
    if (m->position == m->target) {
        MoveDone(m);
        return;
    }

    /* 5—梯形加减速计算下一个间隔 */
    uint32_t n = m->step_count;
    uint32_t c = m->pulse_delay;

    switch (m->state) {

    case MOTOR_ACCEL:
        /* 加速: c -= (2*c)/(4*n+1) */
        if (n > 0 && c > 10) {
            uint32_t dec = (2 * c) / (4 * n + 1);
            if (dec < c) m->pulse_delay = c - dec;
        }
        m->current_speed = (m->pulse_delay > 0) ? STEP_TIM_CLK / m->pulse_delay : 0;

        /* 达到 max_speed 或加速段结束 → 进入匀速 */
        if (m->current_speed >= m->max_speed || n >= (uint32_t)m->decel_start) {
            m->state         = MOTOR_CONST;
            m->pulse_delay   = STEP_TIM_CLK / m->max_speed;
            m->current_speed = m->max_speed;
        }
        break;

    case MOTOR_CONST:
        m->pulse_delay   = STEP_TIM_CLK / m->max_speed;
        m->current_speed = m->max_speed;

        /* 检查是否需要开始减速 (剩余距离 ≤ 减速所需距离) */
        {
            int32_t rem = m->target - m->position;
            if (rem < 0) rem = -rem;
            uint32_t decel_needed = (m->current_speed * m->current_speed
                                    - m->min_speed * m->min_speed)
                                    / (2 * m->accel);
            if ((uint32_t)rem <= decel_needed + 1) {
                m->state = MOTOR_DECEL;
            }
        }
        break;

    case MOTOR_DECEL:
        /* 减速: c += (2*c)/(4*d+1)  (d = 剩余步数) */
        {
            int32_t rem = m->target - m->position;
            if (rem < 0) rem = -rem;

            if (rem <= 1) {
                m->pulse_delay = STEP_TIM_CLK / m->min_speed;
            } else {
                uint32_t d = (uint32_t)rem;
                if (c < 65000) {
                    uint32_t inc  = (2 * c) / (4 * d + 1);
                    uint32_t next = c + inc;
                    if (next > 65535 || next < c) next = 65535;
                    m->pulse_delay = next;
                }
            }
        }
        m->current_speed = (m->pulse_delay > 0) ? STEP_TIM_CLK / m->pulse_delay : 0;
        if (m->current_speed < m->min_speed) {
            m->current_speed = m->min_speed;
            m->pulse_delay   = STEP_TIM_CLK / m->min_speed;
        }
        break;

    default:
        break;
    }

    /* 6—限幅 & 更新 ARR */
    if (m->pulse_delay < 20)    m->pulse_delay = 20;
    if (m->pulse_delay > 65535) m->pulse_delay = 65535;
    m->TIMx->ARR = (uint16_t)m->pulse_delay;
}

/*********************************************END OF FILE**********************/
