/**
  ******************************************************************************
  * @file    main.c
  * @brief   视觉追踪云台 — K230颜色识别 + STM32步进电机控制
  *
  *   K230 → USART1_RX(PA10) → 发送目标坐标 "CX,CY\n"
  *   STM32 解析坐标, 比例控制两轴步进电机追踪目标
  *
  *   X轴(俯仰/Tilt): ±90° 范围, 响应垂直误差
  *   Y轴(水平/Pan):  连续旋转, 响应水平误差
  *
  * 接线:
  *   Tilt(X): STEP=PA6, DIR=PA7, EN=PB0,  UART=PA2(USART2), TIM3
  *   Pan(Y):  STEP=PA0, DIR=PA1, EN=PA4,  UART=PA9(USART1), TIM2
  *   K230:    UART_TX → PA10(USART1_RX),  GND → STM32 GND
  ******************************************************************************
  */

#include "stm32f10x.h"
#include ".\StepMotor\bsp_stepmotor.h"
#include ".\TMC2208\bsp_tmc2208.h"
#include "stm32f10x_it.h"

StepperMotor g_pan;    /* X轴: 水平 (TIM3, PA6/PA7/PB0) */
StepperMotor g_tilt;   /* Y轴: 俯仰 (TIM2, PA0/PA1/PA4) */

static void Delay_ms(uint32_t ms)
{
    uint32_t start = sys_tick_ms;
    while ((sys_tick_ms - start) < ms);
}

/* 画面中心 (640x480 的一半) */
#define FRAME_CX     320
#define FRAME_CY     240

/* 追踪参数 (快速响应) */
#define DEAD_ZONE     20     /* 死区 (像素) */
#define MAX_STEP      400    /* 单帧最大步数 */
#define TRACK_GAIN    8      /* 步数 = 误差 × GAIN */

/* 俯仰轴软限位: ±90° = ±12800 步 */
#define TILT_LIMIT    12800

/* K230 发来的最新坐标 */
extern volatile int16_t vision_cx;
extern volatile int16_t vision_cy;
extern volatile uint8_t  vision_ready;

int main(void)
{
    /* 系统时钟 72MHz */
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    if (RCC_WaitForHSEStartUp() == SUCCESS) {
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        FLASH_SetLatency(FLASH_Latency_2);
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLK1Config(RCC_HCLK_Div2);
        RCC_PCLK2Config(RCC_HCLK_Div1);
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        RCC_PLLCmd(ENABLE);
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while (RCC_GetSYSCLKSource() != 0x08);
    }

    SysTick_Config(SystemCoreClock / 1000);

    /* GPIO */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);
    GPIO_ResetBits(GPIOA, gpio.GPIO_Pin);

    gpio.GPIO_Speed = GPIO_Speed_10MHz;
    gpio.GPIO_Pin   = GPIO_Pin_4;
    GPIO_Init(GPIOA, &gpio);  GPIO_SetBits(GPIOA, GPIO_Pin_4);
    gpio.GPIO_Pin   = GPIO_Pin_0;
    GPIO_Init(GPIOB, &gpio);  GPIO_SetBits(GPIOB, GPIO_Pin_0);

    /* ---- USART3 RX (PB11) ← K230 IO4 TX ---- */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    gpio.GPIO_Pin  = GPIO_Pin_11;              /* PB11 = USART3_RX */
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio);

    USART_InitTypeDef uart3;
    uart3.USART_BaudRate            = 115200;
    uart3.USART_WordLength          = USART_WordLength_8b;
    uart3.USART_StopBits            = USART_StopBits_1;
    uart3.USART_Parity              = USART_Parity_No;
    uart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart3.USART_Mode                = USART_Mode_Rx;
    USART_Init(USART3, &uart3);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);

    /* TIM2(Pan/Y) + TIM3(Tilt/X) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_Period = 65535; tim.TIM_Prescaler = 0;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tim); TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_TimeBaseInit(TIM3, &tim); TIM_ClearFlag(TIM3, TIM_FLAG_Update);

    /* NVIC */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannel            = TIM2_IRQn;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel            = TIM3_IRQn;
    nvic.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel                   = USART3_IRQn;   /* K230 via PB11 */
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&nvic);

    /* TMC2208 */
    TMC2208_Init();

    /* X轴=水平: TIM3, PA6/PA7/PB0, USART2(PA2) */
    Stepper_Init(&g_pan, TIM3,
                 GPIOA, GPIO_Pin_6, GPIOA, GPIO_Pin_7, GPIOB, GPIO_Pin_0,
                 1, 10000, 8000);

    /* Y轴=俯仰: TIM2, PA0/PA1/PA4, USART1(PA9) */
    Stepper_Init(&g_tilt, TIM2,
                 GPIOA, GPIO_Pin_0, GPIOA, GPIO_Pin_1, GPIOA, GPIO_Pin_4,
                 0, 10000, 8000);

    Delay_ms(500);
    Stepper_Enable(&g_pan, 1);
    Stepper_Enable(&g_tilt, 1);

    /* 主循环: 追踪 + 丢目标沿原方向继续找 */
    uint32_t last_data = sys_tick_ms;
    int32_t  last_pan  = 0;       /* 最后水平方向 */
    int32_t  last_tilt = 0;       /* 最后俯仰方向 */

    while (1) {
        if (vision_ready) {
            vision_ready = 0;
            last_data = sys_tick_ms;

            int16_t ex = (int16_t)vision_cx - FRAME_CX;
            int16_t ey = (int16_t)vision_cy - FRAME_CY;

            /* X轴=水平 */
            if (ex > DEAD_ZONE || ex < -DEAD_ZONE) {
                int32_t s = (int32_t)ex * TRACK_GAIN;
                if (s >  MAX_STEP) s =  MAX_STEP;
                if (s < -MAX_STEP) s = -MAX_STEP;
                last_pan = (s > 0) ? 1 : -1;
                if (!Stepper_IsRunning(&g_pan))
                    Stepper_MoveRel(&g_pan, s);
            }

            /* Y轴=俯仰 (反向, 带限位) */
            if (ey > DEAD_ZONE || ey < -DEAD_ZONE) {
                int32_t s = (int32_t)(-ey) * TRACK_GAIN;
                if (s >  MAX_STEP) s =  MAX_STEP;
                if (s < -MAX_STEP) s = -MAX_STEP;
                int32_t np = g_tilt.position + s;
                if (np >  TILT_LIMIT) s = TILT_LIMIT - g_tilt.position;
                if (np < -TILT_LIMIT) s = -TILT_LIMIT - g_tilt.position;
                last_tilt = (s > 0) ? 1 : -1;
                if (!Stepper_IsRunning(&g_tilt) && s != 0)
                    Stepper_MoveRel(&g_tilt, s);
            }
        }

        /* 丢目标 > 300ms → 沿最后方向继续转着找 */
        if ((sys_tick_ms - last_data) > 300) {
            if (!Stepper_IsRunning(&g_pan) && last_pan != 0)
                Stepper_MoveRel(&g_pan, last_pan * 200);
            if (!Stepper_IsRunning(&g_tilt) && last_tilt != 0) {
                int32_t np = g_tilt.position + last_tilt * 200;
                if (np <= TILT_LIMIT && np >= -TILT_LIMIT)
                    Stepper_MoveRel(&g_tilt, last_tilt * 200);
                else
                    last_tilt = -last_tilt;   /* 碰限位回头找 */
            }
        }
    }
}
