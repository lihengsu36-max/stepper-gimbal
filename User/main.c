/**
  ******************************************************************************
  * @file    main.c
  * @brief   双轴同步控制
  *
  *   X轴: 顺时针45° → 逆时针90° → 循环 (90°摆动范围)
  *   Y轴: 持续匀速旋转
  *   两轴同时运行
  *
  * 接线:
  *   X: STEP=PA6, DIR=PA7, EN=PB0,  UART=PA2(USART2), TIM3
  *   Y: STEP=PA0, DIR=PA1, EN=PA4,  UART=PA9(USART1), TIM2
  ******************************************************************************
  */

#include "stm32f10x.h"
#include ".\StepMotor\bsp_stepmotor.h"
#include ".\TMC2208\bsp_tmc2208.h"
#include "stm32f10x_it.h"

StepperMotor g_motor_x;
StepperMotor g_motor_y;

/* 90° = 12800 步, 180° = 25600 步 */
#define STEP_90     12800
#define STEP_180    (STEP_90 * 2)

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
    GPIO_Init(GPIOA, &gpio);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    gpio.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &gpio);
    GPIO_SetBits(GPIOB, GPIO_Pin_0);

    /* TIM2 + TIM3 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_Period            = 65535;
    tim.TIM_Prescaler         = 0;
    tim.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim.TIM_CounterMode       = TIM_CounterMode_Up;
    tim.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tim); TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_TimeBaseInit(TIM3, &tim); TIM_ClearFlag(TIM3, TIM_FLAG_Update);

    /* NVIC */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;

    nvic.NVIC_IRQChannel            = TIM2_IRQn;    /* Y轴 */
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel            = TIM3_IRQn;    /* X轴 */
    nvic.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&nvic);

    /* TMC2208 */
    TMC2208_Init();

    /* X轴: TIM3, PA6/PA7/PB0, USART2(PA2) */
    Stepper_Init(&g_motor_x, TIM3,
                 GPIOA, GPIO_Pin_6, GPIOA, GPIO_Pin_7, GPIOB, GPIO_Pin_0,
                 1, 8000, 6000);

    /* Y轴: TIM2, PA0/PA1/PA4, USART1(PA9) */
    Stepper_Init(&g_motor_y, TIM2,
                 GPIOA, GPIO_Pin_0, GPIOA, GPIO_Pin_1, GPIOA, GPIO_Pin_4,
                 0, 8000, 6000);

    Delay_ms(500);
    Stepper_Enable(&g_motor_x, 1);
    Stepper_Enable(&g_motor_y, 1);

    /* 两轴同时起步 */
    Stepper_MoveRel(&g_motor_x, STEP_90);
    Stepper_MoveRel(&g_motor_y, REV_TO_STEPS(1));
    int8_t x_sign = -1;

    /* Y轴: 持续转, X轴完成第一步后也进入独立循环 */
    while (1) {
        /* X轴: 一到头立刻回头 180° */
        if (!Stepper_IsRunning(&g_motor_x)) {
            Stepper_MoveRel(&g_motor_x, (int32_t)x_sign * STEP_180);
            x_sign = -x_sign;
        }

        /* Y轴: 持续转圈 */
        if (!Stepper_IsRunning(&g_motor_y)) {
            Stepper_MoveRel(&g_motor_y, REV_TO_STEPS(1));
        }
    }
}
