/**
  ******************************************************************************
  * @file    stm32f10x_it.c
  * @brief   中断服务 — TIM2(Y轴) + TIM3(X轴) + SysTick + USART1
  ******************************************************************************
  */

#include "stm32f10x_it.h"
#include ".\StepMotor\bsp_stepmotor.h"
#include <stdio.h>

extern StepperMotor g_pan;       /* X轴: 水平, TIM3 */
extern StepperMotor g_tilt;      /* Y轴: 俯仰, TIM2 */

volatile uint32_t sys_tick_ms = 0;

volatile uint8_t  cmd_buf[CMD_BUF_SIZE];
volatile uint8_t  cmd_head = 0;
volatile uint8_t  cmd_tail = 0;
volatile uint8_t  cmd_new  = 0;

void SysTick_Handler(void)
{
    sys_tick_ms++;
}

/********************** TIM2 — Tilt(Y轴俯仰) STEP 脉冲 ************/
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        Stepper_TIM_IRQHandler(&g_tilt);
    }
}

/********************** TIM3 — Pan(X轴水平) STEP 脉冲 **************/
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        Stepper_TIM_IRQHandler(&g_pan);
    }
}

/********************** USART3 — K230 视觉坐标接收 *********************/
/* 数据格式: "CX,CY\n"  例: "320,240\n" */
/* 接线: K230 IO4(TX) → STM32 PB11(USART3_RX),  GND↔GND */
#define VISION_BUF_SIZE  32
volatile uint8_t  vision_buf[VISION_BUF_SIZE];
volatile uint8_t  vision_idx = 0;
volatile uint8_t  vision_ready = 0;
volatile int16_t  vision_cx = 320;
volatile int16_t  vision_cy = 240;

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        uint8_t ch = (uint8_t)USART_ReceiveData(USART3);

        /* 清除可能的错误标志, 防止卡死 */
        if (USART_GetFlagStatus(USART3, USART_FLAG_ORE)) {
            (void)USART_ReceiveData(USART3);
        }

        if (ch == '\n') {
            vision_buf[vision_idx] = 0;
            vision_idx = 0;
            int16_t cx = 320, cy = 240;
            sscanf((const char*)vision_buf, "%hd,%hd", &cx, &cy);
            vision_cx = cx;
            vision_cy = cy;
            vision_ready = 1;
        } else if ((ch >= '0' && ch <= '9' || ch == ',' || ch == '-')
                   && vision_idx < VISION_BUF_SIZE - 1) {
            vision_buf[vision_idx++] = ch;
        }
    }
}

/********************** 系统异常 ****************************/
void NMI_Handler(void)                 {}
void HardFault_Handler(void)           { while(1); }
void MemManage_Handler(void)           { while(1); }
void BusFault_Handler(void)            { while(1); }
void UsageFault_Handler(void)          { while(1); }
void SVC_Handler(void)                 {}
void DebugMon_Handler(void)            {}
void PendSV_Handler(void)              {}

/*********************************************END OF FILE**********************/
