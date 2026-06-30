/**
  ******************************************************************************
  * @file    stm32f10x_it.h
  * @brief   Interrupt handler header
  ******************************************************************************
  */

#ifndef __STM32F10x_IT_H
#define __STM32F10x_IT_H

#include "stm32f10x.h"

/* SysTick 毫秒计数 (全局, 供延时使用) */
extern volatile uint32_t sys_tick_ms;

/* 串口指令接收环形缓冲区 */
#define CMD_BUF_SIZE    16
extern volatile uint8_t  cmd_buf[CMD_BUF_SIZE];
extern volatile uint8_t  cmd_head;
extern volatile uint8_t  cmd_tail;
extern volatile uint8_t  cmd_new;

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/* 外设中断 */
/* K230 视觉坐标 (USART1 接收) */
extern volatile int16_t vision_cx;
extern volatile int16_t vision_cy;
extern volatile uint8_t  vision_ready;

void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void USART3_IRQHandler(void);

#endif /* __STM32F10x_IT_H */
