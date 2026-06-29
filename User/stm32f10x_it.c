/**
  ******************************************************************************
  * @file    stm32f10x_it.c
  * @brief   中断服务 — TIM2(Y轴) + TIM3(X轴) + SysTick + USART1
  ******************************************************************************
  */

#include "stm32f10x_it.h"
#include ".\StepMotor\bsp_stepmotor.h"

extern StepperMotor g_motor_x;   /* X轴: PA6/PA7/PB0/PA2, TIM3 */
extern StepperMotor g_motor_y;   /* Y轴: PA0/PA1/PA4/PA9, TIM2 */

volatile uint32_t sys_tick_ms = 0;

volatile uint8_t  cmd_buf[CMD_BUF_SIZE];
volatile uint8_t  cmd_head = 0;
volatile uint8_t  cmd_tail = 0;
volatile uint8_t  cmd_new  = 0;

void SysTick_Handler(void)
{
    sys_tick_ms++;
}

/********************** TIM2 — Y轴 STEP 脉冲 **********************/
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        Stepper_TIM_IRQHandler(&g_motor_y);
    }
}

/********************** TIM3 — X轴 STEP 脉冲 **********************/
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        Stepper_TIM_IRQHandler(&g_motor_x);
    }
}

/********************** USART1 — 串口指令接收 **********************/
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t ch = (uint8_t)USART_ReceiveData(USART1);
        uint8_t next = (cmd_tail + 1) % CMD_BUF_SIZE;
        if (next != cmd_head) {
            cmd_buf[cmd_tail] = ch;
            cmd_tail = next;
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
