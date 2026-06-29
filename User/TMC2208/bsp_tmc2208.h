#ifndef __BSP_TMC2208_H
#define __BSP_TMC2208_H

#include "stm32f10x.h"

/**********************TMC2208 寄存器定义****************************/

/* TMC2208 单线 UART 通信参数 */
#define TMC2208_SYNC            0x05    /* 同步字节 */
#define TMC2208_WRITE           0x80    /* 写标志位 bit7=1 */
#define TMC2208_SLAVE_ADDR      0x00    /* UART 地址 (独立 USART 模式下均用 0x00) */
#define TMC2208_BAUDRATE        115200  /* 波特率 115200 */

/* 寄存器地址 (7-bit) */
#define REG_GCONF           0x00
#define REG_IHOLD_IRUN      0x10
#define REG_TPOWERDOWN      0x11
#define REG_TPWMTHRS        0x13
#define REG_CHOPCONF        0x6C
#define REG_PWMCONF         0x70

/* ---- GCONF 位定义 ---- */
#define GCONF_PDN_DISABLE     (1 << 6)   /* PDN 禁用 (UART 模式必须置1) */
#define GCONF_MSTEP_REG_SEL   (1 << 7)   /* 微步由寄存器控制 (必须置1) */

/* ---- IHOLD_IRUN 位定义 ---- */
/* 电流计算: Irms = (value/31) * (325mV/Rsense)   (Rsense 典型 110mΩ) */
/*          value=16 → 0.47A   value=20 → 0.59A   value=24 → 0.71A   */
#define IHOLD_SHIFT         0
#define IRUN_SHIFT          8
#define IHOLDDELAY_SHIFT    16

/* ---- CHOPCONF 位定义 ---- */
#define CHOPCONF_TOFF_SHIFT    0
#define CHOPCONF_HSTRT_SHIFT   4
#define CHOPCONF_HEND_SHIFT    7
#define CHOPCONF_TBL_SHIFT     15
#define CHOPCONF_VSENSE        (1 << 17)   /* 低灵敏度(高电流用) */
#define CHOPCONF_MRES_SHIFT    24
#define CHOPCONF_INTPOL        (1 << 28)   /* 256微步插值使能 */

/* 微步分辨率 MRES 值 */
#define MRES_256   0   /* 1/256 微步 ★ 最高精度 */
#define MRES_128   1   /* 1/128 微步 */
#define MRES_64    2   /* 1/64  微步 */
#define MRES_32    3   /* 1/32  微步 */
#define MRES_16    4   /* 1/16  微步 */
#define MRES_8     5   /* 1/8   微步 */
#define MRES_4     6   /* 1/4   微步 */
#define MRES_2     7   /* 1/2   微步 */
#define MRES_FULL  8   /* 全步 */

/* 每圈微步数 (200 全步电机 @ 256 细分 = 51200) */
#define STEPS_PER_REV       51200

/* ---- PWMCONF 位定义 (StealthChop2 静音驱动) ---- */
#define PWMCONF_PWM_OFS_SHIFT      0
#define PWMCONF_PWM_GRAD_SHIFT     8
#define PWMCONF_PWM_FREQ_SHIFT     16
#define PWMCONF_PWM_AUTOSCALE      (1 << 18)
#define PWMCONF_PWM_AUTOGRAD       (1 << 19)

#define PWM_FREQ_36KHZ   2

/********************** TMC2208 USART 选择 ****************************/
/* 每个 TMC2208 接一个独立 USART:
   Motor X → USART1 (PA9 TX), Motor Y → USART2 (PA2 TX) */
#define TMC2208_USART1_X   1
#define TMC2208_USART2_Y   1

/* ---- Motor X 的 USART (USART1) ---- */
#if TMC2208_USART1_X
#define  TMC2208_X_USART               USART1
#define  TMC2208_X_USART_CLK           RCC_APB2Periph_USART1
#define  TMC2208_X_USART_CLK_CMD       RCC_APB2PeriphClockCmd
#define  TMC2208_X_USART_IRQ           USART1_IRQn
#define  TMC2208_X_TX_PORT             GPIOA
#define  TMC2208_X_TX_PIN              GPIO_Pin_9
#define  TMC2208_X_RX_PORT             GPIOA
#define  TMC2208_X_RX_PIN              GPIO_Pin_10
#define  TMC2208_X_TX_CLK             (RCC_APB2Periph_GPIOA)
#define  TMC2208_X_TX_CLK_CMD          RCC_APB2PeriphClockCmd
#endif

/* ---- Motor Y 的 USART (USART2) ---- */
#if TMC2208_USART2_Y
#define  TMC2208_Y_USART               USART2
#define  TMC2208_Y_USART_CLK           RCC_APB1Periph_USART2
#define  TMC2208_Y_USART_CLK_CMD       RCC_APB1PeriphClockCmd
#define  TMC2208_Y_TX_PORT             GPIOA
#define  TMC2208_Y_TX_PIN              GPIO_Pin_2
#define  TMC2208_Y_TX_CLK             (RCC_APB2Periph_GPIOA)
#define  TMC2208_Y_TX_CLK_CMD          RCC_APB2PeriphClockCmd
#endif

/********************** 电机默认参数 ****************************/
/* 42步进电机 + 256 细分 */
#define MOTOR_FULL_STEPS        200         /* 全步/圈 (1.8°步距角) */
#define MOTOR_MICROSTEPS        STEPS_PER_REV /* 微步/圈 = 51200 */

/* TMC2208 电流默认值 (iRun/iHold 范围 0~31) */
#define DEFAULT_IRUN            25          /* 运行电流 ~0.74A rms */
#define DEFAULT_IHOLD           13          /* 保持电流 ~0.38A rms (50%) */
#define DEFAULT_IHOLDDELAY      4           /* 保持延时 x2^18 clk */

/********************** 宏: 单位换算 ****************************/
#define DEG_TO_STEPS(d)    ((int32_t)((float)(d) * STEPS_PER_REV / 360.0f))
#define REV_TO_STEPS(r)    ((int32_t)((r) * STEPS_PER_REV))

/********************** API 声明 ****************************/
uint8_t TMC2208_CalcCRC(uint8_t *buf, uint8_t len);
void TMC2208_WriteReg(uint8_t axis, uint8_t reg, uint32_t data);
void TMC2208_Init(void);
void TMC2208_SetMicrosteps(uint8_t axis, uint8_t mres);
void TMC2208_SetCurrent(uint8_t axis, uint8_t iRun, uint8_t iHold);
void TMC2208_Enable(uint8_t axis, uint8_t enable);

#endif /* __BSP_TMC2208_H */
