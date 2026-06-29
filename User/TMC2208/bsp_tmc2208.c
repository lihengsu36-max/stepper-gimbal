#include "bsp_tmc2208.h"
#include <stddef.h>

/********************** 内部变量 ****************************/
/* 每轴 TMC2208 的配置缓存 (用于读-改-写) */
static uint32_t gconf_cache[2];
static uint32_t chopconf_cache[2];
static uint32_t pwmconf_cache[2];

/********************** CRC 计算 (多项式 0x07) **********************/
uint8_t TMC2208_CalcCRC(uint8_t *buf, uint8_t len)
{
    /* TMC2208 CRC-8: 多项式 0x07, LSB 优先 */
    uint8_t crc = 0;
    uint8_t i, j;
    for (i = 0; i < len; i++) {
        uint8_t byte = buf[i];
        for (j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (byte & 0x01))
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc = (uint8_t)(crc << 1);
            byte >>= 1;            /* LSB 优先: 右移 */
        }
    }
    return crc;
}

/********************** USART 发送单字节 ****************************/
static void USART_SendByte(USART_TypeDef *USARTx, uint8_t data)
{
    USART_SendData(USARTx, data);
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
}

/********************** USART 发送完成等待 ****************************/
static void USART_WaitTX(USART_TypeDef *USARTx)
{
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
}

/********************** 获取 axis 对应的 USART ************************/
static USART_TypeDef *TMC2208_GetUSART(uint8_t axis)
{
    if (axis == 0) return TMC2208_X_USART;
    if (axis == 1) return TMC2208_Y_USART;
    return NULL;
}

/********************** 软件延时 (SysTick ms) ************************/
extern volatile uint32_t sys_tick_ms;
static void delay_ms(uint32_t ms)
{
    uint32_t start = sys_tick_ms;
    while ((sys_tick_ms - start) < ms);
}

/********************** 写 TMC2208 寄存器 ****************************/
/*
 * 数据帧格式 (8 字节):
 *   [0] Sync    = 0x05
 *   [1] Address = 0x00 (8-bit 从机地址)
 *   [2] Reg     = 寄存器地址 | 0x80 (bit7=写)
 *   [3] Data[31:24]
 *   [4] Data[23:16]
 *   [5] Data[15:8]
 *   [6] Data[7:0]
 *   [7] CRC     = 前 7 字节的 CRC-8
 */
void TMC2208_WriteReg(uint8_t axis, uint8_t reg, uint32_t data)
{
    uint8_t buf[8];
    USART_TypeDef *USARTx = TMC2208_GetUSART(axis);
    if (!USARTx) return;

    buf[0] = TMC2208_SYNC;
    buf[1] = TMC2208_SLAVE_ADDR;
    buf[2] = reg | TMC2208_WRITE;
    buf[3] = (uint8_t)(data >> 24);
    buf[4] = (uint8_t)(data >> 16);
    buf[5] = (uint8_t)(data >> 8);
    buf[6] = (uint8_t)(data);
    buf[7] = TMC2208_CalcCRC(buf, 7);

    for (uint8_t i = 0; i < 8; i++) {
        USART_SendByte(USARTx, buf[i]);
    }
    USART_WaitTX(USARTx);
}

/********************** USART 初始化 (X/Y 两路) **********************/
static void TMC2208_USART_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* ===== USART1 (TX → Y轴 TMC2208) ===== */
    TMC2208_X_TX_CLK_CMD(TMC2208_X_TX_CLK, ENABLE);
    TMC2208_X_USART_CLK_CMD(TMC2208_X_USART_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = TMC2208_X_TX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TMC2208_X_TX_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = TMC2208_BAUDRATE;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx;
    USART_Init(TMC2208_X_USART, &USART_InitStructure);
    USART_Cmd(TMC2208_X_USART, ENABLE);

    /* ===== USART2 (TX→X轴TMC2208) ===== */
    TMC2208_Y_TX_CLK_CMD(TMC2208_Y_TX_CLK, ENABLE);
    TMC2208_Y_USART_CLK_CMD(TMC2208_Y_USART_CLK, ENABLE);

    /* PA2 = TX (必须重设 Mode/Speed, 之前被 PA10 覆盖了) */
    GPIO_InitStructure.GPIO_Pin   = TMC2208_Y_TX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TMC2208_Y_TX_PORT, &GPIO_InitStructure);

    USART_Init(TMC2208_Y_USART, &USART_InitStructure);
    USART_Cmd(TMC2208_Y_USART, ENABLE);
}

/********************** TMC2208 初始化 (两路) ***************************/
/*
 * 配置序列:
 *   1. GCONF — PDN 禁用 + 微步寄存器选择
 *   2. IHOLD_IRUN — 运行/保持电流
 *   3. TPOWERDOWN — 掉电延时 (~2s)
 *   4. TPWMTHRS — StealthChop 速度上限 (0=全速域静音)
 *   5. CHOPCONF — 256 微步细分 + 插值
 *   6. PWMCONF — StealthChop2 静音 PWM 参数
 */
void TMC2208_Init(void)
{
    /* 初始化两路 USART (PA9→TMC2208 X, PA2→TMC2208 Y) */
    TMC2208_USART_Config();

    /* 对 X/Y 两个轴依次配置 */
    for (uint8_t axis = 0; axis < 2; axis++) {

        /* 1—GCONF: 使能 UART 控制微步 */
        gconf_cache[axis] = GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SEL;
        TMC2208_WriteReg(axis, REG_GCONF, gconf_cache[axis]);
        delay_ms(10);   /* 加大延时, 确保 TMC2208 处理完 */

        /* 2—IHOLD_IRUN: 电流 */
        uint32_t v = ((uint32_t)DEFAULT_IHOLD      << IHOLD_SHIFT)
                   | ((uint32_t)DEFAULT_IRUN       << IRUN_SHIFT)
                   | ((uint32_t)DEFAULT_IHOLDDELAY << IHOLDDELAY_SHIFT);
        TMC2208_WriteReg(axis, REG_IHOLD_IRUN, v);
        delay_ms(5);

        /* 3—TPOWERDOWN */
        TMC2208_WriteReg(axis, REG_TPOWERDOWN, 10);
        delay_ms(1);

        /* 4—TPWMTHRS: 0=全部速度 StealthChop 静音 */
        TMC2208_WriteReg(axis, REG_TPWMTHRS, 0);
        delay_ms(1);

        /* 5—CHOPCONF: 256 微步 ★ */
        chopconf_cache[axis] = ((uint32_t)5  << CHOPCONF_TOFF_SHIFT)
                             | ((uint32_t)5  << CHOPCONF_HSTRT_SHIFT)
                             | ((uint32_t)0  << CHOPCONF_HEND_SHIFT)
                             | ((uint32_t)1  << CHOPCONF_TBL_SHIFT)
                             | CHOPCONF_VSENSE
                             | ((uint32_t)MRES_256 << CHOPCONF_MRES_SHIFT)
                             | CHOPCONF_INTPOL;
        TMC2208_WriteReg(axis, REG_CHOPCONF, chopconf_cache[axis]);
        delay_ms(10);   /* 确保 CHOPCONF 写入完成 */

        /* 6—PWMCONF: StealthChop2 */
        pwmconf_cache[axis] = ((uint32_t)36 << PWMCONF_PWM_OFS_SHIFT)
                            | ((uint32_t)14 << PWMCONF_PWM_GRAD_SHIFT)
                            | ((uint32_t)PWM_FREQ_36KHZ << PWMCONF_PWM_FREQ_SHIFT)
                            | PWMCONF_PWM_AUTOSCALE
                            | PWMCONF_PWM_AUTOGRAD;
        TMC2208_WriteReg(axis, REG_PWMCONF, pwmconf_cache[axis]);
        delay_ms(5);
    }
}

/********************** 设置微步细分 **********************************/
void TMC2208_SetMicrosteps(uint8_t axis, uint8_t mres)
{
    chopconf_cache[axis] &= ~(0x0FUL << CHOPCONF_MRES_SHIFT);
    chopconf_cache[axis] |= ((uint32_t)mres << CHOPCONF_MRES_SHIFT);
    /* ≤64 细分时使能插值，提高平滑度 */
    if (mres <= MRES_64)
        chopconf_cache[axis] |= CHOPCONF_INTPOL;
    else
        chopconf_cache[axis] &= ~CHOPCONF_INTPOL;
    TMC2208_WriteReg(axis, REG_CHOPCONF, chopconf_cache[axis]);
}

/********************** 设置电流 *************************************/
void TMC2208_SetCurrent(uint8_t axis, uint8_t iRun, uint8_t iHold)
{
    uint32_t v = ((uint32_t)iHold << IHOLD_SHIFT)
               | ((uint32_t)iRun  << IRUN_SHIFT)
               | ((uint32_t)DEFAULT_IHOLDDELAY << IHOLDDELAY_SHIFT);
    TMC2208_WriteReg(axis, REG_IHOLD_IRUN, v);
}

/********************** 使能/禁用电机 (TOFF 控制) *************************/
void TMC2208_Enable(uint8_t axis, uint8_t enable)
{
    if (enable) {
        chopconf_cache[axis] |= (5 << CHOPCONF_TOFF_SHIFT);
    } else {
        chopconf_cache[axis] &= ~(0x0FUL << CHOPCONF_TOFF_SHIFT);
    }
    TMC2208_WriteReg(axis, REG_CHOPCONF, chopconf_cache[axis]);
}

/*********************************************END OF FILE**********************/
