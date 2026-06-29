#ifndef __BSP_MOTIONPLAN_H
#define __BSP_MOTIONPLAN_H

#include "stm32f10x.h"
#include "..\StepMotor\bsp_stepmotor.h"

/********************** 运动队列容量 ****************************/
#define MOVE_QUEUE_SIZE     16

/********************** 坐标结构 ****************************/
typedef struct {
    int32_t x;
    int32_t y;
} Point2D;

/********************** 命令类型 ****************************/
typedef enum {
    CMD_LINEAR = 0,    /* 直线插补 */
    CMD_X_ONLY,        /* 仅 X 轴移动 */
    CMD_Y_ONLY,        /* 仅 Y 轴移动 */
    CMD_DELAY,         /* 延时 (ms) */
    CMD_WAIT           /* 等待所有轴停止 */
} CmdType;

/********************** 运动命令 ****************************/
typedef struct {
    CmdType  type;
    int32_t  x;            /* 目标 X 或 未使用 */
    int32_t  y;            /* 目标 Y 或 未使用 */
    uint32_t speed;        /* 速度 (0=默认) */
    uint32_t accel;        /* 加速度 (0=默认) */
    uint32_t delay_ms;     /* CMD_DELAY 的毫秒数 */
} MoveCmd;

/********************** 运动规划器 ****************************/
typedef struct {
    StepperMotor *mx;       /* X 轴电机 */
    StepperMotor *my;       /* Y 轴电机 */

    /* 运动队列 (环形缓冲区) */
    MoveCmd  queue[MOVE_QUEUE_SIZE];
    uint8_t  head;
    uint8_t  tail;
    uint8_t  count;

    uint8_t  busy;         /* 当前命令执行中 */
} MotionPlanner;

/********************** API 声明 ****************************/
void MotionPlanner_Init(MotionPlanner *p, StepperMotor *mx, StepperMotor *my);

/* 阻塞式直线插补 */
void MotionPlanner_MoveLinear(MotionPlanner *p,
                              int32_t x, int32_t y,
                              uint32_t speed, uint32_t accel);

/* 队列命令 (非阻塞) */
void MotionPlanner_QueueLinear(MotionPlanner *p,
                               int32_t x, int32_t y,
                               uint32_t speed, uint32_t accel);
void MotionPlanner_QueueX(MotionPlanner *p, int32_t x,
                          uint32_t speed, uint32_t accel);
void MotionPlanner_QueueY(MotionPlanner *p, int32_t y,
                          uint32_t speed, uint32_t accel);
void MotionPlanner_QueueDelay(MotionPlanner *p, uint32_t ms);

/* 更新 & 等待 */
void MotionPlanner_Update(MotionPlanner *p);
void MotionPlanner_WaitDone(MotionPlanner *p);

/* 控制 */
void MotionPlanner_Stop(MotionPlanner *p);
void MotionPlanner_Reset(MotionPlanner *p);
Point2D MotionPlanner_GetPos(MotionPlanner *p);
uint8_t MotionPlanner_IsBusy(MotionPlanner *p);

#endif /* __BSP_MOTIONPLAN_H */
