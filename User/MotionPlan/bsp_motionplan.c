#include "bsp_motionplan.h"
#include <string.h>

/********************** 软件延时 (SysTick ms) ************************/
extern volatile uint32_t sys_tick_ms;
static void delay_ms(uint32_t ms)
{
    uint32_t start = sys_tick_ms;
    while ((sys_tick_ms - start) < ms);
}

/********************** 队列管理 ************************************/
static uint8_t Queue_Enqueue(MotionPlanner *p, MoveCmd *cmd)
{
    if (p->count >= MOVE_QUEUE_SIZE) return 0;    /* 队列满 */
    p->queue[p->tail] = *cmd;
    p->tail = (p->tail + 1) % MOVE_QUEUE_SIZE;
    p->count++;
    return 1;
}

static uint8_t Queue_Dequeue(MotionPlanner *p, MoveCmd *cmd)
{
    if (p->count == 0) return 0;                   /* 队列空 */
    *cmd = p->queue[p->head];
    p->head = (p->head + 1) % MOVE_QUEUE_SIZE;
    p->count--;
    return 1;
}

/********************** 直线插补执行 (速度比例法) ************************/
/*
 * 原理: 两轴同时启动同时到达 → 精确直线轨迹
 *   主导轴 (距离大) = 全速
 *   从动轴 (距离小) = 速度按比例缩放
 *
 * 例: 从 (0,0) 到 (1000,500)
 *   X 主导 (1000步), 速度 V
 *   Y 从动 (500步),  速度 V × 500/1000 = V/2
 *   → 两轴同时完成 → 直线
 */
static void ExecuteLinearMove(MotionPlanner *p,
                              int32_t x_target, int32_t y_target,
                              uint32_t speed, uint32_t accel)
{
    int32_t dx = x_target - p->mx->position;
    int32_t dy = y_target - p->my->position;

    uint32_t dx_abs = (dx >= 0) ? (uint32_t)dx : (uint32_t)(-dx);
    uint32_t dy_abs = (dy >= 0) ? (uint32_t)dy : (uint32_t)(-dy);

    uint32_t      speed_main, speed_sub, accel_main, accel_sub;
    StepperMotor *main_motor, *sub_motor;
    int32_t       main_target, sub_target;

    if (dx_abs >= dy_abs) {
        /* X 轴主导 */
        main_motor = p->mx;
        sub_motor  = p->my;
        main_target = x_target;
        sub_target  = y_target;
        speed_main  = (speed > 0) ? speed : DEFAULT_MAX_SPEED;
        accel_main  = (accel > 0) ? accel : DEFAULT_ACCEL;
        /* 从动轴按比例缩放 */
        if (dx_abs > 0) {
            speed_sub = (uint32_t)((uint64_t)speed_main * dy_abs / dx_abs);
            accel_sub = (uint32_t)((uint64_t)accel_main * dy_abs / dx_abs);
        } else {
            speed_sub = speed_main;
            accel_sub = accel_main;
        }
    } else {
        /* Y 轴主导 */
        main_motor = p->my;
        sub_motor  = p->mx;
        main_target = y_target;
        sub_target  = x_target;
        speed_main  = (speed > 0) ? speed : DEFAULT_MAX_SPEED;
        accel_main  = (accel > 0) ? accel : DEFAULT_ACCEL;
        if (dy_abs > 0) {
            speed_sub = (uint32_t)((uint64_t)speed_main * dx_abs / dy_abs);
            accel_sub = (uint32_t)((uint64_t)accel_main * dx_abs / dy_abs);
        } else {
            speed_sub = speed_main;
            accel_sub = accel_main;
        }
    }

    /* 从动轴速度下限保护 */
    if (speed_sub < MIN_SPEED) speed_sub = MIN_SPEED;
    if (accel_sub < accel_main / 10) accel_sub = accel_main / 10;

    /* 设置两轴参数 */
    main_motor->max_speed = speed_main;
    main_motor->accel     = accel_main;
    sub_motor->max_speed  = speed_sub;
    sub_motor->accel      = accel_sub;

    /* 同时启动两轴 */
    Stepper_MoveTo(main_motor, main_target);
    Stepper_MoveTo(sub_motor,  sub_target);

    p->busy = 1;
}

/********************** 初始化 ************************************/
void MotionPlanner_Init(MotionPlanner *p, StepperMotor *mx, StepperMotor *my)
{
    memset(p, 0, sizeof(MotionPlanner));
    p->mx   = mx;
    p->my   = my;
    p->busy = 0;
}

/********************** 阻塞式直线插补 ******************************/
void MotionPlanner_MoveLinear(MotionPlanner *p,
                              int32_t x, int32_t y,
                              uint32_t speed, uint32_t accel)
{
    ExecuteLinearMove(p, x, y, speed, accel);
    /* 等待两轴都停止 */
    while (p->mx->running || p->my->running) {
        /* 实际项目中可加入超时检测 */
    }
    p->busy = 0;
}

/********************** 队列: 直线 ********************************/
void MotionPlanner_QueueLinear(MotionPlanner *p,
                               int32_t x, int32_t y,
                               uint32_t speed, uint32_t accel)
{
    MoveCmd cmd;
    cmd.type     = CMD_LINEAR;
    cmd.x        = x;
    cmd.y        = y;
    cmd.speed    = speed;
    cmd.accel    = accel;
    cmd.delay_ms = 0;
    Queue_Enqueue(p, &cmd);
}

/********************** 队列: 单轴 X ********************************/
void MotionPlanner_QueueX(MotionPlanner *p, int32_t x,
                          uint32_t speed, uint32_t accel)
{
    MoveCmd cmd;
    cmd.type     = CMD_X_ONLY;
    cmd.x        = x;
    cmd.y        = 0;
    cmd.speed    = speed;
    cmd.accel    = accel;
    cmd.delay_ms = 0;
    Queue_Enqueue(p, &cmd);
}

/********************** 队列: 单轴 Y ********************************/
void MotionPlanner_QueueY(MotionPlanner *p, int32_t y,
                          uint32_t speed, uint32_t accel)
{
    MoveCmd cmd;
    cmd.type     = CMD_Y_ONLY;
    cmd.x        = 0;
    cmd.y        = y;
    cmd.speed    = speed;
    cmd.accel    = accel;
    cmd.delay_ms = 0;
    Queue_Enqueue(p, &cmd);
}

/********************** 队列: 延时 ********************************/
void MotionPlanner_QueueDelay(MotionPlanner *p, uint32_t ms)
{
    MoveCmd cmd;
    cmd.type     = CMD_DELAY;
    cmd.delay_ms = ms;
    cmd.speed    = 0;
    cmd.accel    = 0;
    Queue_Enqueue(p, &cmd);
}

/********************** 等待队列完成 ******************************/
void MotionPlanner_WaitDone(MotionPlanner *p)
{
    while (p->count > 0 || p->mx->running || p->my->running || p->busy) {
        MotionPlanner_Update(p);
    }
}

/********************** 主循环更新 (驱动队列) *************************/
void MotionPlanner_Update(MotionPlanner *p)
{
    /* 当前命令还在执行中 → 等待 */
    if (p->busy) {
        if (p->mx->running || p->my->running) {
            return;
        }
        p->busy = 0;
    }

    /* 空闲 & 队列非空 → 取下一命令 */
    if (!p->busy && p->count > 0) {
        MoveCmd cmd;
        if (!Queue_Dequeue(p, &cmd)) return;

        switch (cmd.type) {

        case CMD_LINEAR:
            ExecuteLinearMove(p, cmd.x, cmd.y, cmd.speed, cmd.accel);
            break;

        case CMD_X_ONLY:
            p->mx->max_speed = (cmd.speed > 0) ? cmd.speed : DEFAULT_MAX_SPEED;
            p->mx->accel     = (cmd.accel > 0) ? cmd.accel : DEFAULT_ACCEL;
            Stepper_MoveTo(p->mx, cmd.x);
            p->busy = 1;
            break;

        case CMD_Y_ONLY:
            p->my->max_speed = (cmd.speed > 0) ? cmd.speed : DEFAULT_MAX_SPEED;
            p->my->accel     = (cmd.accel > 0) ? cmd.accel : DEFAULT_ACCEL;
            Stepper_MoveTo(p->my, cmd.y);
            p->busy = 1;
            break;

        case CMD_DELAY:
            delay_ms(cmd.delay_ms);
            break;

        case CMD_WAIT:
            while (p->mx->running || p->my->running) {}
            break;

        default:
            break;
        }
    }
}

/********************** 紧急停止 ************************************/
void MotionPlanner_Stop(MotionPlanner *p)
{
    Stepper_Stop(p->mx);
    Stepper_Stop(p->my);
    p->busy  = 0;
    p->head  = 0;
    p->tail  = 0;
    p->count = 0;
}

/********************** 重置 ************************************/
void MotionPlanner_Reset(MotionPlanner *p)
{
    MotionPlanner_Stop(p);
}

/********************** 获取当前位置 ********************************/
Point2D MotionPlanner_GetPos(MotionPlanner *p)
{
    Point2D pos;
    pos.x = Stepper_GetPosition(p->mx);
    pos.y = Stepper_GetPosition(p->my);
    return pos;
}

/********************** 忙碌状态 ************************************/
uint8_t MotionPlanner_IsBusy(MotionPlanner *p)
{
    return (p->busy || p->mx->running || p->my->running || p->count > 0);
}

/*********************************************END OF FILE**********************/
