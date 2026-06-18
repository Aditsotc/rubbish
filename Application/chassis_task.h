/**
 ******************************************************************************
 * @file    chassis_task.h
 * @brief   Omnidirectional (Mecanum) chassis control — RC driving
 ******************************************************************************
 */
#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include <stdint.h>
#include "motor.h"
#include "controller.h"

/* Chassis geometry -----------------------------------------------*/
#define wheel_radius       76.00f       /* 轮子半径 (mm) */
#define WHEEL_OPPOSITE     0.2675f      /* 轮子到中心的距离 (m) */
#define SQRT2              1.4142135f
#define VELOCITY_RATIO     3.0f
#define CHASSIS_WHEEL_REDUCTION_RATIO (3591.0f / 187.0f)  /* 电机减速比 */

/* Task period (ms) ------------------------------------------------*/
#define CHASSIS_TASK_PERIOD 2

/* RC stick to velocity scaling ------------------------------------*/
#define RC_TO_VELOCITY      0.004f       /* RC [-660,660] → m/s */
#define RC_TO_ROTATION      0.005f       /* RC [-660,660] → rad/s, max≈3.3 */

/* Motor indices ---------------------------------------------------*/
enum {
    FR = 0,  /* 右前 */
    FL = 1,  /* 左前 */
    HL = 2,  /* 左后 */
    HR = 3   /* 右后 */
};

/* Chassis modes (used by bsp_CAN.c) --------------------------------*/
enum {
    Follow_Mode = 0,
    Spinning_Mode,
    Side_Mode,
    Silence_Mode,
};

/* Chassis state ---------------------------------------------------*/
typedef struct {
    float Vx, Vy, Vr;              /* 目标速度: X(m/s), Y(m/s), 旋转(rad/s) */
    float V1, V2, V3, V4;          /* 四个轮子的目标转速 (rad/s) */

    float VxTransfer, VyTransfer;  /* 坐标系转换后的速度 */
    float VelocityRatio;           /* 速度比例系数 */
    float WheelRadius;
    float WheelReductionRatio;

    Motor_t ChassisMotor[4];       /* 四个电机对象 */

    uint8_t Mode;
    uint8_t status;

    /* Used by bsp_CAN.c navigation dispatch (no-op in RC mode) */
    float nowFaceX;
    float nowFaceY;
    float TargetVelocityXtemp;
    float TargetVelocityYtemp;
} Chassis_t;

extern Chassis_t Chassis;
extern Motor_t ChassisMotor[4];
extern float limited_current[4];

/* API -------------------------------------------------------------*/
void Chassis_Init(void);
void Chassis_Control(void);

#endif /* _CHASSIS_TASK_H */
