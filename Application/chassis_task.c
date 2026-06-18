/**
 ******************************************************************************
 * @file    chassis_task.c
 * @brief   Omnidirectional chassis control — RC → Omni-wheel kinematics →
 *          PID speed loop → CAN motor current
 ******************************************************************************
 */
#include "chassis_task.h"
#include "motor.h"
#include "remote_control.h"
#include "detect_task.h"
#include "bsp_dwt.h"
#include "bsp_CAN.h"
#include "user_lib.h"

/* External CAN handle from can.c */
extern CAN_HandleTypeDef hcan1;

/* Global instances */
Chassis_t Chassis = {0};
Motor_t ChassisMotor[4] = {0};
float limited_current[4] = {0};

/* Local state */
static uint32_t Chassis_DWT_Count = 0;
static float dt = 0.001f;

/* ========================= Chassis Init ================================== */

void Chassis_Init(void)
{
    Chassis.WheelRadius = wheel_radius;
    Chassis.WheelReductionRatio = CHASSIS_WHEEL_REDUCTION_RATIO;
    Chassis.VelocityRatio = VELOCITY_RATIO;
    Chassis.Mode = 0;

    /* Init motor speed-loop PID for all 4 wheels
       PID_Init(pid, max_out, integral_limit, deadband,
                kp, ki, kd, A, B, output_lpf_rc, derivative_lpf_rc,
                ols_order, improve) */
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_Init(&ChassisMotor[i].PID_Velocity,
                 16384.0f,    /* Max output = C620 current range */
                 2000.0f,     /* Integral limit */
                 0.0f,        /* Dead band */
                 50.0f,       /* Kp */
                 0.1f,        /* Ki */
                 0.0f,        /* Kd */
                 65.0f,       /* A (changing integration rate) */
                 15.0f,       /* B (changing integration rate) */
                 0.005f,      /* Output LPF RC */
                 0.0f,        /* Derivative LPF RC */
                 1,           /* OLS order */
                 Integral_Limit | OutputFilter | ChangingIntegrationRate);

        ChassisMotor[i].Max_Out = 16384.0f;
        ChassisMotor[i].ReductionRatio = CHASSIS_WHEEL_REDUCTION_RATIO;
        ChassisMotor[i].Direction = 0;
    }
}

/* ======================= Chassis Control Loop ============================ */

void Chassis_Control(void)
{
    /* --- Delta time -------------------------------------------- */
    dt = DWT_GetDeltaT(&Chassis_DWT_Count);
    if (dt < 0.0001f || dt > 0.1f) dt = 0.001f;  /* clamp abnormal dt */

    /* Update PID sample period */
    for (uint8_t i = 0; i < 4; i++)
        ChassisMotor[i].PID_Velocity.dt = dt;

    /* --- Read RC and compute target body velocity --------------- */
    /* ch3: forward(+)/backward(-), ch4: left(+)/right(-) */
    if (!is_TOE_Error(RC_TOE))
    {
        Chassis.Vx = (float)remote_control.ch3 * RC_TO_VELOCITY;  /* m/s */
        Chassis.Vy = (float)remote_control.ch4 * RC_TO_VELOCITY;  /* m/s */
        Chassis.Vr = (float)remote_control.ch2 * RC_TO_ROTATION;  /* rad/s */
    }
    else
    {
        /* RC lost → stop */
        Chassis.Vx = 0.0f;
        Chassis.Vy = 0.0f;
        Chassis.Vr = 0.0f;
    }

    /* Coordinate transform: body → chassis frame
       (without IMU, body frame = chassis frame, no rotation compensation) */
    Chassis.VxTransfer = Chassis.Vx;
    Chassis.VyTransfer = Chassis.Vy;

    /* --- Omni-wheel inverse kinematics: body velocity → wheel speed ----- */
    /* V_wheel(rad/s) = f(Vx, Vy, Vr)
       FR:  V1 = [ (Vx - Vy)*ratio + (W*d*sqrt2)*Vr ] / (r * 0.001 * sqrt2)
       FL:  V2 = [ (Vx + Vy)*ratio + (W*d*sqrt2)*Vr ] / (r * 0.001 * sqrt2)
       HL:  V3 = [ (-Vx + Vy)*ratio + (W*d*sqrt2)*Vr ] / (r * 0.001 * sqrt2)
       HR:  V4 = [ (-Vx - Vy)*ratio + (W*d*sqrt2)*Vr ] / (r * 0.001 * sqrt2)
    */
    float denom = wheel_radius * 0.001f * SQRT2;
    float wheel_base_term = WHEEL_OPPOSITE * SQRT2 * Chassis.Vr;

    Chassis.V1 = (( Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio + wheel_base_term) / denom;
    Chassis.V2 = (( Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio + wheel_base_term) / denom;
    Chassis.V3 = ((-Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio + wheel_base_term) / denom;
    Chassis.V4 = ((-Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio + wheel_base_term) / denom;

    /* --- PID speed control: target speed → motor current ----------------- */
    /* Target: V1~V4 in rad/s,  Feedback: OutputVel_RadPS in rad/s (wheel shaft) */
    limited_current[0] = Motor_Speed_Calculate(&ChassisMotor[FR],
        ChassisMotor[FR].OutputVel_RadPS, Chassis.V1);
    limited_current[1] = Motor_Speed_Calculate(&ChassisMotor[FL],
        ChassisMotor[FL].OutputVel_RadPS, Chassis.V2);
    limited_current[2] = Motor_Speed_Calculate(&ChassisMotor[HL],
        ChassisMotor[HL].OutputVel_RadPS, Chassis.V3);
    limited_current[3] = Motor_Speed_Calculate(&ChassisMotor[HR],
        ChassisMotor[HR].OutputVel_RadPS, Chassis.V4);

    /* --- Send motor currents via CAN1 (C620 ESC protocol) ---------------- */
    if (is_TOE_Error(RC_TOE))
    {
        /* RC offline → zero current (safety) */
        Send_Motor_Current_1_4(&hcan1, 0, 0, 0, 0);
    }
    else
    {
        Send_Motor_Current_1_4(&hcan1,
            (int16_t)limited_current[0],
            (int16_t)limited_current[1],
            (int16_t)limited_current[2],
            (int16_t)limited_current[3]);
    }
}
