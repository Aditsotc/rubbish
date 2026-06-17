//
// Created by 25664 on 2026/6/15.
//

#include "chassis_task.h"
#include "kalman_filter.h"
#include "controller.h"

Chassis_t Chassis = {0};
SentryInfo decision_info = {0};
uint32_t Chassis_DWT_Count = 0;
uint32_t time_temp = 0;
uint32_t nanCount = 0;
uint32_t debugvalue = 0;
float preFollowTheta;
float SpinningValidTheta;
float NAVTransmitTheta;
float TargetTransmitTheta;
float YAWTransmitTheta;
float SpinningValidVx;
float SpinningValidVy;
uint32_t SpinCount = 0;
uint8_t hurtflag = 0;
uint8_t lasthurtflag = 0;

float lastVr;

float TempAerialX;
float TempAerialY;
float LastTempAerialX;
float LastTempAerialY;
uint8_t TargetChangeFlag;

uint8_t AMP = 50;
uint16_t VR = 600;
float MAX_RPM = 8000;

float Vx_k = 2000; // 最大阶跃速度比
float Vy_k = 2000;
float Vr_k = 300;
float c[3] = {1, 1, 1};

static float dt = 0,
             t = 0;

float user_count_time = 0;
uint8_t last_game_status = 0;

uint16_t ReachCount = 0;
uint8_t ReachFinished = 0;
uint32_t ReachFlag = 0;

uint8_t aimassist_online = 0;
uint32_t aimssistLoseCount = 0;
uint8_t Sendcount = 0;

static void Chassis_Get_CtrlValue(void);
static void Chassis_Set_Control(void);
static float Max_4(float num1, float num2, float num3, float num4);
static void Velocity_MAXLimit(void);

float float_constrain(float Value, float minValue, float maxValue) {
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}


static void Send_Chassis_Current(void) /*发送底盘电机控制电流*/
{
    static uint32_t count = 0;
    if (count % 10 == 0)
    {
        Send_Power_Data(&hcan1, robot_state.chassis_power_limit, power_heat_data.buffer_energy);
        count = 0;
    }

    if (is_TOE_Error(RC_TOE)) // RC——遥控器接收器  VTM——图传
    {
        if (Send_Motor_Current_1_4(&hcan1, 0, 0, 0, 0) == HAL_OK)
            ;
    }
    else
    {

        if (Send_Motor_Current_1_4(&hcan1,
                                   (int16_t)limited_current[0],
                                   (int16_t)limited_current[1],
                                   (int16_t)limited_current[2],
                                   (int16_t)limited_current[3]) == HAL_OK)
            ;

        // if (Send_Motor_Current_1_4(&hcan1, 0, 0, 0, 0) == HAL_OK)
        //     ;
    }

    count++;
}


void Chassis_Init(void) {
    Chassis.WheelRadius = 0.0765f;

    Chassis.WheelReductionRatio = (3591.0f / 187.0f);

    Chassis.VelocityRatio = VELOCITY_RATIO;

    Chassis.rcStickRotateRatio = 0;

    Chassis.GravityCenter_Adjustment = GRAVUTYCENTER_ADJUSTMENT;

    Chassis.YawCorrectionScale = 0;

    for (uint8_t i = 0; i < 4; i++)
    {

        PID_Init(&Chassis.ChassisMotor[i].PID_Velocity, 16384, 2000, 0, 50, 0.1, 0, 65, 15, 0.005, 0, 1,
                 Integral_Limit | OutputFilter | ChangingIntegrationRate);
        Chassis.ChassisMotor[i].Max_Out = 16384.0f;
    }
    decision_info.isInvincible = 1;
    decision_info.isDead = 0;
    decision_info.isAlive = 1;

    Power_Control.Is_Cap_On = FALSE;
    Power_Control.Is_Cap_Used = TRUE;
    chassisPowerInit();
}
void Chassis_Control(void) {
    dt = DWT_GetDeltaT(&Chassis_DWT_Count);
    t += dt ;
    Chassis_Get_CtrlValue();
    Send_Chassis_Current();
    Chassis.V1 = ((Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio + (WHEEL_OPPOSITE * SQRT2) * Chassis.Vr) / (wheel_radius * 0.001 * SQRT2);
    Chassis.V2 = ((Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio + (WHEEL_OPPOSITE * SQRT2) * Chassis.Vr) / (wheel_radius * 0.001 * SQRT2);
    Chassis.V3 = ((-Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio + (WHEEL_OPPOSITE * SQRT2) * Chassis.Vr) / (wheel_radius * 0.001 * SQRT2);
    Chassis.V4 = ((-Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio + (WHEEL_OPPOSITE * SQRT2) * Chassis.Vr) / (wheel_radius * 0.001 * SQRT2);


}
void Chassis_Get_CtrlValue(void) {
    static float Temp_Vx, Temp_Vy; // 速度的期望值
    static float tempVal;
    static float RC = 0.000001f;
    Temp_Vx += (remote_control.ch3 * 2.64f / 660.0f);
    Temp_Vy += (remote_control.ch4 * 2.64f / 660.0f);
    tempVal = (Temp_Vx - Chassis.Vx)/(0.001+dt);
    tempVal = (Temp_Vy - Chassis.Vy)/(0.001+dt);
    Chassis.Vx += (Temp_Vx - Chassis.Vx) / (RC + dt) * dt;   // RC 时间常数缓启动
    Chassis.Vy += (Temp_Vy - Chassis.Vy) / (0.01f + dt) * dt;
    Chassis.Vx = float_constrain(Chassis.Vx, -2.64f, 2.64f);
    Chassis.Vy = float_constrain(Chassis.Vy, -2.64f, 2.64f);
    // Mecanum 逆运动学：车体速度 → 轮子转速
    Chassis.V1 = ((Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio + (WHEEL_OPPOSITE * SQRT2) * Chassis.Vr) / (wheel_radius * 0.001 * SQRT2);
    Chassis.V2 = ((Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio + (WHEEL_OPPOSITE * SQRT2) * Chassis.Vr) / (wheel_radius * 0.001 * SQRT2);
    Chassis.V3 = ((-Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio + (WHEEL_OPPOSITE * SQRT2) * Chassis.Vr) / (wheel_radius * 0.001 * SQRT2);
    Chassis.V4 = ((-Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio + (WHEEL_OPPOSITE * SQRT2) * Chassis.Vr) / (wheel_radius * 0.001 * SQRT2);

    Motor_Speed_Calculate(&Chassis.ChassisMotor[0], Chassis.ChassisMotor[0].Velocity_RPM / WheelReductionRatio, Chassis.V1);
    Motor_Speed_Calculate(&Chassis.ChassisMotor[1], Chassis.ChassisMotor[1].Velocity_RPM / WheelReductionRatio, Chassis.V2);
    Motor_Speed_Calculate(&Chassis.ChassisMotor[2], Chassis.ChassisMotor[2].Velocity_RPM / WheelReductionRatio, Chassis.V3);
    Motor_Speed_Calculate(&Chassis.ChassisMotor[3], Chassis.ChassisMotor[3].Velocity_RPM / WheelReductionRatio, Chassis.V4);

}
