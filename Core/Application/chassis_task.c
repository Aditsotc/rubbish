#include "chassis_task.h"

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

void *decision_tree = NULL; // 决策树句柄

static void Chassis_Get_Theta(void);     // 获取底盘与云台偏角
static void Chassis_Set_Mode(void);      // 设置底盘运动模式
static void Chassis_Get_CtrlValue(void); // 处理来自摇杆与键盘的控制数据
static void Chassis_Set_Control(void);   // 底盘运动解算以及PID计算
static void Send_Chassis_Current(void);  // 发送底盘电机控制电流
static float Max_4(float num1, float num2, float num3, float num4);
static void Velocity_MAXLimit(void); // 最大速度限制

void Chassis_Init(void)
{
    // 机械参数（保留）
    Chassis.WheelRadius = 0.0765f;
    Chassis.WheelReductionRatio = (3591.0f / 187.0f);
    Chassis.VelocityRatio = 10.0f;   // 固定速度系数，可调

    // 初始化 4 个电机速度 PID（原代码保留，参数可调）
    for (uint8_t i = 0; i < 4; i++) {
        PID_Init(&Chassis.ChassisMotor[i].PID_Velocity,
                 16384, 2000, 0, 50, 0.1, 0, 65, 15, 0.005, 0, 1,
                 Integral_Limit | OutputFilter | ChangingIntegrationRate);
        Chassis.ChassisMotor[i].Max_Out = 16384.0f;
    }


    PID_Init(&Chassis.RotateFollow, 310, 0, 0, 10, 0, 0, 0,0,0,0,1,
             Integral_Limit | OutputFilter);
}
//{
//     TacticsManager_Init();
//     decision_tree = DecisionTree_create(); // 构建决策树
//
//     decision_info.decision_choose = Balanced;       // 策略选择
//     decision_info.healing_strategy = Heal_Balanced; // 补给策略选择
//
//     Chassis.WheelRadius = 0.0765f;
//
//     Chassis.WheelReductionRatio = (3591.0f / 187.0f);
//
//     Chassis.VelocityRatio = VELOCITY_RATIO;
//
//     Chassis.rcStickRotateRatio = 0;
//
//     Chassis.GravityCenter_Adjustment = GRAVUTYCENTER_ADJUSTMENT;
//
//     ReachFlag = 1;
//
//     Chassis.TotalTheta = (Gimbal.YawMotor.total_angle - Gimbal.YawMotor.zero_offset) * ENCODERCOEF * YAW_REDUCTION_RATIO;
//
//     Chassis.Theta = loop_float_constrain((Chassis.TotalTheta + Gimbal.YawMotor.offset_angle * ENCODERCOEF * YAW_REDUCTION_RATIO), -180, 180);
//
//     Chassis.YawCorrectionScale = 0;
//
//     // 电机速度环PID计算
//     for (uint8_t i = 0; i < 4; i++)
//     {
//
//         PID_Init(&Chassis.ChassisMotor[i].PID_Velocity, 16384, 2000, 0, 50, 0.1, 0, 65, 15, 0.005, 0, 1,
//                  Integral_Limit | OutputFilter | ChangingIntegrationRate);
//         Chassis.ChassisMotor[i].Max_Out = 16384.0f;
//     }
//
//     // 底盘跟随云台PID初始化
//     PID_Init(&Chassis.RotateFollow, 310, 0, 0, RotateFollowKp, RotateFollowKi, RotateFollowKd, 0,
//              0, 0, 0, 1,
//              Integral_Limit | Derivative_On_Measurement | OutputFilter | DerivativeFilter);
//
//     PID_Init(&Chassis.SpinningValid, 50, 20, 0, 0.1, 0.0001, 0, 0,
//              0, 0, 0, 1,
//              Integral_Limit | Derivative_On_Measurement | OutputFilter | DerivativeFilter);
//
//     PID_Init(&Chassis.BuffEnergy, 60, 5, 0, 0.1, -0.001, 0, 0,
//              0, 0, 0, 1,
//              Integral_Limit | Derivative_On_Measurement | OutputFilter | DerivativeFilter); // problem
//
//     PID_Init(&Chassis.NAVFollow, 300, 100, 0, 8, 0, 1, 0,
//              0, 0, 0, 1,
//              Integral_Limit | Derivative_On_Measurement | OutputFilter | DerivativeFilter);
//
//     PID_Init(&Chassis.Vx_Compensate, 50, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0);
//
//     PID_Init(&Chassis.Vy_Compensate, 50, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0);
//
//     PID_Init(&Chassis.Vr_Compensate, 50, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0);
//
//     TD_Init(&Chassis.SpinningTD, 100000, 0.001);
//
//     Matrix_Init(&Chassis.ChassisMotionEst.H, 4, 6, (float *)Chassis.ChassisMotionEst.H_data);
//     Matrix_Init(&Chassis.ChassisMotionEst.HT, 6, 4, (float *)Chassis.ChassisMotionEst.HT_data);
//
//     decision_info.isInvincible = 1;
//     decision_info.isDead = 0;
//     decision_info.isAlive = 1;
//
//     Power_Control.Is_Cap_On = FALSE;
//     Power_Control.Is_Cap_Used = TRUE;
//     chassisPowerInit();
// }

void Chassis_Control(void)
{

    dt = DWT_GetDeltaT(&Chassis_DWT_Count);
    t += dt;
    // 处理云台手发来的数据
    //AerialKeyBoardCmd();
    // 获取底盘与云台偏角
    //Chassis_Get_Theta();
    // 设置底盘运动模式
    //Chassis_Set_Mode();
    // 处理来自摇杆与键盘的控制数据
    Chassis_Get_CtrlValue();
    // 底盘运动解算以及PID计算
    Chassis_Set_Control();
    // 发送底盘电机控制电流
    Send_Chassis_Current();
    // 发送云台手数据
    // if (TargetChangeFlag == 1)
    //     Sendcount++;
    // else
    //     Sendcount = 0;
    // if (Sendcount <= 10)
    //     Send_Nav_Data_RMUC(&hcan2);
}

static void Chassis_Get_Theta(void)
{
    // 底盘与云台偏角获取
    Chassis.TotalTheta = (Gimbal.YawMotor.total_angle - Gimbal.YawMotor.zero_offset) * ENCODERCOEF * YAW_REDUCTION_RATIO;

    Chassis.Theta = loop_float_constrain((Chassis.TotalTheta +
                                          Gimbal.YawMotor.offset_angle * ENCODERCOEF * YAW_REDUCTION_RATIO),
                                         -180, 180);

    if (is_TOE_Error(GIMBAL_YAW_MOTOR_TOE))
    {
        Chassis.TotalTheta = 0;
        Chassis.Theta = 0;
    }

    Chassis.FollowTheta = float_deadband(Chassis.Theta, -0.005f, 0.005f); // 0.005
    Insert_thetaFrame(Chassis.thetaFrame, Chassis.FollowTheta, INS_GetTimeline());
}

static void Chassis_Get_CtrlValue(void)
{
    static float Temp_Vx, Temp_Vy; // 速度的期望值
    static float tempVal;
    static float RC = 0.000001f;
    float max_speed = 2.64f;   // 最大平移速度 m/s
    float max_rotate = 3.0f;   // 最大旋转速度 rad/s

    // 摇杆值范围 -660 ~ +660
    Chassis.Vx = remote_control.ch3 * max_speed / 660.0f;
    Chassis.Vy = remote_control.ch4 * max_speed / 660.0f;


    可加低通滤波（原代码的缓启动可保留，这里用最简单线性映射）
    遥控器数值转化为车体速度
        Temp_Vx += (remote_control.ch3 * 2.64f / 660.0f);
        Temp_Vy += (remote_control.ch4 * 2.64f / 660.0f);

        // 缓启动
        if (fabsf(Temp_Vx) > 1e-3f) // 上升部分 1e-3f ~~ 0
        {
            if (Chassis.Vx * Temp_Vx < 0)
                Chassis.Vx = 0;

            tempVal = (Temp_Vx - Chassis.Vx) / (RC + dt);
            if (tempVal > Vx_k)
                tempVal = Vx_k;
            else if (tempVal < -Vx_k)
                tempVal = -Vx_k;
            Chassis.Vx += tempVal * dt;
        }
        else
        {
            tempVal = (Temp_Vx - Chassis.Vx) / (0.01f + dt);
            Chassis.Vx += tempVal * dt;
        }

        if (fabsf(Temp_Vy) > 1e-3f)
        {
            if (Chassis.Vy * Temp_Vy < 0)
                Chassis.Vy = 0;

            tempVal = (Temp_Vy - Chassis.Vy) / (RC + dt);
            if (tempVal > Vy_k)
                tempVal = Vy_k;
            else if (tempVal < -Vy_k)
                tempVal = -Vy_k;
            Chassis.Vy += 0.8 * tempVal * dt;
        }
        else
        {
            tempVal = (Temp_Vy - Chassis.Vy) / (0.01f + dt);
            Chassis.Vy += 0.8 * tempVal * dt;
         }

    // 加上限幅
    Chassis.Vx = float_constrain(Chassis.Vx, -max_speed, max_speed);
    Chassis.Vy = float_constrain(Chassis.Vy, -max_speed, max_speed);

}
//{
//     static float Temp_Vx, Temp_Vy; // 速度的期望值
//     static float tempVal;
//     static float RC = 0.000001f;
//
//     // 功率限制与实际值不符时
//     if (robot_state.chassis_power_limit >= 10240) // 数据包体积的最大值
//         robot_state.chassis_power_limit /= 256;
//     // 功率限幅
//     if (robot_state.chassis_power_limit > 120) // 数据包的体积的最小值
//         robot_state.chassis_power_limit = 120;
//     // 功率最低限制
//     if (robot_state.chassis_power_limit == 0)
//         robot_state.chassis_power_limit = 45;
//
//     if (robot_state.chassis_power_limit != 0)
//         Chassis.VelocityRatio = float_constrain(robot_state.chassis_power_limit / 10.0f + 2, 8, 13); // 通过power_limit 调整机器人跑得快慢
//
//     Temp_Vx = 0;
//     Temp_Vy = 0;
//
//     // 遥控器数值转化为车体速度
//     Temp_Vx += (remote_control.ch3 * 2.64f / 660.0f);
//     Temp_Vy += (remote_control.ch4 * 2.64f / 660.0f);
//
//     // 缓启动
//     if (fabsf(Temp_Vx) > 1e-3f) // 上升部分 1e-3f ~~ 0
//     {
//         if (Chassis.Vx * Temp_Vx < 0)
//             Chassis.Vx = 0;
//
//         tempVal = (Temp_Vx - Chassis.Vx) / (RC + dt);
//         if (tempVal > Vx_k)
//             tempVal = Vx_k;
//         else if (tempVal < -Vx_k)
//             tempVal = -Vx_k;
//         Chassis.Vx += tempVal * dt;
//     }
//     else
//     {
//         tempVal = (Temp_Vx - Chassis.Vx) / (0.01f + dt);
//         Chassis.Vx += tempVal * dt;
//     }
//
//     if (fabsf(Temp_Vy) > 1e-3f)
//     {
//         if (Chassis.Vy * Temp_Vy < 0)
//             Chassis.Vy = 0;
//
//         tempVal = (Temp_Vy - Chassis.Vy) / (RC + dt);
//         if (tempVal > Vy_k)
//             tempVal = Vy_k;
//         else if (tempVal < -Vy_k)
//             tempVal = -Vy_k;
//         Chassis.Vy += 0.8 * tempVal * dt;
//     }
//     else
//     {
//         tempVal = (Temp_Vy - Chassis.Vy) / (0.01f + dt);
//         Chassis.Vy += 0.8 * tempVal * dt;
//     }
//     Chassis.Vx = float_constrain(Chassis.Vx, -2.64f, 2.64f);
//     Chassis.Vy = float_constrain(Chassis.Vy, -2.64f, 2.64f);
// }

static void Chassis_Set_Mode(void)
{
    if (is_TOE_Error(CHASSIS_MOTOR1_TOE) && is_TOE_Error(CHASSIS_MOTOR2_TOE) && is_TOE_Error(CHASSIS_MOTOR3_TOE) && is_TOE_Error(CHASSIS_MOTOR4_TOE))
        Chassis.Mode = Silence_Mode;

    // 拨杆切换运动模式
    switch (remote_control.switch_right)
    {
    case Switch_Up:
        Chassis.Mode = CHECK_IN_MODE;
        break;

    case Switch_Middle:
        Chassis.Mode = Follow_Mode;
        Chassis.FollowCoef = 0;
        break;

    case Switch_Down:
        Chassis.Mode = Battle_MODE;
        Chassis.FollowCoef = 0;
        break;
    }

    Chassis.last_status = Chassis.status;

    if (Cap0727.Voltage > 15.0f)
    {
        Power_Control.Is_Cap_On = TRUE;
    }
    else
    {
        Power_Control.Is_Cap_On = FALSE;
    }
}

static void Chassis_Set_Control(void)
{
    static uint8_t thetaFrameNum;
    static uint16_t outpost_HP, last_sentry_HP, last_outpost_HP, base_HP, last_base_HP, enemy_outpost_HP, enemy_base_HP, last_enemy_base_HP;
    static uint16_t sentry_HP;
    static uint8_t remain_energy;
    static uint16_t ExchangedBulletNum, LastExchangedBulletNum;
    static uint8_t last_commd_keyboard;
    static int16_t last_ch4;
    static uint8_t ch4_change_flag = 0;

    const float lpf_r = 2 * PI * 1 * dt / (1 + 2 * PI * 1 * dt);
    static float ovx = 0, ovy = 0, ovr = 0;
    static int flag = 0;
    static int last = 0;
    static int compensate = 0;
    const float lpf_a = 2 * PI * 4 * dt / (1 + 2 * PI * 4 * dt);

    Chassis.TargetVelocityX = Chassis.TargetVelocityXtemp;
    Chassis.TargetVelocityY = Chassis.TargetVelocityYtemp;

    NAVTransmitTheta = atan2(Chassis.nowFaceY, Chassis.nowFaceX);
    TargetTransmitTheta = atan2(Chassis.TargetVelocityY, Chassis.TargetVelocityX);
    YAWTransmitTheta = Chassis.Theta;
    thetaFrameNum = Find_thetaFrame(Chassis.thetaFrame, (uint32_t)(INS_GetTimeline() - debugvalue));
    preFollowTheta = Chassis.thetaFrame[thetaFrameNum].FollowTheta / RADIAN_COEF;
    SpinningValidTheta = STD_RADIAN(-preFollowTheta + Chassis.posZ);

    sentry_HP = robot_state.current_HP;
    outpost_HP = game_robot_HP.ally_outpost_HP;
    base_HP = game_robot_HP.ally_base_HP;
    ExchangedBulletNum = sentry_info.ExchangedBulletNum;
    remain_energy = buff_musk.remaining_energy;

    // 将决策树需要的变量传入decision_info结构体
    decision_info.sentry_HP = sentry_HP;
    decision_info.base_HP = base_HP;
    decision_info.outpost_HP = outpost_HP;
    decision_info.remain17mmBullet = projectile_allowance.projectile_allowance_17mm;
    decision_info.remain_gold = projectile_allowance.remaining_gold_coin;
    decision_info.remain_energy = remain_energy;
    decision_info.stage_remain_time = game_status.stage_remain_time;
    decision_info.gameProgress = game_status.game_progress;
    decision_info.sentryPresentPose = sentry_info.SentryPresentPose;

    // 执行决策树
    Decision_execute(decision_tree, &decision_info, dt);
    Chassis.status = decision_info.decision;

    switch (Chassis.Mode)
    {
    case Follow_Mode:
        if (fabsf(Chassis.FollowTheta) < 0.005f) // 判断是否正对头部
            Chassis.Vr = remote_control.ch1 * Chassis.rcStickRotateRatio;
        else
        {
            // if (remote_control.switch_left == Switch_Up)
            //     Chassis.Vr = PID_Calculate(&Chassis.RotateFollow, Chassis.FollowTheta, 0.0f);
            // else
            // {
            //     Chassis.Vr = PID_Calculate(&Chassis.RotateFollow, Chassis.FollowTheta, 0.0f) +
            //                  remote_control.ch1 * Chassis.rcStickRotateRatio;
            // }
        }

        Chassis.Vr = 0; // 底盘关闭跟头，需要跟头时注释该行代码

        Chassis.VxTransfer = user_cos(-Chassis.FollowTheta / RADIAN_COEF) * Chassis.Vx +
                             user_sin(-Chassis.FollowTheta / RADIAN_COEF) * Chassis.Vy;
        Chassis.VyTransfer = -user_sin(-Chassis.FollowTheta / RADIAN_COEF) * Chassis.Vx +
                             user_cos(-Chassis.FollowTheta / RADIAN_COEF) * Chassis.Vy;

        break;

    case Silence_Mode:
        Chassis.Vr = 0;
        Chassis.VxTransfer = Chassis.Vx;
        Chassis.VyTransfer = Chassis.Vy;
        break;

    case Battle_MODE:
        Chassis.Vr = 50.0f;

        if (decision_info.isRevive == 1)
        {
            Chassis.Vr = PID_Calculate(&Chassis.RotateFollow, Chassis.FollowTheta, 0.0f);
        }

        Chassis.Vr *= decision_info.rotate_speed_ratio;

        SpinningValidTheta = STD_RADIAN(-preFollowTheta + Chassis.posZ + NAVTransmitTheta);

        Chassis.FollowTheta = 0.0f;
        Chassis.VxTransfer = -(-user_sin(SpinningValidTheta) * (Chassis.Vx + Chassis.TargetVelocityX) +
                               user_cos(SpinningValidTheta) * (Chassis.Vy + Chassis.TargetVelocityY));
        Chassis.VyTransfer = user_cos(SpinningValidTheta) * (Chassis.Vx + Chassis.TargetVelocityX) +
                             user_sin(SpinningValidTheta) * (Chassis.Vy + Chassis.TargetVelocityY);

        Chassis.last_status = Chassis.status;
        last_outpost_HP = outpost_HP;
        last_base_HP = base_HP;
        last_sentry_HP = sentry_HP;
        last_enemy_base_HP = enemy_base_HP;
        last_commd_keyboard = map_command.cmd_keyboard;
        LastExchangedBulletNum = ExchangedBulletNum;
        last_game_status = game_status.game_progress;
        lasthurtflag = hurtflag;

        decision_info.last_sentry_HP = last_sentry_HP;
        break;
    case CHECK_IN_MODE:
        if (Chassis.Mode == CHECK_IN_MODE)
        {
            Chassis.Vr = 50.0f;
        }

        Chassis.VxTransfer = user_cos(-Chassis.FollowTheta / RADIAN_COEF) * (Chassis.Vx + Chassis.FollowXVelocity * Chassis.FollowCoef * 14.0f * 10.0f / wheel_radius / 2 / (pi / 60)) +
                             user_sin(-Chassis.FollowTheta / RADIAN_COEF) * (Chassis.Vy + Chassis.FollowYVelocity * Chassis.FollowCoef * 14.0f * 10.0f / wheel_radius / 2 / (pi / 60));
        Chassis.VyTransfer = -user_sin(-Chassis.FollowTheta / RADIAN_COEF) * (Chassis.Vx + Chassis.FollowXVelocity * Chassis.FollowCoef * 14.0f * 10.0f / wheel_radius / 2 / (pi / 60)) +
                             user_cos(-Chassis.FollowTheta / RADIAN_COEF) * (Chassis.Vy + Chassis.FollowYVelocity * Chassis.FollowCoef * 14.0f * 10.0f / wheel_radius / 2 / (pi / 60));

        last_ch4 = remote_control.ch4;
        last_sentry_HP = sentry_HP;
        lasthurtflag = hurtflag;
        break;
    case HOME_TRAIN_MODE:
        Chassis.Vr = 0.0f;

        SpinningValidTheta = STD_RADIAN(-preFollowTheta + Chassis.posZ + NAVTransmitTheta);

        Chassis.VxTransfer = -(-user_sin(SpinningValidTheta) * (Chassis.Vx + Chassis.TargetVelocityX) +
                               user_cos(SpinningValidTheta) * (Chassis.Vy + Chassis.TargetVelocityY));
        Chassis.VyTransfer = user_cos(SpinningValidTheta) * (Chassis.Vx + Chassis.TargetVelocityX) +
                             user_sin(SpinningValidTheta) * (Chassis.Vy + Chassis.TargetVelocityY);

        Chassis.Vr *= decision_info.rotate_speed_ratio;

        last_ch4 = remote_control.ch4;
        last_sentry_HP = sentry_HP;
        lasthurtflag = hurtflag;

        decision_info.last_sentry_HP = last_sentry_HP;
        break;
    }

    Chassis.V1 = ((Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio + (WHEEL_OPPOSITE * SQRT2) * Chassis.Vr) / (wheel_radius * 0.001 * SQRT2);
    Chassis.V2 = ((Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio + (WHEEL_OPPOSITE * SQRT2) * Chassis.Vr) / (wheel_radius * 0.001 * SQRT2);
    Chassis.V3 = ((-Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio + (WHEEL_OPPOSITE * SQRT2) * Chassis.Vr) / (wheel_radius * 0.001 * SQRT2);
    Chassis.V4 = ((-Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio + (WHEEL_OPPOSITE * SQRT2) * Chassis.Vr) / (wheel_radius * 0.001 * SQRT2);

    Velocity_MAXLimit();

    Motor_Speed_Calculate(&Chassis.ChassisMotor[0], Chassis.ChassisMotor[0].Velocity_RPM / Chassis.WheelReductionRatio, Chassis.V1);
    Motor_Speed_Calculate(&Chassis.ChassisMotor[1], Chassis.ChassisMotor[1].Velocity_RPM / Chassis.WheelReductionRatio, Chassis.V2);
    Motor_Speed_Calculate(&Chassis.ChassisMotor[2], Chassis.ChassisMotor[2].Velocity_RPM / Chassis.WheelReductionRatio, Chassis.V3);
    Motor_Speed_Calculate(&Chassis.ChassisMotor[3], Chassis.ChassisMotor[3].Velocity_RPM / Chassis.WheelReductionRatio, Chassis.V4);

    for (int i = 0; i < 4; i++)
    {
        powerManagerInput(Chassis.ChassisMotor[i].Torque, Chassis.ChassisMotor[i].Velocity_Rad, i); // 对电机输出进行功率控制
    }
    if (!isnormal(Chassis.ChassisMotor[0].Output) && Chassis.ChassisMotor[0].Output != 0.0f)
    {
        Chassis.ChassisMotor[0].Output = 0.0f;
        nanCount++;
    }
    if (!isnormal(Chassis.ChassisMotor[1].Output) && Chassis.ChassisMotor[1].Output != 0.0f)
    {
        Chassis.ChassisMotor[1].Output = 0.0f;
        nanCount++;
    }
    if (!isnormal(Chassis.ChassisMotor[2].Output) && Chassis.ChassisMotor[2].Output != 0.0f)
    {
        Chassis.ChassisMotor[2].Output = 0.0f;
        nanCount++;
    }
    if (!isnormal(Chassis.ChassisMotor[3].Output) && Chassis.ChassisMotor[3].Output != 0.0f)
    {
        Chassis.ChassisMotor[3].Output = 0.0f;
        nanCount++;
    }

    if (Chassis.Vx != 0 || Chassis.Vy != 0)
        time_temp = USER_GetTick();
    if (PowerManager.is_initiallized_ && PowerManager.rls_.inited_) // 确保所有的初始化工作都完成之后，才进入到功率控制环节，如果没有初始化，函数中可能会产生野指针
    {
        powerManagerTask(); // 新版功率控制
    }

    // Chassis_Power_Cal();
    // if (Power_Control.Is_Cap_Used == TRUE)
    //     Cap0727_Power_Control();
    // else if (Power_Control.Is_Cap_Used == FALSE)
    //     Chassis_Power_Control_Without_Cap();
    // Cap_unused_correct();

    Chassis.LastMode = Chassis.Mode;

    ovx = Chassis.Observed_Vx;
    ovy = Chassis.Observed_Vy;
    ovr = Chassis.Vr;
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
        Send_Motor_Current_1_4(&hcan1, 0, 0, 0, 0);
    }
    else
    {

        Send_Motor_Current_1_4(&hcan1,
                               (int16_t)limited_current[0],
                               (int16_t)limited_current[1],
                               (int16_t)limited_current[2],
                               (int16_t)limited_current[3]);

        // Send_Motor_Current_1_4(&hcan1, 0, 0, 0, 0)
    }

    count++;
}

static float Max_4(float num1, float num2, float num3, float num4)
{
    static float max_num = 0;

    max_num = fabsf(num1);
    if (fabsf(num2) > max_num)
        max_num = fabsf(num2);
    if (fabsf(num3) > max_num)
        max_num = fabsf(num3);
    if (fabsf(num4) > max_num)
        max_num = fabsf(num4);

    return max_num;
}

static void Velocity_MAXLimit(void)
{
    static float TempMaxVelocity;
    if (Max_4(fabsf(Chassis.V1), fabsf(Chassis.V2), fabsf(Chassis.V3), fabsf(Chassis.V4)) > MAX_RPM)
    {
        TempMaxVelocity = Max_4(fabsf(Chassis.V1), fabsf(Chassis.V2), fabsf(Chassis.V3), fabsf(Chassis.V4));
        Chassis.V1 *= MAX_RPM / TempMaxVelocity;
        Chassis.V2 *= MAX_RPM / TempMaxVelocity;
        Chassis.V3 *= MAX_RPM / TempMaxVelocity;
        Chassis.V4 *= MAX_RPM / TempMaxVelocity;
    }
}

void Insert_thetaFrame(thetaFrame_t *theta_frame, float follow_theta, uint32_t time_stamp_ms)
{
    memmove(theta_frame + 1, theta_frame, (FOLLOW_THETA_LEN - 1) * sizeof(thetaFrame_t));
    theta_frame[0].TimeStamp_ms = time_stamp_ms;
    theta_frame[0].FollowTheta = follow_theta;
}

uint16_t Find_thetaFrame(thetaFrame_t *theta_frame, uint32_t match_time_stamp_ms)
{
    uint16_t num = 0;
    float min_time_error = fabsf((float)(theta_frame[0].TimeStamp_ms) - match_time_stamp_ms);

    for (uint16_t i = 0; i < FOLLOW_THETA_LEN; i++)
    {
        if (fabsf((float)(theta_frame[i].TimeStamp_ms) - match_time_stamp_ms) < min_time_error)
        {
            min_time_error = fabsf((float)(theta_frame[i].TimeStamp_ms) - match_time_stamp_ms);
            num = i;
        }
    }
    return num;
}

void AerialKeyBoardCmd(void)
{
    static float Tempx = 0, Tempy = 0;

    Tempx = map_command.target_position_x;
    Tempy = map_command.target_position_y;

    if (Tempx != LastTempAerialX || Tempy != LastTempAerialY)
    {
        TargetChangeFlag = 1;
    }
    else
        TargetChangeFlag = 0;

    LastTempAerialX = Tempx;
    LastTempAerialY = Tempy;
}
