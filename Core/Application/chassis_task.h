#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include <stdint.h>
#include "kalman_filter.h"
#include "motor.h"
#include "decision_task.h"
#include "power_manager.h"
#include "bsp_CAN.h"
#include "bsp_dwt.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "ins_task.h"
#include "judgement_info.h"
#include "remote_control.h"
#include "cap.h"
#include "decision_task.h"
#include <math.h>

#define Chassis_Vr_FFC_MAXOUT 800
#define Chassis_Vr_FCC_LPF 0.001
#define Chassis_Vr_C0 1
#define Chassis_Vr_C1 1
#define Chassis_Vr_C2 1
#define Change_Boundary 500

#define CHASSIS_TASK_PERIOD 2

#define UseAttitudeControl
#define GRAVUTYCENTER_ADJUSTMENT 1.0f

#define VELOCITY_RATIO 3
#define RC_STICK_ROTATE_RATIO 0.72
#define RC_MOUSE_ROTATE_RATIO 0.5f

#ifndef STD_RADIAN
#define STD_RADIAN(angle) ((angle) + round((0 - (angle)) / (2 * PI)) * (2 * PI))
#endif

#define wheel_radius 76.00f
#define pi 3.1415926f
#define Kx 0.250f
#define Ky 0.250f

#define SQRT2 1.4142135f // √2的精确值

#define YAW_REDUCTION_RATIO 1 / 1.0f // YAW轴电机的传动系统的减速比

#define WHEEL_OPPOSITE 0.2675f // 轮子到中间的距离

#define YAW_REDUCTION_CORRECTION_ANGLE 90.0f

#ifdef ARM_MATH_DSP
#define user_cos arm_cos_f32
#define user_sin arm_sin_f32
#else
#define user_cos cosf
#define user_sin sinf
#endif

#define FOLLOW_THETA_LEN 200

#define RotateFollowKp 7
#define RotateFollowKi 1
#define RotateFollowKd 0.5

#define LowEnergy 0x07

typedef struct
{
  float FollowTheta;
  float TimeStamp_ms;
} thetaFrame_t;

typedef struct
{
  int16_t vx;
  int16_t vy;
  int32_t theta;

  int16_t FollowPosition[3];
  float SpinningPosition[3];

  uint8_t FollowTargetStatus;
  uint8_t IsSpinning;

  uint8_t MiniPC_Update;

  PID_t PID_Follow;
} MiniPC_ControlFrame;

typedef struct _Chassis_t
{
  float Vx, Vy, Vr; /*XY轴速度与角速度 */
  float LastVr;
  float VxTransfer, VyTransfer; /*用于云台坐标系与底盘坐标系的转换*/
  float GimbalVxTransfer, GimbalVyTransfer;
  float AngularVelocity; /*角速度*/

  float VelocityRatio;      /*调整小陀螺在总功率中的占比*/
  float rcStickRotateRatio; /*摇杆运动与电机间的比例系数*/
  float rcMouseRotateRatio; /*鼠标运动与电机间的比例系数*/

  KalmanFilter_t ChassisMotionEst;
  float V1, V2, V3, V4;

  float Vx_is_Chassis, Vy_is_Chassis; /*底盘坐标系下反解出的车速度*/
  float Vx_is_Body, Vy_is_Body;       /*云台坐标系下反解出的车速度*/
  float Ax_Chassis, Ay_Chassis;       /*底盘坐标系下在X,Y方向上的加速度*/
  float Ax_Body, Ay_Body;             /*云台坐标系下在X,Y方向上的加速度*/
  float V1_is, V2_is, V3_is, V4_is;   /*反解出四个轮子的速度*/
  float Vx_is, Vy_is, Vr_is;
  float VxTransfer_is, VyTransfer_is;

  float Position[2];
  float V_Position[2];
  float Velocity[2];
  float Accel[2];

  float x;
  float y;

  uint8_t status;
  uint8_t last_status;
  uint8_t Mode;
  uint8_t LastMode;

  int YawCorrectionScale; /*YAW修正后的范围*/
  uint32_t ChassisSwitchTick;

  float GravityCenter_Adjustment; /*云台重心修正*/
  float Theta;
  float TotalTheta;
  float FollowTheta;

  Motor_t ChassisMotor[4];

  TD_t ChassisVxTD;
  TD_t ChassisVyTD;
  TD_t SpinningTD;

  int16_t FollowXVelocity1000;
  int16_t FollowYVelocity1000;
  float FollowXVelocity;
  float FollowYVelocity;
  float FollowCoef;

  PID_t SpinningValid;
  PID_t NAVFollow;
  PID_t BuffEnergy;
  PID_t RotateFollow;
  PID_t Vx_Compensate;
  PID_t Vy_Compensate;
  PID_t Vr_Compensate;

  int16_t posX1000;
  int16_t posY1000;
  int16_t posZ1000;
  float posX;
  float posY;
  float posZ;
  float spinnig_center[2];
  int16_t PlanX1000;
  int16_t PlanY1000;
  float PlanX;
  float PlanY;
  uint8_t NavigationMode; // 0平地 1上坡 2下坡

  float nowFaceX;
  float nowFaceY;

  float NAV2CHASSISTheta;

  float TargetVelocityXtemp;
  float TargetVelocityYtemp;
  float TargetVelocityX;
  float TargetVelocityY;

  float GimbalVelocityX;
  float GimbalVelocityY;

  float NavVelocityX;
  float NavVelocityY;

  float ShiftingValueChangeFrq;
  float ShiftingValue;

  float YawAngle;
  float YawRefAngle;
  float YawCtrlAngle;

  float Observed_Vx;
  float Observed_Vy;
  float Observed_Vr;
  float WheelRadius;
  float WheelReductionRatio;

  thetaFrame_t thetaFrame[FOLLOW_THETA_LEN];

} Chassis_t;

enum
{
  Follow_Mode = 0, // 跟随
  Spinning_Mode,   // 小陀螺
  Side_Mode,       // 横着走
  Silence_Mode,    // 静止模式
  Count_Mode,
  Debug_Mode,
  HOME_TRAIN_MODE,
  RMUL_MODE,
  RMUC_MODE,
  Battle_MODE,
  CHECK_IN_MODE,
};

enum
{
  NavigationFlat = 0,
  NavigationUp,
  NavigationDown,
  PH,
  NavigationError,
};

enum
{
  FR = 0,
  FL = 1,
  HL = 2,
  HR = 3
};

enum TgtPosPredictStatus
{
  TgtLost = 0,
  TgtTracking,
  TgtSwitch,
  TgtConjecture,
};

void Chassis_Control(void);
void Chassis_Init(void);
void Insert_thetaFrame(thetaFrame_t *theta_frame, float follow_theta, uint32_t time_stamp_ms);
uint16_t Find_thetaFrame(thetaFrame_t *theta_frame, uint32_t match_time_stamp_ms);
void AerialKeyBoardCmd(void);

extern Chassis_t Chassis;
extern SentryInfo decision_info;
extern MiniPC_ControlFrame MiniPC_CtrlFrame;
extern uint8_t aimassist_online;
extern float TempAerialX;
extern float TempAerialY;
extern uint32_t ReachFlag;
extern float *limited_current;
extern uint8_t TargetChangeFlag;
#endif
