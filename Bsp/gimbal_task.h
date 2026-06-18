#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H
#include "motor.h"

#define YAW_MOTOR_ID        0x209
#define CAN_GIMBAL_Info_ID  0x666

typedef struct { Motor_t YawMotor; Motor_t PitchMotor; } Gimbal_t;
typedef struct { uint8_t MiniPC_state; uint8_t slope; uint8_t isrollover; } Gimbal_Data_t;

extern Gimbal_t Gimbal;
extern Gimbal_Data_t Gimbal_Data;
#endif
