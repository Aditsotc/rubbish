/**
 * @file    globals.c
 * @brief   Storage for extern variables used by chassis system.
 *          These are referenced by bsp_CAN.c and other modules but
 *          not needed for basic RC driving — they exist to satisfy the linker.
 */
#include "chassis_task.h"
#include "detect_task.h"
#include "cap.h"
#include "gimbal_task.h"
#include "decision_task.h"
#include "judgement_info.h"
#include "client_interact.h"
#include "power_manager.h"

/* Cap */
Cap_t Cap = {0};
Cap0727_t Cap0727 = {0};

/* Gimbal */
Gimbal_t Gimbal = {0};
Gimbal_Data_t Gimbal_Data = {0};

/* Decision */
SentryInfo decision_info = {0};

/* Judgement */
GameStatus_t game_status = {0};
RobotState_t robot_state = {0};
PowerHeatData_t power_heat_data = {0};

/* Client interact */
MapCommand_t map_command = {0};

/* Power */
PowerControl_t Power_Control = {0};
