#ifndef __JUDGEMENT_INFO_H
#define __JUDGEMENT_INFO_H
#include <stdint.h>

typedef struct {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
} GameStatus_t;

typedef struct {
    uint8_t robot_id; uint8_t robot_level;
    uint16_t remain_HP; uint16_t max_HP; uint16_t current_HP;
    uint16_t shooter_barrel_heat_limit; uint16_t shooter_barrel_cooling_value;
    uint16_t chassis_power_limit;
} RobotState_t;

typedef struct {
    float chassis_power; float chassis_power_buffer;
    uint16_t shooter_17mm_barrel_heat; uint16_t buffer_energy;
} PowerHeatData_t;

extern GameStatus_t game_status;
extern RobotState_t robot_state;
extern PowerHeatData_t power_heat_data;
#endif
