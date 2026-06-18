#ifndef DECISION_TASK_H
#define DECISION_TASK_H
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t isInvincible; uint8_t isDead; uint8_t isAlive;
    uint8_t tgtStatus; uint8_t allow_to_lock; uint8_t reached_target;
    uint8_t sentryPosture; uint8_t sentryPresentPose; uint8_t attack_outpost;
    uint8_t isEnemyPostAlive;
    float cruise_begin; float cruise_end; float cruise_speed;
    float move_speed_ratio; float rotate_speed_ratio;
    uint16_t remain_gold;
} SentryInfo;

extern SentryInfo decision_info;
#endif
