#ifndef DECISION_TASK_H
#define DECISION_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "judgement_info.h"

#define COMPETITION_RMUC // 启用RMUC决策
// #define COMPETITION_RMUL // 启用RMUL决策
// #define COMPETITION_TEST  // 启用决策树测试函数

// 姿态切换参数
#define POSTURE_DEGRADE_TIME 180.0f // 衰减阈值
#define POSTURE_PREEMPT_TIME 170.0f // 主动轮换阈值
#define POSTURE_SWITCH_CD 5.0f      // 切换冷却

// 补给配置默认值
#define RECOVERING_ENTER_HP 50
#define RECOVERING_ENTER_BULLET 30
#define RECOVERING_EXIT_HP 200
#define RECOVERING_EXIT_BULLET 200
#define RECOVERING_COOLDOWN 30.0f
#define RECOVERING_TIME_OUT 5.0f

#define HP_Emergency 100 // 血量紧急
#define HP_Warning 250   // 血量预警
#define HP_Healthy 300   // 血量健康
#define Base_Danger 1500 // 基地告急
#define Base_Safe 3000   // 基地安全

#define Bullet_Emergency 80 // 弹药告罄
#define Bullet_Caution 120  // 弹量紧张
#define Bullet_Healthy 200  // 弹量健康

#define Time_Late 60   // 比赛快结束
#define Time_Low 100   // 时间所剩无几
#define Time_Mid 200   // 比赛过半
#define Time_Early 300 // 比赛初期
#define Time_Total 420 // 比赛总时间

#define Gold_Rich 100 // 金币充足

// 决策点位
enum position
{
    stay_in_place = 0, // 原地待命
    pos_source_point,  // 起始点/出生岛
    pos_healing_zone,  // 补血点（资源岛兑矿区）
    pos_central,       // 中央点（RMUL）
    listen_to_aerial,  // 云台手接管点位

    // 防守点位
    pos_defense_1,
    pos_defense_2,
    pos_defense_3,
    pos_defense_4,

    // 进攻点位
    pos_attack_1,

    // 定点巡航点位
    pos_cruise_1,
    pos_cruise_2,
    pos_cruise_3,
    pos_cruise_4,

    // 巡逻点位
    pos_patrol_left,
    pos_patrol_right,

    // 测试点位
    pos_test,
};

// 特征值类型（用于决策树条件判断）
enum FeatureType
{
    SENTRY_HP,
    BULLET_REMAINING,
    BASE_HP,
    STAGE_REMAIN_TIME,
    GAME_PROGRESS,
    AERIAL_CONTROL,
    SENTRY_POSTURE,
    ENEMY_OUTPOST_ALIVE,
    ALLY_OUTPOST_ALIVE,
    FREE_BULLET_NUM,
    IS_ENEMY_INVINCIBLE,
    DEAD_COUNT,
    REMAIN_GOLD,
    STRATEGY_CHOICE,
    REACHED_TARGET,
};

// 哨兵机器人姿态
enum RobotPosture
{
    No_Posture,
    Attack_Posture  = 1,
    Defense_Posture = 2,
    Mobile_Posture  = 3,
};

// 策略选择
enum StrategyChoice
{
    Radical      = 0,
    Balanced     = 1,
    Conservative = 2,
    UltraRadical = 3,
};

// 补给策略选择
enum HealingStrategyChoice
{
    Heal_Radical      = 0,
    Heal_Balanced     = 1,
    Heal_Conservative = 2,
};

// 哨兵行为状态
enum SentryStateType
{
    Sentry_Idle,          // 待机（比赛未开始）
    Sentry_Heal,          // 补给（前往补给区，位置固定为 pos_healing_zone）
    Sentry_Follow_Aerial, // 听令（云台手接管，位置固定为 listen_to_aerial）
    Sentry_Aims,          // 瞄准（检测到目标）
    Sentry_Patrol,        // 巡逻（持续在区域内往返）
    Sentry_Guard,         // 驻守（固定点防守）
    Sentry_Engage,        // 交战（主动进攻点位）
    Sentry_Moving,        // 在路上
    Sentry_Point_Cruise,    // 定点巡航（几个点位来回巡航）
    Sentry_Rotating_Guard,  // 旋转驻守（固定点，巡航角每20s+90°）
    Sentry_Attack_Outpost,  // 进攻前哨站（停止自旋，attack_outpost标志置1）
};

#define POINT_CRUISE_MAX 4 // 定点巡航最大点位数

typedef struct
{
    uint16_t sentry_HP;
    uint16_t base_HP;
    int32_t outpost_HP;
    uint16_t stage_remain_time;
    int remain17mmBullet;
    uint8_t gameProgress;
    uint8_t InAerialCmdFlag;
    uint16_t aerial_cmd_count;
    uint8_t sentryPosture;
    uint8_t sentryPresentPose;
    uint8_t decision;

    uint8_t isInvincible;
    uint8_t isDead;
    uint8_t dead_count;
    uint8_t isRevive;
    uint8_t isAlive;
    uint8_t have_defense_buff;
    uint16_t remain_gold;
    uint8_t attack_outpost;
    uint8_t isEnemyPostAlive;
    uint8_t isPostAlive;
    uint8_t allowToAttackEngineer;
    uint8_t tgtStatus;
    uint8_t allow_to_lock;
    uint16_t free_bullet_num;
    uint8_t isEnemyInvincible;
    int16_t patrol_remain_time;
    uint8_t open_cap;
    uint8_t do_spin;
    uint8_t remain_energy;

    uint8_t decision_choose;
    uint8_t healing_strategy;

    float move_speed_ratio;
    float rotate_speed_ratio;
    float cruise_begin;
    float cruise_end;
    float cruise_speed;

    uint16_t last_sentry_HP;
    uint8_t reached_target;
    uint8_t isUnderFire;
} SentryInfo;

#ifdef __cplusplus
extern "C"
{
#endif

    void *DecisionTree_create(void);
    void DecisionTree_Delete(void *tree_handle);
    void TacticsManager_Init(void);
    void Decision_execute(void *tree_handle, SentryInfo *info, float dt);

#ifdef __cplusplus
}
#endif

#endif // DECISION_TASK_H
