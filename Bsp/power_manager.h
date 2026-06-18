#ifndef __POWER_MANAGER_H
#define __POWER_MANAGER_H
#include <stdint.h>
typedef struct { uint8_t Is_Cap_On; uint8_t Is_Cap_Used; } PowerControl_t;
extern PowerControl_t Power_Control;
#endif
