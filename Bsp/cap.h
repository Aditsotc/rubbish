#ifndef __CAP_H
#define __CAP_H
#include <stdint.h>

typedef struct { float Voltage; float Power; float PowerIn; uint8_t Cap_Normol_Chassis_Flag; } Cap_t;
typedef struct { float Voltage; float Power; } Cap0727_t;

extern Cap_t Cap;
extern Cap0727_t Cap0727;
#endif
