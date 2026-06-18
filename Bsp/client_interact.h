#ifndef __CLIENT_INTERACT_H
#define __CLIENT_INTERACT_H
#include <stdint.h>
typedef struct { uint8_t cmd_keyboard; float target_position_x; float target_position_y; } MapCommand_t;
extern MapCommand_t map_command;
#endif
