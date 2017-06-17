#ifndef __GOLDO_ARMS_H__
#define __GOLDO_ARMS_H__
#include "../goldo_config.h"

typedef enum GOLDO_ARM_SIDE
{
	GOLDO_ARM_LEFT,
	GOLDO_ARM_RIGHT
} GOLDO_ARM_SIDE;

typedef enum GOLDO_ARM_STATE
{
	GOLDO_ARM_SATE_1
} GOLDO_ARM_STATE;

typedef struct gold_arm_position_s
{
	uint16_t positions[5];
} goldo_arm_position_s;

//1 81 82 2 3

int goldo_arms_init(void);
int goldo_arms_release(void);

/* add new functions*/
int goldo_arms_set_enabled(GOLDO_ARM_SIDE side, bool enabled);
int goldo_arms_move_to_position(GOLDO_ARM_SIDE side, int pos);
int goldo_arms_grab_in_position(GOLDO_ARM_SIDE side, int pos);
int goldo_arms_grab(GOLDO_ARM_SIDE side);
int goldo_arms_drop(GOLDO_ARM_SIDE side);

int goldo_arms_init_barrels(void);
int goldo_arms_move_barrel(int index);
#endif /* __GOLDO_ARMS_H__ */