#ifndef __GOLDO_ROBOT_HAL_H__
#define __GOLDO_ROBOT_HAL_H__
#include "goldo_arms.h"
#include "goldo_adversary_detection.h"

int goldo_robot_init(void);
int goldo_barrels_init(void);

int goldo_robot_release(void);
int goldo_robot_do_funny_action(void);


int goldo_robot_wait_for_match_begin(void);


#endif /* __GOLDO_ROBOT_HAL_H__ */