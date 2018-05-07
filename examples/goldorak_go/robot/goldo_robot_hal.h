#ifndef __GOLDO_ROBOT_HAL_H__
#define __GOLDO_ROBOT_HAL_H__
#include "arms.h"
int goldo_robot_init(void);
int goldo_arms_init(void);
int goldo_adversary_detection_init(void);
int goldo_barrels_init(void);

int goldo_robot_release_robot(void);
int goldo_robot_release_adversary_detection(void);
int goldo_robot_release_barrels(void);





#endif /* __GOLDO_ROBOT_HAL_H__ *.