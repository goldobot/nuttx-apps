#ifndef __GOLDO_STRATEGY_H__
#define __GOLDO_STRATEGY_H__
#include "goldo_config.h"

#define GOLDO_ACTION_NONE 0
#define GOLDO_ACTION_GRAB_CYLINDER_ROCKET 1
#define GOLDO_ACTION_GRAB_CYLINDER_FLOOR 2
#define GOLDO_ACTION_DROP_CYLINDER 4

#define GOLDO_SIDE_BLUE 0
#define GOLDO_SIDE_YELLOW 1
#define GOLDO_SIDE_TEST 2

#define GOLDO_ACTION_STATUS_AVAILABLE 1
#define GOLDO_ACTION_STATUS_DONE 2
#define GOLDO_ACTION_STATUS_FAILED 3

typedef struct goldo_strategy_action_s
{
	int point_id;
	void* data;
	int (*get_status_func)(void*);
	int (*execute_func)(void*);
} goldo_strategy_action_s;

typedef struct goldo_strategy_waypoint_s
{
	float pos_x;
	float pos_y;
	int edges_out_index;
	goldo_strategy_action_s* actions;
} goldo_strategy_waypoint_s;

#endif /* __GOLDO_STRATEGY_H__*/