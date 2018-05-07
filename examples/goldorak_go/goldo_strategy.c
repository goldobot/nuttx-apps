#include "goldo_strategy.h"

#define GOLDO_NUM_WAYPOINTS 16
typedef struct bfs_point
{
	int previous_point;
	int distance;
} bfs_point;

static goldo_strategy_waypoint_s s_waypoints[2] = {
	{0.5,1.0,NULL},//zone depart bleue
	{0.6,1,0,NULL},
	{}
};

static bfs_point s_bfs_points[GOLDO_NUM_WAYPOINTS];

static int s_edges[] = {1};

static int s_starting_point_blue;
static int s_starting_point_yellow;

static int s_objectives_list_blue[] = {};
static int s_objectives_list_yellow[] = {};

static int s_current_point;
static int* s_current_objective;

static int s_current_path[16];

static int goldo_strategy_compute_path(int target_waypoint);

int goldo_strategy_main(int side)
{
	switch(side)
	{
		case GOLDO_SIDE_BLUE:
			s_current_point = s_starting_point_blue;
			s_current_objective = s_objectives_list_blue;
			break;
		case GOLDO_SIDE_YELLOW:
			s_current_point = s_starting_point_yellow;
			s_current_objective = s_objectives_list_yellow;
			break;
	}
	while(*s_current_objective != -1)
	{
		goldo_strategy_compute_path(*s_current_objective);
	}
}

static int bfs_explore_node(int index)
{
	int i;
	goldo_strategy_waypoint_s* pt = &s_waypoints[index];
	for(i = pt->edges_out_index;s_edges[i] != -1;i++)
	{
		goldo_strategy_waypoint_s* pt2 = &s_waypoints[s_edges[i]];
	}
}

/* Do a breadth first search for a path to target point*/
int goldo_strategy_compute_path(int target_waypoint)
{
	int i;
	printf("goldo_strategy: compute path to waypoint %i",target_waypoint);
	for(i=0; i<GOLDO_NUM_WAYPOINTS;i++)
	{
		s_bfs_points[i].previous_point = -1;
	}
}
