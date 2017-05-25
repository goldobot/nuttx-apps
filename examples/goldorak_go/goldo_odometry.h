
#ifndef __GOLDO_ODOMETRY_H__
#define __GOLDO_ODOMETRY_H__
#include "goldo_config.h"

typedef struct goldo_odometry_config_s
{
  float dist_per_count_left;
  float dist_per_count_right;
  float wheel_spacing;
  float update_period;

} goldo_odometry_config_s;

typedef struct goldo_odometry_s
{ 
  int counts_left;
  int counts_right;
  float elapsed_distance;
  float heading_change;
  float pos_x;
  float pos_y;
  float heading;
  float speed;
  float speed_x;
  float speed_y;
  float yaw_rate;
} goldo_odometry_state_s;

extern goldo_odometry_state_s g_odometry_state;

int goldo_odometry_init(void);
int goldo_odometry_quit(void);
int goldo_odometry_set_config(goldo_odometry_config_s* config);
int goldo_odometry_update(void);
int goldo_odometry_set_position(float x, float y, float heading);
int goldo_odometry_reset_elapsed(void);

#endif /* __GOLDO_ODOMETRY_H___ */