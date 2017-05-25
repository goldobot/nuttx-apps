#include "goldo_odometry.h"

#include <stdint.h>
#include <stdlib.h>
#include <math.h>




goldo_odometry_state_s g_odometry_state;

static goldo_odometry_config_s odometry_config;

static float heading_factor = 0;
static float update_frequency = 0;

extern int goldo_odometry_hal_init(void);
extern int goldo_odometry_hal_release(void);
extern int goldo_odometry_hal_read_encoders(int* left, int* right);

int goldo_odometry_init(void)
{
	/* Set initial values */
	g_odometry_state.elapsed_distance = 0;
	g_odometry_state.heading_change = 0;
	g_odometry_state.pos_x = 0;
	g_odometry_state.pos_y = 0;
	g_odometry_state.heading = 0;
	g_odometry_state.speed = 0;
	g_odometry_state.speed_x = 0;
	g_odometry_state.speed_y = 0;
	g_odometry_state.yaw_rate = 0;

  return goldo_odometry_hal_init();	
}

int goldo_odometry_quit(void)
{
  return goldo_odometry_hal_release();
}


int goldo_odometry_set_config(goldo_odometry_config_s* config)
{
	odometry_config = *config;
	heading_factor = 1.0f / odometry_config.wheel_spacing;
	update_frequency = 1.0f / odometry_config.update_period;
	return OK;
}

int goldo_odometry_update(void)
{
	int32_t qe_r;
  int32_t qe_l;

  goldo_odometry_hal_read_encoders(&qe_l,&qe_r);  

  g_odometry_state.counts_left = qe_l;
  g_odometry_state.counts_right = qe_r;

/* todo: manage encoder wraparound */
  int delta_cnt_l, delta_cnt_r;
  delta_cnt_l = (int16_t)(qe_l - g_odometry_state.counts_left);
  delta_cnt_r = (int16_t)(qe_r - g_odometry_state.counts_right);
  float dx_l = delta_cnt_l * odometry_config.dist_per_count_left;
  float dx_r = delta_cnt_r * odometry_config.dist_per_count_right;
  float d_dist = (dx_l+dx_r)*0.5f;
  float d_heading = (dx_r-dx_l) * heading_factor;

  g_odometry_state.elapsed_distance += d_dist;
  /* todo add config option for speed low pass filter frequency*/
  g_odometry_state.speed = g_odometry_state.speed * 0.9f + d_dist * update_frequency * 0.1f;
  g_odometry_state.heading_change += d_heading;

  /* Update position */
  float cos_h = cosf(g_odometry_state.heading + d_heading*0.5f);
  float sin_h = sinf(g_odometry_state.heading + d_heading*0.5f);
  g_odometry_state.pos_x += d_dist * cos_h;
  g_odometry_state.pos_y += d_dist * sin_h;
  g_odometry_state.speed_x += g_odometry_state.speed * cos_h;
  g_odometry_state.speed_y += g_odometry_state.speed * sin_h;
  g_odometry_state.heading += d_heading;
  return OK;
}


int goldo_odometry_set_position(float x, float y, float heading)
{
	g_odometry_state.pos_x = x;
	g_odometry_state.pos_y = y;
	g_odometry_state.heading = heading;
	float cos_h = cosf(g_odometry_state.heading);
  float sin_h = sinf(g_odometry_state.heading);
  g_odometry_state.speed_x += g_odometry_state.speed * cos_h;
  g_odometry_state.speed_y += g_odometry_state.speed * sin_h;
	return OK;
}