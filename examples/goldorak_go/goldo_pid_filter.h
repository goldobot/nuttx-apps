#ifndef __GOLDO_PID_FILTER_H__
#define __GOLDO_PID_FILTER_H__
#include "goldo_config.h"

typedef struct goldo_pid_filter_config_s
{
  float k_p;
  float k_i;
  float k_d;
  float lim_i;
  float ff_speed;
} goldo_pid_filter_config_s;

typedef struct goldo_pid_filter_s
{
  float cur_pos;
  float cur_speed;
  float tar_pos;
  float tar_speed;
  float out;
  float k_p;
  float k_i;
  float k_d;
  float lim_i;
  float ff_speed;
  float integrator;
  
} goldo_pid_filter_s;

int goldo_pid_filter_init(goldo_pid_filter_s* s);
int goldo_pid_filter_release(goldo_pid_filter_s* s);
int goldo_pid_filter_set_config(goldo_pid_filter_s* s, goldo_pid_filter_config_s* c);
int goldo_pid_set_target(goldo_pid_filter_s* s, float pos, float speed);
int goldo_pid_filter_do_step(goldo_pid_filter_s* s,float dt, float pos,float speed,float* out);

#endif /* __GOLDO_PID_FILTER_H__ */