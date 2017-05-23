#ifndef __GOLDO_PID_FILTER_H__
#define __GOLDO_PID_FILTER_H__

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
  float integrator;
  
} goldo_pid_filter_s;

int goldo_pid_filter_init(goldo_pid_filter_s* s);
int goldo_pid_filter_release(goldo_pid_filter_s* s);
int goldo_pid_set_target(goldo_pid_filter_s* s, float pos, float speed);
int goldo_pid_filter_do_step(goldo_pid_filter_s* s,float pos,float speed,float* out);

#endif /* __GOLDO_PID_FILTER_H__ */