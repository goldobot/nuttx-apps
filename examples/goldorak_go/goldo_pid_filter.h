#ifndef __GOLDO_PID_FILTER_H__
#define __GOLDO_PID_FILTER_H__

typedef struct 
{
  float cur_val;
  float prev_val;
  float tar_val;
  float out;
  float k_p;
  float k_i;
  float k_d;
  
} goldo_pid_filter_s;

int goldo_pid_filter_init(goldo_pid_filter_s* s);
int goldo_pid_filter_release(goldo_pid_filter_s* s);
int goldo_pid_set_target_value(goldo_pid_filter_s* s, float val);
int goldo_pid_filter_do_step(goldo_pid_filter_s* s,float val,float* out);

#endif /* __GOLDO_PID_FILTER_H__ */