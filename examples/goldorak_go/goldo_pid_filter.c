#include "goldo_pid_filter.h"

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
  float ff_speed;
  float lim_i;
  float integrator;
  
} goldo_pid_filter_s;

int goldo_pid_filter_init(goldo_pid_filter_s* s)
{
	s->tar_pos = 0;
	s->tar_speed = 0;
	s->cur_pos = 0;
	s->cur_speed = 0;
	s->integrator = 0;
	s->lim_i = 0;
	s->ff_speed = 0;
	
	return OK;
}

int goldo_pid_filter_release(goldo_pid_filter_s* s)
{
	return OK;
}

int goldo_pid_set_target(goldo_pid_filter_s* s, float pos, float speed)
{
	s->tar_pos = pos;
	s->tar_speed = speed;
}

int goldo_pid_filter_do_step(goldo_pid_filter_s* s,float pos,float speed,float* out)
{
	s->integrator += (pos - s->tar_pos) * s->k_i;
	if(s->integrator > s->lim_i) s->integrator = s->lim_i;
	if(s->integrator < -s->lim_i) s->integrator = -s->lim_i;
	return (pos-s->tar_pos) * s->k_p + (speed - s->tar_speed) * s->k_d + speed * s->ff_speed + s->integrator;
}
