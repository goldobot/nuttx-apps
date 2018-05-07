#include "goldo_pid_filter.h"



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

int goldo_pid_filter_set_config(goldo_pid_filter_s* f, goldo_pid_filter_config_s* c)
{
	f->k_p = c->k_p;
	f->k_d = c->k_d;
	f->k_i = c->k_i;
	f->lim_i = c->lim_i;	
	f->ff_speed = c->ff_speed;
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
	return OK;
}

int goldo_pid_filter_do_step(goldo_pid_filter_s* s,float dt, float pos,float speed,float* out)
{
	s->cur_pos = pos;
	s->cur_speed = speed;
	s->integrator += (s->tar_pos - pos) * dt * s->k_i;
	if(s->integrator > s->lim_i) s->integrator = s->lim_i;
	if(s->integrator < -s->lim_i) s->integrator = -s->lim_i;
	*out= (s->tar_pos - pos) * s->k_p + (s->tar_speed - speed) * s->k_d + s->tar_speed * s->ff_speed + s->integrator;

	return OK;
}
