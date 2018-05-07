#include "goldo_match_timer.h"
#include "goldo_asserv.h"
#include "robot/goldo_robot.h"


#include <pthread.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>


static pthread_t s_match_timer_thread_id;
static bool s_match_timer_stop = false;
static uint32_t s_match_timer_begin = 0;
static uint32_t s_match_timer_end = -1;

static void *thread_match_timer(void *arg);


int goldo_match_timer_init(void)
{
	goldo_log(0,"goldo_match_timer: init\n");	
	s_match_timer_stop = false;	
	pthread_create(&s_match_timer_thread_id, NULL, thread_match_timer, NULL);
	return OK;
}

int goldo_match_timer_release(void)
{
	goldo_log(0,"goldo_match_timer: release\n");
	s_match_timer_stop = true;
	pthread_join(s_match_timer_thread_id,NULL);
	return OK;
}

int goldo_match_timer_start(int time_s)
{
  s_match_timer_begin = g_asserv.elapsed_time_ms;
  s_match_timer_end = g_asserv.elapsed_time_ms + time_s*1000;
  return OK;
}

int goldo_match_timer_get_value(void)
{
  return g_asserv.elapsed_time_ms - s_match_timer_begin;
}

bool goldo_match_timer_is_finished(void)
{
	return g_asserv.elapsed_time_ms >= s_match_timer_end;
}

void *thread_match_timer(void *arg)
{
	goldo_log(0,"goldo_match_timer: start thread\n");	
	while(!s_match_timer_stop)
	{
		if( g_asserv.elapsed_time_ms >= s_match_timer_end)
		{
			goldo_asserv_match_finished();
			printf("goldo_match_timer: match finished\n");
			break;
		}
		usleep(100000);
	}
	goldo_log(0,"goldo_match_timer: thread finished\n");
	return NULL;
}