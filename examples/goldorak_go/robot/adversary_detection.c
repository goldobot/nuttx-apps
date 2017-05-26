#include "goldo_adversary_detection.h"
#include "../goldo_asserv.h"

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

static pthread_t s_adversary_detection_thread_id;
bool s_adversary_detection_enabled = false;
bool s_adversary_detection_stop = false;

extern int goldo_get_obstacle_gpio_state(void);

static void *thread_adversary_detection(void *arg);

int goldo_adversary_detection_init(void)
{
	goldo_log(0,"goldo_adversary_detection: init\n");	
	s_adversary_detection_stop = false;
	s_adversary_detection_enabled = false;
	pthread_create(&s_adversary_detection_thread_id, NULL, thread_adversary_detection, NULL);
	return OK;
}

int goldo_adversary_detection_release(void)
{
	goldo_log(0,"goldo_adversary_detection: release\n");
	s_adversary_detection_stop = true;
	pthread_join(s_adversary_detection_thread_id,NULL);
	return OK;
}

int goldo_adversary_detection_set_enable(bool enable)
{
	s_adversary_detection_enabled = enable;
	return OK;
}

void *thread_adversary_detection(void *arg)
{
	goldo_log(0,"goldo_adversary_detection: start thread\n");
	while(!s_adversary_detection_stop)
	{
		if(goldo_get_obstacle_gpio_state() && s_adversary_detection_enabled)
		{
			goldo_asserv_emergency_stop();
		}
		usleep(100000);
	}
	goldo_log(0,"goldo_adversary_detection: thread finished\n");
	return NULL;
}