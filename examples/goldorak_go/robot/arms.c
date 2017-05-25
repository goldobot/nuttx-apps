#include "goldo_arms.h"
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

static pthread_t s_arms_thread_id;


static void *thread_arms(void *arg);


int goldo_arms_init(void)
{
	goldo_log(0,"goldo_arms: init\n");
	pthread_create(&s_arms_thread_id, NULL, thread_arms, NULL);
	return OK;
}

int goldo_arms_release(void)
{
	goldo_log(0,"goldo_arms: release\n");
	return OK;
}

void *thread_arms(void *arg)
{
	goldo_log(0,"goldo_arms: start thread\n");
	while(1)
	{
		usleep(10000);
	}
}