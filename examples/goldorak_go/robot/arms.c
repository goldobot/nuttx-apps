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


int dynamixel_get_current_position(int id);

void SetTorque(int id,int value);
void SetSpeed(int id,int value);
void SetPosition(int id,int pos);


static pthread_t s_arms_thread_id;
static bool s_arms_stop = false;
static uint8_t s_left_arm_servo_ids[5] = {4,83,84,5,6};
static uint8_t s_right_arm_servo_ids[5] = {1,81,82,2,3};
static void *thread_arms(void *arg);

static goldo_arm_position_s s_left_arm_positions[2] = {{{550,2048,2383,411,364}},{{0,0,0,0,0}}};

void goldo_pump_init(void);
int goldo_arms_init(void)
{
	goldo_log(0,"goldo_arms: init\n");
	goldo_pump_init();
	s_arms_stop = false;
	pthread_create(&s_arms_thread_id, NULL, thread_arms, NULL);
	goldo_arms_move_to_position(0);
	return OK;
}

int goldo_arms_release(void)
{
	goldo_log(0,"goldo_arms: release\n");
	s_arms_stop = true;
	pthread_join(s_arms_thread_id,NULL);
	return OK;
}

int goldo_arms_move_to_position(int pos)
{
	for(int i =0;i<5;i++)
	{
		SetPosition(s_left_arm_servo_ids[i],s_left_arm_positions[pos].positions[i]);
		SetTorque(s_left_arm_servo_ids[i],512);
	}
};

void *thread_arms(void *arg)
{
	goldo_log(0,"goldo_arms: start thread\n");
	while(!s_arms_stop)
	{
		usleep(10000);
	}
	goldo_log(0,"goldo_arms: thread finished\n");
	return NULL;
}