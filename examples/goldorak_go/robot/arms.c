#include "goldo_arms.h"
#include "goldo_dynamixels.h"
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

extern void goldo_pump1_speed(int32_t s);

int dynamixel_get_current_position(int id);

void SetTorque(int id,int value);
void SetSpeed(int id,int value);
void SetPosition(int id,int pos);


static pthread_t s_arms_thread_id;
static bool s_arms_stop = false;
static uint8_t s_left_arm_servo_ids[5] = {4,83,84,5,6};
static uint8_t s_right_arm_servo_ids[5] = {1,81,82,2,3};
static s_left_barrel_positions[4] = {0,255,300,400};
static void *thread_arms(void *arg);

int s_current_barrel_index;
/*
{{550,2048,2380,435,364}},// prise cylindre bras avancé
	{{550,2048,2380,435,364}},// prise cylindre bras reculé
	{{760,2048,1800,340,364}},// cylindre à l'horizontale devant le robot
*/
static goldo_arm_position_s s_left_arm_positions[] = {
	{{550,2048,2380,435,364}},// prise cylindre bras avancé
	{{550,2048,2380,435,364}},// prise cylindre bras reculé
	{{760,2048,1800,340,364}},// position_depart
	{{720,2048,2383,411,364}},// pre_attrape_cylindre
	{{550,2048,2383,411,364}},// 4:attrape_cylindre
	{{520,2048,2383,411,364}},// 5:attrape_cylindre_2 
	{{720,2299,2382,409,353}},// 6:decalage_cylindre
	{{720,2302,1728,409,363}},// 7:soulever_cylindre
	{{700,2050,1728,409,363}},// 8:cylindre pris
	{{689,2048,1135,217,362}},// 9:depose_barillet
	{{689,2048,1150,140,362}},// 10:depose_barillet_2
	{{550,2005,2120,420,39}},// 11:transport_cylindre_tourne
	{{550,2002,2385,420,39}} //12:depose_cylindre_tourne
	};

//760: avance replié

	extern void goldo_pump2_speed(int32_t s);

int goldo_arms_init(void)
{
	goldo_log(0,"goldo_arms: init\n");
	s_arms_stop = false;
	pthread_create(&s_arms_thread_id, NULL, thread_arms, NULL);	
	return OK;
}

int goldo_arms_release(void)
{
	goldo_log(0,"goldo_arms: release\n");
	s_arms_stop = true;
	pthread_join(s_arms_thread_id,NULL);
	return OK;
}

int goldo_arms_set_enabled(GOLDO_ARM_SIDE side, bool enabled)
{
	for(int i =0;i<5;i++)
	{
		SetTorque(s_left_arm_servo_ids[i],512);
	}
	

	//set barrel to current position
}
int goldo_arms_grab_in_position(GOLDO_ARM_SIDE side, int pos)
{
	goldo_pump2_speed(65535);
	goldo_arms_move_to_position(side, pos);
	sleep(1);
	goldo_pump2_speed(40000);
}

int goldo_arms_grab(GOLDO_ARM_SIDE side)
{
	goldo_pump2_speed(65535);
	sleep(2);
	goldo_pump2_speed(40000);
}

int goldo_arms_drop(GOLDO_ARM_SIDE side)
{
	goldo_pump2_speed(0);	
}

int goldo_arms_move_to_position(GOLDO_ARM_SIDE side, int pos)
{
	for(int i =0;i<5;i++)
	{
		goldo_dynamixels_set_position_sync(s_left_arm_servo_ids[i],s_left_arm_positions[pos].positions[i]);
		usleep(1000);
	}
	goldo_dynamixels_do_action();
};

int goldo_arms_init_barrels(void)
{
	s_current_barrel_index=0;
	// left barrel. 8 pour rotation modules
	SetTorque(62,512);
	goldo_dynamixels_set_position(62,s_left_barrel_positions[0]);
}

int goldo_arms_move_barrel(int index)
{
	goldo_dynamixels_set_position(62,s_left_barrel_positions[index]);
	usleep(500000);
}

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