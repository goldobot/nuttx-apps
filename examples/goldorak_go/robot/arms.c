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
static int s_arms_servo_torques[5] = {256,256,512,512,256};
static int s_arms_servo_boost_torques[5] = {383,384,778,778,256};
static int s_left_barrel_positions[4] = {900,1924,2948,3972};
static void *thread_arms(void *arg);

//900 1924
int s_current_barrel_index;
bool s_current_barrel_filled = false;
/*
{{550,2048,2380,435,364}},// prise cylindre bras avancé
	{{550,2048,2380,435,364}},// prise cylindre bras reculé
	{{760,2048,1800,340,364}},// cylindre à l'horizontale devant le robot
*/
static goldo_arm_position_s s_left_arm_positions[] = {
	{{550,2048,2380,435,364}},// prise cylindre bras avancé
	{{550,2048,2380,435,364}},// prise cylindre bras reculé
	{{630,2048,1050,550,40}},// position_depart
	{{720,2048,2383,411,364}},// pre_attrape_cylindre
	{{700,2048,2383,430,350}},// 4:attrape_cylindre
	{{500,2048,2383,430,350}},// 5:attrape_cylindre_2 
	{{730,2299,2382,430,353}},// 6:decalage_cylindre
	{{730,2302,1728,409,363}},// 7:soulever_cylindre
	{{680,2050,1728,409,363}},// 8:cylindre pris
	{{680,2048,1135,217,362}},// 9:depose_barillet
	{{730,2048,1100,200,362}},// 10:depose_barillet_2
	{{650,2005,2120,420,39}},// 11:transport_cylindre_tourne
	{{550,2002,2300,420,39}}, //12:depose_cylindre_tourne
	{{550,2002,2250,600,39}}, //13 depose_cylindre_tourne_2
	{{650,2048,1130,200,362}},// 14:attrape_barillet_1
	{{550,2500,2300,440,39}},//
	{{600,2048,2383,430,350}},// 16:attrape_cylindre_3 
	{{670,2050,1300,250,363}},// 17:Depose barriler 3
	{{680,2050,1300,250,363}}, // 18
	{{680,2050,1300,250,363}}};//19: position legerement decalée pour attrape cylindre
//330 1135
	// 700 130à
static int s_left_arm_current_position=2;
static int s_left_barrel_current_index=0;
//760: avance replié

	//sequence depliage: 2,8
	//sequence prise: 8 3 4 grab 5 6 7 8
	//sequence mise en barillet: 8 9 10
	//sequence depos: grab 10 9 8 11 12 drop

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

int boost_torque(bool boost)
{
	if(boost)
	{
		for(int i =0;i<5;i++)
		{
			SetTorque(s_left_arm_servo_ids[i],s_arms_servo_boost_torques[i]);
			usleep(10000);
		}
	} else
	{
		for(int i =0;i<5;i++)
		{
			SetTorque(s_left_arm_servo_ids[i],s_arms_servo_torques[i]);
			usleep(10000);
		}
	}
}
int goldo_arms_set_enabled(GOLDO_ARM_SIDE side, bool enabled)
{
	if(enabled)
	{
		for(int i =0;i<5;i++)
		{
			SetTorque(s_left_arm_servo_ids[i],s_arms_servo_torques[i]);
			usleep(10000);
		}
		
	} else
	{
		for(int i =0;i<5;i++)
		{
			SetTorque(s_left_arm_servo_ids[i],0);
		}
	}
	return OK;
	//set barrel to current position
}

int goldo_arms_grab_in_position(GOLDO_ARM_SIDE side, int pos)
{
	goldo_pump2_speed(65535);
	goldo_arms_move_to_position(side, pos);
	usleep(500000);
	goldo_pump2_speed(40000);
	return OK;
}

int goldo_arms_grab(GOLDO_ARM_SIDE side)
{
	goldo_pump2_speed(65535);
	sleep(2);
	goldo_pump2_speed(40000);
	return OK;
}

int goldo_arms_drop(GOLDO_ARM_SIDE side)
{
	goldo_pump2_speed(0);
	return OK;	
}

int goldo_arms_move_to_position(GOLDO_ARM_SIDE side, int pos)
{
	for(int i =0;i<5;i++)
	{
		goldo_dynamixels_set_position_sync(s_left_arm_servo_ids[i],s_left_arm_positions[pos].positions[i]);
		usleep(1000);
	}
	goldo_dynamixels_do_action();
	s_left_arm_current_position = pos;
	return OK;
};

int goldo_arms_init_barrels(void)
{
	s_current_barrel_filled = false;
	// left barrel. 8 pour rotation modules
	SetTorque(62,256);
	goldo_arms_move_barrel(0);
	return OK;
}

int goldo_arms_move_barrel(int index)
{
	printf("goldo_arms_move_barrel: %i\n",index);
	goldo_dynamixels_set_position(62,s_left_barrel_positions[index]);
	usleep(500000);
	s_left_barrel_current_index = index;
	return OK;
}

int goldo_arms_start_match(void)
{
	goldo_arms_init_barrels();
	goldo_arms_move_barrel(0);
	goldo_arms_set_enabled(GOLDO_ARM_LEFT,true);
	goldo_arms_move_to_position(GOLDO_ARM_LEFT,2);
	s_left_arm_current_position = 2;
	s_left_barrel_current_index = 0;
	return OK;
}

int goldo_arms_goto_rest_position(GOLDO_ARM_SIDE side)
{
	while(1)
	{
		switch(s_left_arm_current_position)
		{
			case 2:
				goldo_arms_move_to_position(side,8);
				usleep(500000);
				break;
			case 3:
				goldo_arms_move_to_position(side,8);
				usleep(500000);
				break;
			case 4:
				goldo_arms_move_to_position(side,3);
				usleep(500000);
				break;
			case 5:
				goldo_arms_move_to_position(side,4);
				usleep(500000);
				break;
			case 10:
				goldo_arms_move_to_position(side,9);
				usleep(500000);
				break;
			case 9:
				goldo_arms_move_to_position(side,8);
				usleep(500000);
				break;
			case 13:
				goldo_arms_move_to_position(side,12);
				usleep(500000);
				break;
			case 12:
				goldo_arms_move_to_position(side,11);
				usleep(500000);
				break;
			case 11:
				goldo_arms_move_to_position(side,8);
				usleep(500000);
				break;
			case 16:
				goldo_arms_move_to_position(side,4);
				usleep(500000);
				break;

			case 8:
				return;
			default:
				goldo_arms_move_to_position(side,8);
				usleep(500000);
		}
	}
	return OK;
}

int goldo_arms_grab_rocket_cylinder(GOLDO_ARM_SIDE side)
{
	goldo_arms_goto_rest_position(side);
	goldo_arms_move_to_position(side,3);
	usleep(500000);
	goldo_arms_move_to_position(side,4);
	usleep(500000);
	goldo_arms_move_to_position(side,16);
	usleep(500000);
	SetTorque(s_left_arm_servo_ids[0],256);
	usleep(100000);
	goldo_arms_move_to_position(side,16);
	usleep(500000);
	goldo_arms_grab_in_position(side,5);
	goldo_arms_move_to_position(side,4);
	SetTorque(s_left_arm_servo_ids[0],256);
	usleep(500000);
	goldo_arms_move_to_position(side,6);
	usleep(500000);
	goldo_arms_move_to_position(side,7);
	usleep(500000);
	goldo_arms_move_to_position(side,8);
	usleep(500000);
}

int goldo_arms_store_cylinder(GOLDO_ARM_SIDE side)
{
	if(s_current_barrel_filled && s_left_barrel_current_index<3)
	{
		goldo_arms_move_barrel(s_left_barrel_current_index+1);
	}
	goldo_arms_goto_rest_position(side);
	boost_torque(true);
	goldo_arms_move_to_position(side,17);
	usleep(500000);
	goldo_arms_move_to_position(side,9);
	usleep(500000);
	goldo_arms_move_to_position(side,10);
	usleep(500000);
	goldo_arms_drop(side);
	boost_torque(false);
	goldo_arms_goto_rest_position(side);
	s_current_barrel_filled = true;	
}

int goldo_arms_load_cylinder(GOLDO_ARM_SIDE side)
{
	if(!s_current_barrel_filled & s_left_barrel_current_index>0)
	{
		goldo_arms_move_barrel(s_left_barrel_current_index-1);
	}
	goldo_arms_goto_rest_position(side);
	goldo_arms_move_to_position(side,14);
	usleep(500000);
	boost_torque(true);
	goldo_arms_grab_in_position(side,10);
	usleep(500000);
	goldo_arms_goto_rest_position(side);
	boost_torque(false);
	s_current_barrel_filled = false;
}

int goldo_arms_drop_cylinder_front(GOLDO_ARM_SIDE side)
{
	goldo_arms_goto_rest_position(side);
	goldo_arms_move_to_position(side,11);
	usleep(500000);
	goldo_arms_grab_in_position(side,12);
	usleep(500000);
	goldo_arms_grab_in_position(side,13);
	usleep(500000);
	goldo_arms_drop(side);
	goldo_arms_goto_rest_position(side);	
}

int goldo_arms_drop_cylinder_front_swipe(GOLDO_ARM_SIDE side)
{
	goldo_arms_goto_rest_position(side);
	goldo_arms_move_to_position(side,11);
	usleep(500000);
	goldo_arms_move_to_position(side,15);
	usleep(500000);
	goldo_arms_grab_in_position(side,12);
	usleep(500000);
	goldo_arms_grab_in_position(side,13);
	usleep(500000);
	goldo_arms_drop(side);
	goldo_arms_goto_rest_position(side);	
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