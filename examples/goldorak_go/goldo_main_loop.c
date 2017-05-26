#define _USE_MATH_DEFINES
#include "goldo_main_loop.h"
#include "robot/goldo_robot.h"
#include "goldo_asserv.h"
#include "goldo_asserv_hal.h"
#include "goldo_odometry.h"
#include "goldo_match_timer.h"
#include "robot/goldo_adversary_detection.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <system/readline.h>


int main_loop_match(void)
{
  /* Wait for start of match*/
  printf("main_loop: Wait for start of match\n");
  /*replace true by check on start gpio*/
  while(true)
  {
    usleep(10000);
  }
  goldo_robot_wait_for_match_begin();
  printf("main_loop_match: Start match\n");
  goldo_asserv_enable();
  goldo_match_timer_start(90);  
  goldo_asserv_straight_line(0.5, 0.5, 0.5, 0.5);  

  /* STOP motors */

}

int main_loop_homologation(void)
{
  /* Wait for start of match*/
  printf("main_loop_homologation: start main loop\n");
  /*replace true by check on start gpio*/
  goldo_robot_wait_for_match_begin();
  printf("main_loop_homologation: start match\n");
  goldo_asserv_enable();
  goldo_match_timer_start(90);
  
  goldo_asserv_straight_line(0.5, 0.1, 0.5, 0.5);
  //goldo_asserv_wait(0.5);
  //goldo_asserv_rotation(M_PI * 0.5f, 0.5, 0.5, 0.5);
  //goldo_asserv_wait(0.5);
  //goldo_asserv_straight_line(0.5, 0.5, 0.5, 0.5);
  //sleep(3);
  //goldo_asserv_emergency_stop();
  //goldo_asserv_wait_finished();
  while(!goldo_match_timer_is_finished())
  {
    usleep(100000);
  }
  goldo_robot_do_funny_action();
  /* STOP motors */
  return OK;
}

int main_loop_test_motors(void)
{
  int command=0;
  int pwm_left=0;
  int pwm_right=0;
  char buffer[32];
  goldo_asserv_hal_set_motors_enable(true,true);
  while(1)
  {
    printf("Pwm left: %i, right: %i\n",pwm_left,pwm_right);
    printf("(1) Set left motor pwm\n(2) Set right motor_pwm\n(3) Quit\n Enter command: \n");
    command = 0;
    readline(buffer,32,stdin,stdout);
    sscanf(buffer,"%d",&command);
    
    switch(command)
    {
      case 1:
          printf("Input pwm (-60000,60000): ");
          fflush(stdout);
          readline(buffer,32,stdin,stdout);
          sscanf(buffer,"%d",&pwm_left);
        break;
      case 2:
          printf("Input pwm (-60000,60000): ");
          fflush(stdout);
          readline(buffer,32,stdin,stdout);
          sscanf(buffer,"%d",&pwm_right);
          break;         
      case 3:
        return OK;
      default:
        break;
    }
    goldo_asserv_hal_set_motors_pwm(pwm_left,pwm_right);
  }
}

int main_loop_test_odometry(void)
{
  while(1)
  {
    printf("encoders:%i, %i pos: %i,%i heading:%i speed: %i\r", 
      (int)g_odometry_state.counts_left,(int)g_odometry_state.counts_right,
      (int)(g_odometry_state.pos_x*1000),(int)(g_odometry_state.pos_y*1000),(int)(g_odometry_state.heading*180/M_PI),
      (int)(g_odometry_state.speed*1000));
    usleep(100000);
  }
  return OK;
}

int main_loop_test_asserv(void)
{
  goldo_asserv_enable();

  int command=0;  
  char buffer[32];
  while(1)
  {
    printf("(1) Straight line\n(2) Rotation\n(q) Quit\n Enter command: \n");
    command = 0;
    readline(buffer,32,stdin,stdout);
    sscanf(buffer,"%d",&command);
    if(buffer[0] == 'q')
    {
      return OK;
    }

    switch(command)
    {
      case 1:
      {
        int distance;
        int speed;
        int acceleration;
        int decceleration;
        printf("\nDistance (mm): ");
        fflush(stdout);
        readline(buffer,32,stdin,stdout);
        sscanf(buffer,"%d",&distance);

        printf("Speed (mm/s): ");
        fflush(stdout);
        readline(buffer,32,stdin,stdout);
        sscanf(buffer,"%d",&speed);
        fflush(stdout);

        printf("Acceleration (mm/s^2): ");
        fflush(stdout);
        readline(buffer,32,stdin,stdout);
        sscanf(buffer,"%d",&acceleration);
        fflush(stdout);

        printf("Decceleration (mm/s^2): ");
        fflush(stdout);
        readline(buffer,32,stdin,stdout);
        sscanf(buffer,"%d",&decceleration);
        goldo_asserv_straight_line(distance*1e-3,speed*1e-3,acceleration*1e-3,decceleration*1e-3);
        goldo_asserv_wait_finished();
      }          
      break;
      case 2:
      {
        int angle;
        int speed;
        int acceleration;
        int decceleration;
        printf("\nAngle (deg): ");
        fflush(stdout);
        readline(buffer,32,stdin,stdout);
        sscanf(buffer,"%d",&angle);

        printf("Speed (deg/s): ");
        fflush(stdout);
        readline(buffer,32,stdin,stdout);
        sscanf(buffer,"%d",&speed);
        fflush(stdout);

        printf("Acceleration (deg/s^2): ");
        fflush(stdout);
        readline(buffer,32,stdin,stdout);
        sscanf(buffer,"%d",&acceleration);
        fflush(stdout);

        printf("Decceleration (deg/s^2): ");
        fflush(stdout);
        readline(buffer,32,stdin,stdout);
        sscanf(buffer,"%d",&decceleration);
        goldo_asserv_rotation(angle*M_PI/180,speed*M_PI/180,acceleration*M_PI/180,decceleration*M_PI/180);
        goldo_asserv_wait_finished();
      }
      break;         
      case 3:
        return OK;
      default:
        break;
    }
  }

  goldo_asserv_straight_line(0.5, 0.5, 0.5, 0.5);
  goldo_asserv_straight_line(-0.5, 0.5, 0.5, 0.5);
   getchar();
  goldo_asserv_wait_finished();
  sleep(10);
  printf("finished movement sequence\n");
  return OK;
}

int main_loop_utest_start_match(void)
{
   /* Wait for start of match*/
  printf("main_loop_utest_start_match: start main loop\n");
  /*replace true by check on start gpio*/
  goldo_robot_wait_for_match_begin();
  printf("main_loop_utest_start_match: start match\n");
  return OK;
}

int main_loop_utest_adversary_detection(void)
{
  goldo_adversary_detection_set_enable(true);
  goldo_asserv_straight_line(0.2,0.01,0.5,0.5);
  while(1)
  {
    usleep(100000);
  }
  return OK;
}

int main_loop_utest_match_timer(void)
{
  goldo_match_timer_start(10);
  sleep(20);
  return OK;
}