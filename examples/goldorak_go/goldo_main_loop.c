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

static char s_input_buffer[32];

int get_int_value(const char* prompt,int* val)
{
  printf(prompt);
  fflush(stdout);
  readline(s_input_buffer,32,stdin,stdout);  
  sscanf(s_input_buffer,"%d",val); 
  return OK;
}

int get_float_value(const char* prompt,float* val)
{
  printf(prompt);
  fflush(stdout);
  readline(s_input_buffer,32,stdin,stdout);
  sscanf(s_input_buffer,"%f",val);
  return OK;
}

int get_char_value(const char* prompt,char* val)
{
  printf(prompt);
  fflush(stdout);
  readline(s_input_buffer,32,stdin,stdout);
  *val = s_input_buffer[0];
  return OK;
}

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
    printf("encoders:%i, %i pos: %i,%i heading:%i speed: %i          \r", 
      (int)g_odometry_state.counts_left,(int)g_odometry_state.counts_right,
      (int)(g_odometry_state.pos_x*1000),(int)(g_odometry_state.pos_y*1000),(int)(g_odometry_state.heading*180/M_PI),
      (int)(g_odometry_state.speed*1000));
    usleep(100000);
  }
  return OK;
}

static int tune_pid_distance(void)
{
  
  float k_p=0;
  float k_i=0;
  float k_d=0;
  float i_limit;
  float speed_ff;
  char command;
  goldo_asserv_get_distance_pid_values(&k_p,&k_i,&k_d,&i_limit,&speed_ff);
  while(1)
  {
    printf("Tune distance pid\n");
    printf("K_P: %f\n",k_p);
    printf("K_I: %f\n",k_i);
    printf("K_D: %f\n",k_d);
    printf("Integrator limit: %f\n",i_limit);
    printf("Feedforward speed: %f\n",speed_ff);
    printf("\n");
    printf("Set (p), Set(i), Set (d), Set(l), Set (f), Test (t,T), BLock (b), Quit (q)\n");
    get_char_value("Command: ",&command);
    switch(command)
    {
      case 'p':
        get_float_value("K_P: ",&k_p);
        goldo_asserv_set_distance_pid_values(k_p,k_i,k_d,i_limit,speed_ff);
        break;
      case 'i':
        get_float_value("K_I: ",&k_i);
        goldo_asserv_set_distance_pid_values(k_p,k_i,k_d,i_limit,speed_ff);
        break;
      case 'd':
        get_float_value("K_D: ",&k_d);
        goldo_asserv_set_distance_pid_values(k_p,k_i,k_d,i_limit,speed_ff);
        break;
      case 'l':
        get_float_value("Integral limit: ",&i_limit);
        goldo_asserv_set_distance_pid_values(k_p,k_i,k_d,i_limit,speed_ff);
        break;
      case 'f':
        get_float_value("Speed feedforward: ",&speed_ff);
        goldo_asserv_set_distance_pid_values(k_p,k_i,k_d,i_limit,speed_ff);
        break;
      case 't':
        goldo_asserv_straight_line(0.1,0.2,0.5,0.5);
        goldo_asserv_straight_line(-0.1,0.2,0.5,0.5);
        goldo_asserv_wait_finished();
        break;
      case 'T':
        goldo_asserv_straight_line(0.3,0.2,0.5,0.5);
        goldo_asserv_straight_line(-0.3,0.2,0.5,0.5);
        goldo_asserv_wait_finished();
        break;
      case 'b':
        goldo_asserv_wait(20);
        goldo_asserv_wait_finished();
      case 'q':
        return OK;
    }
  }
}


static int tune_pid_heading(void)
{
  
  float k_p=0;
  float k_i=0;
  float k_d=0;
  float i_limit;
  float speed_ff;
  char command;
  goldo_asserv_get_heading_pid_values(&k_p,&k_i,&k_d,&i_limit,&speed_ff);
  while(1)
  {
    printf("Tune heading pid\n");
    printf("K_P: %f\n",k_p);
    printf("K_I: %f\n",k_i);
    printf("K_D: %f\n",k_d);
    printf("Integrator limit: %f\n",i_limit);
    printf("Feedforward speed: %f\n",speed_ff);
    printf("\n");
    printf("Set (p), Set(i), Set (d), Set(l), Set (f), Test (t,T), BLock (b), Quit (q)\n");
    get_char_value("Command: ",&command);
    switch(command)
    {
      case 'p':
        get_float_value("K_P: ",&k_p);
        goldo_asserv_set_heading_pid_values(k_p,k_i,k_d,i_limit,speed_ff);
        break;
      case 'i':
        get_float_value("K_I: ",&k_i);
        goldo_asserv_set_heading_pid_values(k_p,k_i,k_d,i_limit,speed_ff);
        break;
      case 'd':
        get_float_value("K_D: ",&k_d);
        goldo_asserv_set_heading_pid_values(k_p,k_i,k_d,i_limit,speed_ff);
        break;
      case 'l':
        get_float_value("Integral limit: ",&i_limit);
        goldo_asserv_set_heading_pid_values(k_p,k_i,k_d,i_limit,speed_ff);
        break;
      case 'f':
        get_float_value("Speed feedforward: ",&speed_ff);
        goldo_asserv_set_heading_pid_values(k_p,k_i,k_d,i_limit,speed_ff);
        break;
      case 't':
        goldo_asserv_rotation(M_PI*0.5,0.2,0.5,0.5);
        goldo_asserv_rotation(-M_PI*0.5,0.2,0.5,0.5);
        goldo_asserv_wait_finished();  
        break;
      case 'b':
        goldo_asserv_wait(20);
        goldo_asserv_wait_finished();
      case 'q':
        return OK;
    }
  }
}

int main_loop_test_asserv(void)
{
  goldo_asserv_enable();

  int command=0;  
  char buffer[32];
  while(1)
  {
    printf("(1) Straight line\n(2) Rotation\n(4) Tune distance PID\n(5) Tune heading PID\n(q) Quit\n Enter command: \n");
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
        get_int_value("Distance (mm): ",&distance);   
        get_int_value("Speed (mm/s): ",&speed);    
        get_int_value("Acceleration (mm/s^2): ",&acceleration); 
        get_int_value("Decceleration (mm/s^2): ",&decceleration);       
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
      case 4:
        tune_pid_distance();
        break;
      case 5:
        tune_pid_heading();
        break;
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

int main_loop_utest_funny_action(void)
{
  goldo_robot_do_funny_action();
}


int dynamixel_get_current_position(int id);
int goldo_dynamixels_init(void);
dynamixel_set_led(int id, int enable);
void SetTorque(int id,int value);
void SetPosition(int id,int pos);

int main_loop_test_dynamixels(void)
{
  goldo_dynamixels_init();
  int id;
  int position;
  char command;
 

 while(1)
  {
    printf("Dynamixels test\n");    
    printf("\n");
    printf("Set position (s), Get position (g), Led(l), Torque (t), Quit (q)\n");
    get_char_value("Command: ",&command);
    switch(command)
    {
      case 'g':
        get_int_value("Id: ",&id);
        printf("Position: %i\n",dynamixel_get_current_position(id));
        break;
      case 's':
        get_int_value("Id: ",&id);
        get_int_value("Position: ",&position);
        SetPosition(id,position);
        break;
      case 't':
        get_int_value("Id: ",&id);
        get_int_value("Torque: ",&position);
        SetTorque(id,position);
        break;
      case 'l':
        get_int_value("Id: ",&id);
        get_int_value("Enable: ",&position);       
        dynamixel_set_led(id,position);
        break;
      case 'q':
        return OK;
        break;
    }
  }
}

int main_loop_test_arms(void)
{

}