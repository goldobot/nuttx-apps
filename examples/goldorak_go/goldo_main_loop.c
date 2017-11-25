#define _USE_MATH_DEFINES
#include "goldo_main_loop.h"
#include "robot/goldo_robot.h"
#include "goldo_asserv.h"
#include "goldo_asserv_hal.h"
#include "goldo_odometry.h"
#include "goldo_match_timer.h"
#include "robot/goldo_adversary_detection.h"
#include "robot/goldo_arms.h"
#include "goldo_dynamixels.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <system/readline.h>

static char s_input_buffer[32];

typedef enum GOLDO_OPCODE
{
  GOP_MOVE_TO_WAYPOINT,
  GOP_LOAD_CYLINDER_FROM_ROCKET,
  GOP_DROP_CYLINDER_FRONT,
  GOP_DROP_CYLINDER_FRONT_SWIPE
} GOLDO_OPCODE;

typedef struct goldo_waypoint_s
{
  float x;
  float y;
} goldo_waypoint_s;

static goldo_waypoint_s s_waypoints[] = {
  {90,2090},//yellow start against the border
  {210,2090},//yellow start center of start area
  {510,1924},
  {350,1924},//yellow grab rocket position
  {900,1924},
  {900,2300},
  {850,2400},
  {850,2620}

};

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

void point_to(float x, float y)
{
  float dx = (x - g_odometry_state.pos_x);
  float dy = (y - g_odometry_state.pos_y);
  float direction = atan2(dy,dx);  
  float delta = direction - g_odometry_state.heading;
  if(delta>M_PI)
  {
    delta -= M_PI*2;
  } else if(delta < -M_PI*2) 
  {
    delta += M_PI*2;
  }
   goldo_asserv_rotation(delta,M_PI/4,M_PI/4,M_PI/4);
   goldo_asserv_wait(1.5);
   goldo_asserv_wait_finished();  
}

void move_to(float x, float y, bool forward)
{
  float dx = (x - g_odometry_state.pos_x);
  float dy = (y - g_odometry_state.pos_y);
  float direction = atan2(dy,dx);
  float distance = sqrt(dx*dx+dy*dy);
  if(!forward)
  {
    distance *= -1;
    direction += M_PI;
    if(direction > M_PI)
    {
      direction -= 2*M_PI;
    }
  }
  float delta = direction - g_odometry_state.heading;
  
  if(delta>M_PI)
  {
    delta -= M_PI*2;
  } else if(delta < -M_PI*2) 
  {
    delta += M_PI*2;
  }
   goldo_asserv_rotation(delta,M_PI/4,M_PI/4,M_PI/4);
   goldo_asserv_wait(0.5);
   goldo_asserv_straight_line(distance,0.1,0.1,0.2);
   goldo_asserv_wait(0.5);
   goldo_asserv_wait_finished();  
}

void move_to_waypoint(int i, bool forward)
{
  move_to(s_waypoints[i].x*1e-3,s_waypoints[i].y*1e-3,forward);
}

int match_core(bool yellow)
{
  move_to_waypoint(1,true);
  move_to_waypoint(2,true);
  move_to_waypoint(3,true);
   goldo_arms_grab_rocket_cylinder(GOLDO_ARM_LEFT);
        goldo_arms_store_cylinder(GOLDO_ARM_LEFT);

         goldo_arms_grab_rocket_cylinder(GOLDO_ARM_LEFT);
        goldo_arms_store_cylinder(GOLDO_ARM_LEFT);

         goldo_arms_grab_rocket_cylinder(GOLDO_ARM_LEFT);
        goldo_arms_store_cylinder(GOLDO_ARM_LEFT);

         goldo_arms_grab_rocket_cylinder(GOLDO_ARM_LEFT);
        goldo_arms_store_cylinder(GOLDO_ARM_LEFT);

        move_to_waypoint(4,true);
        move_to_waypoint(5,true);
        move_to_waypoint(6,true);
        move_to_waypoint(6,true);
        move_to_waypoint(7,true);
        goldo_arms_load_cylinder(GOLDO_ARM_LEFT);
        goldo_arms_drop_cylinder_front(GOLDO_ARM_LEFT);

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

goldo_asserv_trace_point_s asserv_trace_buffer[50];

static int tune_pid_distance(void)
{
  int i;
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
    printf("Set (p), Set(i), Set (d), Set(l), Set (f), Test (t,T,s), BLock (b), Quit (q)\n");
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
        goldo_asserv_wait_finished();
        sleep(1);
        goldo_asserv_straight_line(-0.1,0.2,0.5,0.5);
        goldo_asserv_wait_finished();
        break;
      case 'T':
        goldo_asserv_start_trace(asserv_trace_buffer,50,10);
        goldo_asserv_straight_line(0.5,0.2,0.2,0.2);
        goldo_asserv_wait(1);
        goldo_asserv_straight_line(-0.5,0.2,0.2,0.2);        
        goldo_asserv_wait_finished();
        for(i=0;i<50;i++)
        {
          printf("%i,%i,%i,%i,%i\t",
           (int)(asserv_trace_buffer[i].elapsed_distance_setpoint*1000),
           (int)(asserv_trace_buffer[i].elapsed_distance*1000),
           (int)(asserv_trace_buffer[i].speed_setpoint*1000),
           (int)(asserv_trace_buffer[i].speed*1000),
           (int)((asserv_trace_buffer[i].motor_pwm_left+asserv_trace_buffer[i].motor_pwm_right)*50));
          if(i%5 == 0)
          {
            printf("\n");
          }
        }
        break;
      case 's':
        goldo_asserv_start_trace(asserv_trace_buffer,50,10);
        goldo_asserv_position_step(0.2);
        goldo_asserv_wait(5);
        goldo_asserv_wait_finished();
        for(i=0;i<50;i++)
        {
          printf("%i,%i,%i,%i,%i\t",
           (int)(asserv_trace_buffer[i].elapsed_distance_setpoint*1000),
           (int)(asserv_trace_buffer[i].elapsed_distance*1000),
           (int)(asserv_trace_buffer[i].speed_setpoint*1000),
           (int)(asserv_trace_buffer[i].speed*1000),
           (int)((asserv_trace_buffer[i].motor_pwm_left+asserv_trace_buffer[i].motor_pwm_right)*50));
          if(i%5 == 0)
          {
            printf("\n");
          }
        }
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
  int i;
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
        goldo_asserv_start_trace(asserv_trace_buffer,50,10);
        goldo_asserv_rotation(M_PI,0.2,0.2,0.2);
        goldo_asserv_rotation(-M_PI,0.2,0.2,0.2);
        goldo_asserv_wait_finished();
        for(i=0;i<50;i++)
        {
          printf("%i,%i,%i,%i,%i\t",
           (int)(asserv_trace_buffer[i].heading_change_setpoint*180/M_PI),
           (int)(asserv_trace_buffer[i].heading_change*180/M_PI),
           (int)(asserv_trace_buffer[i].yaw_rate_setpoint*180/M_PI),
           (int)(asserv_trace_buffer[i].yaw_rate*180/M_PI),
           (int)((asserv_trace_buffer[i].motor_pwm_right-asserv_trace_buffer[i].motor_pwm_left)*50));
          if(i%5 == 0)
          {
            printf("\n");
          }
        }
        break;
      case 's':
        goldo_asserv_start_trace(asserv_trace_buffer,50,10);
        goldo_asserv_heading_step(90*M_PI/180);
        goldo_asserv_wait(5);
        goldo_asserv_wait_finished();
        for(i=0;i<50;i++)
        {
          printf("%i,%i,%i,%i,%i\t",
          (int)(asserv_trace_buffer[i].heading_change_setpoint*180/M_PI),
           (int)(asserv_trace_buffer[i].heading_change*180/M_PI),
           (int)(asserv_trace_buffer[i].yaw_rate_setpoint*180/M_PI),
           (int)(asserv_trace_buffer[i].yaw_rate*180/M_PI),
           (int)((asserv_trace_buffer[i].motor_pwm_right-asserv_trace_buffer[i].motor_pwm_left)*50));
          if(i%5 == 0)
          {
            printf("\n");
          }
        }
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
    printf("(1) Straight line\n(2) Rotation\n(4) Tune distance PID\n(5) Tune heading PID\n(6) Recalage\n (7) Switch\n(q) Quit\n Enter command: \n");
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
      case 6:
      {
        float pwm;
        get_float_value("PWM: ",&pwm);
        goldo_asserv_recalage(pwm);
      }
      break;
      case 7:
      {
        int en=0;
        get_int_value("Enabled: ",&en);
        if(en)
        {
          goldo_asserv_enable();
        } else
        {
          goldo_asserv_disable();
        }
      }
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


int main_loop_test_match(void)
{
  goldo_asserv_enable();
  goldo_arms_start_match();
  goldo_odometry_set_position(s_waypoints[0].x*1e-3,s_waypoints[0].y*1e-3,0);
  int command=0;  
  while(1)
  {
    printf("(1) Set position\n (q) Quit\n Enter command: \n");
    printf("(2) Move to\n");
    printf("(3) Point to\n");
    printf("(4) Grab cylinder from rocket\n");
    printf("(5) Drop cylinder in front\n");
    printf("(6) Straight line\n");
    printf("(7) Rotation line\n");
    printf("(8) Show odometry\n");
    printf("(9) Position arm\n");
    printf("(a) Move to waypoint\n");
    command = 0;
    get_char_value("Command: ",&command);
    switch(command)
    {
      case '1':
      {
        int x,y,heading;        
        get_int_value("Pos x (mm): ",&x);   
        get_int_value("Pos y (mm): ",&y);    
        get_int_value("Heading (deg): ",&heading); 
        goldo_odometry_set_position(x*1e-3,y*1e-3,heading*M_PI/180);
      }          
      break;
      case '2':
      {
        int x,y;        
        get_int_value("Pos x (mm): ",&x);   
        get_int_value("Pos y (mm): ",&y);
        move_to(x*1e-3,y*1e-3,true);
      }
      break;
      case '3':
      {
        int x,y;        
        get_int_value("Pos x (mm): ",&x);   
        get_int_value("Pos y (mm): ",&y);
        point_to(x*1e-3,y*1e-3);
      }
      break;

      case '6':
      {
        int distance;
        get_int_value("Distance x (mm): ",&distance);
        goldo_asserv_straight_line(distance*1e-3,0.2,0.2,0.2); 
        goldo_asserv_wait_finished(); 
      }
      break;    
      case '7':
      {
        int angle;
        get_int_value("Angle x (deg): ",&angle);
        goldo_asserv_rotation(angle*M_PI/180,M_PI/2,M_PI/2,M_PI/2);
        goldo_asserv_wait_finished(); 
      }
      break;
      case '8':
        printf("(x,y,heading) = %i,%i,%i\n",(int)(g_odometry_state.pos_x*1e3),(int)(g_odometry_state.pos_y*1e3),(int)(g_odometry_state.heading*180/M_PI));
        break; 
      case '9':
      {
        int pos;
        get_int_value("Position (id): ",&pos);
        goldo_arms_move_to_position(GOLDO_ARM_LEFT,pos);
      }
      break; 
       case 'a':
      {
        int id;
        get_int_value("Waypoint (id): ",&id);
        move_to(s_waypoints[id].x*1e-3,s_waypoints[id].y*1e-3,true);
      }
      break;
      case 'f':
        goldo_robot_do_funny_action();
        break;
      case '5':
        goldo_arms_load_cylinder(GOLDO_ARM_LEFT);
        goldo_arms_drop_cylinder_front_swipe(GOLDO_ARM_LEFT);
        break;
        break;
      case '4':
        goldo_arms_grab_rocket_cylinder(GOLDO_ARM_LEFT);
        goldo_arms_store_cylinder(GOLDO_ARM_LEFT);
        break;
      case 5:
        tune_pid_heading();
        break;
        case 'm':
        match_core(true);
        break;
      case 'q':
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

int main_loop_utest_funny_action(void)
{
  goldo_robot_do_funny_action();
}


int dynamixel_get_current_position(int id);
int goldo_dynamixels_init(void);
void dynamixel_set_led(int id, int enable);
void SetTorque(int id,int value);

//void goldo_dynamixels_set_position(int id,int pos);
//void goldo_dynamixels_set_position_sync(int id,int pos);
//void goldo_dynamixels_do_action(int id);

extern void goldo_pump1_speed(int32_t s);
extern void goldo_pump2_speed(int32_t s);

int main_loop_test_dynamixels(void)
{
  int id;
  int position;
  char command;

  char last_pump_id='\0';
  int last_pump_speed=0;


  while(1)
  {
    printf("Dynamixels test\n");    
    printf("\n");
    printf("Set position (s), Get position (g), Led(l), Torque (t), Pump(p), Grab(G), Quit (q)\n");
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
        goldo_dynamixels_set_position(id,position);
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
      case 'p':
        get_char_value("Left(l), Right(r): ",&command);
        get_int_value("PWM (1-65000): ",&position);
        if (position>65535) position=65535;
        if (position<-65535) position=-65535;
        if (command=='l') {
          last_pump_id='l';
          last_pump_speed=position;
          goldo_pump2_speed(position);
        } else if (command=='r') {
          last_pump_id='r';
          last_pump_speed=position;
          goldo_pump1_speed(position);
        }
        break;
      case 'G':
        get_char_value("Left(l), Right(r): ",&command);
        goldo_arms_grab(0);
        break;
      case '!':
        goldo_pump1_speed(0);
        goldo_pump2_speed(0);
        break;
      case '*':
        printf ("last_pump_id= %c ; last_pump_speed= %d\n", last_pump_id, last_pump_speed);
        if (last_pump_id=='l') {
          printf ("l\n");
          goldo_pump2_speed(last_pump_speed);
        } else if (last_pump_id=='r') {
          printf ("r\n");
          goldo_pump1_speed(last_pump_speed);
        }
        break;
      case 'q':
        return OK;
        break;
    }
  }
}

int main_loop_test_arms(void)
{
  int position;
  char command;
  goldo_arms_start_match();

 while(1)
  {
    printf("Arms test\n");    
    printf("\n");
    printf("Set position (s), Move Barrel (b), Grab in position(g), Grab(G), Drop(d), Quit (q)\n");
    get_char_value("Command: ",&command);
    switch(command)
    {      
      case 's':
        get_int_value("Position: ",&position);
        goldo_arms_move_to_position(GOLDO_ARM_LEFT,position);
        break;
      case 'b':
        get_int_value("Position: ",&position);
        goldo_arms_move_barrel(position);
        break;
      case 'g':
        get_int_value("Position: ",&position);
        goldo_arms_grab_in_position(GOLDO_ARM_LEFT,position);
        break;
      case 'd':
        goldo_arms_drop(0);
        break;
      case 'G':
        goldo_arms_grab(0);
        break;
      case '1':
        goldo_arms_goto_rest_position(GOLDO_ARM_LEFT);
        break;
      case '2':
        goldo_arms_grab_rocket_cylinder(GOLDO_ARM_LEFT);
        break;
      case '3':
        goldo_arms_store_cylinder(GOLDO_ARM_LEFT);
        break;
      case '4':
        goldo_arms_load_cylinder(GOLDO_ARM_LEFT);
        break;
      case '5':
        goldo_arms_drop_cylinder_front(GOLDO_ARM_LEFT);
        break;
      case '6':
        goldo_arms_drop_cylinder_front_swipe(GOLDO_ARM_LEFT);
        break;
      case 'q':
        return OK;
        break;
    }
  }
}
