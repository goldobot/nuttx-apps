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

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <semaphore.h>
#include <pthread.h>
#include <signal.h>
#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <string.h>

#include <nuttx/semaphore.h>
#include <nuttx/timers/timer.h>
#include <nuttx/drivers/pwm.h>
#include <nuttx/sensors/qencoder.h>

#include <system/readline.h>
#include <math.h>

#if 1 /* FIXME : DEBUG : test Goldo 03/05/2018 */
int enable_simul_obstacle = 0;
int enable_detect_obstacle = 0;
volatile int simul_obstacle_flag = 0;
volatile int obstacle_flag = 0;
#define MATCH_VERT   1
//#define MATCH_ORANGE 1

void *simul_obstacle(void *arg);
void *detect_obstacle(void *arg);
void match_goldo(void);
void servo_bac(int pos);
void servo_bac_d(int pos);
void servo_bac_g(int pos);
void servo_trappe(int pos);

extern int goldo_get_start_gpio_state(void);
extern int goldo_get_obstacle_gpio_state(void);
extern int goldo_get_couleur_gpio_state(void);

int fd_servo_bac_g = -1;
int fd_servo_bac_d = -1;
int fd_servo_trappe = -1;
#endif

static char s_input_buffer[32];

typedef enum GOLDO_OPCODE
{
  GOP_MOVE_TO_WAYPOINT,
  GOP_LOAD_CYLINDER_FROM_ROCKET,
  GOP_DROP_CYLINDER_FRONT,
  GOP_DROP_CYLINDER_FRONT_SWIPE
} GOLDO_OPCODE;

typedef enum _waypoint_type
{
  WAYPOINT_INVALID = 0,
  WAYPOINT_X_Y,
  WAYPOINT_X_Y_THETA,
  WAYPOINT_HOME_FRONT,
  WAYPOINT_HOME_BACK,
  WAYPOINT_HOME_RIGHT,
  WAYPOINT_HOME_LEFT,
} waypoint_type_t;

typedef struct goldo_waypoint_s
{
  waypoint_type_t type;
  float x;
  float y;
  float theta;
} goldo_waypoint_s;

#if 0 /* FIXME : TODO : remove : ancien code 2017 */
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
#else

#if 0
static goldo_waypoint_s s_waypoints_green[] = {
  {WAYPOINT_X_Y_THETA,    550,   290,  45}, //  0
  {WAYPOINT_X_Y,          800,   640,   0}, //  1
  {WAYPOINT_X_Y,         1400,   640,   0}, //  2
  {WAYPOINT_X_Y,         1600,   250,   0}, //  3
  {WAYPOINT_X_Y,         1800,   250,   0}, //  4
  {WAYPOINT_HOME_FRONT,  1900,   250,   0}, //  5
  {WAYPOINT_X_Y,         1600,   250,   0}, //  6
  {WAYPOINT_X_Y,         1200,  1000,   0}, //  7
  {WAYPOINT_X_Y,         1200,  2000,   0}, //  8
  {WAYPOINT_X_Y,         1600,  2600,   0}, //  9
  {WAYPOINT_X_Y,          550,  1150,   0}, // 10
  {WAYPOINT_X_Y,          300,  1050,   0}, // 11
  {WAYPOINT_HOME_BACK,    100,  1050, 180}, // 12
  {WAYPOINT_INVALID,      100,  1050,   0}, // 13
};

static goldo_waypoint_s s_waypoints_orange[] = {
  {WAYPOINT_X_Y_THETA,    550,  -290, -45}, //  0
  {WAYPOINT_X_Y,          800,  -640,   0}, //  1
  {WAYPOINT_X_Y,         1400,  -640,   0}, //  2
  {WAYPOINT_X_Y,         1600,  -160,   0}, //  3
  {WAYPOINT_X_Y,         1800,  -140,   0}, //  4
  {WAYPOINT_HOME_FRONT,  1900,  -140,   0}, //  5
  {WAYPOINT_X_Y,         1600,  -160,   0}, //  6
  {WAYPOINT_X_Y,         1200, -1000,   0}, //  7
  {WAYPOINT_X_Y,         1200, -2000,   0}, //  8
  {WAYPOINT_X_Y,         1900, -2400,   0}, //  9
  {WAYPOINT_X_Y,          550, -1320,   0}, // 10
  {WAYPOINT_X_Y,          300, -1350,   0}, // 11
  {WAYPOINT_HOME_BACK,    100, -1350, 180}, // 12
  {WAYPOINT_INVALID,      100, -1350,   0}, // 13
};
#else
static goldo_waypoint_s s_waypoints_green[] = {
  {WAYPOINT_X_Y_THETA,    520,   260,  45}, //  0
  {WAYPOINT_X_Y,          800,   640,   0}, //  1
  {WAYPOINT_X_Y,         1400,   640,   0}, //  2
  {WAYPOINT_X_Y,         1600,   250,   0}, //  3
  {WAYPOINT_X_Y,         1800,   250,   0}, //  4
  {WAYPOINT_HOME_FRONT,  1900,   250,   0}, //  5
  {WAYPOINT_X_Y,         1600,   250,   0}, //  6

  {WAYPOINT_X_Y,         1200,  1000,   0}, //  7
  {WAYPOINT_X_Y,          600,   850,   0}, //  8
  {WAYPOINT_X_Y,          300,   850,   0}, //  9

  {WAYPOINT_X_Y,          550,  1150,   0}, // 10
  {WAYPOINT_X_Y,          300,  1050,   0}, // 11
  {WAYPOINT_HOME_BACK,    100,  1050, 180}, // 12
  {WAYPOINT_INVALID,      100,  1050,   0}, // 13
};

static goldo_waypoint_s s_waypoints_orange[] = {
  {WAYPOINT_X_Y_THETA,    520,  -260, -45}, //  0
  {WAYPOINT_X_Y,          800,  -640,   0}, //  1
  {WAYPOINT_X_Y,         1400,  -640,   0}, //  2
  {WAYPOINT_X_Y,         1600,  -160,   0}, //  3
  {WAYPOINT_X_Y,         1800,  -140,   0}, //  4
  {WAYPOINT_HOME_FRONT,  1900,  -140,   0}, //  5
  {WAYPOINT_X_Y,         1600,  -160,   0}, //  6

  {WAYPOINT_X_Y,         1200, -1000,   0}, //  7
  {WAYPOINT_X_Y,          600,  -850,   0}, //  8
  {WAYPOINT_X_Y,          300,  -800,   0}, //  9

  {WAYPOINT_X_Y,          550, -1320,   0}, // 10
  {WAYPOINT_X_Y,          300, -1350,   0}, // 11
  {WAYPOINT_HOME_BACK,    100, -1350, 180}, // 12
  {WAYPOINT_INVALID,      100, -1350,   0}, // 13
};
#endif

#ifdef MATCH_VERT /* VERT */
static goldo_waypoint_s *s_waypoints = s_waypoints_green;
#else /* ORANGE */
static goldo_waypoint_s *s_waypoints = s_waypoints_orange;
#endif

#endif

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
  while(true) {
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
  if(delta>M_PI) {
    delta -= M_PI*2;
  } else if(delta < -M_PI*2) {
    delta += M_PI*2;
  }
#if 0 /* FIXME : DEBUG : test Goldo 03/05/2018 */
  goldo_asserv_rotation(delta,M_PI/4,M_PI/4,M_PI/4);
#else
  goldo_asserv_rotation(delta,M_PI/3,M_PI/6,M_PI/6);
#endif
  goldo_asserv_wait(1.5);
  goldo_asserv_wait_finished();  
}

#if 1 /* FIXME : DEBUG : test Goldo 03/05/2018 */

void turn_to(float new_theta)
{
  float delta = new_theta - g_odometry_state.heading;
  if(delta>M_PI) {
    delta -= M_PI*2;
  } else if(delta < -M_PI*2) {
    delta += M_PI*2;
  }
  goldo_asserv_rotation(delta,M_PI/3,M_PI/6,M_PI/6);
  goldo_asserv_wait(1.5);
  goldo_asserv_wait_finished();  
}

int move_to(float x, float y, bool forward)
{
  int result;

  float dx;
  float dy;
  float direction;
  float distance;
  float delta;

  dx = (x - g_odometry_state.pos_x);
  dy = (y - g_odometry_state.pos_y);
  direction = atan2(dy,dx);
  distance = sqrt(dx*dx+dy*dy);

  if(!forward) {
    distance *= -1;
    direction += M_PI;
    if(direction > M_PI) {
      direction -= 2*M_PI;
    }
  }
  delta = direction - g_odometry_state.heading;
  
  if(delta>M_PI) {
    delta -= M_PI*2;
  } else if(delta < -M_PI*2) {
    delta += M_PI*2;
  }

  //goldo_asserv_rotation(delta,M_PI/3,M_PI/6,M_PI/6);
  goldo_asserv_rotation(delta,2.0*M_PI/3,M_PI/3,M_PI/3);
  goldo_asserv_wait(0.5);

  //goldo_asserv_straight_line(distance,0.3,0.2,0.2);
  goldo_asserv_straight_line(distance,0.2,0.1,0.1);
  //goldo_asserv_straight_line(distance,0.4,0.2,0.2);
  //goldo_asserv_straight_line(distance,0.25,0.2,0.2);

  goldo_asserv_wait(0.5);
  result = goldo_asserv_wait_finished();  

  if (result==ASSERV_STATE_EMERGENCY_STOP) {
    goldo_asserv_disable();
    goldo_asserv_enable();
    goldo_asserv_wait(0.5);
    goldo_asserv_wait_finished();  
  }

  return result;
}

#else /* FIXME : DEBUG : old code (Thomas) */

void move_to(float x, float y, bool forward)
{
  float dx = (x - g_odometry_state.pos_x);
  float dy = (y - g_odometry_state.pos_y);
  float direction = atan2(dy,dx);
  float distance = sqrt(dx*dx+dy*dy);
  if(!forward) {
    distance *= -1;
    direction += M_PI;
    if(direction > M_PI) {
      direction -= 2*M_PI;
    }
  }
  float delta = direction - g_odometry_state.heading;
  
  if(delta>M_PI) {
    delta -= M_PI*2;
  } else if(delta < -M_PI*2) {
    delta += M_PI*2;
  }
  goldo_asserv_rotation(delta,M_PI/4,M_PI/4,M_PI/4);
  goldo_asserv_wait(0.5);
  goldo_asserv_straight_line(distance,0.1,0.1,0.2);
  goldo_asserv_wait(0.5);
  goldo_asserv_wait_finished();  
}

#endif

void move_to_waypoint(int i, bool forward)
{
  move_to(s_waypoints[i].x*1e-3,s_waypoints[i].y*1e-3,forward);
}


/* FIXME : TODO : remove : ancien code Thomas 2017 */
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

  return OK;
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
    printf("(1) Set left motor pwm\n(2) Set right motor_pwm\n(3) Quit (and disable motors)\n Enter command: \n");
    command = 0;
    readline(buffer,32,stdin,stdout);
    sscanf(buffer,"%d",&command);

    switch(command) {
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
      goldo_asserv_hal_set_motors_pwm(0,0);
      goldo_asserv_hal_set_motors_enable(false,false);
      return OK;
    default:
      break;
    }
    goldo_asserv_hal_set_motors_pwm(pwm_left,pwm_right);
  }
}

int main_loop_test_odometry(void)
{
  while(1) {
    printf("encoders:%i, %i pos: %i,%i heading:%f(%i) speed: %i          \r", 
           (int)g_odometry_state.counts_left,(int)g_odometry_state.counts_right,
           (int)(g_odometry_state.pos_x*1000),
           (int)(g_odometry_state.pos_y*1000),
           (g_odometry_state.heading),
           (int)(g_odometry_state.heading*180/M_PI),
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
    switch(command) {
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
          if(i%5 == 0) {
            printf("\n");
          }
        }
      break;
    case 's':
      goldo_asserv_start_trace(asserv_trace_buffer,50,10);
      goldo_asserv_position_step(0.2);
      goldo_asserv_wait(5);
      goldo_asserv_wait_finished();
      for(i=0;i<50;i++) {
        printf("%i,%i,%i,%i,%i\t",
               (int)(asserv_trace_buffer[i].elapsed_distance_setpoint*1000),
               (int)(asserv_trace_buffer[i].elapsed_distance*1000),
               (int)(asserv_trace_buffer[i].speed_setpoint*1000),
               (int)(asserv_trace_buffer[i].speed*1000),
               (int)((asserv_trace_buffer[i].motor_pwm_left+asserv_trace_buffer[i].motor_pwm_right)*50));
        if(i%5 == 0) {
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
  while(1) {
    printf("Tune heading pid\n");
    printf("K_P: %f\n",k_p);
    printf("K_I: %f\n",k_i);
    printf("K_D: %f\n",k_d);
    printf("Integrator limit: %f\n",i_limit);
    printf("Feedforward speed: %f\n",speed_ff);
    printf("\n");
    printf("Set (p), Set(i), Set (d), Set(l), Set (f), Test (t,T), BLock (b), Quit (q)\n");
    get_char_value("Command: ",&command);
    switch(command) {
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
      for(i=0;i<50;i++) {
        printf("%i,%i,%i,%i,%i\t",
               (int)(asserv_trace_buffer[i].heading_change_setpoint*180/M_PI),
               (int)(asserv_trace_buffer[i].heading_change*180/M_PI),
               (int)(asserv_trace_buffer[i].yaw_rate_setpoint*180/M_PI),
               (int)(asserv_trace_buffer[i].yaw_rate*180/M_PI),
               (int)((asserv_trace_buffer[i].motor_pwm_right-asserv_trace_buffer[i].motor_pwm_left)*50));
        if(i%5 == 0) {
          printf("\n");
        }
      }
      break;
    case 's':
      goldo_asserv_start_trace(asserv_trace_buffer,50,10);
      goldo_asserv_heading_step(90*M_PI/180);
      goldo_asserv_wait(5);
      goldo_asserv_wait_finished();
      for(i=0;i<50;i++) {
        printf("%i,%i,%i,%i,%i\t",
               (int)(asserv_trace_buffer[i].heading_change_setpoint*180/M_PI),
               (int)(asserv_trace_buffer[i].heading_change*180/M_PI),
               (int)(asserv_trace_buffer[i].yaw_rate_setpoint*180/M_PI),
               (int)(asserv_trace_buffer[i].yaw_rate*180/M_PI),
               (int)((asserv_trace_buffer[i].motor_pwm_right-asserv_trace_buffer[i].motor_pwm_left)*50));
        if(i%5 == 0) {
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
  while(1) {
    printf("(1) Straight line\n(2) Rotation\n(4) Tune distance PID\n(5) Tune heading PID\n(6) Recalage\n (7) Switch\n(q) Quit\n Enter command: \n");
    command = 0;
    readline(buffer,32,stdin,stdout);
    sscanf(buffer,"%d",&command);
    if(buffer[0] == 'q') {
      return OK;
    }

    switch(command) {
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
        if(en) {
          goldo_asserv_enable();
        } else {
          goldo_asserv_disable();
        }
      }
    default:
      break;
    }
  }

  //goldo_asserv_straight_line(0.5, 0.5, 0.5, 0.5);
  //goldo_asserv_straight_line(-0.5, 0.5, 0.5, 0.5);
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
  while(1) {
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
    printf("(m) MATCH!\n");
    printf("(q) Quit\n");
    command = 0;
    get_char_value("Command: ",(char *)&command);
    switch(command) {
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
  while(1) {
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
  return OK;
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


  while(1) {
    printf("Dynamixels test\n");    
    printf("\n");
    printf("Set position (s), Get position (g), Led(l), Torque (t), Pump(p), Grab(G), Quit (q)\n");
    get_char_value("Command: ",&command);
    switch(command) {
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

 while(1) {
   printf("Arms test\n");    
   printf("\n");
   printf("Set position (s), Move Barrel (b), Grab in position(g), Grab(G), Drop(d), Quit (q)\n");
   get_char_value("Command: ",&command);
   switch(command) {      
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



/*************************************************************************/
/* GOLDO * GOLDO * GOLDO * GOLDO * GOLDO * GOLDO * GOLDO * GOLDO * GOLDO */
/*************************************************************************/

void goldo_print_pos(void)
{
  printf("\n\n\n");
  printf("TEST_GOLDO encoders:%i, %i pos: %i,%i heading:%f(%i) speed: %i\n", 
    (int)g_odometry_state.counts_left,(int)g_odometry_state.counts_right,
    (int)(g_odometry_state.pos_x*1000),
    (int)(g_odometry_state.pos_y*1000),
    (g_odometry_state.heading),
    (int)(g_odometry_state.heading*180/M_PI),
    (int)(g_odometry_state.speed*1000));
  printf("\n\n\n");
}

void *simul_obstacle(void *arg)
{
  unsigned char c;
  while(1) {
    if (enable_simul_obstacle!=0) {
      c = getchar();
      if (c==115) {
        printf("BIM!\n");
        simul_obstacle_flag = 1;
      } else if (c==103) {
        printf("GO!\n");
        simul_obstacle_flag = 0;
      }
    }

    if (simul_obstacle_flag!=0) {
      goldo_asserv_emergency_stop();
      usleep (500000);
    }

    usleep (20000);
  }
  return NULL;
}

void *detect_obstacle(void *arg)
{
  volatile int obstacle_gpio_state_old = 0;
  volatile int obstacle_gpio_state = 0;

  while(1) {
    if (enable_detect_obstacle!=0) {
      obstacle_gpio_state_old = obstacle_gpio_state;
      obstacle_gpio_state = goldo_get_obstacle_gpio_state();
      if (obstacle_gpio_state!=0) {
        //printf("BIM!\n");
        obstacle_flag = 1;
      } else {
        //printf("GO!\n");
        obstacle_flag = 0;
      }
    }

    if ((obstacle_gpio_state_old==0) && (obstacle_gpio_state!=0)) {
      printf("\n\n");
      printf("********************\n");
      printf("      BLOCAGE      *\n");
      printf("********************\n");
      printf("\n\n");
      goldo_asserv_emergency_stop();
      usleep (500000);
    } else if ((obstacle_gpio_state_old!=0) && (obstacle_gpio_state==0)) {
      printf("\n\n");
      printf("********************\n");
      printf("     DEBLOCAGE     *\n");
      printf("********************\n");
      printf("\n\n");
    }

    //usleep (20000);
    usleep (100000);
  }
  return NULL;
}

void goldo_homing_init(void)
{
  int i;
  goldo_asserv_disable();
  goldo_asserv_hal_set_motors_enable(true,true);
  goldo_asserv_hal_set_motors_pwm(20000,20000);
  for (i=0; i<10; i++) {
    usleep(100000);
  }
  goldo_asserv_hal_set_motors_pwm(40000,40000);
  for (i=0; i<10; i++) {
    usleep(100000);
  }
}

void goldo_homing_end(void)
{
  int i;
  goldo_asserv_hal_set_motors_pwm(-20000,-20000);
  for (i=0; i<5; i++) {
    usleep(100000);
  }
  goldo_asserv_hal_set_motors_pwm(0,0);
  goldo_asserv_hal_set_motors_enable(false,false);
  goldo_asserv_enable();
}

void goldo_homing(waypoint_type_t type)
{
  switch (type) {
  case WAYPOINT_HOME_FRONT:
    goldo_homing_init();
    g_odometry_state.pos_x = 1.900;
    g_odometry_state.heading = 0.0f*M_PI/180;
    goldo_homing_end();
    break;
  case WAYPOINT_HOME_BACK:
    goldo_homing_init();
    g_odometry_state.pos_x = 0.100;
    g_odometry_state.heading = 180.0f*M_PI/180;
    goldo_homing_end();
    break;
  case WAYPOINT_HOME_RIGHT:
    goldo_homing_init();
    /* FIXME : TODO : OK? (pour "deviner" la couleur) */
    if (g_odometry_state.pos_y>0) g_odometry_state.pos_y = 0.100; /*vert*/
    else g_odometry_state.pos_y = -0.100; /*orange*/
    g_odometry_state.heading = -90.0f*M_PI/180;
    goldo_homing_end();
    break;
  case WAYPOINT_HOME_LEFT:
    goldo_homing_init();
    /* FIXME : TODO : OK? (pour "deviner" la couleur) */
    if (g_odometry_state.pos_y>0) g_odometry_state.pos_y = 2.900; /*vert*/
    else g_odometry_state.pos_y = -2.900; /*orange*/
    g_odometry_state.heading = 90.0f*M_PI/180;
    goldo_homing_end();
    break;
  case WAYPOINT_INVALID:
  case WAYPOINT_X_Y:
  case WAYPOINT_X_Y_THETA:
  default:
    break;
  }
}

void init_servos(void)
{
  struct pwm_info_s info_servo;
  int ret;

  fd_servo_bac_g = open("/dev/pwm16", O_RDONLY);
  if (fd_servo_bac_g < 0) {
    printf("servo_bac_g: open /dev/pwm16 failed: %d\n", errno);
  }

  fd_servo_bac_d = open("/dev/pwm15", O_RDONLY);
  if (fd_servo_bac_d < 0) {
    printf("servo_bac_d: open /dev/pwm15 failed: %d\n", errno);
  }

  fd_servo_trappe = open("/dev/pwm8", O_RDONLY);
  if (fd_servo_trappe < 0) {
    printf("servo_trappe: open /dev/pwm8 failed: %d\n", errno);
  }

  info_servo.frequency = 100;
  info_servo.duty = 0x1c28;
  if (fd_servo_bac_g >= 0) {
    ret = ioctl(fd_servo_bac_g, PWMIOC_SETCHARACTERISTICS,
                (unsigned long)((uintptr_t)&info_servo));
    if (ret < 0) {
      printf("servo_bac_g: ioctl(PWMIOC_SETCHARACTERISTICS) failed:%d\n",errno);
    }
    ret = ioctl(fd_servo_bac_g, PWMIOC_START, 0);
    if (ret < 0) {
      printf("servo_bac_g: ioctl(PWMIOC_START) failed: %d\n", errno);
    }
  }

  info_servo.frequency = 100;
  info_servo.duty = 0x30a3;
  if (fd_servo_bac_d >= 0) {
    ret = ioctl(fd_servo_bac_d, PWMIOC_SETCHARACTERISTICS,
                (unsigned long)((uintptr_t)&info_servo));
    if (ret < 0) {
      printf("servo_bac_d: ioctl(PWMIOC_SETCHARACTERISTICS) failed:%d\n",errno);
    }
    ret = ioctl(fd_servo_bac_d, PWMIOC_START, 0);
    if (ret < 0) {
      printf("servo_bac_d: ioctl(PWMIOC_START) failed: %d\n", errno);
    }
  }

  info_servo.frequency = 100;
  info_servo.duty = 0x3000;
  if (fd_servo_trappe >= 0) {
    ret = ioctl(fd_servo_trappe, PWMIOC_SETCHARACTERISTICS,
                (unsigned long)((uintptr_t)&info_servo));
    if (ret < 0) {
      printf("servo_trappe:ioctl(PWMIOC_SETCHARACTERISTICS) failed:%d\n",errno);
    }
    ret = ioctl(fd_servo_trappe, PWMIOC_START, 0);
    if (ret < 0) {
      printf("servo_trappe: ioctl(PWMIOC_START) failed: %d\n", errno);
    }
  }
}

#define SERVO_BAC_MEDIAN_VAL 10650
#define SERVO_BAC_MAX_DELTA   5000
void servo_bac(int pos)
{
  struct pwm_info_s info_servo;
  int ret;

  if ((fd_servo_bac_g < 0) || (fd_servo_bac_d < 0)) {
    return;
  }

  if ((pos < (-SERVO_BAC_MAX_DELTA)) || (pos > (SERVO_BAC_MAX_DELTA))) {
    return;
  }

  info_servo.frequency = 100;
  info_servo.duty = SERVO_BAC_MEDIAN_VAL - pos;
  ret = ioctl(fd_servo_bac_g, PWMIOC_SETCHARACTERISTICS,
              (unsigned long)((uintptr_t)&info_servo));
  if (ret < 0) {
    printf("servo_bac_g: ioctl(PWMIOC_SETCHARACTERISTICS) failed:%d\n",errno);
    return;
  }

  info_servo.frequency = 100;
  info_servo.duty = SERVO_BAC_MEDIAN_VAL + pos;
  ret = ioctl(fd_servo_bac_d, PWMIOC_SETCHARACTERISTICS,
              (unsigned long)((uintptr_t)&info_servo));
  if (ret < 0) {
    printf("servo_bac_d: ioctl(PWMIOC_SETCHARACTERISTICS) failed:%d\n",errno);
    return;
  }
}

void servo_trappe(int pos)
{
  struct pwm_info_s info_servo;
  int ret;

  if ((fd_servo_trappe < 0)) {
    return;
  }

  info_servo.frequency = 100;
  info_servo.duty = pos;
  ret = ioctl(fd_servo_trappe, PWMIOC_SETCHARACTERISTICS,
              (unsigned long)((uintptr_t)&info_servo));
  if (ret < 0) {
    printf("servo_trappe: ioctl(PWMIOC_SETCHARACTERISTICS) failed:%d\n",errno);
    return;
  }
}

void servo_bac_d(int pos)
{
  struct pwm_info_s info_servo;
  int ret;

  if ((fd_servo_bac_d < 0)) {
    return;
  }

  info_servo.frequency = 100;
  info_servo.duty = pos;
  ret = ioctl(fd_servo_bac_d, PWMIOC_SETCHARACTERISTICS,
              (unsigned long)((uintptr_t)&info_servo));
  if (ret < 0) {
    printf("servo_bac_d: ioctl(PWMIOC_SETCHARACTERISTICS) failed:%d\n",errno);
    return;
  }
}

void servo_bac_g(int pos)
{
  struct pwm_info_s info_servo;
  int ret;

  if ((fd_servo_bac_g < 0)) {
    return;
  }

  info_servo.frequency = 100;
  info_servo.duty = pos;
  ret = ioctl(fd_servo_bac_g, PWMIOC_SETCHARACTERISTICS,
              (unsigned long)((uintptr_t)&info_servo));
  if (ret < 0) {
    printf("servo_bac_g: ioctl(PWMIOC_SETCHARACTERISTICS) failed:%d\n",errno);
    return;
  }
}

void match_goldo(void)
{
  int start_gpio_state = 0;
  int obstacle_gpio_state = 0;
  int couleur_gpio_state = 0;

  pthread_t id_simul_obstacle;
  pthread_t id_detect_obstacle;

  int result;
  goldo_waypoint_s *my_wp;

  //init_servos();

  enable_simul_obstacle = 0;
  enable_detect_obstacle = 0;

  pthread_create(&id_simul_obstacle, NULL, simul_obstacle, NULL);
  pthread_setschedprio(id_simul_obstacle, PTHREAD_DEFAULT_PRIORITY);

  pthread_create(&id_detect_obstacle, NULL, detect_obstacle, NULL);
  pthread_setschedprio(id_detect_obstacle, PTHREAD_DEFAULT_PRIORITY);


  obstacle_gpio_state = goldo_get_obstacle_gpio_state();
  printf ("obstacle_gpio_state = %d\n", obstacle_gpio_state);

  start_gpio_state = goldo_get_start_gpio_state();
  printf ("start_gpio_state = %d\n", start_gpio_state);

  couleur_gpio_state = goldo_get_couleur_gpio_state();
  printf ("couleur_gpio_state = %d\n", couleur_gpio_state);

  while (start_gpio_state==0) {
    start_gpio_state = goldo_get_start_gpio_state();
    usleep(20000);
  }

  /* GO * GO * GO!!!!! */

  //servo_bac(-3000);
  servo_bac_d(11000);

  goldo_asserv_enable();

  enable_simul_obstacle = 1;
  enable_detect_obstacle = 1;
  obstacle_flag = 0;
  simul_obstacle_flag = 0;

  if (couleur_gpio_state==0) { /* 0=vert */
    my_wp = s_waypoints_green;
  } else { /* 1=orange */
    my_wp = s_waypoints_orange;
  }

  goldo_odometry_set_position(my_wp->x*1e-3, my_wp->y*1e-3, 
                              my_wp->theta*M_PI/180);
  my_wp++;

  while (my_wp->type!=WAYPOINT_INVALID) {
    switch (my_wp->type) {
    case WAYPOINT_INVALID:
      goto end;
    case WAYPOINT_X_Y:
      do {
        result = move_to(my_wp->x*1e-3, my_wp->y*1e-3, /*forward=*/true);
        if (result==ASSERV_STATE_EMERGENCY_STOP) {
          usleep(500000);
          while ((obstacle_flag != 0) || 
                 (simul_obstacle_flag != 0)) usleep(100000);
        }
      } while (result==ASSERV_STATE_EMERGENCY_STOP);
      break;
    case WAYPOINT_X_Y_THETA:
      do {
        result = move_to(my_wp->x*1e-3, my_wp->y*1e-3, /*forward=*/true);
        if (result==ASSERV_STATE_EMERGENCY_STOP) {
          usleep(500000);
          while ((obstacle_flag != 0) || 
                 (simul_obstacle_flag != 0)) usleep(100000);
        }
      } while (result==ASSERV_STATE_EMERGENCY_STOP);
      turn_to(my_wp->theta*M_PI/180);
      break;
    case WAYPOINT_HOME_FRONT:
    case WAYPOINT_HOME_BACK:
    case WAYPOINT_HOME_RIGHT:
    case WAYPOINT_HOME_LEFT:
      goldo_homing(my_wp->type);
      break;
    default:
      goto end;
    }
    my_wp++;
  }

 end:
  enable_simul_obstacle = 0;
  enable_detect_obstacle = 0;
  obstacle_flag = 0;
  simul_obstacle_flag = 0;
}

void main_loop_test_servos(void)
{
  int command=0;
  int servo_pos=0;
  char buffer[32];

  //init_servos();

  while(1)
  {
    printf("(1) Cmd servo 'bac'\n(2) Cmd servo 'trappe'\n(3) Cmd servo bac d\n(4) Cmd servo bac g\n(5) Quit\n Enter command: \n");
    command = 0;
    readline(buffer,32,stdin,stdout);
    sscanf(buffer,"%d",&command);

    switch(command) {
    case 1:
      printf("Input pos (0,60000): ");
      fflush(stdout);
      readline(buffer,32,stdin,stdout);
      sscanf(buffer,"%d",&servo_pos);
      servo_bac(servo_pos);
      break;
    case 2:
      printf("Input pos (0,60000): ");
      fflush(stdout);
      readline(buffer,32,stdin,stdout);
      sscanf(buffer,"%d",&servo_pos);
      servo_trappe(servo_pos);
      break;
    case 3:
      printf("Input pos (0,60000): ");
      fflush(stdout);
      readline(buffer,32,stdin,stdout);
      sscanf(buffer,"%d",&servo_pos);
      servo_bac_d(servo_pos);
      break;
    case 4:
      printf("Input pos (0,60000): ");
      fflush(stdout);
      readline(buffer,32,stdin,stdout);
      sscanf(buffer,"%d",&servo_pos);
      servo_bac_g(servo_pos);
      break;
    case 5:
    default:
      goto end;
    }
  }
 end:
  return;
}


