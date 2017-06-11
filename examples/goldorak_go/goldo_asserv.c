#include "goldo_asserv.h"
#include "goldo_asserv_hal.h"
#include "goldo_odometry.h"
#include "goldo_polyline.h"

//#include <sys/types.h>
//#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
//#include <debug.h>
#include <semaphore.h>
#include <pthread.h>

//#include <nuttx/init.h>
//#include <nuttx/arch.h>
//#include <string.h>

/* Configuration */


typedef enum GOLDO_ASSERV_COMMAND
{
  GOLDO_ASSERV_COMMAND_STRAIGHT_LINE=0,
  GOLDO_ASSERV_COMMAND_ROTATION=1,
  GOLDO_ASSERV_COMMAND_WAIT=2,
  GOLDO_ASSERV_COMMAND_GO_TO=3,
  GOLDO_ASSERV_COMMAND_POINT_TO

} GOLDO_ASSERV_COMMAND;

typedef struct goldo_asserv_command_s
{
  GOLDO_ASSERV_COMMAND command;
  float distance;
  float speed;
  float acceleration;
  float decceleration;
  float target_x;
  float target_y;
  float target_heading;
} goldo_asserv_command_s;


typedef struct goldo_asserv_command_fifo_s
{
  goldo_asserv_command_s commands[GOLDO_ASSERV_MAX_COMMANDS];
  int index;
  int end;
  pthread_mutex_t mutex;

} goldo_asserv_command_fifo_s;


goldo_asserv_s g_asserv;
goldo_asserv_command_fifo_s s_command_fifo;

pthread_cond_t s_asserv_cond;/* Condition variable used to signal asserv events */
pthread_mutex_t s_asserv_mutex;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int command_fifo_init(void);
static void command_fifo_clear(void);
static goldo_asserv_command_s* command_fifo_current_command(void);
static void command_fifo_advance(void);
static goldo_asserv_command_s* command_fifo_begin_write(void);
static int command_fifo_end_write(void);

/* Trajectory for current command is in the form of a polyline */
static goldo_polyline_pt_s s_poly_distance_points[16];
static goldo_polyline_pt_s s_poly_heading_points[16];
static goldo_polyline_s s_poly_distance;
static goldo_polyline_s s_poly_heading;
static int s_poly_distance_index=0;
static int s_poly_heading_index=0;

uint32_t s_command_wait_end_time;

/* PID filters */
static goldo_pid_filter_s s_pid_distance;
static goldo_pid_filter_s s_pid_heading;

static float s_emergency_stop_dist;
static float s_emergency_stop_speed;

/*
  Support functions
*/


int goldo_asserv_init(void)
{
  /* init synchronization structures */
  command_fifo_init();
  pthread_cond_init(&s_asserv_cond,NULL);
  pthread_mutex_init(&s_asserv_mutex,NULL);
  goldo_asserv_hal_init();
  goldo_asserv_hal_set_motors_enable(false, false);
  goldo_asserv_hal_set_motors_pwm(0, 0);
  g_asserv.asserv_state = ASSERV_STATE_DISABLED;


  /* Initialize feedback loop values*/
  goldo_pid_filter_init(&s_pid_distance);
  goldo_pid_filter_init(&s_pid_heading);
  goldo_pid_filter_config_s pid_config_distance;
  pid_config_distance.k_p = 5;
  pid_config_distance.k_d = 0;
  pid_config_distance.k_i = 1;
  pid_config_distance.lim_i = 0.25;
  pid_config_distance.ff_speed = 0.1;
  goldo_pid_filter_set_config(&s_pid_distance,&pid_config_distance);

  goldo_pid_filter_config_s pid_config_heading;
  pid_config_heading.k_p = 2;
  pid_config_heading.k_d = 0;
  pid_config_heading.k_i = 1;
  pid_config_heading.lim_i = 0.25;
  pid_config_heading.ff_speed = 0;
  goldo_pid_filter_set_config(&s_pid_heading,&pid_config_heading);

  /*Todo cleanup: init polyline pointers*/
  s_poly_distance.points = s_poly_distance_points;
  s_poly_distance.num_points = 0;
  s_poly_heading.points = s_poly_heading_points;
  s_poly_heading.num_points = 0;
  /* Launch realtime thread */
  goldo_asserv_arch_init();
  g_asserv.initialized = true;
  return OK;
}

int goldo_asserv_quit(void)
{
  if(!g_asserv.initialized)
  {
    return OK;
  }
  pthread_cond_destroy(&s_asserv_cond);
  return goldo_asserv_arch_release();
}

int goldo_asserv_enable(void)
{
  printf("goldo_asserv: enable asserv\n");
  pthread_mutex_lock(&s_asserv_mutex);          
  if(g_asserv.asserv_state == ASSERV_STATE_DISABLED)
  {
    g_asserv.asserv_state = ASSERV_STATE_IDLE;
    g_asserv.elapsed_distance_setpoint = g_odometry_state.elapsed_distance;
    g_asserv.heading_change_setpoint = g_odometry_state.heading_change;
    pthread_cond_broadcast(&s_asserv_cond);
    pthread_mutex_unlock(&s_asserv_mutex);    
    return OK;
  }
  pthread_mutex_unlock(&s_asserv_mutex);  
  return OK;  
}



static void goldo_asserv_begin_command_straight_line(goldo_asserv_command_s* c)
{
    goldo_log(0,"%f goldo_asserv: begin execute straight line\n",g_asserv.elapsed_time_ms*1e-3);
    goldo_log(0,"goldo_asserv: speed=%f, distance=%f\n",c->speed,c->distance);
    goldo_polyline_gen_trapezoidal_1(&s_poly_distance,
                                     g_asserv.elapsed_time_ms*1e-3,
                                     g_odometry_state.elapsed_distance,
                                     g_odometry_state.elapsed_distance+c->distance,
                                     c->speed,
                                     c->acceleration,
                                     c->decceleration,
                                     1);
    s_poly_distance_index = 0;
}

static void goldo_asserv_begin_command_rotation(goldo_asserv_command_s* c)
{
    goldo_log(0,"goldo_asserv: begin execute rotation line\n");
    goldo_log(0,"goldo_asserv: speed=%f, angle=%f\n",c->speed,c->distance);
    goldo_polyline_gen_trapezoidal_1(&s_poly_heading,
                                     g_asserv.elapsed_time_ms*1e-3,
                                     g_odometry_state.heading_change,
                                     g_odometry_state.heading_change+c->distance,
                                     c->speed,
                                     c->acceleration,
                                     c->decceleration,
                                     1);
    s_poly_heading_index = 0;
}
static void goldo_asserv_begin_command(goldo_asserv_command_s* c)
{
  switch(c->command)
  {
    case GOLDO_ASSERV_COMMAND_STRAIGHT_LINE:
      goldo_asserv_begin_command_straight_line(c);
      break;
    case GOLDO_ASSERV_COMMAND_ROTATION:
      goldo_asserv_begin_command_rotation(c);
      break;
    case GOLDO_ASSERV_COMMAND_WAIT:
      goldo_log(0,"%f goldo_asserv: begin execute wait\n",g_asserv.elapsed_time_ms*1e-3);
      s_command_wait_end_time = g_asserv.elapsed_time_ms + (int)(c->distance*1000);
      break;
  }
}

/*return true if finished*/
static bool goldo_asserv_update_command(goldo_asserv_command_s* c)
{
  switch(c->command)
  {
    case GOLDO_ASSERV_COMMAND_STRAIGHT_LINE:
    {

      if(!goldo_polyline_sample(&s_poly_distance,g_asserv.elapsed_time_ms*1e-3,
        &g_asserv.elapsed_distance_setpoint,&g_asserv.speed_setpoint,&s_poly_distance_index))
      {        
        goldo_log(0,"%f goldo_asserv: end execute straight line\n",g_asserv.elapsed_time_ms*1e-3);
        return true;
      }
    }
    break;
  case GOLDO_ASSERV_COMMAND_ROTATION:
    {

      if(!goldo_polyline_sample(&s_poly_heading,g_asserv.elapsed_time_ms*1e-3,
        &g_asserv.heading_change_setpoint,&g_asserv.yaw_rate_setpoint,&s_poly_heading_index))
      {        
        goldo_log(0,"goldo_asserv: end execute rotation\n");
        return true;
      }
    }
    break;
    case GOLDO_ASSERV_COMMAND_WAIT:
      if(s_command_wait_end_time < g_asserv.elapsed_time_ms)
      {
        goldo_log(0,"%f goldo_asserv: end execute wait\n",g_asserv.elapsed_time_ms*1e-3);
        return true;
      }
      break;
    
  }

  return false;
}

int goldo_asserv_do_step(int dt_ms)
{
  g_asserv.elapsed_time_ms += dt_ms;
  goldo_asserv_command_s* current_command = command_fifo_current_command();

  switch(g_asserv.asserv_state)
  {
    case ASSERV_STATE_DISABLED:
      g_asserv.motor_pwm_left = 0;
      g_asserv.motor_pwm_right = 0;
      break;
    case ASSERV_STATE_IDLE:
      if(current_command != NULL)
      {
        goldo_log(0,"goldo_asserv: IDLE->MOVING\n");
        goldo_asserv_begin_command(current_command);

        pthread_mutex_lock(&s_asserv_mutex);
        g_asserv.asserv_state = ASSERV_STATE_MOVING;
        pthread_cond_signal(&s_asserv_cond);
        pthread_mutex_unlock(&s_asserv_mutex);
      }
      break;
    case ASSERV_STATE_MOVING:
      if(goldo_asserv_update_command(current_command))
      {
        /* If current command finished, fetch next command
           If next comand, continue moving, else switch back to idle.
        */
        command_fifo_advance();
        current_command = command_fifo_current_command();
        if(current_command != NULL)
        {
          goldo_asserv_begin_command(current_command);
        } else
        {
          goldo_log(0,"goldo_asserv: MOVING->IDLE\n");

          pthread_mutex_lock(&s_asserv_mutex);
          g_asserv.asserv_state = ASSERV_STATE_IDLE;
          pthread_cond_broadcast(&s_asserv_cond);
          pthread_mutex_unlock(&s_asserv_mutex);
        }
      };
      break;
    case ASSERV_STATE_EMERGENCY_STOP:
      {
        float d_s = dt_ms * 1e-3f*2;
        if(s_emergency_stop_speed > d_s)
        {
          s_emergency_stop_speed -= d_s;
        } 
        else if(s_emergency_stop_speed < -d_s)
        {
          s_emergency_stop_speed += d_s;
        } else
        {
          s_emergency_stop_speed =0;
        }
        s_emergency_stop_dist += s_emergency_stop_speed * dt_ms * 1e-3f;
        g_asserv.elapsed_distance_setpoint = s_emergency_stop_dist;
        g_asserv.speed_setpoint = s_emergency_stop_speed;
        g_asserv.yaw_rate_setpoint = 0;
        if(g_odometry_state.speed < 0.05 && g_odometry_state.speed > -0.05 )
        {
          goldo_log(0,"%f goldo_asserv: EMERGENCY_STOP->STOPPED\n",g_asserv.elapsed_time_ms*1e-3);
          pthread_mutex_lock(&s_asserv_mutex);
          g_asserv.asserv_state = ASSERV_STATE_STOPPED;
          pthread_cond_broadcast(&s_asserv_cond);
          pthread_mutex_unlock(&s_asserv_mutex);
        }

      }
    default:
      break;
  }
  /* Update PID loop*/
  if(g_asserv.asserv_state != ASSERV_STATE_DISABLED)
  {
    float out_pid_distance=0;
    float out_pid_heading=0;

    goldo_pid_set_target(&s_pid_distance,g_asserv.elapsed_distance_setpoint,g_asserv.speed_setpoint);
    goldo_pid_filter_do_step(&s_pid_distance,dt_ms*1e-3,g_odometry_state.elapsed_distance, g_odometry_state.speed,&out_pid_distance);

    goldo_pid_set_target(&s_pid_heading,g_asserv.heading_change_setpoint,g_asserv.yaw_rate_setpoint);
    goldo_pid_filter_do_step(&s_pid_heading,dt_ms*1e-3,g_odometry_state.heading_change, g_odometry_state.yaw_rate,&out_pid_heading);

    g_asserv.motor_pwm_left = out_pid_distance - out_pid_heading;
    g_asserv.motor_pwm_right = out_pid_distance + out_pid_heading;

  }
  

  return OK;
}

GOLDO_ASSERV_STATE goldo_asserv_wait_finished(void)
{
  pthread_mutex_lock(&s_asserv_mutex);
  while(g_asserv.asserv_state == ASSERV_STATE_MOVING || command_fifo_current_command() != NULL)
  {
    pthread_cond_wait(&s_asserv_cond,&s_asserv_mutex);
  }
  return g_asserv.asserv_state;
}

int goldo_asserv_emergency_stop(void)
{
  goldo_log(0,"%f goldo_asserv: EMERGENCY_STOP\n",g_asserv.elapsed_time_ms*1e-3);
  pthread_mutex_lock(&s_asserv_mutex);
  if(g_asserv.asserv_state == ASSERV_STATE_MOVING)
  {
    g_asserv.asserv_state = ASSERV_STATE_EMERGENCY_STOP;
    s_emergency_stop_speed = g_odometry_state.speed;
    s_emergency_stop_dist = g_odometry_state.elapsed_distance;
    command_fifo_clear();
    pthread_cond_broadcast(&s_asserv_cond);
  }  
  pthread_mutex_unlock(&s_asserv_mutex);
  return OK;
}

int goldo_asserv_match_finished(void)
{
  goldo_log(0,"%f goldo_asserv: MATCH_FINISHED\n",g_asserv.elapsed_time_ms*1e-3);
  pthread_mutex_lock(&s_asserv_mutex);
  g_asserv.asserv_state = ASSERV_STATE_MATCH_FINISHED;   
  command_fifo_clear();
  pthread_cond_broadcast(&s_asserv_cond);
  pthread_mutex_unlock(&s_asserv_mutex);
  return OK;
}



int goldo_asserv_straight_line(float distance, float speed, float accel, float deccel)
{
  goldo_asserv_command_s* c = command_fifo_begin_write();
  goldo_log(0,"goldo_asserv: enqueue straight line\n");
  if(c != NULL)
  {
    c->command = GOLDO_ASSERV_COMMAND_STRAIGHT_LINE;
    c->distance = distance;
    c->speed = speed;
    c->acceleration = accel;
    c->decceleration = deccel;
    command_fifo_end_write();
  } else
  {
    goldo_log(0,"goldo_asserv: command fifo full\n");
  }  
  return OK;
}

int goldo_asserv_rotation(float heading_change, float yaw_rate, float angular_accel, float angular_deccel)
{
  goldo_asserv_command_s* c = command_fifo_begin_write();
  goldo_log(0,"goldo_asserv: enqueue rotation\n");
  if(c != NULL)
  {
    c->command = GOLDO_ASSERV_COMMAND_ROTATION;
    c->distance = heading_change;
    c->speed = yaw_rate;
    c->acceleration = angular_accel;
    c->decceleration = angular_deccel;
    command_fifo_end_write();
  } else
  {
    goldo_log(0,"goldo_asserv: command fifo full\n");
  }  
  return OK;
}

int goldo_asserv_wait(float t)
{
  goldo_asserv_command_s* c = command_fifo_begin_write();
  goldo_log(0,"goldo_asserv: enqueue wait\n");
  if(c != NULL)
  {
    c->command = GOLDO_ASSERV_COMMAND_WAIT;
    c->distance = t;    
    command_fifo_end_write();
  } else
  {
    goldo_log(0,"goldo_asserv: command fifo full\n");
  }  
  return OK;
}
/*********************************************************************
 * Command fifo functions.
 ********************************************************************/

int command_fifo_init(void)
{
  s_command_fifo.index = 0;
  s_command_fifo.end = 0;
  pthread_mutex_init(&s_command_fifo.mutex,NULL);
  return OK;
}

goldo_asserv_command_s* command_fifo_current_command(void)
{
  if(s_command_fifo.index != s_command_fifo.end)
  {
    return s_command_fifo.commands + s_command_fifo.index;
  } else
  {
    return NULL;
  }
}

void command_fifo_clear(void)
{
  pthread_mutex_lock(&s_command_fifo.mutex);
  s_command_fifo.index = 0;
  s_command_fifo.end = 0;
  pthread_mutex_unlock(&s_command_fifo.mutex);
}


void command_fifo_advance(void)
{
  goldo_log(0,"goldo_asserv: command_fifo_advance\n");
  pthread_mutex_lock(&s_command_fifo.mutex);
  if(s_command_fifo.index != s_command_fifo.end)
  {
    s_command_fifo.index++;
    if(s_command_fifo.index==GOLDO_ASSERV_MAX_COMMANDS)
    {
      s_command_fifo.index=0;
    }
  }
  pthread_mutex_unlock(&s_command_fifo.mutex);
}

goldo_asserv_command_s* command_fifo_begin_write(void)
{
  pthread_mutex_lock(&s_command_fifo.mutex);
  int next_command_index = s_command_fifo.end+1;
  if(next_command_index == GOLDO_ASSERV_MAX_COMMANDS)
  {
    next_command_index = 0;
  }
  if(next_command_index == s_command_fifo.index)
  {
    pthread_mutex_unlock(&s_command_fifo.mutex);
    return NULL;
  } else
  {
    return s_command_fifo.commands + s_command_fifo.end;
  }  
}

int command_fifo_end_write(void)
{
  int next_command_index = s_command_fifo.end+1;
  if(next_command_index == GOLDO_ASSERV_MAX_COMMANDS)
  {
    next_command_index = 0;
  }
  s_command_fifo.end = next_command_index;
  pthread_mutex_unlock(&s_command_fifo.mutex);
  return OK;
}

int goldo_asserv_get_distance_pid_values(float* k_p, float* k_i, float* k_d, float* lim_i, float* speed_ff)
{
  *k_p = s_pid_distance.k_p;
  *k_i = s_pid_distance.k_i;
  *k_d = s_pid_distance.k_d;
  *lim_i = s_pid_distance.lim_i;
  *speed_ff = s_pid_distance.ff_speed;
}

int goldo_asserv_set_distance_pid_values(float k_p, float k_i, float k_d, float lim_i, float ff_speed)
{
  s_pid_distance.k_p = k_p;
  s_pid_distance.k_i = k_i;
  s_pid_distance.k_d = k_d;
  s_pid_distance.lim_i = lim_i;
  s_pid_distance.ff_speed = ff_speed;
}

int goldo_asserv_get_heading_pid_values(float* k_p, float* k_i, float* k_d, float* lim_i, float* speed_ff)
{
  *k_p = s_pid_heading.k_p;
  *k_i = s_pid_heading.k_i;
  *k_d = s_pid_heading.k_d;
  *lim_i = s_pid_heading.lim_i;
  *speed_ff = s_pid_heading.ff_speed;
}

int goldo_asserv_set_heading_pid_values(float k_p, float k_i, float k_d, float lim_i, float ff_speed)
{
  s_pid_heading.k_p = k_p;
  s_pid_heading.k_i = k_i;
  s_pid_heading.k_d = k_d;
  s_pid_heading.lim_i = lim_i;
  s_pid_heading.ff_speed = ff_speed;
}