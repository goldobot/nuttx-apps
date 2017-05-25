#include "goldo_asserv.h"
#include "goldo_asserv_hal.h"
#include "goldo_odometry.h"

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
  GOLDO_ASSERV_COMMAND_ROTATION=1
} GOLDO_ASSERV_COMMAND;

typedef struct goldo_asserv_command_s
{
  GOLDO_ASSERV_COMMAND command;
  float distance;
  float speed;
  float acceleration;
  float decceleration;
  float heading_change;
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
static uint32_t s_match_timer_begin = 0;
static uint32_t s_match_timer_end = -1;
pthread_cond_t s_asserv_cond;/* Condition variable used to signal asserv events */
pthread_mutex_t s_asserv_mutex;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int emergency_stop_begin(void);
static bool emergency_stop_update(void);
static int command_fifo_init(void);
static void command_fifo_clear(void);
static goldo_asserv_command_s* command_fifo_current_command(void);
static void command_fifo_advance(void);
static goldo_asserv_command_s* command_fifo_begin_write(void);
static int command_fifo_end_write(void);

/*polynomial curve segment*/
typedef struct goldo_asserv_segment_pt_s
{
  uint32_t t;
  float d0                                                                                                                                         ;
  float d1;
  float d2;
  float d3;
  float h0;
  float h1;
  float h2;
  float h3;
} goldo_asserv_polyline_pt_s;

/* Trajectory for current command is in the form of a polyline */
static goldo_asserv_polyline_pt_s s_asserv_segments[16];
static int s_asserv_segment_index=0;
static int s_asserv_segment_end=0;

static goldo_pid_filter_s s_pid_distance;
static goldo_pid_filter_s s_pid_heading;

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
  g_asserv.asserv_state = ASSERV_STATE_IDLE;


  /* Initialize feedback loop values*/
  goldo_pid_filter_init(&s_pid_distance);
  goldo_pid_filter_init(&s_pid_heading);
  goldo_pid_filter_config_s pid_config_distance;
  pid_config_distance.k_p = 50;
  pid_config_distance.k_d = 0;
  pid_config_distance.k_i = 0;
  pid_config_distance.ff_speed = 0;
  goldo_pid_filter_set_config(&s_pid_distance,&pid_config_distance);

  /* Launch realtime thread */
  goldo_asserv_arch_init();
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
    if(g_asserv.asserv_state == ASSERV_STATE_DISABLED)
    {
      g_asserv.asserv_state = ASSERV_STATE_IDLE;      
      return OK;
    }    
  
  return OK;  
}

int goldo_match_timer_start(int time_s)
{
  s_match_timer_begin = g_asserv.elapsed_time_ms;
  s_match_timer_end = g_asserv.elapsed_time_ms + time_s*1000;
  return OK;
}

int goldo_match_timer_get_value(void)
{
  return g_asserv.elapsed_time_ms - s_match_timer_begin;
}
bool goldo_match_timer_is_finished(void)
{
return g_asserv.elapsed_time_ms < s_match_timer_end;
}

static bool goldo_asserv_update_segment(void)
{
  while(s_asserv_segment_index != s_asserv_segment_end && s_asserv_segments[s_asserv_segment_index+1].t < g_asserv.elapsed_time_ms)
  {
    goldo_log(0,"goldo_asserv: advance polyline segment\n");
    s_asserv_segment_index++;
  }
  if(s_asserv_segment_index == s_asserv_segment_end)
  {

    return false;
  }
  goldo_asserv_polyline_pt_s* pt = s_asserv_segments+s_asserv_segment_index;
  float u = (g_asserv.elapsed_time_ms - pt->t) * 1e-3; 
  g_asserv.elapsed_distance_setpoint = pt->d0 + u*(pt->d1 + u*(pt->d2 + u* pt->d3));
  g_asserv.speed_setpoint = pt->d1 + u*(2*pt->d2 + 3*u* pt->d3);
  return true;  
}

static void goldo_asserv_begin_command(goldo_asserv_command_s* c)
{
  switch(c->command)
  {
    case GOLDO_ASSERV_COMMAND_STRAIGHT_LINE:
    {
      goldo_log(0,"goldo_asserv: begin execute straight line\n");
      uint32_t start_time = g_asserv.elapsed_time_ms;
      float speed = c->speed;
      goldo_log(0,"goldo_asserv: speed=%f, distance=%f\n",speed,c->distance);
      float d_a = (speed * speed) * 0.5f / c->acceleration;
      float d_d = (speed * speed) * 0.5f / c->decceleration;
      float t_a = 0;
      float t_d = 0;
      float t_c = 0;
      
      if(d_a+d_d > c->distance)
      {
        goldo_log(0,"goldo_asserv: distance too short %f\n",t_a);
      } else
      {
        t_a = speed/c->acceleration;
        t_d = speed/c->decceleration;
        t_c = (c->distance - d_a - d_d)/speed;
      }
      /*set polyline*/
      uint32_t ct = g_asserv.elapsed_time_ms;
      float cd = g_odometry_state.elapsed_distance;
      s_asserv_segments[0].t=ct;
      s_asserv_segments[0].d0=cd;
      s_asserv_segments[0].d1=0;
      s_asserv_segments[0].d2=0.5f*c->acceleration;
      s_asserv_segments[0].d3=0;
      ct+=t_a*1e3;
      cd+=d_a;
      s_asserv_segments[1].t=ct;
      s_asserv_segments[1].d0=cd;
      s_asserv_segments[1].d1=speed;
      s_asserv_segments[1].d2=0;
      s_asserv_segments[1].d3=0;
      ct+=t_c*1e3;
      cd+=c->distance - d_a - d_d;
      s_asserv_segments[2].t=ct;
      s_asserv_segments[2].d0=cd;
      s_asserv_segments[2].d1=speed;
      s_asserv_segments[2].d2=-0.5f*c->decceleration;
      s_asserv_segments[2].d3=0;
      ct+=t_d*1e3;
      s_asserv_segments[3].t=ct;      
      s_asserv_segment_index=0;
      s_asserv_segment_end=3;



      goldo_log(0,"goldo_asserv: %f\n",d_a);
    }
    
  }
}

/*return true if finished*/
static bool goldo_asserv_update_command(goldo_asserv_command_s* c)
{
  switch(c->command)
  {
    case GOLDO_ASSERV_COMMAND_STRAIGHT_LINE:
    {
      if(!goldo_asserv_update_segment())
      {
        goldo_log(0,"goldo_asserv: end execute straight line\n");
        return true;
      }
      //goldo_log(0,"goldo_asserv: start execute straight line\n");
    }
    
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
  goldo_log(0,"goldo_asserv: EMERGENCY_STOP\n");
  pthread_mutex_lock(&s_asserv_mutex);
  g_asserv.asserv_state = ASSERV_STATE_EMERGENCY_STOP;
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

/*********************************************************************
 * Command fifo functions.
/********************************************************************/

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