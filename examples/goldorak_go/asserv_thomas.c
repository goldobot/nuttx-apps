#include "asserv_thomas.h"
#include "goldo_asserv_hal.h"
#include "goldo_odometry.h"

#include <sys/types.h>
#include <sys/ioctl.h>

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
  float max_speed;
  float acceleration;
  float decceleration;
  float heading_change;
} goldo_asserv_command_s;




typedef struct goldo_asserv_command_fifo_s
{
  goldo_asserv_command_s commands[GOLDO_ASSERV_MAX_COMMANDS];
  int index;
  int end;

} goldo_asserv_command_fifo_s;

goldo_asserv_s g_asserv;
goldo_asserv_command_fifo_s s_command_fifo;

/*
  Support functions
*/

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

void command_fifo_advance(void)
{
  if(s_command_fifo.index != s_command_fifo.end)
  {
    s_command_fifo.index++;
    if(s_command_fifo.index==GOLDO_ASSERV_MAX_COMMANDS)
    {
      s_command_fifo.index=0;
    }
  }
}


int goldo_asserv_init(void)
{
  goldo_asserv_hal_init();
  goldo_asserv_hal_set_motors_enable(false, false);
  goldo_asserv_hal_set_motors_pwm(0, 0);
  g_asserv.asserv_state = ASSERV_STATE_IDLE;
  s_command_fifo.index = 0;
  s_command_fifo.end = 0;


  /* Initialize feedback loop values*/
  goldo_asserv_arch_init();
}

int goldo_asserv_quit(void)
{
  if(!g_asserv.initialized)
  {
    return OK;
  }
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

/* update elapsed distance, speed, heading and yaw rate setpoints based on current trajectory*/
static int asserv_trajectory_generator(void)
{
  return OK;
}

static void goldo_asserv_begin_command(goldo_asserv_command_s* c)
{

}

/*return true if finished*/
static bool goldo_asserv_update_command(goldo_asserv_command_s* c)
{

}

int goldo_asserv_do_step(int dt_ms)
{
  g_asserv.elapsed_time_ms += dt_ms;
  goldo_asserv_command_s* current_command = command_fifo_current_command(void);
  switch(g_asserv.asserv_state)
  {
    case ASSERV_STATE_DISABLED:
      g_asserv.motor_pwm_left = 0;
      g_asserv.motor_pwm_right = 0;
      break;
    case ASSERV_STATE_IDLE:
      if(current_command != NULL)
      {
        goldo_asserv_begin_command(current_command);
        g_asserv.asserv_state = ASSERV_MOVING;
      }
    /* fall trough*/
    case ASSERV_STATE_MOVING:
      if(goldo_asserv_update_command(current_command))
      {
        /* If current command finished, fetch next command
           If next comand, continue moving, else switch back to idle.
        */
        ommand_fifo_advance();
        current_command = command_fifo_current_command(void);
        if(current_command != NULL)
        {
          goldo_asserv_begin_command(current_command);
          g_asserv.asserv_state = ASSERV_MOVING;
        } else
        {
          g_asserv.asserv_state = ASSERV_IDLE;
        }
      };
      break;
    default:
      break;


  }
  return OK;
}

int goldo_asserv_straight_line(float distance, float speed, float accel, float deccel)
{
  return OK;
}