#include "asserv_thomas.h"
#include "goldo_asserv_hal.h"
#include "goldo_odometry.h"
#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>


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


goldo_asserv_s g_asserv;


int goldo_asserv_init(void)
{
  goldo_asserv_hal_init();
  goldo_asserv_hal_set_motors_enable(false, false);
  goldo_asserv_hal_set_motors_pwm(0, 0);
  g_asserv.asserv_state = ASSERV_STATE_DISABLED;


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

int goldo_asserv_enable(bool en)
{
  ASSERV_STATE current_state = g_asserv.asserv_state;
  if(en)
  {
    if(current_state == ASSERV_STATE_DISABLED)
    {
      g_asserv->asserv_state = ASSERV_STATE_IDLE;
      goldo_asserv_hal_set_motors_enable(true, true);
      return OK;
    }    
    
  } else
  {
    g_asserv.asserv_state = ASSERV_STATE_DISABLED;
    goldo_asserv_hal_set_motors_enable(false, false);
  }
  return OK;  
}

/* update elapsed distance, speed, heading and yaw rate setpoints based on current trajectory*/
static int asserv_trajectory_generator(void)
{
  return OK;
}

int asserv_do_step(int dt_ms)
{

}

