#include "goldo_asserv.h"
#include "goldo_asserv_hal.h"
#include "goldo_odometry.h"
#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>

#include <nuttx/timers/timer.h>

/* Configuration */

#define CONFIG_RT_DEVNAME "/dev/timer0"
#define CONFIG_RT_INTERVAL 10000 //timer interval in usec


typedef struct goldo_asserv_stm32_s
{
  int initialized;
  int tim_fd;
  pthread_t rt_id;
  int rt_stop;
} goldo_asserv_stm32_s;


goldo_asserv_stm32_s s_asserv_arch;

/* Real time feedback loop thread */
static void *thread_asserv(void *arg);
static int start_realtime_thread(goldo_asserv_stm32_s* asserv);



int goldo_asserv_arch_init(void)
{
  /* Initialize feedback loop values*/

  /* Start realtime thread and timer*/
  if(start_realtime_thread(&s_asserv_arch)!=OK)
    {
      return EXIT_FAILURE;
    }
    return OK;
}

int goldo_asserv_arch_release(void)
{
  if(!s_asserv_arch.initialized)
  {
    return OK;
  }
  int ret = 0;
  bool err = false;
  /* Stop the real time thread */
  printf("Stop the realtime thread\n");
  s_asserv_arch.rt_stop = true;
  ret = pthread_join(s_asserv_arch.rt_id,NULL);
  if (ret < 0) {
    fprintf(stderr, "ERROR: Failed to stop the realtime thread: %d\n", errno);
    err = true;
  }
 
  /* Stop the timer */
  printf("Stop the timer\n");
  ret = ioctl(s_asserv_arch.tim_fd, TCIOC_STOP, 0);
  if (ret < 0) {
    fprintf(stderr, "ERROR: Failed to stop the timer: %d\n", errno);    
    err = true;
  }
  close(s_asserv_arch.tim_fd);
  if(err)
  {
    return ERROR;
  } else
  {
    return OK;
  }  
}


static void *thread_asserv(void *arg)
{
  sigset_t set;
  siginfo_t info;
  sigfillset(&set);

  /*************************************************************************/
  /** BOUCLE PRINCIPALE ****************************************************/
  while(!s_asserv_arch.rt_stop) {
    /* 0 : debut d'iteration + barriere synchro +++++++++++++++++++++++++++*/
    sigwaitinfo(&set, &info);
    /* fin debut d'iteration ----------------------------------------------*/

    goldo_odometry_update();
    goldo_asserv_do_step(CONFIG_RT_INTERVAL/1000);
    switch(g_asserv.asserv_state)
    {
      case ASSERV_STATE_DISABLED:
        {
          goldo_asserv_hal_set_motors_enable(false,false);
          goldo_asserv_hal_set_motors_pwm(0,0);
        }
        break;
      case ASSERV_STATE_IDLE:
        {
          goldo_asserv_hal_set_motors_enable(true,true);
          goldo_asserv_hal_set_motors_pwm(0,0);
        }
        break;
      case ASSERV_STATE_MOVING:
      case ASSERV_STATE_EMERGENCY_STOP:
      case ASSERV_STATE_RECALAGE:
        {
          goldo_asserv_hal_set_motors_enable(true,true);
          /*manage friction*/
          int pwm_left = g_asserv.motor_pwm_left*(1 << 16);
          int pwm_right =  g_asserv.motor_pwm_right*(1 << 16);
          int pwm_limit = 40000;
          int pwm_friction_treshold = 15000;

          if(pwm_left > 0) pwm_left += pwm_friction_treshold;
          if(pwm_left < 0) pwm_left += -pwm_friction_treshold;
          if(pwm_left > pwm_limit) pwm_left = pwm_limit;
          if(pwm_left < -pwm_limit) pwm_left = -pwm_limit;

          if(pwm_right > 0) pwm_right += pwm_friction_treshold;
          if(pwm_right < 0) pwm_right += -pwm_friction_treshold;
          if(pwm_right > pwm_limit) pwm_right = pwm_limit;
          if(pwm_right < -pwm_limit) pwm_right = -pwm_limit;

          goldo_asserv_hal_set_motors_pwm(pwm_left,pwm_right);
        }
        break;
      case ASSERV_STATE_ERROR:
        {
          goldo_asserv_hal_set_motors_enable(false,false);
          goldo_asserv_hal_set_motors_pwm(0,0);
        }
        break;
      case ASSERV_STATE_MATCH_FINISHED:
        {
          goldo_asserv_hal_set_motors_enable(false,false);
          goldo_asserv_hal_set_motors_pwm(0,0);
        }
        break;


    }
   
    /* --------------------------------------------------------------------*/
  }
  /** fin BOUCLE PRINCIPALE ************************************************/
  /*************************************************************************/
  printf("thread_asserv done\n");
  return NULL;
}

static int start_realtime_thread(goldo_asserv_stm32_s* asserv)
{
  int ret = 0;  
  asserv->tim_fd = -1;

  /* Initialize real time thread */
  asserv->rt_stop = false;

  pthread_create(&asserv->rt_id, NULL, thread_asserv, NULL);
  pthread_setschedprio(asserv->rt_id, PTHREAD_DEFAULT_PRIORITY);

  /* Setup synchronization timer */
  asserv->tim_fd = open(CONFIG_RT_DEVNAME, O_RDONLY);
  if (asserv->tim_fd < 0) {
    fprintf(stderr, "ERROR: Failed to open %s: %d\n", CONFIG_RT_DEVNAME, errno);
    goto errout;
  }

  printf("Set timer interval to %lu\n", (unsigned long)CONFIG_RT_INTERVAL);

  ret = ioctl(asserv->tim_fd, TCIOC_SETTIMEOUT, CONFIG_RT_INTERVAL);
  if (ret < 0) {
    fprintf(stderr, "ERROR: Failed to set the timer interval: %d\n", errno);
    goto errout;
  }
  /* Attach the timer handler to the real time thread
   *
   * NOTE: If no handler is attached, the timer stop at the first interrupt.
   */
  printf("Attach timer handler\n");
  struct timer_notify_s notify;
  notify.signo = 1;
  notify.pid   = asserv->rt_id;
  notify.arg   = NULL;

  ret = ioctl(asserv->tim_fd, TCIOC_NOTIFICATION, (unsigned long)((uintptr_t)&notify));
  if (ret < 0) {
    fprintf(stderr, "ERROR: Failed to set the timer handler: %d\n", errno);
    goto errout;
  }

  /* Start the timer */
  printf("Start the timer\n");

  ret = ioctl(asserv->tim_fd, TCIOC_START, 0);
  if (ret < 0) {
    fprintf(stderr, "ERROR: Failed to start the timer: %d\n", errno);
    goto errout;
  }

 return OK;
  errout:
  if(asserv->tim_fd >= 0)
  {
    close(asserv->tim_fd);    
  }
  return EXIT_FAILURE;
}