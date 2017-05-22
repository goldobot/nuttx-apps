#include "asserv_thomas.h"
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

#include <nuttx/drivers/pwm.h>
#include <nuttx/timers/timer.h>

/* Configuration */

#define CONFIG_RT_DEVNAME "/dev/timer0"
#define CONFIG_RT_INTERVAL 10000 //timer interval in usec


typedef struct
{
  int command_type;

} goldo_asserv_command;

typedef struct
{
  int initialized;
  int tim_fd;
  pthread_t rt_id;
  int rt_stop;
  int asserv_state;

  float elapsed_distance_setpoint;
  float speed_setpoint;
  float heading_change_setpoint;
  float yaw_rate_setpoint;

} goldo_asserv_s;

typedef struct 
{
  
} goldo_asserv_config_s;

goldo_asserv_s g_asserv;

/* Real time feedback loop thread */
static void *thread_asserv(void *arg);

static int start_realtime_thread(goldo_asserv_s* asserv)
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

int goldo_asserv_init(void)
{
  goldo_asserv_hal_init();
  /* Initialize feedback loop values*/

  /* Start realtime thread and timer*/
  if(start_realtime_thread(&g_asserv)!=OK)
    {
      return EXIT_FAILURE;
    }
    return OK;
}

int goldo_asserv_quit(void)
{
  if(!g_asserv.initialized)
  {
    return OK;
  }
  int ret = 0;
  bool err = false;
  /* Stop the real time thread */
  printf("Stop the realtime thread\n");
  g_asserv.rt_stop = true;
  ret = pthread_join(g_asserv.rt_id,NULL);
  if (ret < 0) {
    fprintf(stderr, "ERROR: Failed to stop the realtime thread: %d\n", errno);
    err = true;
  }
 
  /* Stop the timer */
  printf("Stop the timer\n");
  ret = ioctl(g_asserv.tim_fd, TCIOC_STOP, 0);
  if (ret < 0) {
    fprintf(stderr, "ERROR: Failed to stop the timer: %d\n", errno);    
    err = true;
  }
  close(g_asserv.tim_fd);
  if(err)
  {
    return ERROR;
  } else
  {
    return OK;
  }  
}

/* update elapsed distance, speed, heading and yaw rate setpoints based on current trajectory*/
static int asserv_trajectory_generator(void)
{
  return OK;
}




static void *thread_asserv(void *arg)
{
  sigset_t set;
  siginfo_t info;
  sigfillset(&set);

  int tick_10ms = 0;


  /*************************************************************************/
  /** BOUCLE PRINCIPALE ****************************************************/
  while(!g_asserv.rt_stop) {
    /* 0 : debut d'iteration + barriere synchro +++++++++++++++++++++++++++*/
    sigwaitinfo(&set, &info);
    /* fin debut d'iteration ----------------------------------------------*/

    goldo_odometry_update();

  
    /* 8 : fin d'iteration ++++++++++++++++++++++++++++++++++++++++++++++++*/
    tick_10ms++;
    /* --------------------------------------------------------------------*/
  }
  /** fin BOUCLE PRINCIPALE ************************************************/
  /*************************************************************************/

  printf("thread_asserv done\n");
  return NULL;
}