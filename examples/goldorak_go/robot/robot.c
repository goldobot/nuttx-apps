#include "goldo_robot.h"
#include "../goldo_odometry.h"
#include "../goldo_asserv.h"
#include "../goldo_match_timer.h"

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

#include <nuttx/drivers/pwm.h>

/* Hack */
//Start : PB8
//obstacle PB9


extern int goldo_get_start_gpio_state(void);
extern int goldo_get_obstacle_gpio_state(void);


int goldo_robot_init(void)
{
  goldo_odometry_config_s odometry_config;

  /* Petit robot */
  #if 1
  odometry_config.dist_per_count_left = -0.4025e-3f ;
  odometry_config.dist_per_count_right = 0.3797e-3f;
  odometry_config.wheel_spacing = 0.313213;
  odometry_config.update_period = 10e-3f;
  #endif

  goldo_match_timer_init();
  goldo_odometry_init();
  goldo_odometry_set_config(&odometry_config); 
  goldo_asserv_init();
  goldo_arms_init();
  goldo_adversary_detection_init();


  return OK;
}

int goldo_robot_release(void)
{
  goldo_adversary_detection_release();
  goldo_arms_release();
  goldo_asserv_quit();
  goldo_odometry_quit();
  goldo_match_timer_release();
  return OK;
  
};

int goldo_robot_wait_for_match_begin(void)
{
  printf("goldo_robot: wait for start of match\n");
  while(!goldo_get_start_gpio_state())
  {
    usleep(10000);
  }
  return OK;
}

int goldo_robot_do_funny_action(void)
{
  int ret;
  printf("goldo_robot: do funny action\n");
  /* Open PWM driver */
  struct pwm_info_s info;
  int fd;
  fd = open("/dev/pwm8", O_RDONLY);
  if (fd < 0)
  {
    printf("pwm_main: open %s failed: %d\n", "/dev/pwm8", errno);
    return ERROR;
  }

  info.duty = (50 << 16) / 100;
  info.frequency = 10000;
  ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
  if (ret < 0)
  {
    printf("goldorak_go_ioctl: LEFT : ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
  }
  ret = ioctl(fd, PWMIOC_START, 0);
  usleep(10000000);
  ret = ioctl(fd, PWMIOC_STOP, 0);
  close(fd);
  return OK;
}