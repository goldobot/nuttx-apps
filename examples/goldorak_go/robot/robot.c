#include "goldo_robot.h"
#include "../goldo_odometry.h"
#include "../goldo_asserv.h"
#include "../goldo_match_timer.h"

/* Hack */
//Start : PB8
//obstacle PB9


extern int goldo_get_start_gpio_state(void);
extern int goldo_get_obstacle_gpio_state(void);


int goldo_robot_init(void)
{
  goldo_odometry_config_s odometry_config;
  odometry_config.dist_per_count_left = 0.5e-3f;
  odometry_config.dist_per_count_right = 0.5e-3f;
  odometry_config.wheel_spacing = 100e-3f;
  odometry_config.update_period = 10e-3f;

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
  return OK;
}