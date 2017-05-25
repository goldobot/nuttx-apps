#include "goldo_main_loop.h"
#include "robot/goldo_robot.h"
#include "goldo_asserv.h"
#include "goldo_asserv_hal.h"
#include "goldo_odometry.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


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

int main_loop_homologation(void)
{
  /* Wait for start of match*/
  printf("main_loop_homologation: start main loop\n");
  /*replace true by check on start gpio*/
  goldo_robot_wait_for_match_begin();
  printf("main_loop_homologation: Start match\n");
  goldo_asserv_enable();
  goldo_match_timer_start(90);
  
  goldo_asserv_straight_line(0.5, 0.5, 0.5, 0.5);
  

  /* STOP motors */

}

int main_loop_test_odometry(void)
{
  while(1)
  {
    printf("encoders: %i,%i, %i\n", g_asserv.elapsed_time_ms,g_odometry_state.counts_left,g_odometry_state.counts_right);
    usleep(100000);
  }
}

int main_loop_test_asserv(void)
{
  goldo_asserv_enable();  
  goldo_asserv_straight_line(1, 0.5, 0.5, 0.5);
  goldo_asserv_straight_line(0.5, 0.5, 0.5, 0.5);
  sleep(1);
  goldo_asserv_emergency_stop();
  goldo_asserv_wait_finished();
  sleep(10);
  printf("finished movement sequence\n");
}