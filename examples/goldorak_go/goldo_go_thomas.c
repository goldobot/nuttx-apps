/****************************************************************************
 * examples/goldorak_go/goldorak_go_main.c
 *
 *   Copyright (C) 2011-2012, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*

Cote carte : 1-Vert 2-Noir 3-Jaun

V N J : KO+ OK-
N V J : KO!
J N V : KO!
N J V : OK+ OK-
J V N : OK+ KO-
V J N : KO!

Cote moteur : 1-Gris 2-Maro 3-Jaun

Ce qui marche : (carte-moteur)
 2-Noir-Gris-1 3-Jaun-Maro-2 1-Vert-Jaun-3
 
#define SEUIL_FRICTION_BRUSHLESS_1P 14600 // 15720 16260 15780 16020 // 15720
#define SEUIL_FRICTION_BRUSHLESS_1N 14500 // 15720 16440 16080 17340 // 16395
#define SEUIL_FRICTION_BRUSHLESS_2P 17000 // 14700 15540 14940 15660 // 15210
#define SEUIL_FRICTION_BRUSHLESS_2N 14500 // 15000 15660 16020 16080 // 15690
#define SEUIL_PROTECT_BRUSHLESS 60000

 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/time.h>

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

typedef unsigned int wint_t;
#include <math.h>



#include "goldorak_go.h"
#include "asserv_thomas.h"
#include "goldo_asserv_hal.h"
#include "goldo_odometry.h"

#if 1 /* FIXME : DEBUG : HACK */
int __fpclassifyd(double val)
{
  return 4;
}
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

//config at  10 Hz = 100ms = 100000us
//config at 100 Hz = 10ms  = 10000us


/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * thread_asserv
 ****************************************************************************/

/* FIXME : TODO : mettre tout ce bordel dans asserv.c */

#define ASSERV_CMD_TYPE_NONE         0
#define ASSERV_CMD_TYPE_TRANSLATION  1
#define ASSERV_CMD_TYPE_ROTATION     2
#define ASSERV_CMD_TYPE_STATIC       3

#define ASSERV_STATE_IDDLE             0
#define ASSERV_STATE_MOVING            1
#define ASSERV_STATE_STOP_BLOCKED      2


/****************************************************************************
 * Name: goldorak_go_help
 ****************************************************************************/

static void goldorak_go_help(void)
{

  printf("Usage: goldorak_go <x> <y> <theta>\n");
  printf("         go to point <x, y> and then turn to align with direction <theta>\n");
  printf("       goldorak_go [-h]\n");
  printf("          -h = shows this message and exits\n");
}

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

static void parse_args(int argc, FAR char **argv)
{
  FAR char *ptr;

  if (argc==1) {
    goldorak_go_help();
    exit(0);
  }

  ptr = argv[1];

  if ((ptr[0]=='-') && (ptr[1]=='h')) {
    goldorak_go_help();
    exit(0);
  }

#if 0 /* FIXME : DEBUG */
  if (argc!=4) {
    goldorak_go_help();
    exit(1);
  }

  x_final = strtof(argv[1], NULL);
  y_final = strtof(argv[2], NULL);
  theta_final = strtof(argv[3], NULL);
#else
  if (argc!=2) {
    //goldorak_go_help();
    exit(1);
  }

  //D_final = strtof(argv[1], NULL);
  //rc_corr = strtof(argv[2], NULL);
#endif
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: goldorak_go_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int goldorak_go_main(int argc, char *argv[])
#endif
{
  int ret;


  /* Parse the command line */
  parse_args(argc, argv);

#if 0 /* FIXME : DEBUG ++ */
  printf(" x_final = %.6f\n", x_final);
  printf(" y_final = %.6f\n", y_final);
  printf(" theta_final = %.6f\n", theta_final);
  printf(" sin(theta_final) = %.6f\n", sin(M_PI*theta_final/180.0));
  printf(" cos(theta_final) = %.6f\n", cos(M_PI*theta_final/180.0));
  return OK;
#endif /* FIXME : DEBUG -- */
  goldo_odometry_config_s odometry_config;
  odometry_config.dist_per_count_left = 0.5e-3f;
  odometry_config.dist_per_count_right = 0.5e-3f;
  odometry_config.wheel_spacing = 100e-3f;
  odometry_config.update_period = 10e-3f;
  goldo_odometry_init();
  goldo_odometry_set_config(&odometry_config); 
  goldo_asserv_init();
  goldo_asserv_enable();
  goldo_asserv_straight_line(0.5, 0.5, 0.5, 0.5);
  g_asserv.motor_pwm_left=1.0f;
  while(1)
  {
    printf("encoders: %i,%i, %i\n", g_asserv.elapsed_time_ms,g_odometry_state.counts_left,g_odometry_state.counts_right);
    usleep(100000);
  }

  /* STOP motors */
  printf("goldorak_go_main: disabling motors\n");
  goldo_asserv_quit();
  fflush(stdout);
  return OK;
}

