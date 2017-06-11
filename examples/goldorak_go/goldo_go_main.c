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


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string.h>

//typedef unsigned int wint_t;
//#include <math.h>

#include "robot/goldo_robot.h"
#include "goldo_odometry.h"
#include "goldo_main_loop.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int run_mode=0;
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
  if (strcmp(ptr,"match")==0)
  {
    run_mode = GOLDO_MODE_MATCH;
    return;
  }
  if (strcmp(ptr,"homologation")==0)
  {
    run_mode = GOLDO_MODE_HOMOLOGATION;
    return;
  }
  if (strcmp(ptr,"test_odometry")==0)
  {
    run_mode = GOLDO_MODE_TEST_ODOMETRY;
    return;
  }
  if (strcmp(ptr,"test_motors")==0)
  {
    run_mode = GOLDO_MODE_TEST_MOTORS;
    return;
  }
  if (strcmp(ptr,"test_asserv")==0)
  {
    run_mode = GOLDO_MODE_TEST_ASSERV;
    return;
  }
  if (strcmp(ptr,"utest_start_match")==0)
  {
    run_mode = GOLDO_MODE_UTEST_START_MATCH;
    return;
  }
  if (strcmp(ptr,"utest_adversary_detection")==0)
  {
    run_mode = GOLDO_MODE_UTEST_ADVERSARY_DETECTION;
    return;
  }
  if (strcmp(ptr,"utest_match_timer")==0)
  {
    run_mode = GOLDO_MODE_UTEST_MATCH_TIMER;
    return;
  }
  if (strcmp(ptr,"utest_funny_action")==0)
  {
    run_mode = GOLDO_MODE_UTEST_FUNNY_ACTION;
    return;
  }
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: goldorak_go_main
 ****************************************************************************/
//#define CONFIG_BUILD_KERNEL
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int goldorak_go_main(int argc, char *argv[])
#endif
{
 // int ret;

  /* Parse the command line */
  parse_args(argc, argv);

  goldo_robot_init();
   switch(run_mode)
  {
    case GOLDO_MODE_MATCH:
      main_loop_match();
      break;
    case GOLDO_MODE_HOMOLOGATION:
      main_loop_homologation();
      break;
    case GOLDO_MODE_TEST_ODOMETRY:
      main_loop_test_odometry();
      break;
    case GOLDO_MODE_TEST_ASSERV:
      main_loop_test_asserv();
      break;
    case GOLDO_MODE_TEST_MOTORS:
      main_loop_test_motors();
      break;
    case GOLDO_MODE_UTEST_START_MATCH:
      main_loop_utest_start_match();
      break;
    case GOLDO_MODE_UTEST_ADVERSARY_DETECTION:
      main_loop_utest_adversary_detection();
      break;
    case GOLDO_MODE_UTEST_MATCH_TIMER:
      main_loop_utest_match_timer();
      break;
    case GOLDO_MODE_UTEST_FUNNY_ACTION:
      main_loop_utest_funny_action();
      break;
  }
  goldo_robot_release();
  printf("End of main\n");
  exit(0);
  return OK;
}