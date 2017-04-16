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

/****************************************************************************
 * Included Files
 ****************************************************************************/

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

#include "goldorak_go.h"

extern void goldo_maxon2_dir_p(void);
extern void goldo_maxon2_dir_n(void);
extern void goldo_maxon2_en(void);
extern void goldo_maxon2_dis(void);
extern void goldo_maxon1_dir_p(void);
extern void goldo_maxon1_dir_n(void);
extern void goldo_maxon1_en(void);
extern void goldo_maxon1_dis(void);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct goldorak_go_state_s
{
  bool      initialized;
  FAR char *devpathL;
  FAR char *devpathR;
  int32_t   speedL;
  int32_t   speedR;
  uint32_t  freq;
  int       duration;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct goldorak_go_state_s g_goldorak_go_state =
{
  .initialized = 0,
  .speedL      = 0,
  .speedR      = 0,
  .duration    = 1000,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: goldorak_go_help
 ****************************************************************************/

static void goldorak_go_help(void)
{

  printf("Usage: goldorak_go [-t <duration>] -L <speedL> -R <speedR>\n");
  printf("         <duration> = test duration in ms (default 1000 ms)\n");
  printf("         <speedL> = speed command for left motor (signed integer)\n");
  printf("         <speedR> = speed command for right motor(signed integer)\n");
  printf("       goldorak_go [-h]\n");
  printf("          -h = shows this message and exits\n");
}

/****************************************************************************
 * Name: arg_string
 ****************************************************************************/

static int arg_string(FAR char **arg, FAR char **value)
{
  FAR char *ptr = *arg;

  if (ptr[2] == '\0')
    {
      *value = arg[1];
      return 2;
    }
  else
    {
      *value = &ptr[2];
      return 1;
    }
}

/****************************************************************************
 * Name: arg_decimal
 ****************************************************************************/

static int arg_decimal(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int ret;

  ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

static void parse_args(FAR struct goldorak_go_state_s *gs, int argc, FAR char **argv)
{
  FAR char *ptr;
  long value;
  int index;
  int nargs;

  for (index = 1; index < argc; )
    {
      ptr = argv[index];
      if (ptr[0] != '-')
        {
          printf("Invalid options format: %s\n", ptr);
          exit(0);
        }

      switch (ptr[1])
        {
          case 'L':
            nargs = arg_decimal(&argv[index], &value);
            if (value < -32768 || value > 32767)
              {
                printf("Duty out of range: %ld\n", value);
                exit(1);
              }

            gs->speedL = value;
            index += nargs;
            break;

          case 'R':
            nargs = arg_decimal(&argv[index], &value);
            if (value < -32768 || value > 32767)
              {
                printf("Duty out of range: %ld\n", value);
                exit(1);
              }

            gs->speedR = value;
            index += nargs;
            break;

          case 't':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 1 || value > INT_MAX)
              {
                printf("Duration out of range: %ld\n", value);
                exit(1);
              }

            gs->duration = (int)value;
            index += nargs;
            break;

          case 'h':
            goldorak_go_help();
            exit(0);

          default:
            printf("Unsupported option: %s\n", ptr);
            goldorak_go_help();
            exit(1);
        }
    }
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
  int speed_val;

  /* Initialize the state data */

  if (!g_goldorak_go_state.initialized)
    {
      g_goldorak_go_state.speedL      = 0;
      g_goldorak_go_state.speedR      = 0;
      g_goldorak_go_state.duration    = 1000;
      g_goldorak_go_state.initialized = true;
    }

  /* Parse the command line */

  parse_args(&g_goldorak_go_state, argc, argv);

  /* Send speed command to the motors and START the test */
  printf("goldorak_go_main: enabling motors\n");
  goldo_maxon2_en();
  goldo_maxon1_en();

  /* LEFT command */
  speed_val = g_goldorak_go_state.speedL;
  // FIXME : TODO
  //goldo_maxon2_speed(speed_val);
  printf("goldorak_go_main: LEFT speed: %d\n", speed_val);

  /* RIGHT command */
  speed_val = g_goldorak_go_state.speedR;
  // FIXME : TODO
  //goldo_maxon1_speed(speed_val);
  printf("goldorak_go_main: RIGHT speed: %d\n", speed_val);

  /* Wait for the specified duration */
  usleep(g_goldorak_go_state.duration*1000);

  /* STOP motors */
  printf("goldorak_go_main: disabling motors\n");
  goldo_maxon2_dis();
  goldo_maxon1_dis();

  fflush(stdout);
  return OK;
}
