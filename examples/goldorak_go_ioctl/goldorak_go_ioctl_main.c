/****************************************************************************
 * examples/goldorak_go_ioctl/goldorak_go_ioctl_main.c
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

#include "goldorak_go_ioctl.h"

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

struct goldorak_go_ioctl_state_s
{
  bool      initialized;
  FAR char *devpathL;
  FAR char *devpathR;
  int32_t   dutyL;
  int32_t   dutyR;
  uint32_t  freq;
  int       duration;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct goldorak_go_ioctl_state_s g_goldorak_go_ioctl_state =
{
  .initialized = 0,
  .devpathL    = "/dev/pwm1",
  .devpathR    = "/dev/pwm0",
  .dutyL       = 0,
  .dutyR       = 0,
  .freq        = 10000,
  .duration    = 1000,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: goldorak_go_ioctl_help
 ****************************************************************************/

static void goldorak_go_ioctl_help(void)
{

  printf("Usage: goldorak_go_ioctl [-f <freq>] [-t <duration>] -L <motL> -R <motR>\n");
  printf("         <freq> = PWM frequency (default 10000 Hz)\n");
  printf("         <duration> = duration of the test in ms (default 1000 ms)\n");
  printf("         <motL> = speed command for left motor (-99:99)\n");
  printf("         <motR> = speed command for right motor (-99:99)\n");
  printf("       goldorak_go_ioctl [-h]\n");
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

static void parse_args(FAR struct goldorak_go_ioctl_state_s *gs, int argc, FAR char **argv)
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
          case 'f':
            nargs = arg_decimal(&argv[index], &value);
            if (value < 1)
              {
                printf("Frequency out of range: %ld\n", value);
                exit(1);
              }

            gs->freq = (uint32_t)value;
            index += nargs;
            break;

          case 'L':
            nargs = arg_decimal(&argv[index], &value);
            if (value < -99 || value > 99)
              {
                printf("Duty out of range: %ld\n", value);
                exit(1);
              }

            gs->dutyL = value;
            index += nargs;
            break;

          case 'R':
            nargs = arg_decimal(&argv[index], &value);
            if (value < -99 || value > 99)
              {
                printf("Duty out of range: %ld\n", value);
                exit(1);
              }

            gs->dutyR = value;
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
            goldorak_go_ioctl_help();
            exit(0);

          default:
            printf("Unsupported option: %s\n", ptr);
            goldorak_go_ioctl_help();
            exit(1);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: goldorak_go_ioctl_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int goldorak_go_ioctl_main(int argc, char *argv[])
#endif
{
  struct pwm_info_s info;
  int fdL, fdR;
  int ret;
  int speed_val;
  char speed_sign_c;

  /* Initialize the state data */

  if (!g_goldorak_go_ioctl_state.initialized)
    {
      g_goldorak_go_ioctl_state.dutyL       = 0;
      g_goldorak_go_ioctl_state.dutyR       = 0;
      g_goldorak_go_ioctl_state.freq        = 10000;
      g_goldorak_go_ioctl_state.duration    = 1000;
      g_goldorak_go_ioctl_state.initialized = true;
    }

  /* Parse the command line */

  parse_args(&g_goldorak_go_ioctl_state, argc, argv);

  /* Open the PWM devices for reading */

  fdL = open(g_goldorak_go_ioctl_state.devpathL, O_RDONLY);
  if (fdL < 0)
    {
      printf("pwm_main: open %s failed: %d\n", g_goldorak_go_ioctl_state.devpathL, errno);
      goto errout;
    }

  fdR = open(g_goldorak_go_ioctl_state.devpathR, O_RDONLY);
  if (fdR < 0)
    {
      printf("pwm_main: open %s failed: %d\n", g_goldorak_go_ioctl_state.devpathR, errno);
      goto errout;
    }

  /* Configure the characteristics of the PWM */

  info.frequency = g_goldorak_go_ioctl_state.freq;
  printf("goldorak_go_ioctl_main: frequency = %u\n", info.frequency);

  /* LEFT */
  speed_val = g_goldorak_go_ioctl_state.dutyL;
  if (speed_val>=0) {
    info.duty = (speed_val << 16) / 100;
    goldo_maxon2_dir_p();
    speed_sign_c = '+';
  } else {
    info.duty = ((-speed_val) << 16) / 100;
    goldo_maxon2_dir_n();
    speed_sign_c = '-';
  }
  printf("goldorak_go_ioctl_main: LEFT duty: %c %08x\n",
         speed_sign_c, info.duty);
  ret = ioctl(fdL, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
  if (ret < 0)
    {
      printf("goldorak_go_ioctl: LEFT : ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
      goto errout_with_dev;
    }

  /* RIGHT */
  speed_val = g_goldorak_go_ioctl_state.dutyR;
  if (speed_val>=0) {
    info.duty = (speed_val << 16) / 100;
    goldo_maxon1_dir_n();
    speed_sign_c = '+';
  } else {
    info.duty = ((-speed_val) << 16) / 100;
    goldo_maxon1_dir_p();
    speed_sign_c = '-';
  }
  printf("goldorak_go_ioctl_main: RIGHT duty: %c %08x\n",
         speed_sign_c, info.duty);
  ret = ioctl(fdR, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
  if (ret < 0)
    {
      printf("goldorak_go_ioctl_main: RIGHT : ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
      goto errout_with_dev;
    }

  /* START */
  ret = ioctl(fdL, PWMIOC_START, 0);
  if (ret < 0)
    {
      printf("goldorak_go_ioctl_main: LEFT : ioctl(PWMIOC_START) failed: %d\n", errno);
      goto errout_with_dev;
    }
  goldo_maxon2_en();

  ret = ioctl(fdR, PWMIOC_START, 0);
  if (ret < 0)
    {
      printf("goldorak_go_ioctl_main: RIGHT : ioctl(PWMIOC_START) failed: %d\n", errno);
      goto errout_with_dev;
    }
  goldo_maxon1_en();

  /* Wait for the specified duration */
  usleep(g_goldorak_go_ioctl_state.duration*1000);

  /* STOP */
  printf("goldorak_go_ioctl_main: stopping PWMs\n");

  ret = ioctl(fdL, PWMIOC_STOP, 0);
  if (ret < 0)
    {
      printf("goldorak_go_ioctl_main: LEFT ioctl(PWMIOC_STOP) failed: %d\n", errno);
    }
  goldo_maxon2_dis();

  ret = ioctl(fdR, PWMIOC_STOP, 0);
  if (ret < 0)
    {
      printf("goldorak_go_ioctl_main: RIGHT ioctl(PWMIOC_STOP) failed: %d\n", errno);
    }
  goldo_maxon1_dis();

  close(fdL);
  close(fdR);
  fflush(stdout);
  return OK;

errout_with_dev:
  close(fdL);
  close(fdR);
errout:
  fflush(stdout);
  return ERROR;
}
