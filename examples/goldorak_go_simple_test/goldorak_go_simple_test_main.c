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

#include <nuttx/semaphore.h>
#include <nuttx/timers/timer.h>
#include <nuttx/drivers/pwm.h>
#include <nuttx/sensors/qencoder.h>

#include "goldorak_go.h"


extern void goldo_maxon2_dir_p(void);
extern void goldo_maxon2_dir_n(void);
extern void goldo_maxon2_en(void);
extern void goldo_maxon2_dis(void);
extern void goldo_maxon2_speed(int32_t s);
extern void goldo_maxon1_dir_p(void);
extern void goldo_maxon1_dir_n(void);
extern void goldo_maxon1_en(void);
extern void goldo_maxon1_dis(void);
extern void goldo_maxon1_speed(int32_t s);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

//config at  10 Hz = 100ms = 100000us
//config at 100 Hz = 10ms  = 10000us

#define CONFIG_RT_DEVNAME "/dev/timer0"

#define CONFIG_RT_INTERVAL 10000 //timer interval

#define CONFIG_RT_DELAY 10000 //sleep interval

#define CONFIG_SPEED_THRESH 0
#define CONFIG_MAX_SPEED 60000

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

static bool rt_quit = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * thread_asserv
 ****************************************************************************/

static int fd_qe_r;
static int fd_qe_l;
static int32_t pos_r;
static int32_t pos_l;
static int32_t accel_r;
static int32_t accel_l;
static int32_t speed_r;
static int32_t speed_l;
static int32_t eff_speed_r;
static int32_t eff_speed_l;

static int32_t seuil_friction_brushless_r;
static int32_t seuil_friction_brushless_l;

static void *thread_asserv(void *arg)
{
  int ret;
  sigset_t set;
  siginfo_t info;
  bool init_ok = true;
  sigfillset(&set);

  int tick_10ms = 0;

  accel_r = (float) (g_goldorak_go_state.speedR) / 500.0;
  speed_r = 0;
  eff_speed_r = 0;

  accel_l = (float) (g_goldorak_go_state.speedL) / 500.0;
  speed_l = 0;
  eff_speed_l = 0;


  fd_qe_r = open("/dev/qe0", O_RDONLY);
  if (fd_qe_r < 0) {
    printf("jitest_main: open %s failed: %d\n", "/dev/qe0", errno);
    init_ok = false;
  }
  fd_qe_l = open("/dev/qe1", O_RDONLY);
  if (fd_qe_l < 0) {
    printf("jitest_main: open %s failed: %d\n", "/dev/qe1", errno);
    init_ok = false;
  }

  pos_l = 0;
  pos_r = 0;
  seuil_friction_brushless_r = 0;
  seuil_friction_brushless_l = 0;

  while(!rt_quit) {
    sigwaitinfo(&set, &info);

    if (init_ok) {
      ret= ioctl(fd_qe_r, QEIOC_POSITION, (unsigned long)((uintptr_t)&pos_r));
      if (ret < 0) {
        pos_r = 42424242;
      }
      ret= ioctl(fd_qe_l, QEIOC_POSITION, (unsigned long)((uintptr_t)&pos_l));
      if (ret < 0) {
        pos_l = 42424242;
      }

      if (tick_10ms < 500) { /* si t<5s on accelere */
        speed_l += accel_l;
        speed_r += accel_r;
      }

      if (speed_l>0)
        eff_speed_l = speed_l + CONFIG_SPEED_THRESH;
      else if (speed_l<0)
        eff_speed_l = speed_l - CONFIG_SPEED_THRESH;
      if (speed_r>0)
        eff_speed_r = speed_r + CONFIG_SPEED_THRESH;
      else if (speed_r<0)
        eff_speed_r = speed_r - CONFIG_SPEED_THRESH;

      if (eff_speed_l > CONFIG_MAX_SPEED)
        eff_speed_l = CONFIG_MAX_SPEED;
      else if (eff_speed_l < (-CONFIG_MAX_SPEED))
        eff_speed_l = -CONFIG_MAX_SPEED;
      if (eff_speed_r > CONFIG_MAX_SPEED)
        eff_speed_r = CONFIG_MAX_SPEED;
      else if (eff_speed_r < (-CONFIG_MAX_SPEED))
        eff_speed_r = -CONFIG_MAX_SPEED;

      goldo_maxon2_speed(eff_speed_l);
      goldo_maxon1_speed(eff_speed_r);

      if ((seuil_friction_brushless_r==0) && (abs(pos_r)>5)) {
        seuil_friction_brushless_r = eff_speed_r;
      }

      if ((seuil_friction_brushless_l==0) && (abs(pos_l)>5)) {
        seuil_friction_brushless_l = eff_speed_l;
      }
    }

    tick_10ms++;
  }
  printf("thread_asserv done\n");
  return NULL;
}

static void *hog(void *arg)
{
  int i;
  while(!rt_quit) {
    i++;
    if (getchar()==115)
      rt_quit=true;
  }
  printf("hog done\n");
  return NULL;
}

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
            if (value < -65535 || value > 65535)
              {
                printf("Duty out of range: %ld\n", value);
                exit(1);
              }

            gs->speedL = value;
            index += nargs;
            break;

          case 'R':
            nargs = arg_decimal(&argv[index], &value);
            if (value < -65535 || value > 65535)
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
            goldo_maxon2_dis();
            goldo_maxon1_dis();
            exit(0);

          default:
            printf("Unsupported option: %s\n", ptr);
            goldorak_go_help();
            exit(1);
        }
    }
}

static int fdL, fdR;

static int init_devices(void)
{
  struct pwm_info_s info;
  int ret;

  fdL = open("/dev/pwm1", O_RDONLY);
  if (fdL < 0)
    {
      printf("init_devices: open /dev/pwm1 failed: %d\n", errno);
      goto errout;
    }

  fdR = open("/dev/pwm0", O_RDONLY);
  if (fdR < 0)
    {
      printf("init_devices: open /dev/pwm0 failed: %d\n", errno);
      goto errout;
    }

  info.frequency = 10000;
  info.duty = 0;

  ret = ioctl(fdL,PWMIOC_SETCHARACTERISTICS,(unsigned long)((uintptr_t)&info));
  if (ret < 0)
    {
      printf("init_devices: LEFT : ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
      goto errout;
    }

  ret = ioctl(fdR,PWMIOC_SETCHARACTERISTICS,(unsigned long)((uintptr_t)&info));
  if (ret < 0)
    {
      printf("init_devices: RIGHT : ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
      goto errout;
    }

  ret = ioctl(fdL, PWMIOC_START, 0);
  if (ret < 0)
    {
      printf("init_devices: LEFT : ioctl(PWMIOC_START) failed: %d\n", errno);
      goto errout;
    }

  ret = ioctl(fdR, PWMIOC_START, 0);
  if (ret < 0)
    {
      printf("init_devices: RIGHT : ioctl(PWMIOC_START) failed: %d\n", errno);
      goto errout;
    }

  return OK;

 errout:
  return ERROR;
}

static void stop_devices(void)
{
  (void)ioctl(fdL, PWMIOC_STOP, 0);
  (void)ioctl(fdR, PWMIOC_STOP, 0);
  close(fdL);
  close(fdR);
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
int goldorak_go_simple_test_main(int argc, char *argv[])
#endif
{
  struct timer_notify_s notify;
  int ret;
  int tim_fd;
  int i;
  pthread_t id,idhog;

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

  /* Init PWM devices */
  if (init_devices()!=OK)
    return ERROR;

  /* Enable motors */
  printf("goldorak_go_main: enabling motors\n");
  goldo_maxon2_speed(0);
  goldo_maxon1_speed(0);
  goldo_maxon2_en();
  goldo_maxon1_en();
  goldo_maxon2_speed(0);
  goldo_maxon1_speed(0);

  /* Initialize RT thread */
  rt_quit = false;

  pthread_create(&id, NULL, thread_asserv, NULL);
  pthread_setschedprio(id, PTHREAD_DEFAULT_PRIORITY);

  pthread_create(&idhog, NULL, hog, NULL);
  pthread_setschedprio(idhog, PTHREAD_DEFAULT_PRIORITY);

  tim_fd = open(CONFIG_RT_DEVNAME, O_RDONLY);
  if (tim_fd < 0) {
    fprintf(stderr, "ERROR: Failed to open %s: %d\n", CONFIG_RT_DEVNAME, errno);
    return EXIT_FAILURE;
  }

  printf("Set timer interval to %lu\n", (unsigned long)CONFIG_RT_INTERVAL);

  ret = ioctl(tim_fd, TCIOC_SETTIMEOUT, CONFIG_RT_INTERVAL);
  if (ret < 0) {
    fprintf(stderr, "ERROR: Failed to set the timer interval: %d\n", errno);
    close(tim_fd);
    return EXIT_FAILURE;
  }

  /* Attach the timer handler
   *
   * NOTE: If no handler is attached, the timer stop at the first interrupt.
   */
  printf("Attach timer handler\n");

  notify.signo = 1;
  notify.pid   = id;
  notify.arg   = NULL;

  ret = ioctl(tim_fd, TCIOC_NOTIFICATION, (unsigned long)((uintptr_t)&notify));
  if (ret < 0) {
    fprintf(stderr, "ERROR: Failed to set the timer handler: %d\n", errno);
    close(tim_fd);
    return EXIT_FAILURE;
  }

  /* Start the timer */
  printf("Start the timer\n");

  ret = ioctl(tim_fd, TCIOC_START, 0);
  if (ret < 0) {
    fprintf(stderr, "ERROR: Failed to start the timer: %d\n", errno);
    close(tim_fd);
    return EXIT_FAILURE;
  }


  struct timeval tv0, tv1;

  gettimeofday (&tv0, NULL);
  //printf(" tv0.tv_sec = %d\n", tv0.tv_sec);
  //printf(" tv0.tv_usec = %d\n", tv0.tv_usec);

  /* Wait for an approximative duration .. */
  int nsamples = g_goldorak_go_state.duration*1000 / CONFIG_RT_DELAY;
  for (i = 0; i < nsamples; i++) {
    usleep(CONFIG_RT_DELAY);
    if ((i%10)==0)
      printf("%d : rcG:%d , rcD:%d , motG:%d , motD:%d (%d, %d, %d, %d)\n", 
             i, pos_l, pos_r, eff_speed_l, eff_speed_r, 
	     speed_l, speed_r, accel_l, accel_r);
    if (rt_quit)
      break;
  }

  gettimeofday (&tv1, NULL);
  //printf(" tv1.tv_sec = %d\n", tv1.tv_sec);
  //printf(" tv1.tv_usec = %d\n", tv1.tv_usec);

  int delta_tv_sec;
  int delta_tv_usec;

  delta_tv_sec = tv1.tv_sec - tv0.tv_sec;
  delta_tv_usec = delta_tv_sec*1000000 + (tv1.tv_usec - tv0.tv_usec);
  printf(" delta_tv_usec = %d\n", delta_tv_usec);


  /* STOP motors */
  printf("goldorak_go_main: disabling motors\n");
  goldo_maxon2_dis();
  goldo_maxon1_dis();

  /* Stop PWM devices */
  (void)stop_devices();

  /* Stop the timer */
  printf("Stop the timer\n");

  ret = ioctl(tim_fd, TCIOC_STOP, 0);
  if (ret < 0) {
    fprintf(stderr, "ERROR: Failed to stop the timer: %d\n", errno);
    close(tim_fd);
    return EXIT_FAILURE;
  }

  /* Close the timer driver */
  rt_quit=true;
  kill(id, 1);
  kill(idhog, 1);
  printf("Finished\n\n");
  close(tim_fd);

  printf("seuil_friction_brushless_r = %d\n", seuil_friction_brushless_r);
  printf("seuil_friction_brushless_l = %d\n", seuil_friction_brushless_l);

  fflush(stdout);
  return OK;
}
