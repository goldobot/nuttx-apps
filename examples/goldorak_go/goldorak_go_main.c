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

#include <nuttx/semaphore.h>
#include <nuttx/timers/timer.h>
#include <nuttx/drivers/pwm.h>
#include <nuttx/sensors/qencoder.h>

#include "goldorak_go.h"

#if 1 /* FIXME : DEBUG : HACK */
int __fpclassifyd(double val)
{
  return 4;
}
#endif

#if 1 /* FIXME : DEBUG : HACK homologation 2017 */
extern int goldo_get_start_gpio_state(void);
extern int goldo_get_obstacle_gpio_state(void);
#endif

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

#define CONFIG_RT_INTERVAL 10000 //timer interval in usec

#define CONFIG_RT_DELAY 10000 //sleep interval in usec

#define CONFIG_SPEED_THRESH 0
#define CONFIG_MAX_SPEED 60000

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

static bool rt_quit = false;

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

extern void cmd_gen_set_params (int _init_rmb_1, int _init_rmb_2, 
                                int _max_d2D, int _max_d2Th, 
                                int _max_dD, int _max_dTh);
extern void cmd_corr_set_params (int nKD_D, int nKD_Th, int nKP_D, int nKP_Th);
extern int cmd_init_new_traj (int _type, int _D);
extern void asserv_cmd_gen (void);
extern void asserv_cmd_corr_pd (int rc_val_1, int rc_val_2, 
                                int speed_val_1, int speed_val_2, 
                                int *cmd_motor_1, int *cmd_motor_2);

static int fd_qe_r;
static int fd_qe_l;

static int fd_pwm_l;
static int fd_pwm_r;

static int32_t qe_r;
static int32_t qe_l;

static float x_final;
static float y_final;
static float theta_final;

static float D_final;
static float rc_corr;

int asserv_state=ASSERV_STATE_IDDLE;
int asserv_todo_dist;
int asserv_todo_time;

int robot_rc_val_1;
int robot_rc_val_2;
int robot_rc_val_1_old;
int robot_rc_val_2_old;
int robot_speed_val_1;
int robot_speed_val_2;
int robot_motor_1;
int robot_motor_2;

static void asserv_do_job_blocking(int my_cmd_type, int my_D)
{
  int i;

  int cmd_offset_1 = 150; /* droite */
  int cmd_offset_2 = 150; /* gauche */

/* FIXME : TODO : regler finement les constantes de l'asserv.. */
  int Kpos_D  = 40;
  int Kpos_Th = 40;
  int Kvit_D  = 20;
  int Kvit_Th = 20; //40

  if ((my_cmd_type!=ASSERV_CMD_TYPE_TRANSLATION) && 
      (my_cmd_type!=ASSERV_CMD_TYPE_ROTATION) && 
      (my_cmd_type!=ASSERV_CMD_TYPE_STATIC)) return;

  ioctl(fd_qe_r, QEIOC_RESET, 0);
  ioctl(fd_qe_l, QEIOC_RESET, 0);
  robot_rc_val_1 = 0;
  robot_rc_val_2 = 0;
  robot_rc_val_1_old = 0;
  robot_rc_val_2_old = 0;
  robot_speed_val_1 = 0;
  robot_speed_val_2 = 0;
  robot_motor_1=0;
  robot_motor_2=0;
  cmd_gen_set_params (cmd_offset_1, cmd_offset_2, 
                      1 /*_max_d2D*/, 1 /*_max_d2Th*/, 
                      100 /*_max_dD*/, 100 /*_max_dTh*/);
  cmd_corr_set_params (Kvit_D, Kvit_Th, Kpos_D, Kpos_Th);

  /* this launches the job.. */
  asserv_todo_time = cmd_init_new_traj (my_cmd_type, my_D);
  printf(" my_D = %d, asserv_todo_time = %d\n", my_D, asserv_todo_time);

  usleep (20000);

  i = 0;
  while (asserv_state!=ASSERV_STATE_IDDLE) {
    usleep(CONFIG_RT_DELAY);
    if ((i%10)==0)
      printf("%d : rcG:%d , rcD:%d , motG:%d , motD:%d\n", 
             i, robot_rc_val_2, robot_rc_val_1, 
             robot_motor_2, robot_motor_1);
    i++;
    if (rt_quit)
      break;
  }
  
}

/* FIXME : TODO : mettre thread_asserv() dans asserv.c */
static void *thread_asserv(void *arg)
{
  sigset_t set;
  siginfo_t info;
  sigfillset(&set);

  int tick_10ms = 0;

  int delta_rc_val_1=0;
  int delta_rc_val_2=0;

  asserv_state = ASSERV_STATE_IDDLE;
  asserv_todo_dist = 0;
  asserv_todo_time = 0;

  ioctl(fd_qe_r, QEIOC_RESET, 0);
  ioctl(fd_qe_l, QEIOC_RESET, 0);

  /* FIXME : TODO  */

  /*************************************************************************/
  /** BOUCLE PRINCIPALE ****************************************************/
  while(!rt_quit) {
    /* 0 : debut d'iteration + barriere synchro +++++++++++++++++++++++++++*/
    sigwaitinfo(&set, &info);
    /* fin debut d'iteration ----------------------------------------------*/

    /* 1 : commandes uart (debug essentiellement) +++++++++++++++++++++++++*/
    /* FIXME : TODO */
    /* fin commandes uart -------------------------------------------------*/

    /* 2 : commandes i2c (ou spi..) +++++++++++++++++++++++++++++++++++++++*/
    /* FIXME : TODO */
    /* fin commandes i2c --------------------------------------------------*/

    /* 3 : test capteurs et detection de blockage +++++++++++++++++++++++++*/
    /* FIXME : TODO */
    /* fin test capteurs --------------------------------------------------*/

    /* 4 : machine a etat principale de l'asservissement ++++++++++++++++++*/
    /* FIXME : TODO */
    switch (asserv_state) {
    case ASSERV_STATE_IDDLE:
      if (asserv_todo_time>0) asserv_state = ASSERV_STATE_MOVING;
      break;
    case ASSERV_STATE_MOVING:
      if (asserv_todo_time<=0) {
        asserv_todo_time = 0;
        asserv_state = ASSERV_STATE_IDDLE;
      } else {
        asserv_todo_time -= CONFIG_RT_INTERVAL;
      }
      break;
    case ASSERV_STATE_STOP_BLOCKED:
      /* FIXME : TODO */
      break;
    }
    /* fin machine a etat -------------------------------------------------*/

    /* 5 : odometrie + GPS ++++++++++++++++++++++++++++++++++++++++++++++++*/
    /* Read quadrature encoders */
    ioctl(fd_qe_r, QEIOC_POSITION, (unsigned long)((uintptr_t)&qe_r));
    ioctl(fd_qe_l, QEIOC_POSITION, (unsigned long)((uintptr_t)&qe_l));

    robot_rc_val_1 = qe_r;
    robot_rc_val_2 = qe_l * 0xffc / 0x1000;

    delta_rc_val_1 = robot_rc_val_1 - robot_rc_val_1_old;
    delta_rc_val_2 = robot_rc_val_2 - robot_rc_val_2_old;

    robot_rc_val_1_old = robot_rc_val_1;
    robot_rc_val_2_old = robot_rc_val_2;

    robot_speed_val_1 = delta_rc_val_1;
    robot_speed_val_2 = delta_rc_val_2;

#if 0 /* FIXME : TODO */
    /* GPS */
    robot_theta += delta_rc_val_1 - delta_rc_val_2;
    {
      int delta_dist = (delta_rc_val_1+delta_rc_val_2)/2;
      robot_x += delta_dist * robot_cos (robot_theta)/0x10000;
      robot_y += delta_dist * robot_sin (robot_theta)/0x10000;
    }
#endif
    /* fin odometrie -----------------------------------------------------*/

    /* 6 : GENERATEUR DE TRAJECTOIRE & ASSERV +++++++++++++++++++++++++++++*/
    if ((asserv_state==ASSERV_STATE_MOVING)) {
      /* generateur de trajectoire */
      asserv_cmd_gen ();

      /* asservissement */
      asserv_cmd_corr_pd (robot_rc_val_1,    robot_rc_val_2, 
                          robot_speed_val_1, robot_speed_val_2, 
                          &robot_motor_1,    &robot_motor_2);

      goldo_maxon2_speed(robot_motor_2);
      goldo_maxon1_speed(robot_motor_1);
    }
    /* fin GENERATEUR DE TRAJECTOIRE & ASSERV -----------------------------*/

    /* 7 : affichage d'etat et traces de debug ++++++++++++++++++++++++++++*/
    /* FIXME : TODO  */
    /* fin affichage d'etat et traces de debug ----------------------------*/

    /* 8 : fin d'iteration ++++++++++++++++++++++++++++++++++++++++++++++++*/
    tick_10ms++;
    /* --------------------------------------------------------------------*/
  }
  /** fin BOUCLE PRINCIPALE ************************************************/
  /*************************************************************************/

  goldo_maxon2_speed(0);
  goldo_maxon1_speed(0);

  printf("thread_asserv done\n");
  return NULL;
}

static void *hog(void *arg)
{
  int i;

  while(!rt_quit) {
    i++;
    if (getchar()==115) {
      rt_quit=true;
      goldo_maxon2_speed(0);
      goldo_maxon1_speed(0);
    }
    usleep (20000);
  }
  printf("hog done\n");
  return NULL;
}

static void *detect_obstacle(void *arg)
{
  int i;
  volatile int obstacle_gpio_state = 0;

  while(!rt_quit) {
    i++;
    obstacle_gpio_state = goldo_get_obstacle_gpio_state();
    if (obstacle_gpio_state!=0) {
      rt_quit=true;
      goldo_maxon2_speed(0);
      goldo_maxon1_speed(0);
    }
    usleep (20000);
  }
  printf("detect_obstacle done\n");
  return NULL;
}

/****************************************************************************
 * Name: goldorak_go_help
 ****************************************************************************/

static void goldorak_go_help(void)
{
  /* FIXME : TODO */
}

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

static void parse_args(int argc, FAR char **argv)
{
  /* FIXME : TODO */
}

/****************************************************************************
 * Name: init_devices
 ****************************************************************************/

static int init_devices(void)
{
  struct pwm_info_s info;
  int ret;

  fd_qe_r = open("/dev/qe0", O_RDONLY);
  if (fd_qe_r < 0) 
    {
      printf("init_devices: open %s failed: %d\n", "/dev/qe0", errno);
      goto errout;
    }

  fd_qe_l = open("/dev/qe1", O_RDONLY);
  if (fd_qe_l < 0) 
    {
      printf("init_devices: open %s failed: %d\n", "/dev/qe1", errno);
      goto errout;
    }

  fd_pwm_l = open("/dev/pwm1", O_RDONLY);
  if (fd_pwm_l < 0)
    {
      printf("init_devices: open /dev/pwm1 failed: %d\n", errno);
      goto errout;
    }

  fd_pwm_r = open("/dev/pwm0", O_RDONLY);
  if (fd_pwm_r < 0)
    {
      printf("init_devices: open /dev/pwm0 failed: %d\n", errno);
      goto errout;
    }

  info.frequency = 10000;
  info.duty = 0;

  ret = ioctl(fd_pwm_l,PWMIOC_SETCHARACTERISTICS,(unsigned long)((uintptr_t)&info));
  if (ret < 0)
    {
      printf("init_devices: LEFT : ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
      goto errout;
    }

  ret = ioctl(fd_pwm_r,PWMIOC_SETCHARACTERISTICS,(unsigned long)((uintptr_t)&info));
  if (ret < 0)
    {
      printf("init_devices: RIGHT : ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
      goto errout;
    }

  ret = ioctl(fd_pwm_l, PWMIOC_START, 0);
  if (ret < 0)
    {
      printf("init_devices: LEFT : ioctl(PWMIOC_START) failed: %d\n", errno);
      goto errout;
    }

  ret = ioctl(fd_pwm_r, PWMIOC_START, 0);
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
  (void)ioctl(fd_pwm_l, PWMIOC_STOP, 0);
  (void)ioctl(fd_pwm_r, PWMIOC_STOP, 0);
  close(fd_pwm_l);
  close(fd_pwm_r);
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
  struct timer_notify_s notify;
  int ret;
  int tim_fd;
  //int i;
  pthread_t id,idhog,iddetect;

#if 1 /* FIXME : DEBUG : HACK homologation 2017 */
  int start_gpio_state = 0;
  int obstacle_gpio_state = 0;
  struct timeval tv0, tv1;

  start_gpio_state = goldo_get_start_gpio_state();
  printf ("start_gpio_state = %d\n", start_gpio_state);

  obstacle_gpio_state = goldo_get_obstacle_gpio_state();
  printf ("obstacle_gpio_state = %d\n", obstacle_gpio_state);

  int homo_trans = 0;
  int homo_rot = 0;

  if (argc!=3) {
    printf ("Usage goldorak_go <homo_trans> <homo_rot>\n");
    exit (-1);
  }
  homo_trans = strtol(argv[1], NULL, 10);
  homo_rot = strtol(argv[2], NULL, 10);

  while (start_gpio_state==0) start_gpio_state = goldo_get_start_gpio_state();

  gettimeofday (&tv0, NULL);

  printf ("homo_trans = %d\n", homo_trans);
  printf ("homo_rot = %d\n", homo_rot);

  //return 0;
#endif

#if 0 /* FIXME : DEBUG ++ */
  /* Parse the command line */
  parse_args(argc, argv);
#endif /* FIXME : DEBUG -- */

#if 0 /* FIXME : DEBUG ++ */
  printf(" x_final = %.6f\n", x_final);
  printf(" y_final = %.6f\n", y_final);
  printf(" theta_final = %.6f\n", theta_final);
  printf(" sin(theta_final) = %.6f\n", sin(M_PI*theta_final/180.0));
  printf(" cos(theta_final) = %.6f\n", cos(M_PI*theta_final/180.0));
  return OK;
#endif /* FIXME : DEBUG -- */

  /* Init devices */
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

  pthread_create(&iddetect, NULL, detect_obstacle, NULL);
  pthread_setschedprio(iddetect, PTHREAD_DEFAULT_PRIORITY);

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


#if 0 /* FIXME : DEBUG ++ */
  /* FIXME : TODO */
  /* Wait for asserv job to finish .. */
  asserv_do_job_blocking(ASSERV_CMD_TYPE_ROTATION, D_final);
#else /* FIXME : DEBUG == */
# if 0 /* FIXME : DEBUG ++ */
  asserv_do_job_blocking(ASSERV_CMD_TYPE_TRANSLATION, 1640);
  usleep (500000);
  asserv_do_job_blocking(ASSERV_CMD_TYPE_ROTATION, 940);
  usleep (500000);
  asserv_do_job_blocking(ASSERV_CMD_TYPE_TRANSLATION, 1150);
  usleep (500000);
  asserv_do_job_blocking(ASSERV_CMD_TYPE_ROTATION, 620);
  usleep (500000);
  asserv_do_job_blocking(ASSERV_CMD_TYPE_TRANSLATION, 950);
  usleep (500000);
  //asserv_do_job_blocking(ASSERV_CMD_TYPE_TRANSLATION, -500);
  goldo_maxon2_speed(-25000);
  goldo_maxon1_speed(-25000);
  usleep (500000);
  goldo_maxon2_speed(0);
  goldo_maxon1_speed(0);
# else /* FIXME : DEBUG == */
  asserv_do_job_blocking(ASSERV_CMD_TYPE_TRANSLATION, homo_trans*1640/1150);
  if (rt_quit) goto stop_motors;
  usleep (200000);
  asserv_do_job_blocking(ASSERV_CMD_TYPE_ROTATION, homo_rot*940/620);
  if (rt_quit) goto stop_motors;
  usleep (200000);
  asserv_do_job_blocking(ASSERV_CMD_TYPE_TRANSLATION, homo_trans);
  if (rt_quit) goto stop_motors;
  usleep (200000);
  asserv_do_job_blocking(ASSERV_CMD_TYPE_ROTATION, homo_rot);
  if (rt_quit) goto stop_motors;
  usleep (200000);
  asserv_do_job_blocking(ASSERV_CMD_TYPE_TRANSLATION, homo_trans*950/1150);
  if (rt_quit) goto stop_motors;
  usleep (200000);
  //asserv_do_job_blocking(ASSERV_CMD_TYPE_TRANSLATION, -500);
  goldo_maxon2_speed(-25000);
  goldo_maxon1_speed(-25000);
  usleep (500000);
  goldo_maxon2_speed(0);
  goldo_maxon1_speed(0);
# endif /* FIXME : DEBUG -- */
#endif /* FIXME : DEBUG -- */

 stop_motors:
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
  kill(iddetect, 1);

  goldo_maxon2_speed(0);
  goldo_maxon1_speed(0);
  goldo_maxon2_dis();
  goldo_maxon1_dis();

  printf("Finished\n");
  close(tim_fd);

  while (1) {
    gettimeofday (&tv1, NULL);
    //printf(" tv1.tv_sec = %d\n", tv1.tv_sec);
    //printf(" tv1.tv_usec = %d\n", tv1.tv_usec);

    int delta_tv_sec;
    int delta_tv_usec;

    delta_tv_sec = tv1.tv_sec - tv0.tv_sec;
    delta_tv_usec = delta_tv_sec*1000000 + (tv1.tv_usec - tv0.tv_usec);
    //printf(" delta_tv_usec = %d\n", delta_tv_usec);
    printf(" delta_tv_sec = %d\n", delta_tv_sec);

    if (delta_tv_sec>90) {
      int fd_funny = -1;
      struct pwm_info_s info_funny;
      int ret;

      fd_funny = open("/dev/pwm8", O_RDONLY);
      if (fd_funny < 0) {
        printf("funny: open /dev/pwm8 failed: %d\n", errno);
        exit(-1);
      }

      info_funny.frequency = 100;
      info_funny.duty = 3000;

      ret = ioctl(fd_funny,PWMIOC_SETCHARACTERISTICS,
                  (unsigned long)((uintptr_t)&info_funny));
      if (ret < 0) {
        printf("funny: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
        exit(-1);
      }

      ret = ioctl(fd_funny, PWMIOC_START, 0);
      if (ret < 0) {
        printf("funny: ioctl(PWMIOC_START) failed: %d\n", errno);
        exit(-1);
      }

      sleep (1);

      (void)ioctl(fd_funny, PWMIOC_STOP, 0);
      close(fd_funny);

      break;
    }

    usleep (100000);
  }

  fflush(stdout);
  return OK;
}
