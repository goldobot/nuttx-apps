#include "goldo_asserv_hal.h"
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


typedef struct goldo_asserv_hal_s
{
  int initialized;
  int fd_left;
  int fd_right;
  int pwm_left;
  int pwm_right;

} goldo_asserv_hal_s;

goldo_asserv_hal_s g_asserv_hal;

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

int goldo_asserv_hal_init(void)
{
  int ret = init_devices();
  if(ret == OK)
  {
    g_asserv_hal.initialized = TRUE;
    return OK;
  } else
  {
    g_asserv_hal.initialized = FALSE;
    return ERROR;
  }
}

int goldo_asserv_hat_quit(void)
{
  if(!g_asserv_hal.initialized)
  {
    return ERROR;
  }
  stop_devices();
  return OK;
}

int goldo_asserv_hal_set_motors_enable(int left, int right)
{
  if(!g_asserv_hal.initialized)
  {
    return ERROR;
  }

  if(left)
  {
    goldo_maxon2_en();
  } else
  {
    goldo_maxon2_dis();
  }
  if(right)
  {
    goldo_maxon1_en();
  } else
  {
    goldo_maxon1_dis();
  }
  return OK;
}

int goldo_asserv_hal_set_motors_pwm(int left, int right)
{
  goldo_maxon2_speed(left);
  goldo_maxon1_speed(right);
}

