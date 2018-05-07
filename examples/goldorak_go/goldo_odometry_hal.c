#include "goldo_odometry.h"
#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <math.h>


#include <nuttx/sensors/qencoder.h>



static int fd_qe_r,fd_qe_l;


extern int goldo_odometry_hal_read_encoders(int* left, int* right);

int goldo_odometry_hal_init(void)
{
  /* Open encoder devices */
  fd_qe_r = open("/dev/qe0", O_RDONLY);
  if (fd_qe_r < 0) {
    printf("init_devices: open %s failed: %d\n", "/dev/qe0", errno);
    goto errout;
  }

  fd_qe_l = open("/dev/qe1", O_RDONLY);
  if (fd_qe_l < 0) {
    printf("init_devices: open %s failed: %d\n", "/dev/qe1", errno);
    goto errout;
  }

  /* Read initial encoder values */   
  int32_t qe_r;
  int32_t qe_l;
  ioctl(fd_qe_r, QEIOC_POSITION, (unsigned long)((uintptr_t)&qe_r));
  ioctl(fd_qe_l, QEIOC_POSITION, (unsigned long)((uintptr_t)&qe_l));

#if 0 /* FIXME : DEBUG : correction PR 2018 */
  g_odometry_state.counts_left = qe_l;
  g_odometry_state.counts_right = qe_r;
#else
  g_odometry_state.counts_left = -qe_l;
  g_odometry_state.counts_right = -qe_r;
#endif
  return OK;

 errout:
  return ERROR;
}

int goldo_odometry_hal_release(void)
{
  close(fd_qe_l);
  close(fd_qe_r);
  return OK;
}


int goldo_odometry_hal_read_encoders(int32_t* left, int32_t* right)
{
  ioctl(fd_qe_l, QEIOC_POSITION, (unsigned long)((uintptr_t)left));
  ioctl(fd_qe_r, QEIOC_POSITION, (unsigned long)((uintptr_t)right));
#if 1 /* FIXME : DEBUG : correction PR 2018 */
  *left = -(*left);
  *right = -(*right);
#endif
  return OK;
}
