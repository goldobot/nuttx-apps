#include "goldo_odometry.h"

goldo_odometry_state_s g_odometry_state;

static goldo_odometry_config_s odometry_config;
static int fd_qe_r,fd_qe_l;


int goldo_odometry_init(void)
{
	/* Set initial values */
	g_odometry_state-> elapsed_distance = 0;
	g_odometry_state-> heading_change = 0;
	g_odometry_state-> pos_x = 0;
	g_odometry_state-> pos_y = 0;
	g_odometry_state-> heading = 0;
	g_odometry_state-> speed = 0;
	g_odometry_state-> speed_x = 0;
	g_odometry_state-> speed_y = 0;
	g_odometry_state-> yaw_rate = 0;

	/* Open encoder devices */
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

 	/* Read initial encoder values */   
    int32_t qe_r;
    int32_t qe_l;
    ioctl(fd_qe_r, QEIOC_POSITION, (unsigned long)((uintptr_t)&qe_r));
    ioctl(fd_qe_l, QEIOC_POSITION, (unsigned long)((uintptr_t)&qe_l));
    g_odometry_state->counts_left = qe_l;
    g_odometry_state->counts_right = qe_r;
    return OK;

    errout:
    	return ERROR;

}

int goldo_odometry_quit(void)
{
  close(fd_qe_l);
  close(fd_qe_r);
  return OK;
}

i
int goldo_odometry_set_config(goldo_odometry_config_s* config)
{
	odometry_config = *config;
	return OK;
}

int goldo_odometry_update(void)
{
	int32_t qe_r;
    int32_t qe_l;

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
    return OK;
}


int goldo_odometry_set_position(float x, float y, float heading)
{
	return OK;
}