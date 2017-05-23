
#ifndef __ASSERV_THOMAS_H__
#define __ASSERV_THOMAS_H__
#include <stdbool.h>
#define GOLDO_ASSERV_MAX_COMMANDS 16
#define GOLDO_ASSERV_MAX_BARRIERS 8

typedef enum ASSERV_STATE
{
	ASSERV_STATE_DISABLED=0,
 	ASSERV_STATE_IDLE,
	ASSERV_STATE_MOVING,
 	ASSERV_STATE_ERROR,
  ASSERV_STATE_MATCH_FINISHED
} ASSERV_STATE;

typedef struct goldo_asserv_s
{
  bool initialized;  
  ASSERV_STATE asserv_state;
  int elapsed_time_ms;
  float elapsed_distance_setpoint;
  float speed_setpoint;
  float heading_change_setpoint;
  float yaw_rate_setpoint;
  float motor_pwm_left;
  float motor_pwm_right;
}  goldo_asserv_s;

extern goldo_asserv_s g_asserv;

int goldo_asserv_init(void);
int goldo_asserv_quit(void);

int goldo_asserv_enable(void);

int goldo_asserv_straight_line(float distance, float speed, float accel, float deccel);
int goldo_aserv_rotation(float heading_change,float angular_accel, float angular_deccel);

int goldo_asserv_stop(void);
int goldo_asserv_wait_barrier(int barrier);
int goldo_asserv_wait_finished(void);

int goldo_asserv_arch_init(void);
int goldo_asserv_arch_release(void);
int goldo_asserv_do_step(int dt_ms);

#endif /* __ASSERV_THOMAS_H__ */