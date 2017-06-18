#ifndef __ASSERV_THOMAS_H__
#define __ASSERV_THOMAS_H__
#include "goldo_config.h"
#include "goldo_pid_filter.h"

#include <stdint.h>

#define GOLDO_ASSERV_MAX_COMMANDS 16
#define GOLDO_ASSERV_MAX_BARRIERS 8

typedef enum GOLDO_ASSERV_STATE
{
	ASSERV_STATE_DISABLED=0,
 	ASSERV_STATE_IDLE,
	ASSERV_STATE_MOVING,
  ASSERV_STATE_RECALAGE,
  ASSERV_STATE_EMERGENCY_STOP,
  ASSERV_STATE_STOPPED,
  ASSERV_STATE_MATCH_FINISHED,
  ASSERV_STATE_ERROR
} GOLDO_ASSERV_STATE;

typedef struct goldo_asserv_config_s
{

} goldo_asserv_config_s;

typedef struct goldo_asserv_s
{
  bool initialized;  
  GOLDO_ASSERV_STATE asserv_state;
  uint32_t elapsed_time_ms;
  float elapsed_distance_setpoint;
  float speed_setpoint;
  float heading_change_setpoint;
  float yaw_rate_setpoint;
  float motor_pwm_left;
  float motor_pwm_right;
}  goldo_asserv_s;

typedef struct goldo_asserv_trace_point_s
{
  float elapsed_distance;
  float heading_change;
  float speed;
  float yaw_rate;
  float elapsed_distance_setpoint;
  float speed_setpoint;
  float heading_change_setpoint;
  float yaw_rate_setpoint;
  float motor_pwm_left;
  float motor_pwm_right;
} goldo_asserv_trace_point_s;

extern goldo_asserv_s g_asserv;

int goldo_asserv_init(void);
int goldo_asserv_quit(void);

/* Enable control loop */
int goldo_asserv_enable(void);
int goldo_asserv_disable(void);

/* Call on adversary detection to stop the robot.*/
int goldo_asserv_emergency_stop(void);
int goldo_asserv_match_finished(void);

int goldo_asserv_straight_line(float distance, float speed, float accel, float deccel);
int goldo_asserv_rotation(float heading_change,float yaw_rate, float angular_accel, float angular_deccel);
int goldo_asserv_position_step(float distance);
int goldo_asserv_heading_step(float angle);
int goldo_asserv_wait(float t);
int goldo_asserv_recalage(float pwm);

/* Wait until all commands are finished or an error occurs*/
GOLDO_ASSERV_STATE goldo_asserv_wait_finished(void);

int goldo_asserv_get_distance_pid_values(float* k_p, float* k_i, float* k_d, float* lim_i, float* speed_ff);
int goldo_asserv_set_distance_pid_values(float k_p, float k_i, float k_d, float lim_i, float speed_ff);
int goldo_asserv_get_heading_pid_values(float* k_p, float* k_i, float* k_d, float* lim_i, float* speed_ff);
int goldo_asserv_set_heading_pid_values(float k_p, float k_i, float k_d, float lim_i, float speed_ff);

/* Internal functions
   arch_init and arch_release manage the realtime thread.
   do_step perform all computations */

int goldo_asserv_arch_init(void);
int goldo_asserv_arch_release(void);
int goldo_asserv_do_step(int dt_ms);

int goldo_asserv_start_trace(goldo_asserv_trace_point_s* buffer, int size, int divider);

#endif /* __ASSERV_THOMAS_H__ */