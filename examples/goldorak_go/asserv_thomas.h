
#ifndef __ASSERV_THOMAS_H__
#define __ASSERV_THOMAS_H__



#define ASSERV_STATE_IDLE 0
#define ASSERV_STATE_MOVING 1
#define ASSERV_STATE_ERROR 2

int goldo_asserv_hal_init(void);
int goldo_asserv_hat_quit(void);
int goldo_asserv_hal_set_motors_pwm(int left, int right);
int goldo_asserv_hal_set_motors_enable(int left, int right);

int goldo_asserv_init(void);
int goldo_asserv_stop(void);
int goldo_asserv_wait_barrier(int barrier);
int goldo_asserv_wait_finished(void);
int goldo_asserv_quit(void);

#endif /* __ASSERV_THOMAS_H__ */