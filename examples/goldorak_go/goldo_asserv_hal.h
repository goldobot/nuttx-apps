#ifndef __GOLDO_ASSERV_HAL_H__
#define __GOLDO_ASSERV_HAL_H__
#include "goldo_config.h"

int goldo_asserv_hal_init(void);
int goldo_asserv_hat_quit(void);
int goldo_asserv_hal_set_motors_pwm(int left, int right);
int goldo_asserv_hal_set_motors_enable(bool left, bool right);

#endif /* __GOLDO_ASSERV_HAL_H__ */