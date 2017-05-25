#ifndef __GOLDO_MATCH_TIMER_H__
#define __GOLDO_MATCH_TIMER_H__
#include "goldo_config.h"

int goldo_match_timer_start(int time_s);
int goldo_match_timer_get_value(void);/* value in milliseconds*/
bool goldo_match_timer_is_finished(void);

#endif /* __GOLDO_MATCH_TIMER_H__ */ 