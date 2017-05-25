#include "goldo_match_timer.h"
#include "goldo_asserv.h"

static uint32_t s_match_timer_begin = 0;
static uint32_t s_match_timer_end = -1;

int goldo_match_timer_start(int time_s)
{
  s_match_timer_begin = g_asserv.elapsed_time_ms;
  s_match_timer_end = g_asserv.elapsed_time_ms + time_s*1000;
  return OK;
}

int goldo_match_timer_get_value(void)
{
  return g_asserv.elapsed_time_ms - s_match_timer_begin;
}

bool goldo_match_timer_is_finished(void)
{
return g_asserv.elapsed_time_ms < s_match_timer_end;
}