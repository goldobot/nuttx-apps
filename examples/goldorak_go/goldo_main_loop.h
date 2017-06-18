#ifndef __GOLDO_MAIN_LOOP_H_
#define __GOLDO_MAIN_LOOP_H_
#include "goldo_config.h"
#define GOLDO_MODE_MATCH 1
#define GOLDO_MODE_TEST_ODOMETRY 2
#define GOLDO_MODE_TEST_ASSERV 3
#define GOLDO_MODE_HOMOLOGATION 4
#define GOLDO_MODE_TEST_MOTORS 5
#define GOLDO_MODE_TEST_DYNAMIXELS 6
#define GOLDO_MODE_TEST_ARMS 7
#define GOLDO_MODE_TEST_MATCH 8
#define GOLDO_MODE_UTEST_START_MATCH 20
#define GOLDO_MODE_UTEST_ADVERSARY_DETECTION 21
#define GOLDO_MODE_UTEST_MATCH_TIMER 22
#define GOLDO_MODE_UTEST_FUNNY_ACTION 23
/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
int main_loop_match(void);
int main_loop_test_odometry(void);
int main_loop_test_asserv(void);
int main_loop_homologation(void);
int main_loop_test_motors(void);
int main_loop_test_dynamixels(void);
int main_loop_test_arms(void);
int main_loop_test_match(void);
int main_loop_utest_start_match(void);
int main_loop_utest_adversary_detection(void);
int main_loop_utest_match_timer(void);;
int main_loop_utest_funny_action(void);

#endif /* __GOLDO_MAIN_LOOP_H__ */