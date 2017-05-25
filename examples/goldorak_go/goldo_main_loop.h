#ifndef __GOLDO_MAIN_LOOP_H_
#define __GOLDO_MAIN_LOOP_H_
#include "goldo_config.h"
#define GOLDO_MODE_MATCH 1
#define GOLDO_MODE_TEST_ODOMETRY 2
#define GOLDO_MODE_TEST_ASSERV 3
#define GOLDO_MODE_HOMOLOGATION 4
#define GOLDO_MODE_TEST_MOTORS 5
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

#endif /* __GOLDO_MAIN_LOOP_H__ */