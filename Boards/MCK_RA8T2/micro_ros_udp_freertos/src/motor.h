#ifndef MTR_MAIN_H
#define MTR_MAIN_H

#include <stdint.h>

#define MAX_MTR_SPEED_RPM      (4000)
#define MIN_MTR_SPEED_RPM      (200)
#define MOTOR_STATUS_STOP      (0)
#define MOTOR_STATUS_RUN       (1)
#define MOTOR_STATUS_ERROR     (2)

void mtr_init( void );
void process_motor_command( int rpm );

#endif /* MTR_MAIN_H */
