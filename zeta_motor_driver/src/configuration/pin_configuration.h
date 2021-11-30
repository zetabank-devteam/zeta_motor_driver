#ifndef ZETA_MOTOR_DRIVER_PIN_CONFIGURATION_H_
#define ZETA_MOTOR_DRIVER_PIN_CONFIGURATION_H_
#include "../pre_define.h"
#ifndef TEST_ROBOT
#define MOT1_CW_PIN       8
#define MOT1_CCW_PIN      9
#define MOT1_PRESS_PIN    10
#define MOT1_PWM_PIN      11
#define MOT1_ENCODER_PIN  21
#define MOT2_CW_PIN       4
#define MOT2_CCW_PIN      5
#define MOT2_PRESS_PIN    6
#define MOT2_PWM_PIN      2
#define MOT2_ENCODER_PIN  20
#else
#define MOT1_CW_PIN       6
#define MOT1_CCW_PIN      7
#define MOT1_PRESS_PIN    9
#define MOT1_PWM_PIN      2
#define MOT1_ENCODER_PIN  21
#define MOT2_CW_PIN       4
#define MOT2_CCW_PIN      5
#define MOT2_PRESS_PIN    8
#define MOT2_PWM_PIN      11
#define MOT2_ENCODER_PIN  20
#endif
#endif /* ZETA_MOTOR_DRIVER_PIN_CONFIGURATION_H_ */