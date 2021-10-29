#ifndef ZETA_MOTOR_DRIVER_H_
#define ZETA_MOTOR_DRIVER_H_

#include "src/configuration/pin_configuration.h"
#include "src/serial_helper/serial_helper.h"
#include "src/controller/pid_controller.h"

#define VEL_TRANSMITT_FREQUENCY    30
#define CONTROL_FREQUENCY          100
#define COMMAND_EXECUTE_FREQUENCY  20
#define MA_FILTER_SIZE  15

#define NUM_TASK  3

using namespace zeta_motor_driver;

PidController::motor_t motor1
{
    
    .pwm_pin = MOT1_PWM_PIN,
    .cw_pin = MOT1_CW_PIN,
    .ccw_pin = MOT1_CCW_PIN,
    .float_pin = MOT1_PRESS_PIN,
};
PidController::motor_t motor2
{
    .pwm_pin = MOT2_PWM_PIN,
    .cw_pin = MOT2_CW_PIN,
    .ccw_pin = MOT2_CCW_PIN,
    .float_pin = MOT2_PRESS_PIN,
};
PidController::pid_t pid_param;
PidController controller;
SerialHelper serial_helper;

enum
{
    task_control_motor = 0,
    task_execute_command,
    task_transmitt_velocity,
};

#endif /* ZETA_MOTOR_DRIVER_H_ */
