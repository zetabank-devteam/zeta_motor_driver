#ifndef ZETA_MOTOR_DRIVER_H_
#define ZETA_MOTOR_DRIVER_H_
#define FW_VERSION "0.3.0"

#include "src/configuration/pin_configuration.h"
#include "src/serial_helper/serial_helper.h"
#include "src/controller/pid_controller.h"

#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/String.h>

#define VEL_TRANSMIT_FREQUENCY     20
#define CONTROL_FREQUENCY          30
#define COMMAND_EXECUTE_FREQUENCY  50
#define VERSION_PUBLISH_FREQUENCY  0.1
#define BLINK_LED_FREQUENCY        1
#define MA_FILTER_SIZE             15
#define NUM_TASK                   8

#define CONTROL_STREAM  Serial

zeta_motor_driver::PidController::motor_t motor1
{
    .pwm_pin   = MOTR_PWM_PIN,
    .dir_pin   = MOTR_DIR_PIN,
    .start_pin = MOTR_BK_PIN,
    .float_pin = MOTR_PRESS_PIN,
};
zeta_motor_driver::PidController::motor_t motor2
{
    .pwm_pin   = MOTL_PWM_PIN,
    .dir_pin   = MOTL_DIR_PIN,
    .start_pin = MOTL_BK_PIN,
    .float_pin = MOTL_PRESS_PIN,
};
zeta_motor_driver::PidController::pid_t pid_param;
zeta_motor_driver::PidController        controller;
zeta_motor_driver::SerialHelper         serial_helper;
enum task_name
{
    task_control_motor = 0,
    task_execute_command,
    task_transmit_velocity,
    task_publish_version,
    task_blink_led,
};

ros::NodeHandle nh;
void SerialInputCallback(const std_msgs::UInt8MultiArray);
ros::Subscriber<std_msgs::UInt8MultiArray> serial_input_subscriber("motor_driver_serial_input", &SerialInputCallback);
std_msgs::UInt8MultiArray serial_output_msg;
ros::Publisher serial_output_publisher("motor_driver_serial_output", &serial_output_msg);
std_msgs::String fw_version_msg;
ros::Publisher fw_version_publisher("driver_fw_version", &fw_version_msg);

#endif /* ZETA_MOTOR_DRIVER_H_ */
