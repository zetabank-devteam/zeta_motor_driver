#ifndef ZETA_MOTOR_DRIVER_H_
#define ZETA_MOTOR_DRIVER_H_
// #define NO_ROS
#define FW_VERSION "0.2.9"

#include "src/configuration/pin_configuration.h"
#include "src/serial_helper/serial_helper.h"
#include "src/controller/pid_controller.h"

#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/String.h>

#define VEL_TRANSMIT_FREQUENCY     20
#define CONTROL_FREQUENCY          30
#define COMMAND_EXECUTE_FREQUENCY  50
#define MA_FILTER_SIZE             15
#define NUM_TASK                   5

#define CONTROL_STREAM  Serial

using namespace zeta_motor_driver;

PidController::motor_t motor1
{
    .pwm_pin   = MOT1_PWM_PIN,
    .cw_pin    = MOT1_CW_PIN,
    .ccw_pin   = MOT1_CCW_PIN,
    .float_pin = MOT1_PRESS_PIN,
};
PidController::motor_t motor2
{
    .pwm_pin   = MOT2_PWM_PIN,
    .cw_pin    = MOT2_CW_PIN,
    .ccw_pin   = MOT2_CCW_PIN,
    .float_pin = MOT2_PRESS_PIN,
};
PidController::pid_t pid_param;
PidController        controller;
SerialHelper         serial_helper;
enum task_name
{
    task_control_motor = 0,
    task_execute_command,
    task_transmit_velocity,
};

#ifndef NO_ROS
ros::NodeHandle nh;
void SerialInputCallback(const std_msgs::UInt8MultiArray);
ros::Subscriber<std_msgs::UInt8MultiArray> serial_input_subscriber("motor_driver_serial_input", &SerialInputCallback);
std_msgs::UInt8MultiArray serial_output_msg;
ros::Publisher serial_output_publisher("motor_driver_serial_output", &serial_output_msg);
std_msgs::String fw_version_msg;
ros::Publisher fw_version_publisher("driver_fw_version", &fw_version_msg);
#endif

#endif /* ZETA_MOTOR_DRIVER_H_ */
