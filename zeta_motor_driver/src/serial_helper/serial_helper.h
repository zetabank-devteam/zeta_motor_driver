#ifndef ZETA_MOTOR_DRIVER_SERIAL_HELPER_H_
#define ZETA_MOTOR_DRIVER_SERIAL_HELPER_H_
#include "configuration_helper/configuration_helper.h"
#include "../controller/pid_controller.h"

#include <HardwareSerial.h>
// #define SERIAL_DEBUG
#define COM_PORT                Serial1
#define SERIAL_SPEED            115200 // if too slow, print take too long (print() disables interrupt!)
#define TX_BUFFER_SIZE          16
#define RX_BUFFER_SIZE          16

/* comm */
#define START_BYTE1             0xAA
#define START_BYTE2             0xBB
#define END_BYTE1               0xDD
#define END_BYTE2               0x55
#define RECEIVE_NO_DATA         0x00

#define POS_START_BYTE1         0
#define POS_START_BYTE2         1
#define POS_LENGTH              2
#define POS_CHECKSUM            3
#define POS_PID                 3
#define POS_DATA_START          4
#define POS_MONITORING_UNIT     4
#define POS_DIR                 4
#define POS_VEL_H               0
#define POS_VEL_L               1
#define POS_MOT1_VEL_H          7
#define POS_MOT1_VEL_L          8
#define POS_MOT2_VEL_H          5
#define POS_MOT2_VEL_L          6

#define RETURN_CODE             0x60
#define ERROR_CODE              0x90

#define LENGTH_MONITORING       7

#define FLOAT32_ZERO            0.0f

#define DIGIT_VELOCITY          3
#define DIGIT_RPM               1

#define MOTOR1_FORWARD          (0b01)
#define MOTOR2_FORWARD          (0b10)

#define LENGTH_MONITORING  7

namespace zeta_motor_driver
{
class SerialHelper : public ConfigurationHelper
{
    typedef struct
    {
        float vel_cmd;
        float vel_cur;
        PidController::MotorState motor_state;
    }motor_state_t;
    enum class ComError : uint8_t
    {
        no_error,
        error_mismatch_checksum,
        error_mismatch_format,
        error_mismatch_length,
        error_unknown_pid,
    };
    enum class ParameterID : uint8_t
    {
        pid_monitoring = 0,
        pid_run_motor,
        pid_brake_motor,
        pid_release_motor,
        pid_set_p_gain,
        pid_set_i_gain,
        pid_set_d_gain,
        pid_set_max_speed,
        pid_set_min_speed,
        pid_set_ppr,
        pid_set_wheel_radius,
        pid_set_increasing_time,
        pid_set_decreasing_time,
        pid_get_param,
        pid_imu = 61,
        pid_sonar,
        pid_last,
    };
    enum class MonitoringUnit : uint8_t
    {
        monitoring_mps = 0,
        monitoring_rpm,
        monitoring_last,
    };
    public:
        SerialHelper() : stream(COM_PORT)
        {
            ConfigurationHelper::Update();
            serial_speed    = ConfigurationHelper::GetBaudrate();
            message_index   = 0;
            com_error       = ComError::no_error;
            monitoring_unit = MonitoringUnit::monitoring_mps;
            command_receive = false;
            wheel_radius    = ConfigurationHelper::GetWheelRadius();
        }
        void Begin();
        void ReceiveData();    // from user
        void ExecuteCommand();
        void TransmittVelocity();
        motor_state_t motor1_state;
        motor_state_t motor2_state;
    private:
        HardwareSerial& stream;
        int32_t  serial_speed;
        uint8_t  receive_message[RX_BUFFER_SIZE];
        int16_t  message_index;
        float    wheel_radius;
        bool     command_receive;
        
        ComError       com_error;
        MonitoringUnit monitoring_unit;

        void     FlushReceiveMessage();
        uint8_t  Checksum(uint8_t[],int);
        bool     Run(uint8_t);
        bool     VerifyFormat();
        bool     VerifyLength();
        bool     VerifyChecksum();
        void     ReturnData();
        void     SetMonitoringUnit();
        void     SetVelocity();
        void     ReleaseMotor();
        void     BrakeMotor();
        void     SetPGain();
        void     SetIGain();
        void     SetDGain();
        void     SetMaxSpeed();
        void     SetMinSpeed();
        void     SetPPR();
        void     SetWheelRadius();
        void     SetIncreasingTime();
        void     SetDecreasingTime();

};
} /* namespace zeta_motor_driver */

#endif /* ZETA_MOTOR_DRIVER_SERIAL_HELPER_H_ */