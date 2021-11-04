#ifndef ZETA_MOTOR_DRIVER_SERIAL_HELPER_H_
#define ZETA_MOTOR_DRIVER_SERIAL_HELPER_H_
#include "configuration_helper/configuration_helper.h"
#include "../controller/pid_controller.h"

#include <HardwareSerial.h>
// #define SERIAL_DEBUG
#define COM_PORT        Serial1
#define SERIAL_SPEED    115200 // if too slow, print take too long (print() disables interrupt!)
#define TX_BUFFER_SIZE  16
#define RX_BUFFER_SIZE  16

/* comm */
#define START_BYTE1     0xAA
#define START_BYTE2     0xBB
#define END_BYTE1       0xDD
#define END_BYTE2       0x55
#define POS_LENGTH      2
#define POS_CHECKSUM    3
#define POS_PID         3
#define POS_DATA_START  4
#define RETURN_CODE     0x60

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
        pid_set_version,
        pid_set_p_gain,
        pid_set_i_gain,
        pid_set_d_gain,
        pid_set_max_speed,
        pid_set_min_speed,
        pid_set_ppr,
        pid_set_wheel_radius,
        pid_set_increasing_time,
        pid_set_decreasing_time,
        pid_imu = 61,
        pid_sonar,
        pid_last,
    };
    enum class MonitoringMode : uint8_t
    {
        monitoring_vel = 0,
        monitoring_rpm,
    };
    public:
        SerialHelper() : stream(COM_PORT)
        {
            ConfigurationHelper::Update();
            serial_speed    = ConfigurationHelper::GetBaudrate();
            message_index   = 0;
            com_error       = ComError::no_error;
            monitoring_mode = MonitoringMode::monitoring_vel;
            command_receive = false;
            monitoring      = true;
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
        bool     command_receive;
        bool     monitoring;
        
        ComError       com_error;
        MonitoringMode monitoring_mode;

        void     FlushReceiveMessage();
        uint8_t  Checksum(uint8_t[],int);
        bool     Run(uint8_t);
        bool     VerifyFormat();
        bool     VerifyLength();
        bool     VerifyChecksum();
        void     SetVelocity();
        void     ReturnData();
        

};
} /* namespace zeta_motor_driver */

#endif /* ZETA_MOTOR_DRIVER_SERIAL_HELPER_H_ */