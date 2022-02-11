#ifndef ZETA_MOTOR_DRIVER_SERIAL_HELPER_H_
#define ZETA_MOTOR_DRIVER_SERIAL_HELPER_H_
#include "configuration_helper/configuration_helper.h"

#include <HardwareSerial.h>
#define DEBUG_PORT  Serial1
// #define SERIAL_DEBUG
#define SERIAL_SPEED            115200 // if too slow, print take too long (print() disables interrupt!)
#define TX_BUFFER_SIZE          16
#define RX_BUFFER_SIZE          16

/* comm */
#define RECEIVE_NO_DATA         0x00

#define POS_PID                 0

#define MOTOR_FORWARD           1
#define MOTOR_BACKWARD          -1
#define ROTATION_FORWARD        1
#define ROTATION_BACKWARD       0
#define POS_MONITORING_UNIT     1
#define POS_DIR                 1
#define POS_BYTE_HIGH           0
#define POS_BYTE_LOW            1
#define POS_MOT1_VEL_H          4
#define POS_MOT1_VEL_L          5
#define POS_MOT2_VEL_H          2
#define POS_MOT2_VEL_L          3

#define RETURN_CODE             0x60
#define ERROR_CODE              0x90

#define LENGTH_MONITORING       7

#define FLOAT32_ZERO            0.0f

#define DIGIT_VELOCITY          FLOAT_PRECISION_3DIGIT
#define DIGIT_RPM               FLOAT_PRECISION_1DIGIT
#define DIGIT_PPS               FLOAT_PRECISION_1DIGIT
#define DIGIT_VEL_LINEAR        FLOAT_PRECISION_3DIGIT
#define DIGIT_VEL_ANGULAR       FLOAT_PRECISION_3DIGIT
#define DIGIT_WHEEL_POSITION    FLOAT_PRECISION_3DIGIT

#define MOTOR1_FORWARD          (0b01)
#define MOTOR2_FORWARD          (0b10)

#define LENGTH_SET_MONITORING    2
#define LENGTH_SET_VELOCITY      6

namespace zeta_motor_driver
{
class SerialHelper : public ConfigurationHelper
{
    typedef struct
    {
        float vel_cmd;  // m/s
        float vel_cur;  // m/s
        float position; // radian
        bool  runnable;
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
        pid_configure_mode,
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
        pid_last = 0xff,
    };
    enum class MonitoringUnit : uint8_t
    {
        monitoring_mps = 0,
        monitoring_rpm,
        monitoring_pps,
        monitoring_complex,
        monitoring_last,
    };
    
    public:
        SerialHelper()
        {
            serial_speed     = ConfigurationHelper::GetBaudrate();
            receive_index    = 0;
            com_error        = ComError::no_error;
            monitoring_unit  = MonitoringUnit::monitoring_complex;
            command_receive  = false;
            wheel_radius     = ConfigurationHelper::GetWheelRadius();
            ppr              = ConfigurationHelper::GetPPR();
            wheel_seperation = ConfigurationHelper::GetWheelSeperation();
        }
        void    Begin();
        void    ReceiveData();    // from user
        bool    ExecuteCommand();
        void    SetVelocityMessage();
        void    SetMessage(uint8_t[],uint8_t);
        void    GetMessage(uint8_t[],uint32_t*);
        bool    IsBrake();
        motor_state_t motor1_state;
        motor_state_t motor2_state;
    private:
        HardwareSerial& stream;
        int32_t  serial_speed;
        uint8_t  receive_message[RX_BUFFER_SIZE];
        uint8_t  transmit_message[TX_BUFFER_SIZE];
        uint8_t  transmit_index;
        uint8_t  receive_index;
        float    wheel_radius;
        float    ppr;
        float    wheel_seperation;
        bool     command_receive;
        
        ComError       com_error;
        MonitoringUnit monitoring_unit;

        void     FlushReceiveMessage();
        uint8_t  Checksum(uint8_t[],uint8_t);
        bool     Run(uint8_t);
        bool     VerifyFormat();
        bool     VerifyLength();
        bool     VerifyChecksum();
        void     ReturnData();
        void     ReturnError();
        bool     SetMonitoringUnit();
        bool     SetVelocity();
        bool     ReleaseMotor();
        bool     BrakeMotor();
        bool     SetMaxSpeed();
        bool     SetMinSpeed();
        bool     SetPPR();
        bool     SetWheelRadius();
        bool     SetIncreasingTime();
        bool     SetDecreasingTime();
};
} /* namespace zeta_motor_driver */

#endif /* ZETA_MOTOR_DRIVER_SERIAL_HELPER_H_ */