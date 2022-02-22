#include "serial_helper.h"

using namespace zeta_motor_driver;

// #include <ros.h>
// extern ros::NodeHandle nh;

void SerialHelper::Begin()
{
    motor1_state.runnable = true;
    motor2_state.runnable = true;
}

void SerialHelper::ReceiveData()
{
}

bool SerialHelper::ExecuteCommand()
{
    wheel_radius    = ConfigurationHelper::GetWheelRadius();
    ppr             = ConfigurationHelper::GetPPR();
    uint8_t pid = receive_message[0];
    if(command_receive)
    {
        if(pid != static_cast<uint8_t>(ParameterID::pid_last))
        {
            Run(pid);
        }
        else
        {
            com_error = ComError::no_error;
        }
        command_receive = false;
        FlushReceiveMessage();
        return true;
    }
    else
    {
        FlushReceiveMessage();
        return false;
    }
    
}

bool SerialHelper::Run(uint8_t pid)
{
    float gain = 0.0f;
    switch(static_cast<ParameterID>(pid))
    {
        case ParameterID::pid_monitoring:
            /*if(SetMonitoringUnit())
            {
                ReturnData();
            }*/
            
            break;
        case ParameterID::pid_run_motor:
            if(receive_index != LENGTH_SET_VELOCITY)
            {
                ReturnError();
                break;
            }
            if(!motor1_state.runnable || !motor2_state.runnable)
            {
                ReturnError();
                break;
            }
            if(SetVelocity())
            {
                ReturnData();
            }
            else
            {
                ReturnError();
            }
            SetConfigurable(false);
            break;
        case ParameterID::pid_brake_motor:
            if(receive_index != 1)
            {
                ReturnError();
                break;
            }
            if(BrakeMotor())
            {
                ReturnData();
            }
            break;
        case ParameterID::pid_release_motor:
            if(receive_index != 1)
            {
                ReturnError();
                break;
            }
            if(ReleaseMotor())
            {
                ReturnData();
            }
            break;
        case ParameterID::pid_configure_mode:
            if(receive_index != 1)
            {
                ReturnError();
                break;
            }
            SetConfigurable(true);
            ReturnData();
            break;
        case ParameterID::pid_set_p_gain:
            gain = BytesToFloat(receive_message[1], receive_message[2], FLOAT_PRECISION_1DIGIT);
            if(gain > 0.0f && gain <= (float(0xffff) / 10.0f))
            {
                if(ConfigurationHelper::SetPGain(gain))
                {
                    ReturnData();
                }
                else
                {
                    ReturnError();
                }
            }
            else
            {
                ReturnError();
            }
            break;
        case ParameterID::pid_set_i_gain:
            gain = BytesToFloat(receive_message[1], receive_message[2], FLOAT_PRECISION_1DIGIT);
            if(gain > 0.0f && gain <= (float(0xffff) / 10.0f))
            {
                
                if(ConfigurationHelper::SetIGain(gain))
                {
                    ReturnData();
                }
                else
                {
                    ReturnError();
                }
            }
            else
            {
                ReturnError();
            }
            break;
        case ParameterID::pid_set_d_gain:
            gain = BytesToFloat(receive_message[1], receive_message[2], FLOAT_PRECISION_1DIGIT);
            if(gain > 0.0f && gain <= (float(0xffff) / 10.0f))
            {
                if(ConfigurationHelper::SetDGain(gain))
                {
                    ReturnData();
                }
                else
                {
                    ReturnError();
                }
            }
            else
            {
                ReturnError();
            }
            break;
        case ParameterID::pid_set_max_speed:
            //SetMaxSpeed();
            ReturnData();
            break;
        case ParameterID::pid_set_min_speed:
            //SetMinSpeed();
            ReturnData();
            break;
        case ParameterID::pid_set_ppr:
            //SetPPR();
            ReturnData();
            break;
        case ParameterID::pid_set_wheel_radius:
            //SetWheelRadius();
            ReturnData();
            break;
        case ParameterID::pid_set_increasing_time:
            //SetIncreasingTime();
            ReturnData();
            break;
        case ParameterID::pid_set_decreasing_time:
            // if(SetDecreasingTime())
            // {
            //     ReturnData();
            // }
            break;
        case ParameterID::pid_get_param:
            // GetParam();
        default:
            com_error = ComError::error_unknown_pid;
            return false;
    }
    return true;
}
////////////////////////////////////////////
// Actual execution codes here
////////////////////////////////////////////
bool SerialHelper::SetMonitoringUnit()
{
    //if(receive_index != 2)
    if(receive_message[POS_MONITORING_UNIT] < static_cast<uint8_t>(MonitoringUnit::monitoring_last))
    {
        monitoring_unit = static_cast<MonitoringUnit>(receive_message[POS_MONITORING_UNIT]);
        return true;
    }
}

void SerialHelper::SetVelocityMessage()
{
    uint8_t dir = 0;
    uint8_t two_bytes[2] = {RECEIVE_NO_DATA,};
    float   rotation_per_sec[2] = {motor1_state.vel_cur / TWO_PI / this -> wheel_radius, motor2_state.vel_cur / TWO_PI / this -> wheel_radius};
    float   velocity_diff, velocity_linear, velocity_angular = 0.0f;
    // buffer clear
    memset(transmit_message, RECEIVE_NO_DATA, sizeof(uint8_t) * TX_BUFFER_SIZE);
    transmit_index = 0;

    transmit_message[transmit_index++] = static_cast<uint8_t>(ParameterID::pid_monitoring); // monitoring mode pid
    transmit_message[transmit_index++] = static_cast<uint8_t>(monitoring_unit);
    if(motor1_state.vel_cur > FLOAT32_ZERO)
    {
        dir |= MOTOR1_FORWARD;
    }
    if(motor2_state.vel_cur > FLOAT32_ZERO)
    {
        dir |= MOTOR2_FORWARD;
    }
    transmit_message[transmit_index++] = dir;
    if(monitoring_unit == MonitoringUnit::monitoring_mps)
    {
        FloatToBytes(&(two_bytes[POS_BYTE_HIGH]), &(two_bytes[POS_BYTE_LOW]),motor1_state.vel_cur, DIGIT_VELOCITY);
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_HIGH];
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_LOW];
        FloatToBytes(&(two_bytes[POS_BYTE_HIGH]), &(two_bytes[POS_BYTE_LOW]), motor2_state.vel_cur, DIGIT_VELOCITY);
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_HIGH];
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_LOW];
    }
    else if(monitoring_unit == MonitoringUnit::monitoring_rpm)
    {
        ConfigurationHelper::FloatToBytes(&(two_bytes[POS_BYTE_HIGH]), &(two_bytes[POS_BYTE_LOW]), rotation_per_sec[0] * 60.0f, DIGIT_RPM);
        // v / (2 pi r) * 60 = (# of rot_per_sec) * 60[s] = RPM
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_HIGH];
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_LOW];
        ConfigurationHelper::FloatToBytes(&(two_bytes[POS_BYTE_HIGH]), &(two_bytes[POS_BYTE_LOW]), rotation_per_sec[1] * 60.0f, DIGIT_RPM);
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_HIGH];
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_LOW];
    }
    else if(monitoring_unit == MonitoringUnit::monitoring_pps)
    {
        ConfigurationHelper::FloatToBytes(&(two_bytes[POS_BYTE_HIGH]), &(two_bytes[POS_BYTE_LOW]), rotation_per_sec[0] * this -> ppr, DIGIT_PPS);
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_HIGH];
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_LOW];
        ConfigurationHelper::FloatToBytes(&(two_bytes[POS_BYTE_HIGH]), &(two_bytes[POS_BYTE_LOW]), rotation_per_sec[1] * this -> ppr, DIGIT_PPS);
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_HIGH];
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_LOW];
    }
    else if(monitoring_unit == MonitoringUnit::monitoring_complex)
    {
        ConfigurationHelper::FloatToBytes(&(two_bytes[POS_BYTE_HIGH]), &(two_bytes[POS_BYTE_LOW]), motor1_state.position, DIGIT_WHEEL_POSITION);
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_HIGH];
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_LOW];
        ConfigurationHelper::FloatToBytes(&(two_bytes[POS_BYTE_HIGH]), &(two_bytes[POS_BYTE_LOW]), motor2_state.position, DIGIT_WHEEL_POSITION);
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_HIGH];
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_LOW];
        velocity_linear  = fabs(motor1_state.vel_cur + motor2_state.vel_cur) / 2.0f;
        if(wheel_seperation != 0.0f)
        {
            velocity_diff = motor2_state.vel_cur - motor1_state.vel_cur;
            velocity_angular = fabs(velocity_diff) / wheel_seperation; // w = (v_r - v_l) / d
        }
        ConfigurationHelper::FloatToBytes(&(two_bytes[POS_BYTE_HIGH]), &(two_bytes[POS_BYTE_LOW]), velocity_linear, DIGIT_VEL_LINEAR);
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_HIGH];
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_LOW];
        if(velocity_diff >= 0.0f)
        {
            transmit_message[transmit_index++] = ROTATION_FORWARD;
        }
        else
        {
            transmit_message[transmit_index++] = ROTATION_BACKWARD;
        }
        ConfigurationHelper::FloatToBytes(&(two_bytes[POS_BYTE_HIGH]), &(two_bytes[POS_BYTE_LOW]), velocity_angular, DIGIT_VEL_ANGULAR);
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_HIGH];
        transmit_message[transmit_index++] = two_bytes[POS_BYTE_LOW];
    }
}

bool SerialHelper::SetVelocity()
{
    float vel1, vel2;
    int dir1 = MOTOR_BACKWARD;
    int dir2 = MOTOR_BACKWARD;
    if(receive_index != LENGTH_SET_VELOCITY)
    {
        return false;
    }
    if((receive_message[POS_DIR] & MOTOR1_FORWARD))
    {
        dir1 = MOTOR_FORWARD;
    }
    if((receive_message[POS_DIR] & MOTOR2_FORWARD))
    {
        dir2 = MOTOR_FORWARD;
    }
    motor1_state.vel_cmd = BytesToFloat(receive_message[POS_MOT1_VEL_H], receive_message[POS_MOT1_VEL_L], DIGIT_VELOCITY) * float(dir1);
    motor2_state.vel_cmd = BytesToFloat(receive_message[POS_MOT2_VEL_H], receive_message[POS_MOT2_VEL_L], DIGIT_VELOCITY) * float(dir2);
    return true;
}

bool SerialHelper::BrakeMotor()
{
    motor1_state.vel_cmd  = 0.0f;
    motor2_state.vel_cmd  = 0.0f;
    motor1_state.runnable = false;
    motor2_state.runnable = false;
    return true;
}

bool SerialHelper::ReleaseMotor()
{
    motor1_state.runnable = true;
    motor2_state.runnable = true;
    return true;
}

void SerialHelper::ReturnData()
{
    memset(transmit_message, RECEIVE_NO_DATA, TX_BUFFER_SIZE);
    transmit_index = 0;
    transmit_message[transmit_index++] = RETURN_CODE;
    for(int i = 0; i < receive_index; i++)
    {
        transmit_message[transmit_index++] = receive_message[i];
    }
}

void SerialHelper::ReturnError()
{
    uint8_t error_message[6];
    const uint8_t temp_error_contents = 0x00;
    error_message[0] = ERROR_CODE;
    error_message[1] = receive_message[0];
    error_message[2] = temp_error_contents;
    error_message[3] = temp_error_contents;
    error_message[4] = temp_error_contents;
    error_message[5] = temp_error_contents;
    memset(transmit_message, RECEIVE_NO_DATA, TX_BUFFER_SIZE);
    memcpy(transmit_message, error_message, sizeof(uint8_t) * 6);
    transmit_index = 6;
}

////////////////////////////////////////////
// utilities here
////////////////////////////////////////////
void SerialHelper::FlushReceiveMessage()
{
    memset(receive_message, RECEIVE_NO_DATA, RX_BUFFER_SIZE);
    receive_index = 0;
}

void SerialHelper::SetMessage(uint8_t msg[], uint8_t length)
{
    memcpy(receive_message, msg, sizeof(uint8_t) * length);
    receive_index = length;
    command_receive = true;
}

void SerialHelper::GetMessage(uint8_t dest[], uint32_t* length)
{
    memcpy(dest, transmit_message, sizeof(uint8_t) * transmit_index);
    *length = transmit_index;
}

bool SerialHelper::IsBrake()
{
    if(!motor1_state.runnable || !motor2_state.runnable)
    {
        return true;
    }
    return false;
}

uint8_t SerialHelper::Checksum(uint8_t data[], uint8_t length)
{
    uint16_t sum = 0;
    
    for(int i = 0; i < length; i++)
    {
        sum += data[i];
    }
    return sum & 0xFF;
}
/* serial_helper.cpp */