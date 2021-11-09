#include "serial_helper.h"

using namespace zeta_motor_driver;

extern ros::NodeHandle nh;

void SerialHelper::Begin()
{
}

void SerialHelper::ReceiveData()
{
}

void SerialHelper::ExecuteCommand()
{
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
    }
    FlushReceiveMessage();
}

bool SerialHelper::Run(uint8_t pid)
{
    switch(static_cast<ParameterID>(pid))
    {
        case ParameterID::pid_monitoring:
            SetMonitoringUnit();
            ReturnData();
            break;
        case ParameterID::pid_run_motor:
            SetVelocity();
            ReturnData();
            break;
        case ParameterID::pid_brake_motor:
            //BrakeMotor();
            ReturnData();
            break;
        case ParameterID::pid_release_motor:
            //ReleaseMotor();
            ReturnData();
            break;
        case ParameterID::pid_set_p_gain:
            //SetPGain();
            ReturnData();
            break;
        case ParameterID::pid_set_i_gain:
            //SetIGain();
            ReturnData();
            break;
        case ParameterID::pid_set_d_gain:
            //SetDGain();
            ReturnData();
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
            //SetDecreasingTime();
            ReturnData();
            break;
        default:
            com_error = ComError::error_unknown_pid;
            return false;
    }
    return true;
}
////////////////////////////////////////////
// Actual execution codes here
////////////////////////////////////////////
void SerialHelper::SetMonitoringUnit()
{
    if(receive_message[POS_MONITORING_UNIT] < static_cast<uint8_t>(MonitoringUnit::monitoring_last))
    {
        monitoring_unit = static_cast<MonitoringUnit>(receive_message[POS_MONITORING_UNIT]);
    }
}

void SerialHelper::TransmitVelocity()
{
    uint8_t transmit_message[TX_BUFFER_SIZE] = {RECEIVE_NO_DATA,};
    int     transmit_index = 0;
    uint8_t dir = 0;
    uint8_t vel_byte[2] = {RECEIVE_NO_DATA,};
    if(monitoring_unit == MonitoringUnit::monitoring_mps)
    {
        transmit_message[transmit_index++] = static_cast<uint8_t>(ParameterID::pid_monitoring); // monotoring mode pid
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
        ConfigurationHelper::FloatToBytes(&(vel_byte[POS_VEL_H]), &(vel_byte[POS_VEL_L]),motor1_state.vel_cur, DIGIT_VELOCITY);
        transmit_message[transmit_index++] = vel_byte[POS_VEL_H];
        transmit_message[transmit_index++] = vel_byte[POS_VEL_L];
        ConfigurationHelper::FloatToBytes(&(vel_byte[POS_VEL_H]), &(vel_byte[POS_VEL_L]), motor2_state.vel_cur, DIGIT_VELOCITY);
        transmit_message[transmit_index++] = vel_byte[POS_VEL_H];
        transmit_message[transmit_index++] = vel_byte[POS_VEL_L];
        memcpy(serial_output_msg.data, transmit_message, sizeof(uint8_t) * transmit_index);
        serial_output_msg.data_length = transmit_index;
        data_publisher -> publish(&serial_output_msg);
    }
    else if(monitoring_unit == MonitoringUnit::monitoring_rpm)
    {
        transmit_message[transmit_index++] = static_cast<uint8_t>(ParameterID::pid_monitoring); // monotoring mode pid
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
        ConfigurationHelper::FloatToBytes(&(vel_byte[POS_VEL_H]), &(vel_byte[POS_VEL_L]), motor1_state.vel_cur / TWO_PI / this -> wheel_radius * 60.0f, DIGIT_RPM);
        transmit_message[transmit_index++] = vel_byte[POS_VEL_H];
        transmit_message[transmit_index++] = vel_byte[POS_VEL_L];
        ConfigurationHelper::FloatToBytes(&(vel_byte[POS_VEL_H]), &(vel_byte[POS_VEL_L]), motor2_state.vel_cur / TWO_PI / this -> wheel_radius * 60.0f, DIGIT_RPM);
        transmit_message[transmit_index++] = vel_byte[POS_VEL_H];
        transmit_message[transmit_index++] = vel_byte[POS_VEL_L];
        memcpy(serial_output_msg.data, transmit_message, sizeof(uint8_t) * transmit_index);
        serial_output_msg.data_length = transmit_index;
        data_publisher -> publish(&serial_output_msg);
    }
    
}

void SerialHelper::SetVelocity()
{
    float vel1, vel2;
    int dir1 = MOTOR_BACKWARD;
    int dir2 = MOTOR_BACKWARD;
    if((receive_message[POS_DIR] & MOTOR1_FORWARD))
    {
        dir1 = MOTOR_FORWARD;
    }
    if((receive_message[POS_DIR] & MOTOR2_FORWARD))
    {
        dir2 = MOTOR_FORWARD;
    }
    motor1_state.vel_cmd = ConfigurationHelper::BytesToFloat(receive_message[POS_MOT1_VEL_H], receive_message[POS_MOT1_VEL_L], DIGIT_VELOCITY) * float(dir1);
    motor2_state.vel_cmd = ConfigurationHelper::BytesToFloat(receive_message[POS_MOT2_VEL_H], receive_message[POS_MOT2_VEL_L], DIGIT_VELOCITY) * float(dir2);
#ifdef SERIAL_DEBUG
    Serial.print("cmd vel: ");Serial.print(motor1_state.vel_cmd,3);Serial.print(", ");Serial.println(motor2_state.vel_cmd,3);
#endif
}

void SerialHelper::BrakeMotor()
{
}

void SerialHelper::ReturnData()
{
}

////////////////////////////////////////////
// utilities here
////////////////////////////////////////////
void SerialHelper::FlushReceiveMessage()
{
    memset(receive_message, '\0', RX_BUFFER_SIZE);
    message_index = 0;
    receive_message[0] = static_cast<uint8_t>(ParameterID::pid_last);
}

void SerialHelper::SetMessage(uint8_t msg[])
{
    memcpy(receive_message, msg, sizeof(uint8_t) * RX_BUFFER_SIZE);
    command_receive = true;
}

uint8_t SerialHelper::Checksum(uint8_t data[], int length)
{
    uint16_t sum = 0;
    
    for(int i = 0; i < length; i++)
    {
        sum += data[i];
    }
    return sum & 0xFF;
}
/* serial_helper.cpp */