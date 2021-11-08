#include "serial_helper.h"

using namespace zeta_motor_driver;

void SerialHelper::Begin()
{
    stream.begin(serial_speed);
}

void SerialHelper::ReceiveData()
{
    while(COM_PORT.available())
    {
        receive_message[message_index] = COM_PORT.read();
        if(receive_message[message_index] == END_BYTE2)
        {
            message_index++;
            command_receive = true;
            break;
        }
        message_index++;
        /* flush when overflow */
        if(message_index == RX_BUFFER_SIZE)
        {
            memset(receive_message,'\0',RX_BUFFER_SIZE);
            message_index = 0;
            break;
        }
    }
}

<<<<<<< HEAD
void SerialHelper::TransmittData()
{
    uint8_t transmitt_message[TX_BUFFER_SIZE] = {0x00,};
    int     transmitt_index = 0;
    transmitt_message[transmitt_index++] = START_BYTE1;
    transmitt_message[transmitt_index++] = START_BYTE2;
    if(monitoring)
    {   
        uint8_t dir = 0;
        uint8_t vel_byte[2] = {0x00,};
        transmitt_message[transmitt_index++] = LENGTH_MONITORING;
        transmitt_message[transmitt_index++] = 0x00; // monotoring mode pid
        transmitt_message[transmitt_index++] = static_cast<uint8_t>(monitoring_mode);
        if(motor1_state.vel_cur > 0.0)
        {
            dir |= 0b01;
        }
        if(motor2_state.vel_cur > 0.0)
        {
            dir |= 0b10;
        }
        transmitt_message[transmitt_index++] = dir;
        FloatToBytes(&(vel_byte[1]),&(vel_byte[0]),motor1_state.vel_cur,3);
        transmitt_message[transmitt_index++] = vel_byte[1];
        transmitt_message[transmitt_index++] = vel_byte[0];
        FloatToBytes(&(vel_byte[1]),&(vel_byte[0]),motor2_state.vel_cur,3);
        transmitt_message[transmitt_index++] = vel_byte[1];
        transmitt_message[transmitt_index++] = vel_byte[0];
        transmitt_message[transmitt_index] = Checksum(&(transmitt_message[POS_LENGTH]), transmitt_index);
        transmitt_index++;
    }
    transmitt_message[transmitt_index++] = END_BYTE1;
    transmitt_message[transmitt_index++] = END_BYTE2;
    stream.write(transmitt_message,transmitt_index);
}
=======

>>>>>>> nightly

void SerialHelper::ExecuteCommand()
{
    if(command_receive)
    {
        if(VerifyFormat() && VerifyLength() && VerifyChecksum())
        {
            noInterrupts(); // lock rx buffer for avoiding wrong behavior
            uint8_t pid = receive_message[POS_PID];
            Run(pid);
        }
        else
        {
            com_error = ComError::no_error;
        }
        command_receive = false;
        interrupts();
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

void SerialHelper::TransmittVelocity()
{
    uint8_t transmitt_message[TX_BUFFER_SIZE] = {RECEIVE_NO_DATA,};
    int     transmitt_index = 0;
    uint8_t dir = 0;
    uint8_t vel_byte[2] = {RECEIVE_NO_DATA,};
    if(monitoring_unit == MonitoringUnit::monitoring_mps)
    {
        transmitt_message[transmitt_index++] = START_BYTE1;
        transmitt_message[transmitt_index++] = START_BYTE2;
        transmitt_message[transmitt_index++] = LENGTH_MONITORING;
        transmitt_message[transmitt_index++] = static_cast<uint8_t>(ParameterID::pid_monitoring); // monotoring mode pid
        transmitt_message[transmitt_index++] = static_cast<uint8_t>(monitoring_unit);
        if(motor1_state.vel_cur > FLOAT32_ZERO)
        {
            dir |= MOTOR1_FORWARD;
        }
        if(motor2_state.vel_cur > FLOAT32_ZERO)
        {
            dir |= MOTOR2_FORWARD;
        }
        transmitt_message[transmitt_index++] = dir;
        ConfigurationHelper::FloatToBytes(&(vel_byte[POS_VEL_H]), &(vel_byte[POS_VEL_L]),motor1_state.vel_cur, DIGIT_VELOCITY);
        transmitt_message[transmitt_index++] = vel_byte[POS_VEL_H];
        transmitt_message[transmitt_index++] = vel_byte[POS_VEL_L];
        ConfigurationHelper::FloatToBytes(&(vel_byte[POS_VEL_H]), &(vel_byte[POS_VEL_L]), motor2_state.vel_cur, DIGIT_VELOCITY);
        transmitt_message[transmitt_index++] = vel_byte[POS_VEL_H];
        transmitt_message[transmitt_index++] = vel_byte[POS_VEL_L];
        transmitt_message[transmitt_index] = Checksum(&(transmitt_message[POS_LENGTH]), transmitt_index);
        transmitt_index++;
        transmitt_message[transmitt_index++] = END_BYTE1;
        transmitt_message[transmitt_index++] = END_BYTE2;
        stream.write(transmitt_message,transmitt_index);
    }
    else if(monitoring_unit == MonitoringUnit::monitoring_rpm)
    {
        transmitt_message[transmitt_index++] = START_BYTE1;
        transmitt_message[transmitt_index++] = START_BYTE2;
        transmitt_message[transmitt_index++] = LENGTH_MONITORING;
        transmitt_message[transmitt_index++] = static_cast<uint8_t>(ParameterID::pid_monitoring); // monotoring mode pid
        transmitt_message[transmitt_index++] = static_cast<uint8_t>(monitoring_unit);
        if(motor1_state.vel_cur > FLOAT32_ZERO)
        {
            dir |= MOTOR1_FORWARD;
        }
        if(motor2_state.vel_cur > FLOAT32_ZERO)
        {
            dir |= MOTOR2_FORWARD;
        }
        transmitt_message[transmitt_index++] = dir;
        ConfigurationHelper::FloatToBytes(&(vel_byte[POS_VEL_H]), &(vel_byte[POS_VEL_L]), motor1_state.vel_cur / TWO_PI / this -> wheel_radius * 60.0f, DIGIT_RPM);
        transmitt_message[transmitt_index++] = vel_byte[POS_VEL_H];
        transmitt_message[transmitt_index++] = vel_byte[POS_VEL_L];
        ConfigurationHelper::FloatToBytes(&(vel_byte[POS_VEL_H]), &(vel_byte[POS_VEL_L]), motor2_state.vel_cur / TWO_PI / this -> wheel_radius * 60.0f, DIGIT_RPM);
        transmitt_message[transmitt_index++] = vel_byte[POS_VEL_H];
        transmitt_message[transmitt_index++] = vel_byte[POS_VEL_L];
        transmitt_message[transmitt_index] = Checksum(&(transmitt_message[POS_LENGTH]), transmitt_index);
        transmitt_index++;
        transmitt_message[transmitt_index++] = END_BYTE1;
        transmitt_message[transmitt_index++] = END_BYTE2;
        stream.write(transmitt_message, transmitt_index);
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
    uint8_t transmitt_message[TX_BUFFER_SIZE] = {0,};
    int     transmitt_index = 0;
    transmitt_message[transmitt_index++] = START_BYTE1;
    transmitt_message[transmitt_index++] = START_BYTE2;
    transmitt_message[transmitt_index++] = receive_message[POS_LENGTH];
    transmitt_message[transmitt_index++] = RETURN_CODE;
    for(int i = POS_PID; i < receive_message[POS_LENGTH] + POS_PID; i++)
    {
        transmitt_message[transmitt_index++] = receive_message[i];
    }
    transmitt_message[transmitt_index] = Checksum(&(transmitt_message[POS_LENGTH]),transmitt_index);
    transmitt_index++;
    transmitt_message[transmitt_index++] = END_BYTE1;
    transmitt_message[transmitt_index++] = END_BYTE2;
    stream.write(transmitt_message,transmitt_index);
}

////////////////////////////////////////////
// utilities here
////////////////////////////////////////////
void SerialHelper::FlushReceiveMessage()
{
    memset(receive_message, '\0', RX_BUFFER_SIZE);
    message_index = 0;
}

bool SerialHelper::VerifyFormat()
{
    if((receive_message[POS_START_BYTE1] != START_BYTE1) || (receive_message[POS_START_BYTE2] != START_BYTE2) ||
        (receive_message[message_index - 2] != END_BYTE1) || (receive_message[message_index - 1] != END_BYTE2))
    {
#ifdef SERIAL_DEBUG
        Serial.println("[error] Mismatch format.");
#endif
        com_error = ComError::error_mismatch_format;
        return false;
    }
    return true;
}

bool SerialHelper::VerifyLength()
{
    if(receive_message[POS_LENGTH] != message_index - (POS_LENGTH + POS_CHECKSUM + 1))
    {
        com_error = ComError::error_mismatch_length;
#ifdef SERIAL_DEBUG
        Serial.print("[error] Mismatch length. Expected length is ");Serial.print(receive_message[POS_LENGTH],HEX);
        Serial.print(" receive one is ");Serial.println(message_index - (POS_LENGTH + POS_CHECKSUM + 1),HEX);
#endif
        return false;
    }
    return true;
}

bool SerialHelper::VerifyChecksum()
{
    if(Checksum(&(receive_message[POS_LENGTH]),message_index - (POS_LENGTH + POS_CHECKSUM)) != receive_message[message_index - POS_CHECKSUM])
    {
#ifdef SERIAL_DEBUG
        Serial.print("[error] Mismatch checksum. Expected sum is ");Serial.print(Checksum(&(receive_message[POS_LENGTH]),message_index - (POS_LENGTH + POS_CHECKSUM)),HEX);
        Serial.print(" receive one is ");Serial.println(receive_message[message_index - POS_CHECKSUM],HEX);
#endif
        com_error = ComError::error_mismatch_checksum;
        return false;
    }
    return true;
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