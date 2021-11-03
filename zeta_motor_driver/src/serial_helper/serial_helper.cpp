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
        if(message_index == RX_BUFFER_SIZE)
        {
            memset(receive_message,'\0',RX_BUFFER_SIZE);
            message_index = 0;
            break;
        }
    }
}

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

void SerialHelper::ExecuteCommand()
{
    uint8_t pid;
    if(command_receive)
    {
        if(VerifyFormat() && VerifyLength() && VerifyChecksum())
        {
            pid = receive_message[POS_PID];
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
        case ParameterID::pid_set_velocity:
            SetVelocity();
            ReturnData();
            break;
        default:
            com_error = ComError::error_unknown_pid;
            return false;
    }
    return true;
}

void SerialHelper::SetVelocity()
{
    float vel1, vel2;
    int dir1 = MOTOR_BACKWARD;
    int dir2 = MOTOR_BACKWARD;
    if((receive_message[4] & 0b1))
    {
        dir1 = MOTOR_FORWARD;
    }
    if((receive_message[4] & (0b1 << 1)))
    {
        dir2 = MOTOR_FORWARD;
    }
    motor1_state.vel_cmd = ByteToFloat(receive_message[7], receive_message[8], 3) * float(dir1);
    motor2_state.vel_cmd = ByteToFloat(receive_message[5], receive_message[6], 3) * float(dir2);
#ifdef SERIAL_DEBUG
    Serial.print("cmd vel: ");Serial.print(motor1_state.vel_cmd,3);Serial.print(", ");Serial.println(motor2_state.vel_cmd,3);
#endif
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

void SerialHelper::FlushReceiveMessage()
{
    memset(receive_message, '\0', RX_BUFFER_SIZE);
    message_index = 0;
}

bool SerialHelper::VerifyFormat()
{
    if((receive_message[0] != START_BYTE1) || (receive_message[1] != START_BYTE2) ||
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

float SerialHelper::ByteToFloat(uint8_t byte_h, uint8_t byte_l, int digit)
{
    if(digit == 1)
    {
        return (byte_h * 256 + byte_l) / 10.0f;
    }
    else if(digit == 3)
    {
        return (byte_h * 256 + byte_l) / 1000.0f;
    }
}

uint16_t SerialHelper::ByteToUInt16(uint8_t byte_h, uint8_t byte_l)
{
    return (byte_h * 256 + byte_l);
}

void SerialHelper::FloatToBytes(uint8_t* byte_h, uint8_t* byte_l, float src, int digit)
{
    uint16_t two_bytes;
    src = fabs(src);
    if(digit == 1)
    {
        two_bytes = uint16_t(src * 10.0f);
    }
    else if(digit == 3)
    {
        two_bytes = uint16_t(src * 1000.0f);
    }
    UInt16ToBytes(byte_h, byte_l, two_bytes);
}

void SerialHelper::UInt16ToBytes(uint8_t* byte_h, uint8_t* byte_l, uint16_t src)
{
    *byte_l = src & 0x00ff;
    *byte_h = (src & 0xff00) >> 8;
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