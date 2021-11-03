#include "configuration_helper.h"
using namespace zeta_motor_driver;

void ConfigurationHelper::Update()
{
    uint8_t baudrate_preset = EEPROM.read(BASE_ADDRESS_BAUDRATE);
    switch(baudrate_preset)
    {
        case 0:
            baudrate = 9600L;
            break;
        case 1:
            baudrate = 19200L;
            break;
        case 2:
            baudrate = 38400L;
            break;
        case 3:
            baudrate = 57600L;
            break;
        case 4:
            baudrate = 115200L;
            break;
        default:
            baudrate = 115200L;
            break;
    }
    version[0] = EEPROM.read(BASE_ADDRESS_FW_VERSION);
    version[1] = EEPROM.read(BASE_ADDRESS_FW_VERSION + 0x001);
    last_error[0] = EEPROM.read(BASE_ADDRESS_LAST_ERROR);
    last_error[1] = EEPROM.read(BASE_ADDRESS_LAST_ERROR + 0x001);
    p_gain = (EEPROM.read(BASE_ADDRESS_P_GAIN + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_P_GAIN)) / 10.0f;
    i_gain = (EEPROM.read(BASE_ADDRESS_I_GAIN + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_I_GAIN)) / 10.0f;
    d_gain = (EEPROM.read(BASE_ADDRESS_D_GAIN + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_D_GAIN)) / 10.0f;
    max_speed = (EEPROM.read(BASE_ADDRESS_MAX_SPEED + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_MAX_SPEED)) / 1000.0f;
    min_speed = (EEPROM.read(BASE_ADDRESS_MIN_SPEED + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_MIN_SPEED)) / 1000.0f;
    ppr = (EEPROM.read(BASE_ADDRESS_PPR + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_PPR)) / 10.0f;
    increasing_time = (EEPROM.read(BASE_ADDRESS_INCREASING_TIME + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_INCREASING_TIME));
    decreasing_time = (EEPROM.read(BASE_ADDRESS_DECREASING_TIME + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_DECREASING_TIME));
}

void ConfigurationHelper::SetVersion(uint8_t src[])
{
    EEPROM.update(BASE_ADDRESS_FW_VERSION,         src[0]);
    EEPROM.update(BASE_ADDRESS_FW_VERSION + 0x001, src[1]);
    memcpy(version, src, sizeof(uint8_t) * 2);
}

void ConfigurationHelper::SetError(uint8_t src[])
{
    EEPROM.update(BASE_ADDRESS_LAST_ERROR,         src[0]);
    EEPROM.update(BASE_ADDRESS_LAST_ERROR + 0x001, src[1]);
    memcpy(last_error, src, sizeof(uint8_t) * 2);
}

void ConfigurationHelper::SetPGain(float gain)
{
    uint16_t gain_new = uint16_t(gain * 10.0f); // 6553.5 * 10
    EEPROM.update(BASE_ADDRESS_P_GAIN,         uint8_t(gain_new & 0x00FF));
    EEPROM.update(BASE_ADDRESS_P_GAIN + 0x001, uint8_t((gain_new & 0xFF00) >> 8));
    p_gain = (EEPROM.read(BASE_ADDRESS_P_GAIN + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_P_GAIN)) / 10.0f;
}

void ConfigurationHelper::SetIGain(float gain)
{
    uint16_t gain_new = uint16_t(gain * 10.0f); // 6553.5 * 10
    EEPROM.update(BASE_ADDRESS_I_GAIN,         uint8_t(gain_new & 0x00FF));
    EEPROM.update(BASE_ADDRESS_I_GAIN + 0x001, uint8_t((gain_new & 0xFF00) >> 8));
    i_gain = (EEPROM.read(BASE_ADDRESS_I_GAIN + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_I_GAIN)) / 10.0f;
}

void ConfigurationHelper::SetDGain(float gain)
{
    uint16_t gain_new = uint16_t(gain * 10.0f); // 6553.5 * 10
    EEPROM.update(BASE_ADDRESS_D_GAIN,         uint8_t(gain_new & 0x00FF));
    EEPROM.update(BASE_ADDRESS_D_GAIN + 0x001, uint8_t((gain_new & 0xFF00) >> 8));
    d_gain = (EEPROM.read(BASE_ADDRESS_D_GAIN + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_D_GAIN)) / 10.0f;
}

void ConfigurationHelper::SetMaxSpeed(float speed)
{
    uint16_t speed_new = uint16_t(speed * 1000.0f); // 65.535 * 1000
    EEPROM.update(BASE_ADDRESS_MAX_SPEED,         uint8_t(speed_new & 0x00FF));
    EEPROM.update(BASE_ADDRESS_MAX_SPEED + 0x001, uint8_t((speed_new & 0xFF00) >> 8));
    max_speed = (EEPROM.read(BASE_ADDRESS_MAX_SPEED + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_MAX_SPEED)) / 1000.0f;
}

void ConfigurationHelper::SetMinSpeed(float speed)
{
    uint16_t speed_new = uint16_t(speed * 1000.0f); // 65.535 * 1000
    EEPROM.update(BASE_ADDRESS_MIN_SPEED,         uint8_t(speed_new & 0x00FF));
    EEPROM.update(BASE_ADDRESS_MIN_SPEED + 0x001, uint8_t((speed_new & 0xFF00) >> 8));
    min_speed = (EEPROM.read(BASE_ADDRESS_MIN_SPEED + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_MIN_SPEED)) / 1000.0f;
}

void ConfigurationHelper::SetPPR(float ppr_)
{
    uint16_t ppr_new = uint16_t(ppr_ * 10.0f); // 6553.5 * 10
    EEPROM.update(BASE_ADDRESS_PPR,         uint8_t(ppr_new & 0x00FF));
    EEPROM.update(BASE_ADDRESS_PPR + 0x001, uint8_t((ppr_new & 0xFF00) >> 8));
    this -> ppr = (EEPROM.read(BASE_ADDRESS_PPR + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_PPR)) / 10.0f;
}

void ConfigurationHelper::SetIncreasingTime(uint16_t time)
{
    EEPROM.update(BASE_ADDRESS_INCREASING_TIME,         uint8_t(time & 0x00FF));
    EEPROM.update(BASE_ADDRESS_INCREASING_TIME + 0x001, uint8_t((time & 0xFF00) >> 8));
    increasing_time = (EEPROM.read(BASE_ADDRESS_INCREASING_TIME + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_INCREASING_TIME));
}

void ConfigurationHelper::SetDecreasingTime(uint16_t time)
{
    EEPROM.update(BASE_ADDRESS_DECREASING_TIME,         uint8_t(time & 0x00FF));
    EEPROM.update(BASE_ADDRESS_DECREASING_TIME + 0x001, uint8_t((time & 0xFF00) >> 8));
    decreasing_time = (EEPROM.read(BASE_ADDRESS_DECREASING_TIME + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_DECREASING_TIME));
}

void ConfigurationHelper::SetWheelRadius(float wheel_radius_)
{
    uint16_t wr_new = uint16_t(wheel_radius_ * 1000.0f); // 65.535 * 1000
    EEPROM.update(BASE_ADDRESS_WHEEL_RADIUS,         uint8_t(wr_new & 0x00FF));
    EEPROM.update(BASE_ADDRESS_WHEEL_RADIUS + 0x001, uint8_t((wr_new & 0xFF00) >> 8));
    wheel_radius = (EEPROM.read(BASE_ADDRESS_WHEEL_RADIUS + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_WHEEL_RADIUS)) / 1000.0f;
}
/* configuration_helper.cpp */