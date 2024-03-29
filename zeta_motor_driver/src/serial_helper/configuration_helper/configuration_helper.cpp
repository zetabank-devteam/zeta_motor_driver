#include "configuration_helper.h"
using namespace zeta_motor_driver;

void ConfigurationHelper::Update()
{
    uint8_t baudrate_preset = EEPROM.read(BASE_ADDRESS_BAUDRATE);
    switch(static_cast<BaudratePreset>(baudrate_preset))
    {
        case BaudratePreset::baudrate_9600:
            baudrate = 9600L;
            break;
        case BaudratePreset::baudrate_19200:
            baudrate = 19200L;
            break;
        case BaudratePreset::baudrate_38400:
            baudrate = 38400L;
            break;
        case BaudratePreset::baudrate_57600:
            baudrate = 57600L;
            break;
        case BaudratePreset::baudrate_115200:
            baudrate = 115200L;
            break;
        default:
            baudrate = 115200L;
            break;
    }
    version[0]      = EEPROM.read(BASE_ADDRESS_FW_VERSION);
    version[1]      = EEPROM.read(BASE_ADDRESS_FW_VERSION + 0x001);
    last_error[0]   = EEPROM.read(BASE_ADDRESS_LAST_ERROR);
    last_error[1]   = EEPROM.read(BASE_ADDRESS_LAST_ERROR + 0x001);
    p_gain          = (EEPROM.read(BASE_ADDRESS_P_GAIN + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_P_GAIN)) / 10.0f;
    i_gain          = (EEPROM.read(BASE_ADDRESS_I_GAIN + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_I_GAIN)) / 10.0f;
    d_gain          = (EEPROM.read(BASE_ADDRESS_D_GAIN + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_D_GAIN)) / 10.0f;
    max_speed       = (EEPROM.read(BASE_ADDRESS_MAX_SPEED + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_MAX_SPEED)) / 1000.0f;
    min_speed       = (EEPROM.read(BASE_ADDRESS_MIN_SPEED + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_MIN_SPEED)) / 1000.0f;
    ppr             = (EEPROM.read(BASE_ADDRESS_PPR + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_PPR)) / 10.0f;
    wheel_radius    = (EEPROM.read(BASE_ADDRESS_WHEEL_RADIUS + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_WHEEL_RADIUS))/1000.0f;
    wheel_seperation    = (EEPROM.read(BASE_ADDRESS_WHEEL_SEPERATION + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_WHEEL_SEPERATION))/1000.0f;
    increasing_time = (EEPROM.read(BASE_ADDRESS_INCREASING_TIME + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_INCREASING_TIME));
    decreasing_time = (EEPROM.read(BASE_ADDRESS_DECREASING_TIME + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_DECREASING_TIME));
}

void ConfigurationHelper::SetVersion(uint8_t src[])
{
    SetEEPROM(BASE_ADDRESS_FW_VERSION, src);
    memcpy(version, src, sizeof(uint8_t) * 2);
}

void ConfigurationHelper::SetBaudrate(int32_t baudrate_)
{
    switch(baudrate_)
    {
        case 9600L:
            SetEEPROM(BASE_ADDRESS_BAUDRATE, static_cast<uint8_t>(BaudratePreset::baudrate_9600));
            break;
        case 19200L:
            SetEEPROM(BASE_ADDRESS_BAUDRATE, static_cast<uint8_t>(BaudratePreset::baudrate_19200));
            break;
        case 38400L:
            SetEEPROM(BASE_ADDRESS_BAUDRATE, static_cast<uint8_t>(BaudratePreset::baudrate_38400));
            break;
        case 57600L:
            SetEEPROM(BASE_ADDRESS_BAUDRATE, static_cast<uint8_t>(BaudratePreset::baudrate_57600));
            break;
        case 115200L:
            SetEEPROM(BASE_ADDRESS_BAUDRATE, static_cast<uint8_t>(BaudratePreset::baudrate_115200));
            break;
    }
}

void ConfigurationHelper::SetError(uint8_t src[])
{
    SetEEPROM(BASE_ADDRESS_LAST_ERROR, src);
    memcpy(last_error, src, sizeof(uint8_t) * 2);
}

bool ConfigurationHelper::SetPGain(float gain)
{
    SetEEPROM(BASE_ADDRESS_P_GAIN, gain, 1);
    p_gain = (EEPROM.read(BASE_ADDRESS_P_GAIN + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_P_GAIN)) / 10.0f;
    return configurable;
}

bool ConfigurationHelper::SetIGain(float gain)
{
    SetEEPROM(BASE_ADDRESS_I_GAIN, gain, 1);
    i_gain = (EEPROM.read(BASE_ADDRESS_I_GAIN + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_I_GAIN)) / 10.0f;
    return configurable;
}

bool ConfigurationHelper::SetDGain(float gain)
{
    SetEEPROM(BASE_ADDRESS_D_GAIN, gain, 1);
    d_gain = (EEPROM.read(BASE_ADDRESS_D_GAIN + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_D_GAIN)) / 10.0f;
    return configurable;
}

void ConfigurationHelper::SetMaxSpeed(float speed)
{
    SetEEPROM(BASE_ADDRESS_MAX_SPEED, speed, 3);
    max_speed = (EEPROM.read(BASE_ADDRESS_MAX_SPEED + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_MAX_SPEED)) / 1000.0f;
}

void ConfigurationHelper::SetMinSpeed(float speed)
{
    SetEEPROM(BASE_ADDRESS_MIN_SPEED, speed, 3);
    min_speed = (EEPROM.read(BASE_ADDRESS_MIN_SPEED + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_MIN_SPEED)) / 1000.0f;
}

void ConfigurationHelper::SetPPR(float ppr_)
{
    SetEEPROM(BASE_ADDRESS_PPR, ppr_, 1);
    ppr = (EEPROM.read(BASE_ADDRESS_PPR + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_PPR)) / 10.0f;
}

void ConfigurationHelper::SetIncreasingTime(uint16_t time)
{
    SetEEPROM(BASE_ADDRESS_INCREASING_TIME, time);
    increasing_time = (EEPROM.read(BASE_ADDRESS_INCREASING_TIME + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_INCREASING_TIME));
}

void ConfigurationHelper::SetDecreasingTime(uint16_t time)
{
    SetEEPROM(BASE_ADDRESS_DECREASING_TIME, time);
    decreasing_time = (EEPROM.read(BASE_ADDRESS_DECREASING_TIME + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_DECREASING_TIME));
}

void ConfigurationHelper::SetWheelRadius(float wheel_radius_)
{
    SetEEPROM(BASE_ADDRESS_WHEEL_RADIUS, wheel_radius_, 3);
    wheel_radius = (EEPROM.read(BASE_ADDRESS_WHEEL_RADIUS + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_WHEEL_RADIUS)) / 1000.0f;
}

void ConfigurationHelper::SetWheelSeperation(float wheel_seperation_)
{
    SetEEPROM(BASE_ADDRESS_WHEEL_SEPERATION, wheel_seperation_, 3);
    wheel_seperation = (EEPROM.read(BASE_ADDRESS_WHEEL_SEPERATION + 0x001) * 256 + EEPROM.read(BASE_ADDRESS_WHEEL_SEPERATION)) / 1000.0f;
}

void ConfigurationHelper::SetEEPROM(uint16_t base_address, float val, int digit)
{
    uint16_t val_new;
    uint8_t  two_bytes[2];
    if(digit == 1)
    {
        val_new = uint16_t(val * 10.0f);
    }
    else if(digit == 3)
    {
        val_new = uint16_t(val * 1000.0f);
    }
    UInt16ToBytes(&(two_bytes[1]), &(two_bytes[0]), val_new);
    if(configurable)
    {
        EEPROM.update(base_address,         two_bytes[0]);
        EEPROM.update(base_address + 0x001, two_bytes[1]);
    }
}

void ConfigurationHelper::SetEEPROM(uint16_t base_address, uint16_t val)
{
    uint8_t  two_bytes[2];
    UInt16ToBytes(&(two_bytes[1]), &(two_bytes[0]), val);
    if(configurable)
    {
        EEPROM.update(base_address,         two_bytes[0]);
        EEPROM.update(base_address + 0x001, two_bytes[1]);
    }
}

void ConfigurationHelper::SetEEPROM(uint16_t base_address, uint8_t val[])
{
    if(configurable)
    {
        EEPROM.update(base_address,         val[0]);
        EEPROM.update(base_address + 0x001, val[1]);
    }
}

void ConfigurationHelper::SetEEPROM(uint16_t base_address, uint8_t val)
{
    if(configurable)
    {
        EEPROM.update(base_address, val);
    }
}

void ConfigurationHelper::SetConfigurable(bool configurable_)
{
    configurable = configurable_;
}

void ConfigurationHelper::FloatToBytes(uint8_t* byte_h, uint8_t* byte_l, float src, int digit)
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

float ConfigurationHelper::BytesToFloat(uint8_t byte_h, uint8_t byte_l, int digit)
{
    uint16_t val = (byte_h << 8) | byte_l;
    if(digit == 1)
    {
        return float(val) / 10.0f;
    }
    else if(digit == 3)
    {
        return float(val) / 1000.0f;
    }
    return 0;
}

uint16_t ConfigurationHelper::BytesToUInt16(uint8_t byte_h, uint8_t byte_l)
{
    uint16_t val = (byte_h << 8) | byte_l;
    return val;
}

void ConfigurationHelper::UInt16ToBytes(uint8_t* byte_h, uint8_t* byte_l, uint16_t src)
{
    *byte_l = src & 0x00ff;
    *byte_h = (src & 0xff00) >> 8;
}

void ConfigurationHelper::SetFactoryValue()
{
    uint8_t two_bytes[2] = {0,0};
    SetConfigurable(true);
    SetEEPROM(BASE_ADDRESS_FW_VERSION, two_bytes);
    SetError(two_bytes);
    SetBaudrate(115200); // UBRR?
    SetPGain(1000.0f);
    SetIGain(100.0f);
    SetDGain(0.0f);
    SetMaxSpeed(0.320f);
    SetMinSpeed(0.016f);
    SetPPR(2420.0f);
    SetWheelRadius(0.035f);
    // SetWheelSeperation(0.133f);
    SetWheelSeperation(0.158f);
    // SetWheelSeperation(0.230f);
    SetIncreasingTime(100);
    SetDecreasingTime(100);
    SetEEPROM(EEPROM_SET_FACTORY_VALUE_ADDRESS, uint8_t(FACTORY_SET_STATE_TRUE));
}
/* configuration_helper.cpp */