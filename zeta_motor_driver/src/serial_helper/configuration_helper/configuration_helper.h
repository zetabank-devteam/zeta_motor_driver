#ifndef ZETA_MOTOR_DRIVER_CONFIGURATION_HELPER_H_
#define ZETA_MOTOR_DRIVER_CONFIGURATION_HELPER_H_

#include <Arduino.h>
#include <EEPROM.h>

#define MAX_CONFIGURATION_NUM                 20
#define SIZE_OF_CONFIGURATION                 0x004
#define EEPROM_CONFIGURATION_OFFSET           (SIZE_OF_CONFIGURATION * MAX_CONFIGURATION_NUM)

#define ADDRESS_OFFSET(BASE)                  (BASE + SIZE_OF_CONFIGURATION)

#define EEPROM_BASE_ADDRESS                   0x000
#define BASE_ADDRESS_FW_VERSION               (EEPROM_BASE_ADDRESS)
#define BASE_ADDRESS_BAUDRATE                 ADDRESS_OFFSET(BASE_ADDRESS_FW_VERSION)
#define BASE_ADDRESS_LAST_ERROR               ADDRESS_OFFSET(BASE_ADDRESS_BAUDRATE)
#define BASE_ADDRESS_P_GAIN                   ADDRESS_OFFSET(BASE_ADDRESS_LAST_ERROR)
#define BASE_ADDRESS_I_GAIN                   ADDRESS_OFFSET(BASE_ADDRESS_P_GAIN)
#define BASE_ADDRESS_D_GAIN                   ADDRESS_OFFSET(BASE_ADDRESS_I_GAIN)
#define BASE_ADDRESS_MAX_SPEED                ADDRESS_OFFSET(BASE_ADDRESS_D_GAIN)
#define BASE_ADDRESS_MIN_SPEED                ADDRESS_OFFSET(BASE_ADDRESS_MAX_SPEED)
#define BASE_ADDRESS_PPR                      ADDRESS_OFFSET(BASE_ADDRESS_MIN_SPEED)
#define BASE_ADDRESS_WHEEL_RADIUS             ADDRESS_OFFSET(BASE_ADDRESS_PPR)
#define BASE_ADDRESS_WHEEL_SEPERATION         ADDRESS_OFFSET(BASE_ADDRESS_WHEEL_RADIUS)
#define BASE_ADDRESS_INCREASING_TIME          ADDRESS_OFFSET(BASE_ADDRESS_WHEEL_SEPERATION)
#define BASE_ADDRESS_DECREASING_TIME          ADDRESS_OFFSET(BASE_ADDRESS_INCREASING_TIME)

#define EEPROM_FACTORY_BASE_ADDRESS             (EEPROM_BASE_ADDRESS + EEPROM_CONFIGURATION_OFFSET)
#define EEPROM_SET_FACTORY_VALUE_ADDRESS        (EEPROM_FACTORY_BASE_ADDRESS)
#define BASE_ADDRESS_BAUDRATE_FACTORY           ADDRESS_OFFSET(EEPROM_SET_FACTORY_VALUE_ADDRESS)
#define BASE_ADDRESS_P_GAIN_FACTORY             ADDRESS_OFFSET(BASE_ADDRESS_BAUDRATE_FACTORY)
#define BASE_ADDRESS_I_GAIN_FACTORY             ADDRESS_OFFSET(BASE_ADDRESS_P_GAIN_FACTORY)
#define BASE_ADDRESS_D_GAIN_FACTORY             ADDRESS_OFFSET(BASE_ADDRESS_I_GAIN_FACTORY)
#define BASE_ADDRESS_MAX_SPEED_FACTORY          ADDRESS_OFFSET(BASE_ADDRESS_D_GAIN_FACTORY)
#define BASE_ADDRESS_MIN_SPEED_FACTORY          ADDRESS_OFFSET(BASE_ADDRESS_MAX_SPEED_FACTORY)
#define BASE_ADDRESS_PPR_FACTORY                ADDRESS_OFFSET(BASE_ADDRESS_MIN_SPEED_FACTORY)
#define BASE_ADDRESS_WHEEL_RADIUS_FACTORY       ADDRESS_OFFSET(BASE_ADDRESS_PPR_FACTORY)
#define BASE_ADDRESS_WHEEL_SEPERATION_FACTORY   ADDRESS_OFFSET(BASE_ADDRESS_WHEEL_RADIUS_FACTORY)
#define BASE_ADDRESS_INCREASING_TIME_FACTORY    ADDRESS_OFFSET(BASE_ADDRESS_WHEEL_SEPERATION_FACTORY)
#define BASE_ADDRESS_DECREASING_TIME_FACTORY    ADDRESS_OFFSET(BASE_ADDRESS_INCREASING_TIME_FACTORY)

#define FACTORY_SET_STATE_TRUE                1U
#define FLOAT_PRECISION_1DIGIT                1
#define FLOAT_PRECISION_3DIGIT                3

namespace zeta_motor_driver
{
class ConfigurationHelper
{
    enum class BaudratePreset : uint8_t
    {
        baudrate_9600,
        baudrate_19200,
        baudrate_38400,
        baudrate_57600,
        baudrate_115200,
    };
    public:
        ConfigurationHelper()
        {
            // if(EEPROM.read(EEPROM_SET_FACTORY_VALUE_ADDRESS) != FACTORY_SET_STATE_TRUE) // default 255
            {
                SetFactoryValue();
            }
            Update();
            configurable = false;
        }
        void     GetVersion(uint8_t dest[]) { memcpy(dest, version,    sizeof(uint8_t) * 2); }
        void     GetError(uint8_t dest[])   { memcpy(dest, last_error, sizeof(uint8_t) * 2); }
        int32_t  GetBaudrate()              { return baudrate;}
        float    GetPGain()                 { return p_gain;}
        float    GetIGain()                 { return i_gain;}
        float    GetDGain()                 { return d_gain;}
        float    GetMaxSpeed()              { return max_speed;}
        float    GetMinSpeed()              { return min_speed;}
        float    GetPPR()                   { return ppr;}
        float    GetWheelRadius()           { return wheel_radius;}
        float    GetWheelSeperation()       { return wheel_seperation;}
        uint16_t GetIncreasingTime()        { return increasing_time;}
        uint16_t GetDecreasingTime()        { return decreasing_time;}
        void     SetConfigurable(bool);
    private:
        int32_t  baudrate;
        uint8_t  version[2];
        uint8_t  last_error[2];
        float    p_gain;
        float    i_gain;
        float    d_gain;
        float    max_speed;
        float    min_speed;
        float    ppr;
        float    wheel_radius;
        float    wheel_seperation;
        uint16_t increasing_time;
        uint16_t decreasing_time;
        bool     configurable;
        void     SetFactoryValue();
        void     SetEEPROM(uint16_t,float,int);
        void     SetEEPROM(uint16_t,uint16_t);
        void     SetEEPROM(uint16_t,uint8_t[]);
        void     SetEEPROM(uint16_t,uint8_t);
    protected:
        void     SetVersion(uint8_t[]);
        void     SetError(uint8_t[]);
        void     SetBaudrate(int32_t);
        bool     SetPGain(float);
        bool     SetIGain(float);
        bool     SetDGain(float);
        void     SetMaxSpeed(float);
        void     SetMinSpeed(float);
        void     SetPPR(float);
        void     SetWheelRadius(float);
        void     SetWheelSeperation(float);
        void     SetIncreasingTime(uint16_t);
        void     SetDecreasingTime(uint16_t);
        void     Update();
        float    BytesToFloat(uint8_t,uint8_t,int);
        uint16_t BytesToUInt16(uint8_t,uint8_t);
        void     FloatToBytes(uint8_t*,uint8_t*,float,int);
        void     UInt16ToBytes(uint8_t*,uint8_t*,uint16_t);
}; /* class ConfigurationHelper */
}/* namespace zeta_motor_driver */

#endif /* ZETA_MOTOR_DRIVER_CONFIGURATION_HELPER_H_ */