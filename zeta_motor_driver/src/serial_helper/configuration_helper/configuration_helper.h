#ifndef ZETA_MOTOR_DRIVER_CONFIGURATION_HELPER_H_
#define ZETA_MOTOR_DRIVER_CONFIGURATION_HELPER_H_

#include <Arduino.h>
#include <EEPROM.h>

#define EEPROM_OFFSET                   0x004
#define EEPROM_BASE_ADDRESS             0x000
#define ADDRESS_OFFSET(BASE)            (BASE + EEPROM_OFFSET)
#define BASE_ADDRESS_FW_VERSION         (EEPROM_BASE_ADDRESS)
#define BASE_ADDRESS_BAUDRATE           ADDRESS_OFFSET(BASE_ADDRESS_FW_VERSION)
#define BASE_ADDRESS_LAST_ERROR         ADDRESS_OFFSET(BASE_ADDRESS_BAUDRATE)
#define BASE_ADDRESS_P_GAIN             ADDRESS_OFFSET(BASE_ADDRESS_LAST_ERROR)
#define BASE_ADDRESS_I_GAIN             ADDRESS_OFFSET(BASE_ADDRESS_P_GAIN)
#define BASE_ADDRESS_D_GAIN             ADDRESS_OFFSET(BASE_ADDRESS_I_GAIN)
#define BASE_ADDRESS_MAX_SPEED          ADDRESS_OFFSET(BASE_ADDRESS_D_GAIN)
#define BASE_ADDRESS_MIN_SPEED          ADDRESS_OFFSET(BASE_ADDRESS_MAX_SPEED)
#define BASE_ADDRESS_PPR                ADDRESS_OFFSET(BASE_ADDRESS_MIN_SPEED)
#define BASE_ADDRESS_WHEEL_RADIUS       ADDRESS_OFFSET(BASE_ADDRESS_PPR)
#define BASE_ADDRESS_INCREASING_TIME    ADDRESS_OFFSET(BASE_ADDRESS_WHEEL_RADIUS)
#define BASE_ADDRESS_DECREASING_TIME    ADDRESS_OFFSET(BASE_ADDRESS_INCREASING_TIME)

namespace zeta_motor_driver
{
class ConfigurationHelper
{
    public:
        ConfigurationHelper()
        {
            Update();
        }
        ~ConfigurationHelper() {;}
        uint32_t GetBaudrate() { return baudrate;}
        void GetVersion(uint8_t dest[])
        {
            memcpy(dest, version, sizeof(uint8_t) * 2);
        }
        void GetError(uint8_t dest[])
        {
            memcpy(dest, last_error, sizeof(uint8_t) * 2);
        }
        float GetPGain() { return p_gain;}
        float GetIGain() { return i_gain;}
        float GetDGain() { return d_gain;}
        float GetMaxSpeed() { return max_speed;}
        float GetMinSpeed() { return min_speed;}
        float GetPPR() { return ppr;}
        float GetWheelRadius() { return wheel_radius;}
        uint16_t GetIncreasingTime() { return increasing_time;}
        uint16_t GetDecreasingTime() { return decreasing_time;}
        void SetVersion(uint8_t[]);
        void SetError(uint8_t[]);
        void SetPGain(float);
        void SetIGain(float);
        void SetDGain(float);
        void SetMaxSpeed(float);
        void SetMinSpeed(float);
        void SetPPR(float);
        void SetWheelRadius(float);
        void SetIncreasingTime(uint16_t);
        void SetDecreasingTime(uint16_t);
        void Update();
    private:
        int32_t baudrate;
        uint8_t version[2];
        uint8_t last_error[2];
        float p_gain;
        float i_gain;
        float d_gain;
        float max_speed;
        float min_speed;
        float ppr;
        float wheel_radius;
        uint16_t increasing_time;
        uint16_t decreasing_time;
}; /* class ConfigurationHelper */
}/* namespace zeta_motor_driver */

#endif /* ZETA_MOTOR_DRIVER_CONFIGURATION_HELPER_H_ */