#ifndef ZETA_MOTOR_DRIVER_MOTOR_H_
#define ZETA_MOTOR_DRIVER_MOTOR_H_
namespace zeta_motor_driver
{
template<typename timer_obj>
class Motor
{
    typedef struct
    {
        uint8_t          encoder_pin;
        volatile int32_t pulse_count;
        volatile int32_t pulse_count_pre;
        Init()
        {
            pulse_count = pulse_count_pre = 0;
        }
    }encoder_t;
    enum class MotorState : uint8_t
    {
        ready = 0,
        run,
        stop,
        brake,
        error,
    };
    public:
        
    private:
        timer_obj  timer;
        uint8_t    pwm_pin;
        uint8_t    cw_pin;
        uint8_t    ccw_pin;
        uint8_t    float_pin;
        int        dir;
        float      vel_cmd;
        float      vel_cur;
        float      pps;       // pulse per second
        int16_t    duty;
        encoder_t  encoder;
        MotorState state;
        Init()
        {
            vel_cmd = vel_cur = pps = 0.0f;
            dir = MOTOR_NEUTRAL;
            duty = 0;
            encoder.Init();
            state = MotorState::ready;
        }
}; /* class Motor */
}  /* namespace zeta_motor_driver */

#endif /* ZETA_MOTOR_DRIVER_MOTOR_H_ */