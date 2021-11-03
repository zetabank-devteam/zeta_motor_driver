#ifndef ZETA_MOTOR_DRIVER_MOTOR_H_
#define ZETA_MOTOR_DRIVER_MOTOR_H_
#include <stdint.h>

#include "../include/Timer3/TimerThree.h"
#include "../include/TimerOne/TimerOne.h"

#define TIMER1_A_PIN   11
#define TIMER1_B_PIN   12
#define TIMER1_C_PIN   13
#define TIMER3_A_PIN   5
#define TIMER3_B_PIN   2
#define TIMER3_C_PIN   3

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
        kidnapped,
        error,
    };
    public:
        Motor(uint8_t pwm_pin_, uint8_t cw_pin_, uint8_t ccw_pin_, uint8_t encoder_pin_, uint8_t sw_pin_) 
            : pwm_pin(pwm_pin_), cw_pin(cw_pin_), ccw_pin(ccw_pin_), sw_pin(sw_pin_)
        {
            timer = nullptr;
            if(pwm_pin == TIMER1_A_PIN || pwm_pin == TIMER1_B_PIN || pwm_pin == TIMER1_C_PIN)
            {
                timer = &Timer1;
            }
            else if(pwm_pin == TIMER3_A_PIN || pwm_pin == TIMER3_B_PIN || pwm_pin == TIMER3_C_PIN)
            {
                timer = &Timer3;
            }
            encoder.encoder_pin = encoder_pin_;
            pinMode(pwm_pin,   OUTPUT);
            pinMode(cw_pin,    OUTPUT);
            pinMode(ccw_pin,   OUTPUT);
            pinMode(sw_pin,    INPUT_PULLUP);
            pinMode(encoder.encoder_pin, INPUT);
            timer->initialize(1000000/PWM_FREQUENCY);
            timer->pwm(pwm_pin, 0);
        }
        SetState(MotorState state_) { state = state_; }


    private:
        timer_obj* timer;
        uint8_t    pwm_pin;
        uint8_t    cw_pin;
        uint8_t    ccw_pin;
        uint8_t    sw_pin;
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