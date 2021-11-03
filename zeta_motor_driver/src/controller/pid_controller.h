#ifndef ZETA_MOTOR_DRIVER_CONTROLLER_H_
#define ZETA_MOTOR_DRIVER_CONTROLLER_H_

#include "motor.hpp"

#include <stdint.h>

#include "../include/Timer3/TimerThree.h"
#include "../include/TimerOne/TimerOne.h"
#define MINIMUM_DUTY  80
#define MAXIMUM_DUTY  500
#define VERY_SMALL_FLOAT  0.001f
#define PWM_FREQUENCY     5000UL // the most good wave form & performance(min 8cm/s available)
// #define PWM_FREQUENCY     10000UL
#define MOTOR_FORWARD      1
#define MOTOR_BACKWARD     -1
#define MOTOR_NEUTRAL      0
// #define ENABLE_FLOAT_SENSING
#define WHEEL_FLOATING_THRESHOLD  5

namespace zeta_motor_driver
{
class PidController
{
    public:
        enum class MotorState : uint8_t
        {
            ready = 0,
            run,
            stop,
            brake,
            error,
        };
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
        typedef struct
        {
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
        }motor_t;
        typedef struct
        {
            float kp;
            float kd;
            float ki;
            float err;
            float err_int;
            float err_derv;
            float err_pre;
            float err_int_pre;
            Init()
            {
                kp = kd = ki = err = err_int = err_derv = err_pre = err_int_pre = 0.0f;
            }
            InitError()
            {
                err = err_int = err_derv = err_pre = err_int_pre = 0.0f;
            }
        }pid_t;

        motor_t   motor1, motor2;
        
        PidController();
        ~PidController() {}
        void Begin(motor_t,motor_t,pid_t);
        void SetMotorSpeed(float,float);
        void StopMotor();
        void SetGain(float,float,float);
        void SetIncreasingTime(uint16_t);
        void SetDecreasingTime(uint16_t);
        void SetPPR(float ppr_) { ppr = ppr_;}
        void SetWheelRadius(float wheel_radius_) { wheel_radius = wheel_radius_;}
        void SetMaxSpeed(float);
        void SetMinSpeed(float);

        void GetVelocity(float[]);
        void GetMotorState(MotorState[]);

        void ResetMotor(); // reset motor state

        void ControlVel() __attribute__((always_inline))
        {
            static uint32_t time_control_pre;
            uint32_t        time_control_cur = millis();
            float           sampling_time    = float(time_control_cur - time_control_pre) / 1000.0; // [sec]
            
            /* feedback block */
            ChangeDir();
            motor1.pps = (float(motor1.encoder.pulse_count - motor1.encoder.pulse_count_pre) / float(time_control_cur - time_control_pre)) * 1000;
            motor2.pps = (float(motor2.encoder.pulse_count - motor2.encoder.pulse_count_pre) / float(time_control_cur - time_control_pre)) * 1000;
            motor1.vel_cur = pps_to_velocity(motor1.pps);
            motor2.vel_cur = pps_to_velocity(motor2.pps); // inline in inline -> problem??
            motor1.encoder.pulse_count_pre = motor1.encoder.pulse_count;
            motor2.encoder.pulse_count_pre = motor2.encoder.pulse_count;
            /* controller block */
            pid_motor1.err      = motor1.vel_cmd - motor1.vel_cur;                            // e[k] = r - y[k]
            pid_motor1.err_derv = (pid_motor1.err - pid_motor1.err_pre) / sampling_time;      // derv(e)[k] = (e[k] - e[k-1]) / ts    where, 'ts' is sampling time
            pid_motor1.err_int  = pid_motor1.err_int_pre +  pid_motor1.err * sampling_time;   // int(e)[k] = I[k-1] + e[k] * ts
            motor1.duty += pid_motor1.err * pid_motor1.kp + pid_motor1.err_int * pid_motor1.ki + pid_motor1.err_derv * pid_motor1.kd;
            pid_motor2.err      = motor2.vel_cmd - motor2.vel_cur;                          
            pid_motor2.err_derv = (pid_motor2.err - pid_motor2.err_pre) / sampling_time; 
            pid_motor2.err_int  = pid_motor2.err_int_pre +  pid_motor2.err * sampling_time;
            motor2.duty += pid_motor2.err * pid_motor2.kp + pid_motor2.err_int * pid_motor2.ki + pid_motor2.err_derv * pid_motor2.kd;
            /* profile generator block */
            // @TODO add velocity profiling
            if(abs(motor1.duty) < MINIMUM_DUTY)
            {
                motor1.duty = MINIMUM_DUTY * motor1.dir;
                if(pid_motor1.err * float(motor1.dir) < VERY_SMALL_FLOAT)
                {
                    pid_motor1.err_int = pid_motor1.err_int_pre;
                }
                motor1.state = MotorState::run;
            }
            if(abs(motor1.duty) > MAXIMUM_DUTY)
            {
                motor1.duty = MAXIMUM_DUTY * motor1.dir;
                if(pid_motor1.err * float(motor1.dir) > VERY_SMALL_FLOAT)
                {
                    pid_motor1.err_int = pid_motor1.err_int_pre; // anti wind-up -> output & integration will be staturated
                }
                motor1.state = MotorState::run;
            }
            if(abs(motor2.duty) < MINIMUM_DUTY)
            {
                motor2.duty = MINIMUM_DUTY * motor2.dir;
                if(pid_motor2.err * float(motor2.dir) < VERY_SMALL_FLOAT)
                {
                    pid_motor2.err_int = pid_motor2.err_int_pre;
                }
                motor2.state = MotorState::run;
            }
            if(abs(motor2.duty) > MAXIMUM_DUTY)
            {
                motor2.duty = MAXIMUM_DUTY * motor2.dir;
                if(pid_motor2.err * float(motor2.dir) > VERY_SMALL_FLOAT)
                {
                    pid_motor2.err_int = pid_motor2.err_int_pre; // anti wind-up -> output & integration will be staturated
                }
                motor2.state = MotorState::run;
            }
            time_control_pre = time_control_cur;
            pid_motor1.err_derv_pre = pid_motor1.err_derv;
            pid_motor1.err_int_pre = pid_motor1.err_int;
            pid_motor2.err_derv_pre = pid_motor2.err_derv;
            pid_motor2.err_int_pre = pid_motor2.err_int;
            if(fabs(motor1.vel_cmd) < VERY_SMALL_FLOAT)
            {
                motor1.duty = 0; // velocity profiling
                motor1.state = MotorState::stop;
            }
            if(fabs(motor2.vel_cmd) < VERY_SMALL_FLOAT)
            {
                motor2.duty = 0; // velocity profiling
                motor2.state = MotorState::stop;
            }
            if(runnable && (motor1.state != MotorState::brake && motor2.state != MotorState::brake))
            {
                Timer3.pwm(motor1.pwm_pin,abs(motor1.duty));
                Timer1.pwm(motor2.pwm_pin,abs(motor2.duty));
            }
            else
            {
                Timer3.pwm(motor1.pwm_pin, 0);
                Timer1.pwm(motor2.pwm_pin, 0);
                pid_motor1.InitError();
                pid_motor2.InitError();
            }
            // Serial1.print(motor1.vel_cmd);Serial1.print(", ");Serial1.print(motor1.vel_cur);Serial1.print(", ");
            //Serial1.println(motor1.duty / 500.0f);
            // Serial.print(motor1.duty);Serial.print(", ");Serial.println(motor2.duty);
            // Serial.print(motor1.vel_cmd,3);Serial.print(", ");Serial.println(motor1.vel_cur,3);
            
        }
        void read_encoder1()
        {
            if(motor1.dir == MOTOR_FORWARD)
            {
                motor1.encoder.pulse_count++;
            }
            else if(motor1.dir == MOTOR_BACKWARD)
            {
                motor1.encoder.pulse_count--;
            }
        }

        void read_encoder2()
        {
            if(motor2.dir == MOTOR_FORWARD)
            {
                motor2.encoder.pulse_count++;
            }
            else if(motor2.dir == MOTOR_BACKWARD)
            {
                motor2.encoder.pulse_count--;
            }
        }
        
    private:
        uint16_t  increasing_time;
        uint16_t  decreasing_time;
        float     maximum_speed;
        float     minimum_speed;
        float     ppr; // pulse per rotation
        float     wheel_radius;
        pid_t     pid_motor1, pid_motor2;
        bool      runnable;

        float pps_to_velocity(float pps) __attribute__((always_inline))
        {
            return pps / ppr * TWO_PI * wheel_radius;
        }
        void  ChangeDir() __attribute__((always_inline))
        {
            if(motor1.dir == MOTOR_NEUTRAL && motor1.state != MotorState::stop)
            {
                //delay(decreasing_time); // @TODO: add velocity profiler for smooth stop
                digitalWrite(motor1.ccw_pin, LOW);
                digitalWrite(motor1.cw_pin,  LOW);
                motor1.state = MotorState::stop;
            }
            else if(motor1.dir == MOTOR_BACKWARD)
            {
                digitalWrite(motor1.ccw_pin, LOW);
                digitalWrite(motor1.cw_pin, HIGH);
            }
            else if(motor1.dir == MOTOR_FORWARD)
            {
                digitalWrite(motor1.ccw_pin, HIGH);
                digitalWrite(motor1.cw_pin, LOW);
            }
            if(motor2.dir == MOTOR_NEUTRAL && motor2.state != MotorState::stop)
            {
                //delay(decreasing_time);
                digitalWrite(motor2.ccw_pin, LOW);
                digitalWrite(motor2.cw_pin,  LOW);
                motor2.state = MotorState::stop;
            }
            else if(motor2.dir == MOTOR_BACKWARD)
            {
                digitalWrite(motor2.ccw_pin, HIGH);
                digitalWrite(motor2.cw_pin, LOW);
            }
            else if(motor2.dir == MOTOR_FORWARD)
            {
                digitalWrite(motor2.ccw_pin, LOW);
                digitalWrite(motor2.cw_pin, HIGH);
            }
        }
        void CheckWheelFloating() __attribute__((always_inline))
        {
            static int count;
            if((digitalRead(motor1.float_pin) || digitalRead(motor2.float_pin)))
            {
                if(count < WHEEL_FLOATING_THRESHOLD)
                {
                    count++;
                }
            }
            else
            {
                count = 0;
                runnable = true;
            }
            if(count == WHEEL_FLOATING_THRESHOLD)
            {
                runnable = false;
            }
        }
}; /* class PidController */
} /* namespace zeta_motor_driver */

#endif /* ZETA_MOTOR_DRIVER_CONTROLLER_H_ */