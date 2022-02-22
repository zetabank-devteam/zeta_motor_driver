#ifndef ZETA_MOTOR_DRIVER_CONTROLLER_H_
#define ZETA_MOTOR_DRIVER_CONTROLLER_H_
#include "../pre_define.h"
#include <stdint.h>
#include "../include/Timer3/TimerThree.h"
#include "../include/TimerOne/TimerOne.h"
// #define MINIMUM_DUTY  80  // actual minimum speed(0.06m/s)
#define MINIMUM_DUTY            0
#define MAXIMUM_DUTY            1000 // caution! carefully change this value for preventing overdrive of motor driver
#define MAXIMUM_VELOCITY        0.320f
#define MINIMUM_VELOCITY        0.016f
#define VERY_SMALL_FLOAT        0.001f // epsilon
#define MIN_VELOCITY_ERROR      0.002f
#define PWM_FREQUENCY           20000UL // upper than hearing range
#define MOTOR_FORWARD           1
#define MOTOR_BACKWARD          -1
#define MOTOR_NEUTRAL           0
// #define ENABLE_FLOAT_SENSING // uncomment it for activating wheel drop sensing
#define WHEEL_FLOATING_THRESHOLD  5

#define MOT1_TIMER  Timer3
#define MOT2_TIMER  Timer1

#include "ros.h"
extern ros::NodeHandle nh;

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
            uint8_t           encoder_pin;
            volatile uint32_t pulse_count;
            Init()
            {
                pulse_count = 0;
            }
        }encoder_t;
        typedef struct
        {
            uint8_t    pwm_pin;
            uint8_t    dir_pin;
            uint8_t    start_pin;
            uint8_t    float_pin;
            int        dir;
            int        dir_pre;
            float      vel_cmd;
            float      vel_cur;
            float      position;  // radian
            int16_t    duty;
            encoder_t  encoder;
            MotorState state;
            Init()
            {
                vel_cmd = vel_cur = position = 0.0f;
                dir      = MOTOR_FORWARD;
                dir_pre  = MOTOR_FORWARD;
                duty     = 0; // 0 ~ 1024
                state    = MotorState::ready;
                encoder.Init();
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
        
        PidController() {}
        ~PidController() {}
        void Begin(motor_t,motor_t,pid_t);
        void SetMotorSpeed(float,float,bool = false);
        void BrakeMotor();
        void SetGain(float,float,float);
        void SetPPR(float ppr_) { ppr = ppr_;}
        void SetWheelRadius(float wheel_radius_) { wheel_radius = wheel_radius_;}
        void SetMaxSpeed(float);
        void SetMinSpeed(float);
        void GetVelocity(float[]);
        void GetPosition(float[]);
        void GetMotorState(MotorState[]);
        inline __attribute__((always_inline)) void ControlVel()
        {
            static uint32_t time_control_pre;
            static float    sampling_time;
            uint32_t        time_control_cur = micros();
            if(time_control_cur - time_control_pre < 0xF0000000) sampling_time = float(time_control_cur - time_control_pre) / 1000000.0;
            time_control_pre = time_control_cur;
            if(motor1.state == MotorState::brake && motor2.state == MotorState::brake)
            {
                BrakeMotor();
                return;
            }
            /* measurement block */
            motor1.vel_cur  =  float(motor1.dir) * pps_to_velocity(float(motor1.encoder.pulse_count) / sampling_time);
            motor2.vel_cur  =  float(motor2.dir) * pps_to_velocity(float(motor2.encoder.pulse_count) / sampling_time);
            motor1.position += float(motor1.dir) * float(motor1.encoder.pulse_count) / ppr * TWO_PI;
            motor2.position += float(motor2.dir) * float(motor2.encoder.pulse_count) / ppr * TWO_PI;
            motor1.encoder.pulse_count = 0;
            motor2.encoder.pulse_count = 0;
            if(__builtin_expect(motor1.position > TWO_PI,0))     motor1.position -= TWO_PI;
            else if(__builtin_expect(motor1.position < 0.0f,0))  motor1.position += TWO_PI;
            if(__builtin_expect(motor2.position > TWO_PI,0))     motor2.position -= TWO_PI;
            else if(__builtin_expect(motor2.position < 0.0f,0))  motor2.position += TWO_PI;
            /* controller block */
            pid_motor1.err      = motor1.vel_cmd - motor1.vel_cur;   // e[k] = r - y[k]
            pid_motor1.err_derv = (pid_motor1.err - pid_motor1.err_pre) / sampling_time;      // derv(e)[k] = (e[k] - e[k-1]) / ts    where, 'ts' is sampling time
            pid_motor1.err_int  = pid_motor1.err_int_pre +  pid_motor1.err * sampling_time;   // int(e)[k] = I[k-1] + e[k] * ts
            motor1.duty         += round(pid_motor1.err * pid_motor1.kp + pid_motor1.err_int * pid_motor1.ki + pid_motor1.err_derv * pid_motor1.kd);
            pid_motor2.err      = motor2.vel_cmd - motor2.vel_cur;
            pid_motor2.err_derv = (pid_motor2.err - pid_motor2.err_pre) / sampling_time;
            pid_motor2.err_int  = pid_motor2.err_int_pre +  pid_motor2.err * sampling_time;
            motor2.duty         += round(pid_motor2.err * pid_motor2.kp + pid_motor2.err_int * pid_motor2.ki + pid_motor2.err_derv * pid_motor2.kd);
            // if(motor1.duty * motor1.dir <= MINIMUM_DUTY && motor1.duty * motor1.dir > 0) // 0 < duty < MIN or -MIN < duty < 0
            // {
            //     motor1.duty = MINIMUM_DUTY * motor1.dir;
            //     motor1.state = MotorState::run;
            // }
            if(abs(motor1.duty) >= MAXIMUM_DUTY)
            {
                motor1.duty = MAXIMUM_DUTY * motor1.dir;
                if(pid_motor1.err * float(motor1.dir) > 0.0f) pid_motor1.err_int = pid_motor1.err_int_pre; // anti wind-up -> output & integration will be staturated
                motor1.state = MotorState::run;
            }
            if(motor1.duty * motor1.dir < 0) motor1.dir *= -1; // if reverse disturbance
            // if(motor2.duty * motor2.dir <= MINIMUM_DUTY && motor2.duty * motor2.dir > 0) 
            // {
            //     motor2.duty = MINIMUM_DUTY * motor2.dir;
            //     motor2.state = MotorState::run;
            // }
            if(abs(motor2.duty) >= MAXIMUM_DUTY)
            {
                motor2.duty = MAXIMUM_DUTY * motor2.dir;
                if(pid_motor2.err * float(motor2.dir) > VERY_SMALL_FLOAT) pid_motor2.err_int = pid_motor2.err_int_pre;
                motor2.state = MotorState::run;
            }
            if(motor2.duty * motor2.dir < 0) motor2.dir *= -1;
            /* Run */
            ChangeDir();
            MOT1_TIMER.pwm(motor1.pwm_pin,abs(motor1.duty));
            MOT2_TIMER.pwm(motor2.pwm_pin,abs(motor2.duty));
            if(motor1.duty > 1) motor1.duty--;
            else if(motor1.duty < -1) motor1.duty++;
            if(motor2.duty > 1) motor2.duty--;
            else if(motor2.duty < -1) motor2.duty++;
            pid_motor1.err_int_pre = pid_motor1.err_int;
            pid_motor2.err_int_pre = pid_motor2.err_int;
        }
        void read_encoder1() {motor1.encoder.pulse_count++;}
        void read_encoder2() {motor2.encoder.pulse_count++;}
    private:
        uint16_t  increasing_time;
        uint16_t  braking_time;
        float     maximum_speed;
        float     minimum_speed;
        float     ppr;             // pulse per rotation
        float     wheel_radius;
        pid_t     pid_motor1, pid_motor2;
        inline __attribute__((always_inline)) float pps_to_velocity(float pps) {return pps / ppr * TWO_PI * wheel_radius;}
        inline __attribute__((always_inline)) void  ChangeDir()
        {
            if(motor1.dir_pre != MOTOR_BACKWARD && motor1.dir == MOTOR_BACKWARD)
            {// TODO: check direction for each wheel
                digitalWrite(motor1.dir_pin, HIGH);
            }
            else if(motor1.dir_pre != MOTOR_FORWARD && motor1.dir == MOTOR_FORWARD)
            {
                digitalWrite(motor1.dir_pin, LOW);
            }
            if(motor2.dir_pre != MOTOR_BACKWARD && motor2.dir == MOTOR_BACKWARD)
            {
                digitalWrite(motor2.dir_pin, LOW);
            }
            else if(motor2.dir_pre != MOTOR_FORWARD && motor2.dir == MOTOR_FORWARD)
            {
                digitalWrite(motor2.dir_pin, HIGH);
            }
            motor1.dir_pre = motor1.dir;
            motor2.dir_pre = motor2.dir;
        }

        inline __attribute__((always_inline)) void CheckWheelFloating()
        {
            static int count;
            if(digitalRead(motor1.float_pin) || digitalRead(motor2.float_pin))
            {
                if(count < WHEEL_FLOATING_THRESHOLD) count++;
            }
            else
            {
                count = 0;
                motor1.state = MotorState::ready;
                motor2.state = MotorState::ready;
            }
            if(count == WHEEL_FLOATING_THRESHOLD)
            {
                motor1.state = MotorState::stop;
                motor2.state = MotorState::stop;
            }
        }

        inline __attribute__((always_inline)) float ConstrainSpeed(float speed_)
        {
            if(__builtin_expect(speed_ > MAXIMUM_VELOCITY,0))
            {
                return MAXIMUM_VELOCITY;
            }
            else if(__builtin_expect(speed_ < -1.0f * MAXIMUM_VELOCITY, 0))
            {
                return -1.0f * MAXIMUM_VELOCITY;
            }
            else if(__builtin_expect(speed_ < MINIMUM_VELOCITY && speed_ > VERY_SMALL_FLOAT, 0))
            {
                return MINIMUM_VELOCITY;
            }
            else if(__builtin_expect(speed_ < -1.0f * VERY_SMALL_FLOAT && speed_ > -1.0f * MINIMUM_VELOCITY, 0))
            {
                return -1.0f * MINIMUM_VELOCITY;
            }
            else
            {
                return speed_;
            }
        }
}; /* class PidController */
} /* namespace zeta_motor_driver */

#endif /* ZETA_MOTOR_DRIVER_CONTROLLER_H_ */