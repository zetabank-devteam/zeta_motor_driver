#ifndef ZETA_MOTOR_DRIVER_CONTROLLER_H_
#define ZETA_MOTOR_DRIVER_CONTROLLER_H_
#include "../pre_define.h"
#include <stdint.h>
#include "../include/Timer3/TimerThree.h"
#include "../include/TimerOne/TimerOne.h"
// #define MINIMUM_DUTY  80  // actual minimum speed(0.06m/s)
#define MINIMUM_DUTY            1
#define MAXIMUM_DUTY            700 // caution! carefully change this value for preventing overdrive of motor driver
#define VERY_SMALL_FLOAT        0.001f
#define VELOCITY_PROFILE_STEPS  5
#define MIN_VELOCITY_ERROR      0.002f
#define PWM_FREQUENCY           5000UL // the most good wave form & performance(min 8cm/s available)
#define MOTOR_FORWARD           1
#define MOTOR_BACKWARD          -1
#define MOTOR_NEUTRAL           0
// #define ENABLE_FLOAT_SENSING // uncomment it for activating wheel drop sensing
#define WHEEL_FLOATING_THRESHOLD  5

#ifdef TEST_ROBOT
#define MOT1_TIMER  Timer3
#define MOT2_TIMER  Timer1
#else
#define MOT1_TIMER  Timer1
#define MOT2_TIMER  Timer3
#endif

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
            uint8_t    cw_pin;
            uint8_t    ccw_pin;
            uint8_t    float_pin;
            int        dir;
            int        vel_step;
            float      vel_cmd;
            float      vel_cmd_profile[VELOCITY_PROFILE_STEPS];
            float      vel_cur;
            float      position;  // radian
            int16_t    duty;
            encoder_t  encoder;
            MotorState state;
            Init()
            {
                vel_cmd = vel_cur = position = 0.0f;
                for(int i = 0; i < VELOCITY_PROFILE_STEPS; i++)
                {
                    vel_cmd_profile[i] = 0.0f;
                }
                dir      = MOTOR_NEUTRAL;
                vel_step = 0;
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

        void ResetMotor(); // reset motor state
        void ControlVel() __attribute__((always_inline))
        {
            static uint32_t time_control_pre;
            static float    sampling_time;
            uint32_t        time_control_cur = micros();
            if(time_control_cur - time_control_pre < 0xF0000000)
            {
                sampling_time = float(time_control_cur - time_control_pre) / 1000000.0;
            }
            time_control_pre = time_control_cur;
            /* measurement block */
            if(motor1.state == MotorState::brake && motor2.state == MotorState::brake)
            {
                BrakeMotor();
                return;
            }
            SetDir();
            motor1.vel_cur  =  float(motor1.dir) * pps_to_velocity(float(motor1.encoder.pulse_count) / sampling_time);
            motor2.vel_cur  =  float(motor2.dir) * pps_to_velocity(float(motor2.encoder.pulse_count) / sampling_time);
            motor1.position += float(motor1.dir) * float(motor1.encoder.pulse_count) / ppr * TWO_PI;
            motor2.position += float(motor2.dir) * float(motor2.encoder.pulse_count) / ppr * TWO_PI;
            motor1.encoder.pulse_count = 0;
            motor2.encoder.pulse_count = 0;
            if(motor1.position > TWO_PI)
            {
                motor1.position -= TWO_PI;
            }
            else if(motor1.position < 0.0f)
            {
                motor1.position += TWO_PI;
            }
            if(motor2.position > TWO_PI)
            {
                motor2.position -= TWO_PI;
            }
            else if(motor2.position < 0.0f)
            {
                motor2.position += TWO_PI;
            }
            
            /* controller block */
            pid_motor1.err      = motor1.vel_cmd_profile[motor1.vel_step] - motor1.vel_cur;   // e[k] = r - y[k]
            pid_motor1.err_derv = (pid_motor1.err - pid_motor1.err_pre) / sampling_time;      // derv(e)[k] = (e[k] - e[k-1]) / ts    where, 'ts' is sampling time
            pid_motor1.err_int  = pid_motor1.err_int_pre +  pid_motor1.err * sampling_time;   // int(e)[k] = I[k-1] + e[k] * ts
            motor1.duty         += pid_motor1.err * pid_motor1.kp + pid_motor1.err_int * pid_motor1.ki + pid_motor1.err_derv * pid_motor1.kd;
            pid_motor2.err      = motor2.vel_cmd_profile[motor2.vel_step] - motor2.vel_cur;                 
            pid_motor2.err_derv = (pid_motor2.err - pid_motor2.err_pre) / sampling_time; 
            pid_motor2.err_int  = pid_motor2.err_int_pre +  pid_motor2.err * sampling_time;
            motor2.duty         += pid_motor2.err * pid_motor2.kp + pid_motor2.err_int * pid_motor2.ki + pid_motor2.err_derv * pid_motor2.kd;
            
            
            // static float sum;
            // sum += (motor1.vel_cur - motor2.vel_cur);
            // char tempstr[32];
            // int length = sprintf(tempstr,"%d, %d\r\n",int(motor1.vel_cmd * 1000),int(motor1.vel_cur * 1000));
            
            // int length = sprintf(tempstr,"%d\r\n",int(sum) * 1000);
            // Serial1.write(tempstr,length);
            // Serial1.println(sum,5);
            // sprintf(tempstr,"%d %d %d %d %d",int(motor2.vel_cmd_profile[motor2.vel_step] * 1000), int(motor2.vel_cur * 1000), int(pid_motor2.err*10000),int(pid_motor2.err_int*10000),motor2.duty);
            // nh.loginfo(tempstr);


            /* profile generator block */
            if(abs(motor1.duty) <= MINIMUM_DUTY && abs(motor1.duty) > 0)
            {
                motor1.duty = MINIMUM_DUTY * motor1.dir;
                if(pid_motor1.err * float(motor1.dir) < VERY_SMALL_FLOAT)
                {
                    pid_motor1.err_int = pid_motor1.err_int_pre;
                }
                motor1.state = MotorState::run;
            }
            else if(abs(motor1.duty) >= MAXIMUM_DUTY)
            {
                motor1.duty = MAXIMUM_DUTY * motor1.dir;
                if(pid_motor1.err * float(motor1.dir) > VERY_SMALL_FLOAT)
                {
                    pid_motor1.err_int = pid_motor1.err_int_pre; // anti wind-up -> output & integration will be staturated
                }
                motor1.state = MotorState::run;
            }
            else if(motor1.duty * motor1.dir < 0)
            {
                motor1.dir *= -1;
            }
            if(abs(motor2.duty) <= MINIMUM_DUTY && abs(motor2.duty) > 0)
            {
                motor2.duty = MINIMUM_DUTY * motor2.dir;
                if(pid_motor2.err * float(motor2.dir) < VERY_SMALL_FLOAT)
                {
                    pid_motor2.err_int = pid_motor2.err_int_pre;
                }
                motor2.state = MotorState::run;
            }
            else if(abs(motor2.duty) >= MAXIMUM_DUTY)
            {
                motor2.duty = MAXIMUM_DUTY * motor2.dir;
                if(pid_motor2.err * float(motor2.dir) > VERY_SMALL_FLOAT)
                {
                    pid_motor2.err_int = pid_motor2.err_int_pre; // anti wind-up -> output & integration will be staturated
                }
                motor2.state = MotorState::run;
            }
            else if(motor2.duty * motor2.dir < 0)
            {
                motor2.dir *= -1;
            }
            /* stop condition */
            if(fabs(motor1.vel_cmd_profile[motor1.vel_step]) < VERY_SMALL_FLOAT)
            {
                motor1.duty = 0;
                motor1.state = MotorState::stop;
            }
            if(fabs(motor2.vel_cmd_profile[motor2.vel_step]) < VERY_SMALL_FLOAT)
            {
                motor2.duty = 0;
                motor2.state = MotorState::stop;
            }
            if(motor1.vel_step != (VELOCITY_PROFILE_STEPS - 1))
            {
                motor1.vel_step += 1;
            }
            if(motor2.vel_step != (VELOCITY_PROFILE_STEPS - 1))
            {
                motor2.vel_step += 1;
            }
            /* Run */
            ChangeDir();
            MOT1_TIMER.pwm(motor1.pwm_pin,abs(motor1.duty));
            MOT2_TIMER.pwm(motor2.pwm_pin,abs(motor2.duty));
            pid_motor1.err_int_pre = pid_motor1.err_int;
            pid_motor2.err_int_pre = pid_motor2.err_int;
        }

        void read_encoder1()
        {
            motor1.encoder.pulse_count++;
        }

        void read_encoder2()
        {
            motor2.encoder.pulse_count++;
        }
        
    private:
        uint16_t  increasing_time;
        uint16_t  braking_time;
        float     maximum_speed;
        float     minimum_speed;
        float     ppr;             // pulse per rotation
        float     wheel_radius;
        pid_t     pid_motor1, pid_motor2;

        float pps_to_velocity(float pps) __attribute__((always_inline))
        {
            return pps / ppr * TWO_PI * wheel_radius;
        }

        void  ChangeDir() __attribute__((always_inline))
        {
            if(motor1.dir == MOTOR_NEUTRAL && motor1.state != MotorState::stop)
            {
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

        void SetDir() __attribute__((always_inline))
        {
            if(fabs(motor1.vel_cmd_profile[motor1.vel_step]) < VERY_SMALL_FLOAT)
            {
                motor1.dir = MOTOR_NEUTRAL; // if zero input
                pid_motor1.InitError();
            }
            else if(motor1.vel_cmd_profile[motor1.vel_step] < 0.0)
            {
                motor1.dir = MOTOR_BACKWARD;
            }
            else
            {
                motor1.dir = MOTOR_FORWARD;
            }

            if(fabs(motor2.vel_cmd_profile[motor2.vel_step]) < VERY_SMALL_FLOAT)
            {
                motor2.dir = MOTOR_NEUTRAL; // if zero input
                pid_motor2.InitError();
            }
            else if(motor2.vel_cmd_profile[motor2.vel_step] < 0.0)
            {
                motor2.dir = MOTOR_BACKWARD;
            }
            else
            {
                motor2.dir = MOTOR_FORWARD;
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
                motor1.state = MotorState::ready;
                motor2.state = MotorState::ready;
            }
            if(count == WHEEL_FLOATING_THRESHOLD)
            {
                motor1.state = MotorState::stop;
                motor2.state = MotorState::stop;
            }
        }
}; /* class PidController */
} /* namespace zeta_motor_driver */

#endif /* ZETA_MOTOR_DRIVER_CONTROLLER_H_ */