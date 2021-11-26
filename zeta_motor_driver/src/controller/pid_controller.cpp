#include "pid_controller.h"

using namespace zeta_motor_driver;

void PidController::Begin(motor_t motor1_, motor_t motor2_, pid_t pid_param)
{
    SetPPR(500.0f);
    SetWheelRadius(0.035f);
    memcpy(&motor1,&motor1_,sizeof(motor_t));
    memcpy(&motor2,&motor2_,sizeof(motor_t));
    memcpy(&pid_motor1,&pid_param,sizeof(pid_t));
    memcpy(&pid_motor2,&pid_param,sizeof(pid_t));
    pinMode(motor1.pwm_pin,   OUTPUT);
    pinMode(motor1.cw_pin,    OUTPUT);
    pinMode(motor1.ccw_pin,   OUTPUT);
    pinMode(motor1.float_pin, INPUT_PULLUP);
    pinMode(motor1.encoder.encoder_pin, INPUT);
    pinMode(motor2.pwm_pin,   OUTPUT);
    pinMode(motor2.cw_pin,    OUTPUT);
    pinMode(motor2.ccw_pin,   OUTPUT);
    pinMode(motor2.float_pin, INPUT_PULLUP);
    pinMode(motor2.encoder.encoder_pin, INPUT);
    MOT1_TIMER.initialize(1000000/PWM_FREQUENCY);
    MOT2_TIMER.initialize(1000000/PWM_FREQUENCY);
    MOT1_TIMER.pwm(motor1.pwm_pin,0);
    MOT2_TIMER.pwm(motor2.pwm_pin,0);
    decreasing_time = 10;
    runnable = true;
}



void PidController::StopMotor()
{
    motor1.vel_cmd = 0.0f;
    motor2.vel_cmd = 0.0f;
    motor1.duty    = 0;
    motor2.duty    = 0;
    motor1.dir     *= -1;
    motor2.dir     *= -1;
    ChangeDir();
    MOT1_TIMER.pwm(motor1.pwm_pin, 1024);
    MOT2_TIMER.pwm(motor2.pwm_pin, 1024);
    delay(decreasing_time);
    MOT1_TIMER.pwm(motor1.pwm_pin, 0);
    MOT2_TIMER.pwm(motor2.pwm_pin, 0);
    motor1.dir = MOTOR_NEUTRAL;
    motor2.dir = MOTOR_NEUTRAL;
    motor1.state = MotorState::brake;
    motor2.state = MotorState::brake;
}

void PidController::SetMotorSpeed(float speed_motor1, float speed_motor2, bool brake = false)
{
#ifdef ENABLE_FLOAT_SENSING
    CheckWheelFloating();
#endif
    if(brake)
    {
        motor1.vel_cmd = 0.0f;
        motor2.vel_cmd = 0.0f;
        for(int i = 0; i < VELOCITY_PROFILE_STEPS; i++)
        {
            motor1.vel_cmd_profile[i] = 0.0f;
            motor2.vel_cmd_profile[i] = 0.0f;
        }
        motor1.vel_step = 0;
        motor2.vel_step = 0;
        motor1.state = MotorState::brake;
        motor2.state = MotorState::brake;
        return;
    }
    if((fabs(motor1.vel_cmd - speed_motor1) < VERY_SMALL_FLOAT) && (fabs(motor2.vel_cmd - speed_motor2) < VERY_SMALL_FLOAT))
    {
        return; // if no speed change
    }
    motor1.vel_cmd = speed_motor1;
    motor2.vel_cmd = speed_motor2;
    for(int i = 0; i < VELOCITY_PROFILE_STEPS; i++)
    {
        motor1.vel_cmd_profile[i] = motor1.vel_cur + (motor1.vel_cmd - motor1.vel_cur) / float(VELOCITY_PROFILE_STEPS) * float(i + 1);
        motor2.vel_cmd_profile[i] = motor2.vel_cur + (motor2.vel_cmd - motor2.vel_cur) / float(VELOCITY_PROFILE_STEPS) * float(i + 1);
    }
    motor1.vel_step = 0;
    motor2.vel_step = 0;
    //Serial1.print(pid_motor1.kp);Serial1.print(", ");Serial1.print(pid_motor1.ki);Serial1.print(", ");Serial1.println(pid_motor1.kd);
}

void PidController::GetVelocity(float dest[])
{
    dest[0] = motor1.vel_cur;
    dest[1] = motor2.vel_cur;
}

/* pid_controller.cpp */