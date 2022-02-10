#include "pid_controller.h"

using namespace zeta_motor_driver;

void PidController::Begin(motor_t motor1_, motor_t motor2_, pid_t pid_param)
{
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
    braking_time = 10;
    motor1.Init();
    motor2.Init();
}



void PidController::BrakeMotor()
{
    motor1.dir *= -1;
    motor2.dir *= -1;
    ChangeDir();
    MOT1_TIMER.pwm(motor1.pwm_pin, MAXIMUM_DUTY);
    MOT2_TIMER.pwm(motor2.pwm_pin, MAXIMUM_DUTY);
    delay(braking_time);
    MOT1_TIMER.pwm(motor1.pwm_pin, 0);
    MOT2_TIMER.pwm(motor2.pwm_pin, 0);
    motor1.Init();
    motor2.Init();
    pid_motor1.InitError();
    pid_motor2.InitError();
    ChangeDir();
}

void PidController::SetMotorSpeed(float speed_motor1_, float speed_motor2_, bool brake = false)
{
    float speed_motor1, speed_motor2 = 0.0f;
#ifdef ENABLE_FLOAT_SENSING
    CheckWheelFloating();
#endif
    if(__builtin_expect(brake, 0))
    {
        motor1.Init();
        motor2.Init();
        motor1.state = MotorState::brake;
        motor2.state = MotorState::brake;
    }
    if(__builtin_expect(motor1.state == MotorState::brake && motor2.state == MotorState::brake,0))
    {
        return; // if brake before, no velocity change
    }
    if(__builtin_expect(speed_motor1_ > MAXIMUM_VELOCITY,0))
    {
        speed_motor1 = MAXIMUM_VELOCITY;
    }
    else if(__builtin_expect(speed_motor1_ < -1.0f * MAXIMUM_VELOCITY,0))
    {
        speed_motor1 = -1.0f * MAXIMUM_VELOCITY;
    }
    else if(__builtin_expect(speed_motor1_ < MINIMUM_VELOCITY && speed_motor1_ > VERY_SMALL_FLOAT,0))
    {
        speed_motor1 = MINIMUM_VELOCITY;
    }
    else if(__builtin_expect(speed_motor1_ < (-1.0f * VERY_SMALL_FLOAT) && speed_motor1_ > (-1.0f * MINIMUM_VELOCITY),0))
    {
        speed_motor1 = -1.0f * MINIMUM_VELOCITY;
    }
    else
    {
        speed_motor1 = speed_motor1_;
    }
    if(__builtin_expect(speed_motor2_ > MAXIMUM_VELOCITY,0))
    {
        speed_motor2 = MAXIMUM_VELOCITY;
    }
    else if(__builtin_expect(speed_motor2_ < -1.0f * MAXIMUM_VELOCITY,0))
    {
        speed_motor2 = -1.0f * MAXIMUM_VELOCITY;
    }
    else if(__builtin_expect(speed_motor2_ < MINIMUM_VELOCITY && speed_motor2_ > VERY_SMALL_FLOAT,0))
    {
        speed_motor2 = MINIMUM_VELOCITY;
    }
    else if(__builtin_expect(speed_motor2_ < (-1.0f * VERY_SMALL_FLOAT) && speed_motor2_ > (-1.0f * MINIMUM_VELOCITY),0))
    {
        speed_motor2 = -1.0f * MINIMUM_VELOCITY;
    }
    else
    {
        speed_motor2 = speed_motor2_;
    }
    if((fabs(motor1.vel_cmd - speed_motor1) < VERY_SMALL_FLOAT) && (fabs(motor2.vel_cmd - speed_motor2) < VERY_SMALL_FLOAT))
    {
        return; // if no speed change
    }
    motor1.vel_cmd = speed_motor1;
    motor2.vel_cmd = speed_motor2;
    pid_motor1.InitError();
    pid_motor2.InitError();
    motor1.duty = 0;
    motor2.duty = 0;
}

void PidController::GetVelocity(float dest[])
{
    dest[0] = motor1.vel_cur;
    dest[1] = motor2.vel_cur;
}

void PidController::GetPosition(float dest[])
{
    dest[0] = motor1.position;
    dest[1] = motor2.position;
}

/* pid_controller.cpp */
