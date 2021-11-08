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
    Timer1.initialize(1000000/PWM_FREQUENCY);
    Timer3.initialize(1000000/PWM_FREQUENCY);
    Timer1.pwm(motor1.pwm_pin,0);
    Timer3.pwm(motor2.pwm_pin,0);
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
    Timer1.pwm(motor1.pwm_pin, 1024);
    Timer3.pwm(motor2.pwm_pin, 1024);
    delay(decreasing_time);
    Timer1.pwm(motor1.pwm_pin, 0);
    Timer3.pwm(motor2.pwm_pin, 0);
    motor1.dir = MOTOR_NEUTRAL;
    motor2.dir = MOTOR_NEUTRAL;
    motor1.state = MotorState::brake;
    motor2.state = MotorState::brake;
}

void PidController::SetMotorSpeed(float speed_motor1, float speed_motor2)
{
#ifdef ENABLE_FLOAT_SENSING
    CheckWheelFloating();
#endif
    motor1.vel_cmd = speed_motor1;
    motor2.vel_cmd = speed_motor2;
    if(fabs(motor1.vel_cmd) < VERY_SMALL_FLOAT)
    {
        motor1.dir = MOTOR_NEUTRAL; // if zero input
        pid_motor1.InitError();
    }
    else if(motor1.vel_cmd < 0.0)
    {
        motor1.dir = MOTOR_BACKWARD;
    }
    else
    {
        motor1.dir = MOTOR_FORWARD;
    }
    if(fabs(motor2.vel_cmd) < VERY_SMALL_FLOAT)
    {
        motor2.dir = MOTOR_NEUTRAL; // if zero input
        pid_motor2.InitError();
    }
    else if(motor2.vel_cmd < 0.0)
    {
        motor2.dir = MOTOR_BACKWARD;
    }
    else
    {
        motor2.dir = MOTOR_FORWARD;
    }
    //ChangeDir();
    //ControlVel();
    /*
     * @TODO add output profiler for mapping vel_cur with the real velocity
     * ex) (-) to (+), (-) to 0 within 1/2 of increasing time and 0 to (+) within 1/2 increasing time
    */
    
    //Serial1.print(pid_motor1.kp);Serial1.print(", ");Serial1.print(pid_motor1.ki);Serial1.print(", ");Serial1.println(pid_motor1.kd);
}

void PidController::GetVelocity(float dest[])
{
    dest[0] = motor1.vel_cur;
    dest[1] = motor2.vel_cur;
}

/* pid_controller.cpp */