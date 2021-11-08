#include "zeta_motor_driver.h"
// /----------------------------------------------------------------------------\      
// |                         Serial Data Format                                 |
// | start1  start2  length*  PID**   data1 ... dataN   chksum*** end1    end2  |  
// |  0xAA    0xBB     N       -       -          -       -       0xDD    0x55  |
// |                                                                            |
// \----------------------------------------------------------------------------/
// *length: length of payload (N)
// **PID: Parameter ID.
// ***chksum = sum of payload & length = (len + d1 + d2 + ... dN) & 0xFF

void setup() {
    // put your setup code here, to run once:
    InitDriver();
#ifdef NO_ROS
    Serial.begin(115200);
#else
    InitROS();
#endif
    millis();
    delay(1);
}

void InitROS()
{
    nh.getHardware()->setBaud(serial_helper.GetBaudrate());
    nh.initNode();
    nh.advertise(fw_version_publisher);
    nh.subscribe(serial_data_subscriber);
}

void InitDriver()
{
    serial_helper.Begin();
    InitDriver();
    pid_param.Init();
    pid_param.kp = serial_helper.GetPGain();
    pid_param.ki = serial_helper.GetIGain();
    motor1.encoder.encoder_pin = MOT1_ENCODER_PIN;
    motor2.encoder.encoder_pin = MOT2_ENCODER_PIN;
    controller.SetPPR(serial_helper.GetPPR());
    controller.SetWheelRadius(serial_helper.GetWheelRadius());
    controller.Begin(motor1, motor2, pid_param);
    attachInterrupt(digitalPinToInterrupt(motor1.encoder.encoder_pin), read_encoder1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motor2.encoder.encoder_pin), read_encoder2, CHANGE);
}

void read_encoder1()
{
    controller.read_encoder1();
}

void read_encoder2()
{
    controller.read_encoder2();
}

void RunPeriodicEvent()
{
    static unsigned long last_control_motor, last_transmit_data, last_execute_command;
    unsigned long time_cur = millis();
    static uint32_t time_pre[NUM_TASK];
    if((time_cur - time_pre[task_control_motor]) > (1000 / CONTROL_FREQUENCY))
    {
        controller.ControlVel();
        time_pre[task_control_motor] = time_cur;
    }
    if((time_cur - time_pre[task_execute_command]) > (1000 / COMMAND_EXECUTE_FREQUENCY))
    {
        serial_helper.ExecuteCommand();
        controller.SetMotorSpeed(serial_helper.motor1_state.vel_cmd,serial_helper.motor2_state.vel_cmd);
        time_pre[task_execute_command] = time_cur;
    }
    if((time_cur - time_pre[task_transmit_velocity]) > (1000 / VEL_TRANSMIT_FREQUENCY))
    {
        TransmitVelocity();
        time_pre[task_transmit_velocity] = time_cur;
    }
#ifndef NO_ROS
    nh.spinOnce();
#endif
}

void TransmitVelocity()
{
    float vel[2];
    controller.GetVelocity(vel);
    serial_helper.motor1_state.vel_cur = vel[0];
    serial_helper.motor2_state.vel_cur = vel[1];
    serial_helper.TransmitVelocity();
    //Serial.print(serial_helper.motor1_state.vel_cur,3);Serial.print(", ");Serial.println(serial_helper.motor2_state.vel_cur,3);
}


void loop()
{
    RunPeriodicEvent();
}

void serialEvent1()
{
    serial_helper.ReceiveData();
}

void SerialDataCallback(const std_msgs::UInt8MultiArray msg)
{
    serial_helper.SetMessage(msg.data);
    serial_data_return_msg.data_length = msg.data_length;
    memcpy(serial_data_return_msg.data, msg.data, msg.data_length);
    serial_data_return_publisher.publish(&serial_data_return_msg);
}

/* zeta_motor_driver.ino */