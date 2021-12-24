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
    Serial1.begin(115200);
    InitDriver();
#ifdef NO_ROS
    Serial.begin(115200);
#else
    InitROS();
#endif
    millis();
    delay(1);
}

#ifndef NO_ROS
void InitROS()
{
    nh.getHardware()->setBaud(serial_helper.GetBaudrate());
    nh.initNode();
    nh.advertise(fw_version_publisher);
    nh.advertise(serial_output_publisher);
    nh.subscribe(serial_input_subscriber);
    // nh.advertise(test_publisher);
    // test_msg.data = (uint8_t*)malloc(16);
    serial_output_msg.data = (uint8_t*)malloc(sizeof(uint8_t) * TX_BUFFER_SIZE);
}
#endif

void InitDriver()
{
    serial_helper.Begin();
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
        if(serial_helper.IsBrake())
        {
            controller.SetMotorSpeed(serial_helper.motor1_state.vel_cmd,serial_helper.motor2_state.vel_cmd, true);
        }
        else
        {
            controller.SetMotorSpeed(serial_helper.motor1_state.vel_cmd,serial_helper.motor2_state.vel_cmd);
        }
        controller.ControlVel();
        time_pre[task_control_motor] = time_cur;
    }
    time_cur = millis();
    if((time_cur - time_pre[task_execute_command]) > (1000 / COMMAND_EXECUTE_FREQUENCY))
    {
        ExecuteCommand();
        time_pre[task_execute_command] = time_cur;
    }
    time_cur = millis();
    if((time_cur - time_pre[task_transmit_velocity]) > (1000 / VEL_TRANSMIT_FREQUENCY))
    {
        TransmitVelocity();
        time_pre[task_transmit_velocity] = time_cur;
    }
    if((time_cur - time_pre[3]) > (1000 / 1))
    {
        // uint8_t sending[6] = {0x01, 0x03, 0x01, 0x0c, 0x01, 0x0c};
        // memcpy(test_msg.data,sending,6);
        // test_msg.data_length = 6;
        // test_publisher.publish(&test_msg);
        // time_pre[3] = time_cur;
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
    serial_helper.SetVelocityMessage();
#ifndef NO_ROS
    serial_helper.GetMessage(serial_output_msg.data, &serial_output_msg.data_length);
    serial_output_publisher.publish(&serial_output_msg);
#else
    uint8_t  receive_buf[RX_BUFFER_SIZE];
    uint32_t receive_index;
    serial_helper.GetMessage(receive_buf, &receive_index);
    // Serial1.write(receive_buf, receive_index);
#endif
    //Serial1.print(serial_helper.motor1_state.vel_cur,3);Serial1.print(", ");Serial1.println(serial_helper.motor2_state.vel_cur,3);
}

void ExecuteCommand()
{
    if(serial_helper.ExecuteCommand())
    {
#ifndef NO_ROS
        serial_helper.GetMessage(serial_output_msg.data, &serial_output_msg.data_length);
        serial_output_publisher.publish(&serial_output_msg);
#else
        uint8_t  receive_buf[RX_BUFFER_SIZE];
        uint32_t receive_index;
        serial_helper.GetMessage(receive_buf,&receive_index);
        CONTROL_STREAM.write(receive_buf,receive_index);
#endif
    }
}


void loop()
{
    RunPeriodicEvent();
}

#ifdef NO_ROS
#if (CONTROL_STREAM == Serial)
void serialEvent()
{
    static uint8_t data[6];
    static int     data_length = 0;
    while(CONTROL_STREAM.available())
    {
        data[data_length++] = CONTROL_STREAM.read();
        delay(1);
    }
    //if(data_length == 6)
    {
        serial_helper.SetMessage(data, data_length);
        data_length = 0;
        memset(data,0xff,6);
    }
}
#elif (CONTROL_STREAM == Serial1)
void serialEvent1()
{
    static uint8_t data[6];
    static int     data_length = 0;
    while(CONTROL_STREAM.available())
    {
        data[data_length++] = CONTROL_STREAM.read();
        delay(1);
    }
    //if(data_length == 6)
    {
        serial_helper.SetMessage(data, data_length);
        data_length = 0;
        memset(data,0xff,6);
    }
}
#endif /* CONTROL_STREAM */
#else
void SerialInputCallback(const std_msgs::UInt8MultiArray msg)
{
    serial_helper.SetMessage(msg.data, (uint8_t)msg.data_length);
}
#endif /* NO_ROS */

/* zeta_motor_driver.ino */