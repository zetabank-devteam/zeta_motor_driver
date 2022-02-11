#include "zeta_motor_driver.h"

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
    serial_output_msg.data = (uint8_t*)malloc(sizeof(uint8_t) * TX_BUFFER_SIZE);
    fw_version_msg.data = FW_VERSION;
}
#endif

void InitDriver()
{
    serial_helper.Begin();
    pid_param.Init();
    pid_param.kp = serial_helper.GetPGain();
    pid_param.ki = serial_helper.GetIGain();
    pid_param.kd = serial_helper.GetDGain();
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
        controller.SetMotorSpeed(serial_helper.motor1_state.vel_cmd,
            serial_helper.motor2_state.vel_cmd, serial_helper.IsBrake());
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
    if((time_cur - time_pre[3]) > (1000 / 0.1))
    {
        fw_version_publisher.publish(&fw_version_msg);
    }
#ifndef NO_ROS
    nh.spinOnce();
#endif
}

void TransmitVelocity()
{
    float vel[2];
    float pos[2];
    controller.GetVelocity(vel);
    controller.GetPosition(pos);
    serial_helper.motor1_state.vel_cur  = vel[0];
    serial_helper.motor2_state.vel_cur  = vel[1];
    serial_helper.motor1_state.position = pos[0];
    serial_helper.motor2_state.position = pos[1];
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

#else
void SerialInputCallback(const std_msgs::UInt8MultiArray msg)
{
    serial_helper.SetMessage(msg.data, (uint8_t)msg.data_length);
}
#endif /* NO_ROS */

/* zeta_motor_driver.ino */
