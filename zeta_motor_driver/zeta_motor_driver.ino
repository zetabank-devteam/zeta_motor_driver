#include "zeta_motor_driver.h"

void setup() {
    pinMode(MT_COM_IND,OUTPUT);
    InitDriver();
    InitROS();
    millis();
    delay(1);
}

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

void InitDriver()
{
    serial_helper.Begin();
    pid_param.Init();
    pid_param.kp = serial_helper.GetPGain();
    pid_param.ki = serial_helper.GetIGain();
    pid_param.kd = serial_helper.GetDGain();
    motor1.encoder.encoder_pin = MOTR_ENCODER_PIN;
    motor2.encoder.encoder_pin = MOTL_ENCODER_PIN;
    controller.SetPPR(serial_helper.GetPPR());
    controller.SetWheelRadius(serial_helper.GetWheelRadius());
    controller.Begin(motor1, motor2, pid_param);
    pinMode(motor1.encoder.encoder_pin,INPUT_PULLUP);
    pinMode(motor2.encoder.encoder_pin,INPUT_PULLUP);   
    attachInterrupt(digitalPinToInterrupt(motor1.encoder.encoder_pin), read_encoder1_wrapper, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motor2.encoder.encoder_pin), read_encoder2_wrapper, CHANGE);
}

void read_encoder1_wrapper()
{
    controller.read_encoder1();
}

void read_encoder2_wrapper()
{
    controller.read_encoder2();
}

inline __attribute__((always_inline)) void RunPeriodicEvent()
{
    static unsigned long last_control_motor, last_transmit_data, last_execute_command;
    unsigned long time_cur = millis();
    static uint32_t time_pre[NUM_TASK];
    if((time_cur - time_pre[task_control_motor]) > (1000 / CONTROL_FREQUENCY))
    {
        if(nh.connected())
        {
            controller.SetMotorSpeed(serial_helper.motor1_state.vel_cmd,serial_helper.motor2_state.vel_cmd, serial_helper.IsBrake());
        }
        else
        {
            controller.SetMotorSpeed(0,0, false);
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
        if(nh.connected()) TransmitVelocity();
        time_pre[task_transmit_velocity] = time_cur;
    }
    time_cur = millis();
    if((time_cur - time_pre[task_publish_version]) > (1000 / VERSION_PUBLISH_FREQUENCY))
    {
        if(nh.connected()) fw_version_publisher.publish(&fw_version_msg);
        time_pre[task_publish_version] = time_cur;
    }
    time_cur = millis();
    if((time_cur - time_pre[task_blink_led]) > (1000 / BLINK_LED_FREQUENCY))
    {
        BlinkLED();
        time_pre[task_blink_led] = time_cur;
    }
}

inline __attribute__((always_inline)) void TransmitVelocity()
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
    serial_helper.GetMessage(serial_output_msg.data, &serial_output_msg.data_length);
    serial_output_publisher.publish(&serial_output_msg);
}

inline __attribute__((always_inline)) void ExecuteCommand()
{
    if(serial_helper.ExecuteCommand())
    {
        serial_helper.GetMessage(serial_output_msg.data, &serial_output_msg.data_length);
        if(nh.connected()) serial_output_publisher.publish(&serial_output_msg);
    }
}

inline __attribute__((always_inline))void BlinkLED()
{
    digitalWrite(MT_COM_IND,!digitalRead(MT_COM_IND));
}

void loop()
{
    RunPeriodicEvent();
    nh.spinOnce();
}

void SerialInputCallback(const std_msgs::UInt8MultiArray msg)
{
    serial_helper.SetMessage(msg.data, (uint8_t)msg.data_length);
}

/* zeta_motor_driver.ino */
