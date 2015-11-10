#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

#include <Arduino.h>


// Temperature sensor setting
const int sensorPin = A0;

ros::NodeHandle nh;

std_msgs::UInt16 sensor_val_msg;
std_msgs::Float32 sensor_voltage_msg;
std_msgs::Float32 sensor_temp_msg;
ros::Publisher sensor_val("sensor_val", &sensor_val_msg);
ros::Publisher sensor_voltage("sensor_voltage", &sensor_voltage_msg);
ros::Publisher sensor_temp("sensor_temp", &sensor_temp_msg);

void setup()
{
    nh.initNode();
    nh.advertise(sensor_val);
    nh.advertise(sensor_voltage);
    nh.advertise(sensor_temp);
}

void loop()
{
    int sensorVal = analogRead(sensorPin);
    sensor_val_msg.data = sensorVal;
    sensor_val.publish(&sensor_val_msg);

    float voltage = (sensorVal / 1024.0) * 5.0;
    sensor_voltage_msg.data = voltage;
    sensor_voltage.publish(&sensor_voltage_msg);

    float temperature = (voltage - 0.5) * 100;
    sensor_temp_msg.data = temperature;
    sensor_temp.publish(&sensor_temp_msg);

    nh.spinOnce();
    delay(1000);
}
