#include <ros.h>
#include <string.h>

#include <Arduino.h>

#include "MS5837.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_Sensor.h"
#include "DallasTemperature.h"
#include <Wire.h>

#include "std_msgs/String.h"
#include <geometry_msgs/Vector3.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"


#define STANDARD_GRAVITY 9.08665      // [m/s^2]
#define RAD_PER_DEG 0.01745329252     // [1/deg]
#define PASCAL_PER_MILLIBAR 0.01      // [Pa/mbar]
#define MICROTESLA_PER_TESLA 0.000001 // [uT/T]

int id = -1;

ros::NodeHandle nh;

sensor_msgs::Imu imu_raw_msg;
sensor_msgs::MagneticField compass_msg;

sensor_msgs::Temperature imu_temperature_msg;
sensor_msgs::Temperature sensor_temperature_msg;
sensor_msgs::FluidPressure pressure_msg;

std_msgs::String calibration;

geometry_msgs::Vector3 euler_msg;

ros::Publisher pub_imu("imu/data", &imu_raw_msg);
ros::Publisher pub_mag("imu/mag", &compass_msg);

ros::Publisher pub_pressure("imu/pressure", &pressure_msg);
ros::Publisher pub_imu_temperature("imu/temperature", &imu_temperature_msg);
ros::Publisher pub_sensor_temperature("sensor/temperature", &sensor_temperature_msg);

ros::Publisher pub_calibration("imu/calibration", &calibration);

ros::Publisher pub_euler("imu/euler", &euler_msg);


// float DBG_AXIS_SIGN_VALUE = -1.0;

const int SensorReadDelay = 40;
unsigned long PrevoiusSensorReadMillis = 0;

int dbg_count = 0;

MS5837 barometer;
Adafruit_BNO055 bno055 = Adafruit_BNO055(55);

#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);


// double GyroLsbSens, AccelLsbSens;
void setupIMU();

void setup() {

    //start ROS-node
    nh.initNode();

    // nh.advertise(pub_dbg);
    nh.advertise(pub_imu_temperature);
    nh.advertise(pub_sensor_temperature);

    nh.advertise(pub_pressure);
    nh.advertise(pub_imu);
    nh.advertise(pub_mag);
    nh.advertise(pub_calibration);
    nh.advertise(pub_euler);

    // Initialize the 'Wire' class for communicating over i2c
    Wire.begin();
    barometer.init();
    barometer.read();

    setupIMU();
    // nh.advertise(pub_mag);
}

void setupIMU(){ /*
                  Sets opmode to NDOF by default

                  From Adafruit_BNO055.h:

                  OPERATION_MODE_CONFIG                                   = 0X00,
                  OPERATION_MODE_ACCONLY                                  = 0X01,
                  OPERATION_MODE_MAGONLY                                  = 0X02,
                  OPERATION_MODE_GYRONLY                                  = 0X03,
                  OPERATION_MODE_ACCMAG                                   = 0X04,
                  OPERATION_MODE_ACCGYRO                                  = 0X05,
                  OPERATION_MODE_MAGGYRO                                  = 0X06,
                  OPERATION_MODE_AMG                                      = 0X07,
                  OPERATION_MODE_IMUPLUS                                  = 0X08,
                  OPERATION_MODE_COMPASS                                  = 0X09,
                  OPERATION_MODE_M4G                                      = 0X0A,
                  OPERATION_MODE_NDOF_FMC_OFF                             = 0X0B,
                  OPERATION_MODE_NDOF                                     = 0X0C


                  ACCEL   MAG     GYRO    RELATIVE        ABSOLUTE
                  ORIENTATION     ORIENTATION
                  ---------------------------------------------------------------
                  IMU     |    X    |   -    |   X  |       X        |     -
                  COMPASS |    X    |   X    |   -  |       -        |     X
                  M4G     |    X    |   X    |   -  |       X        |     -
                  NDOF    |    X    |   X    |   X  |       -        |     X
                */

    id = bno055.begin(bno055.OPERATION_MODE_CONFIG);
    bno055.setAxisConfig(bno055.REMAP_CONFIG_P6);
    bno055.setAxisSign(bno055.REMAP_SIGN_P6);
    bno055.setMode(bno055.OPERATION_MODE_IMUPLUS);
}

void getPressure() {

    // float pressure = barometer.pressure();
    float pressure = barometer.pressure();
    pressure_msg.fluid_pressure = pressure * 100;
    pub_pressure.publish(&pressure_msg);

}


#define BARO 10
int baro_counter = 0;
void getBarometerTemp() {

    if(baro_counter < BARO){
        baro_counter++;
        return;
    }
    else{
        baro_counter = 0;
    }

    float temp = barometer.temperature();
    imu_temperature_msg.temperature = temp;

    pub_imu_temperature.publish(&imu_temperature_msg);

}


#define DALL 10
int dallas_counter = 50;
void getDallasTemp() {

    if(dallas_counter < DALL){
        dallas_counter++;
        return;
    }
    else{
        dallas_counter = 0;
    }

    sensors.requestTemperatures();
    float temp = sensors.getTempCByIndex(0);
    sensor_temperature_msg.temperature = temp;

    pub_sensor_temperature.publish(&sensor_temperature_msg);


}

void getRawIMU() {

    // sensors_event_t event;
    // bno055.getEvent(&event);

    imu::Vector<3> gyro = bno055.getVector(bno055.VECTOR_GYROSCOPE);
    imu_raw_msg.linear_acceleration.x = gyro[0];
    imu_raw_msg.linear_acceleration.y = gyro[1];
    imu_raw_msg.linear_acceleration.z = gyro[2];


    imu::Vector<3> lin_acc = bno055.getVector(bno055.VECTOR_LINEARACCEL);
    imu_raw_msg.angular_velocity.x = lin_acc[0];
    imu_raw_msg.angular_velocity.y = lin_acc[1];
    imu_raw_msg.angular_velocity.z = lin_acc[2];


    imu::Quaternion qs = bno055.getQuat();
    imu_raw_msg.orientation.x = qs.x();
    imu_raw_msg.orientation.y = qs.y();
    imu_raw_msg.orientation.z = qs.z();
    imu_raw_msg.orientation.w = qs.w();

    imu_raw_msg.header.stamp = ros::Time(millis()/1000.0, 0);
    pub_imu.publish(&imu_raw_msg);

}

bool calibrated = false;


void getCalibrationStatus(){

    // lol
    uint8_t sys, gyro, accel, mag;
    sys = gyro = accel = mag = 0;
    bno055.getCalibration(&sys, &gyro, &accel, &mag);
    char buffer[128] = { '\0' };

    if (sys == 3 && gyro == 3 && accel == 3 && mag == 3){
        if(!calibrated){
            sprintf(buffer, "\n All systems go\n");
            calibrated = true;
        }
        return;
    }

    calibrated = false;
    sprintf(buffer, "\nSys: %d, Gyro: %d, Acc: %d, mag: %d\n", sys, gyro, accel, mag);
    String s = String(buffer);
    calibration.data = s.c_str();
    pub_calibration.publish(&calibration);

}

void getEuler()
{
    imu::Vector<3> euler = bno055.getVector(bno055.VECTOR_EULER);
    euler_msg.x = euler[0];
    euler_msg.y = euler[1];
    euler_msg.z = euler[2];
    pub_euler.publish(&euler_msg);
}

void loop(){

    nh.spinOnce();

    if( millis() - PrevoiusSensorReadMillis >= SensorReadDelay ) {

        PrevoiusSensorReadMillis = millis();

        barometer.read();
        nh.spinOnce();


        getPressure();
        nh.spinOnce();


        getBarometerTemp();
        nh.spinOnce();


        getDallasTemp();
        nh.spinOnce();


        getRawIMU();
        nh.spinOnce();


        getCalibrationStatus();
        nh.spinOnce();

        getEuler();
        nh.spinOnce();

    }
}
