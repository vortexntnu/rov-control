#include <ros.h>
#include <string.h>

#include "std_msgs/String.h"

//includer for IMU og trykksensor
#include "MS5837.h"
#include <Wire.h>
#include <geometry_msgs/Vector3.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/FluidPressure.h"

#define STANDARD_GRAVITY 9.08665      // [m/s^2]
#define RAD_PER_DEG 0.01745329252     // [1/deg]
#define PASCAL_PER_MILLIBAR 0.01      // [Pa/mbar]
#define MICROTESLA_PER_TESLA 0.000001 // [uT/T]

ros::NodeHandle nh;

// sensor_msgs::Imu imu_raw_msg;
// sensor_msgs::MagneticField compass_msg;
// sensor_msgs::Temperature temperature_msg;
// sensor_msgs::FluidPressure pressure_msg;

// ros::Publisher pub_imu("imu/data_raw", &imu_raw_msg);
// ros::Publisher pub_mag("imu/mag", &compass_msg);
// ros::Publisher pub_pressure("imu/pressure", &pressure_msg);
// ros::Publisher pub_temperature("imu/temperature", &temperature_msg);

const int SensorReadDelay = 500;
unsigned long PrevoiusSensorReadMillis = 0;

int dbg_count = 0;

MS5837 barometer;

// double GyroLsbSens, AccelLsbSens;


void setup() {
  
    //start ROS-node
    nh.initNode();

    // nh.advertise(pub_imu);
    // nh.advertise(pub_mag);
    // nh.advertise(pub_temperature);
    // nh.advertise(pub_pressure);
    
    nh.spinOnce();
    
    // Initialize the 'Wire' class for communicating over i2c
    Wire.begin();
    barometer.init();

    DDRB |= _BV(DDB5);

    nh.spinOnce();
}

float killme = 0.0;

void getPressure() {

    // Is this blocking or not? the world may never know!
    // barometer.read();

    // float pressure = barometer.pressure();
    // pressure_msg.fluid_pressure = killme++;


    // pub_pressure.publish(&pressure_msg);

}

bool on = false;

void loop(){

    nh.spinOnce();

    if( millis() - PrevoiusSensorReadMillis >= SensorReadDelay ) {
        PrevoiusSensorReadMillis = millis();

        on = ~on;
        if(on)
            PORTB |= _BV(PORTB5);

        if(!on)
            PORTB &= ~_BV(PORTB5);

        // getPressure();

    }
}


// void getFsRangeAndSetLsbSensisivity() {
//   
//     uint8_t GyroFsRange, AccelFsRange;
//     
//     GyroFsRange = accelgyro.getFullScaleGyroRange();
//     /* Gyro
//      * FS_SEL | Full Scale Range   | LSB Sensitivity
//      * -------+--------------------+----------------
//      * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
//      * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
//      * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
//      * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
//      */
//     switch(GyroFsRange) {
//     case 0:
//         GyroLsbSens = 131.0;
//         break;
//     case 1:
//         GyroLsbSens = 65.5;
//         break;
//     case 2:
//         GyroLsbSens = 32.8;
//         break;
//     case 3:
//         GyroLsbSens = 16.4;
//         break;  
//     };
// 
//     
//     AccelFsRange = accelgyro.getFullScaleAccelRange();
//     /*Accelerometer
//      * AFS_SEL | Full Scale Range | LSB Sensitivity
//      * --------+------------------+----------------
//      * 0       | +/- 2g           | 16384 LSB/g
//      * 1       | +/- 4g           | 8192 LSB/g
//      * 2       | +/- 8g           | 4096 LSB/g
//      * 3       | +/- 16g          | 2048 LSB/g
//      */
//     switch(AccelFsRange) {
//     case 0:
//         AccelLsbSens = 16384.0;
//         break;
//     case 1:
//         AccelLsbSens = 8192.0;
//         break;
//     case 2:
//         AccelLsbSens = 4096.0;
//         break;
//     case 3:
//         AccelLsbSens = 2048.0;
//         break;  
//     };
// 
// }
// 
// void lesSensorer() {
// 
//     int16_t ax, ay, az;
//     int16_t gx, gy, gz;
//     int16_t mx, my, mz;
// 
//     accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
// 
//     //Accelerometerdata enhet: [m/s^2]
//     imu_raw_msg.linear_acceleration.x = (ax * STANDARD_GRAVITY) / AccelLsbSens; // OBS! MÅ VÆRE m/s^2!
//     imu_raw_msg.linear_acceleration.y = (ay * STANDARD_GRAVITY) / AccelLsbSens; // OBS! MÅ VÆRE m/s^2!
//     imu_raw_msg.linear_acceleration.z = (az * STANDARD_GRAVITY) / AccelLsbSens; // OBS! MÅ VÆRE m/s^2!
// 
//     //Gyrodata: enhet [rad/s]
//     imu_raw_msg.angular_velocity.x = (gx * RAD_PER_DEG) / GyroLsbSens; // OBS! MÅ VÆRE RAD/SEC
//     imu_raw_msg.angular_velocity.y = (gy * RAD_PER_DEG) / GyroLsbSens; // OBS! MÅ VÆRE RAD/SEC
//     imu_raw_msg.angular_velocity.z = (gz * RAD_PER_DEG) / GyroLsbSens; // OBS! MÅ VÆRE RAD/SEC
// 
//     // Kompass, enhet [T]
//     // TODO finn ut om 0.3 er riktig skaleringsfaktor
//     compass_msg.magnetic_field.x = mx * 0.3 * MICROTESLA_PER_TESLA; // OBS! MÅ VÆRE TESLA! (IKKE MILLI/MICRO)
//     compass_msg.magnetic_field.y = my * 0.3 * MICROTESLA_PER_TESLA; // OBS! MÅ VÆRE TESLA! (IKKE MILLI/MICRO)
//     compass_msg.magnetic_field.z = mz * 0.3 * MICROTESLA_PER_TESLA; // OBS! MÅ VÆRE TESLA! (IKKE MILLI/MICRO)
// 
//     temperature_msg.temperature = ( (double)accelgyro.getTemperature() + 12412.0) / 340.0;
// 
//     depthSensor.read();
//     pressure_msg.fluid_pressure = depthSensor.getPreassure() * PASCAL_PER_MILLIBAR; // OBS! MÅ VÆRE I PASCAL
// 
// }
// 
