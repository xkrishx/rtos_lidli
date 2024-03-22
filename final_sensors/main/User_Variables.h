#ifndef USER_VARIABLES_H
#define USER_VARIABLES_H

//WiFi Provisioning Mgr Variables




// MQTT connection status flag



// LittleFS  Variables 


//Sensor File name


//FREERTOS handles 

//Task handles 

// Event Group Handles

//Wifi event group handle 


//Queue Handles 


//Mutex Handles


//Simple NTP configs
         //+5hrs and 30 mins is Indian Standard Time (5.5) set to 0 for UTC (default)

// MQTT handles

//POLLING PERIODS

//Global I2C device handles
// i2c_dev_t i2c_ds3231;
// i2c_dev_t i2c_mpu6050;
 
//Global sensor handles 
static mpu6050_handle_t mpu6050_dev = NULL;

//Global sensor values
static mpu6050_acce_value_t acce;
static mpu6050_gyro_value_t gyro;
static complimentary_angle_t complimentary_angle;
long   rtc_time;
//long   distance;

//Global Sensor status flags
bool MPU_FAIL = false;



// Mac Address 


#endif 