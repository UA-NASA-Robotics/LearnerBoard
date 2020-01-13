
/* 
 * File:   
 * Author: Seth Carpenter 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef GYRO_H
#define	GYRO_H


#define acel_deadzone 8   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
#define gyro_deadzone 2     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
#define GYROACCEL_1_ADDRESS_H 0x69
#define GYROACCEL_1_ADDRESS_L 0x68

//AXIS FOR THE GYRO AND ACCl
#define MPU6050_RA_ACCEL_XOUT_H    0x3B   // R  
#define MPU6050_ACCEL_XOUT_L       0x3C   // R
#define MPU6050_ACCEL_YOUT_H       0x3D   // R  
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R 
#define MPU6050_GYRO_XOUT_H        0x43   // R  
#define MPU6050_GYRO_YOUT_H        0x45   // R  
#define MPU6050_GYRO_ZOUT_H        0x47   // R  

//OFFSET ADDRESSES
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18

//Full Scale Range Addressing
#define MPU6050_GYRO_FS_250         0b00000000
#define MPU6050_GYRO_FS_500         0b00001000
#define MPU6050_GYRO_FS_1000        0b00010000
#define MPU6050_GYRO_FS_2000        0b00011000

#define MPU6050_ACCEL_FS_2          0b00000000
#define MPU6050_ACCEL_FS_4          0b00001000
#define MPU6050_ACCEL_FS_8          0b00010000
#define MPU6050_ACCEL_FS_16         0b00011000

#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C

#define MPU6050_PWR1_SLEEP_BIT         6
#define MPU6050_RA_PWR_MGMT_1       0x6B

#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3

#define IMPACT_DEADZONE_H  3000
#define IMPACT_DEADZONE_L  3000

#define IMPACT_RING_BUFF_SIZE 200


  /*forwardAxis Value Number scheme(ACCELEROMETER ORIENTATION)
  0 = X is Front, Y is Sides, and Z is Up
  1 = X is Front, Z is Sides, and Y is Up
  2 = Y is Front, X is Sides, and Z is Up
  3 = Y is Front, Z is Sides, and X is Up
  4 = Z is Front, Y is Sides, and X is Up
  5 = Z is Front, X is Sides, and Y is Up
*/ 

extern short forwardAxis;
extern int rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ;       //stores the raw data values from the gyro/accel
extern int mocAX, mocAY, mocAZ, mocGX, mocGY, mocGZ; 
extern long ax_average , ay_average, az_average, gx_average,	gy_average, gz_average;
extern float angleX[2], angleY[2], angleZ[2];
extern long base_x_gyro, base_y_gyro, base_z_gyro;


long get_RawGyro();
//initialize: Must by called before calling any other function 
//this will calibrate and verify that there is a connection to the mpu
bool GYRO_initialize(int address);
//averages 1024 values from the mpu throwing out an additional first 100
void meanAcclGyro();
//Calibrates the gyro and accel (non-specific)
void calibrate();
void getData6();            //Retrieving the data from the I2C device
void getAngles();           //Will calculate the the angles form the gyro
void getAcceleration();     //Will calculate the the acceleration vectors
char getImpactStatus();
void get_Movement_6();
void get_BaseMotion();
void ResetGyro();

//will send a byte through i2c to the mpu's DMP to set the offsets

void setAcclOffset(int, int, int);
void setGyroOffset(int, int, int);
void setFullScaleAccelRange(int);
void setFullScaleGyroRange(int);
void SetSleepEnabled(bool);
void setClockSource(char source);
unsigned char* offsetRegisterSet(int);
void setSampleRate(char);
float get_ZAngle();





//union gyro_union
//{
//    struct
//    {
//        unsigned char gyro_L;
//        unsigned char gyro_H;
//    }
//    reg;
//    struct
//    {
//        int gyroInData;
//    }
//    value;
//};
//extern union gyro_union myData;

#endif	/* XC_HEADER_TEMPLATE_H */

