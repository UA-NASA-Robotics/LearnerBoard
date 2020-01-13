/*
 * File:   AccelGyro.c
 * Author: Seth Carpenter
 *
 * Created on April 15, 2016, 4:32 PM
 */


#include <xc.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "Initialize.h"

#include "main.h"
#define FOSC    (72000000ULL)
#define FCY     (FOSC/2)
#include <libpic30.h>
#include "gyro.h"
#include "I2C_API_GYRO.h"
#include "UART_handler.h"
#include "comDefs.h"
#include "STD_Methods.h"
//union gyro_union myData;


volatile unsigned int time;
short forwardAxis = 0;
int rawAX,rawAY, rawAZ, rawGX, rawGY, rawGZ;       //stores the raw data values from the gyro/accel
int mocAX, mocAY, mocAZ, mocGX, mocGY, mocGZ; 
long ax_average = 0, ay_average = 0, az_average = 0, gx_average = 0,	gy_average = 0, gz_average = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
long base_x_gyro, base_y_gyro, base_z_gyro;
const double RAD_TO_DEGREE = 57.295779513082320876798154814105;
float angleX[2], angleY[2], angleZ[2];
float accelerationX, accelerationY, accelerationZ;
float frontGrav = 0;
float sideGrav = 0;
float upGrav = 0;
unsigned char DataIn[12];
static unsigned char offSet[2];
int gyroAddress;
int highBoundX[IMPACT_RING_BUFF_SIZE], LowBoundX[IMPACT_RING_BUFF_SIZE];
int highBoundY[IMPACT_RING_BUFF_SIZE], LowBoundY[IMPACT_RING_BUFF_SIZE];
int HighBoundAverage[2] , LowBoundAverage[2] ;
int highHead[2], lowHead[2];
bool SOFTWARE_OFFSET = false;
bool RawValuesOffset = true;    //this bool only takes affect when SOFTWARE_OFFSET is set to TRUE
long rawGyro;


long get_RawGyro()
{
    return rawGyro/127;
}
bool GYRO_initialize(int address)
{
    
    //As of 5/6/2016 the gyro internal offsets DO NOT function 
    SOFTWARE_OFFSET = true;
    RawValuesOffset = false; 
    gyroAddress = address;
    
    //Reset the data paths of the gyro
    ResetGyro();
    while(StatusI2Cone() == PENDING);
    SetSleepEnabled(false); 
    //The seClockSource is set to zero when the sleep register is written to. Meaning The clock source is set to the "internal 8MHz oscillator
    //setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    //__delay_ms(50);
    while(StatusI2Cone() == PENDING);
    setSampleRate((unsigned char) 99);
    while(StatusI2Cone() == PENDING);
    setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    //__delay_ms(50);
    while(!(StatusI2Cone() == SUCCESS || StatusI2Cone() == FAILED));
    setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    
    if(SOFTWARE_OFFSET == false)
    {
        setAcclOffset(0, 0, 0);
        setGyroOffset(0, 0, 0);
    }
    calibrate();
     
    //TODO: setGyroOffset to O and accelOffset to 0
    //TODO: make and is valid function verifying the the gyros are all giving the same reliable data
    //TODO: initializer for the gyro from the mpu sampling sketch
    return 0;
}


void meanGyroAccel()
{
    int iterator = 0;
    const short SAMPLE_SIZE = 100;    //this number is the result of 2 to the power of 10 which allows a bit shift when finding the average
	
	while (iterator < (SAMPLE_SIZE + 201)){
		
		//Get the raw data values from the gyro and accel
        if(getGData())
        {
                  
            //We toss the first 100 reading
            if (iterator > 100 && iterator <= (SAMPLE_SIZE + 200)){
                //adding the the next 1024 raw data values to the variable so they can eventually be averaged 
                ax_average += rawAX;
                ay_average += rawAY;
                az_average += rawAZ;

                gx_average += rawGX;
                gy_average += rawGY;
                gz_average += rawGZ;

            }
            if (iterator == (SAMPLE_SIZE + 200))
            {
                //Averaging the values....
                ax_average /= SAMPLE_SIZE;
                ay_average /= SAMPLE_SIZE;
                az_average /= SAMPLE_SIZE;

                gx_average /= SAMPLE_SIZE;
                gy_average /= SAMPLE_SIZE;
                gz_average /= SAMPLE_SIZE;
            }	
            iterator++;
        }
		__delay_ms(1);
    }
     
}

void calibrate()
{
  	/*We start off with sending a guess offset value to the mpu6050. (the averaged value divided by 8)
	* Then we take another set of averaged raw data values and look to
	* see how close we get to the dead zone(to zero within a set amount).
	*
	*/
	//getmotionData();
	get_Movement_6();
    
    
    meanGyroAccel();
    if(SOFTWARE_OFFSET == true)//checking to see if we are using the gyro's offset functionality or a software contained one
    {
        ax_offset = -ax_average;
        ay_offset = -ay_average;
        az_offset = -az_average + 16384;

        gx_offset = -gx_average;
        gy_offset = -gy_average;
        gz_offset = -gz_average;
        
    }
    else
    {
        //this is divided by 8 to be sure that we don't over shoot the zero by to far
        ax_offset = -ax_average / 8;
        ay_offset = -ay_average / 8;
        az_offset = -az_average / 8;

        gx_offset = -gx_average / 4;
        gy_offset = -gy_average / 4;
        gz_offset = -gz_average / 4;

        while (1){
            //#IMPORTANT
            //setAcclOffset(ax_offset, ay_offset, az_offset);

            //setGyroOffset(gx_offset, gy_offset, gz_offset);

            meanGyroAccel();
            __delay_ms(2);

            int ifReady = 0;

            if (abs(ax_average) <= acel_deadzone)
                ifReady++;
            else ax_offset -= ax_average / acel_deadzone;


            if (abs(ay_average) <= acel_deadzone)
                ifReady++;
            else ay_offset = ay_offset - ay_average/ acel_deadzone;


            if (abs(az_average) <= acel_deadzone)
                ifReady++;
            else az_offset = az_offset +(16384 - az_average) / acel_deadzone;


            if (abs(gx_average) <= gyro_deadzone)
                ifReady++;
            else gx_offset -= gx_average / gyro_deadzone;


            if (abs(gy_average) <= gyro_deadzone)
                ifReady++;
            else gy_offset -= gy_average / gyro_deadzone;


            if (abs(gz_average) <= gyro_deadzone)
                ifReady++;
            else gz_offset -= gz_average / gyro_deadzone;


            if (ifReady == 6) break;
        }
    }
}

extern struct UART_ring_buff GYRO_buffer;
void getAngles()
{
    
	//Finding the accelerometer angle relative to gravity
	float accel_angle_x = atan2(mocAX, mocAZ)*RAD_TO_DEGREE;
	float accel_angle_y = atan2(mocAY, mocAZ)*RAD_TO_DEGREE;

	//Stabilizes the data and zeros it(depending on the accuracy of the initial calibration this may not be necessary(?)) 
	//gyro_axis = (gyroData - (Calibrated Value When Motionless(average of first 20 values:see - Calibrate_Sensor()) / (Sensitivity Scale Factor);
	//rawGyro=((rawGyro*2)+((mocGZ-base_z_gyro)))/3;
    int gyro_x = (mocGX - base_x_gyro) / 800;
	int gyro_y = ((mocGY - base_y_gyro) / 131) * -1;
	int gyro_z = (mocGZ - base_z_gyro) /  127;//84.5;
    
	float dt = 0.01;
	//looking at the change in rotation and adds it the last angle value
	float gyro_angle_x = gyro_x*dt + angleX[1];
	float gyro_angle_y = gyro_y*dt + angleY[1];
	float gyro_angle_z = gyro_z*dt + angleZ[1];
    

	//Low_Pass_Filter/complementary 
	//It is important the the accel_angle_<axis> is mixed on the OPPOSITE Gyro axis. 
	//Because When you rotate around an axis of the gyro you are changing the angle of the accelerometer on the opposite axis.(GyroX to AccelY)(GyroY to AccelX)

	float beta = 0.94;
	angleX[0]= beta*gyro_angle_x + (1.0 - beta)*accel_angle_y;
	angleY[0] = beta*gyro_angle_y + (1.0 - beta)*accel_angle_x;
	angleZ[0] = gyro_angle_z;  //Accelerometer doesn't give z-angle
//    if(gyro_angle_y>0)
//    {
//        LED4=LED_ON;
//    }
//    else
//    {
//        LED4=LED_OFF;
//    }
//    if(gyro_angle_x>0)
//    {
//        LED5=LED_ON;
//    }
//    else
//    {
//        LED5=LED_OFF;
//    }
    if(gyro_angle_z>0)
    {
        LED6=LED_ON;
    }
    else
    {
        LED6=LED_OFF;
    }
	angleX[1] = angleX[0];
	angleY[1] = angleY[0];
	angleZ[1] = angleZ[0];
    
    
    //MessageParsser(angleX[0],angleY[0],angleZ[0]);
    //LED8 = 1;

}
float get_ZAngle()
{
    return angleZ[0];
}
void getAcceleration()
{
     //A global variable named frontGrav, sideGrav, upGrav need to be made
    const float alpha = 0.1;
    accelerationX = mocAX;
    accelerationY = mocAY;
    accelerationZ = mocAZ;
    
    //Implementing a simple lowpass filter to remove the gravity vectors
    frontGrav = alpha * frontGrav + (1 - alpha) * accelerationX;
        //Subtracting the gravity from the Accelerometer data
    accelerationX = accelerationX - frontGrav;
    
    sideGrav = alpha * sideGrav + (1 - alpha) * accelerationY;
    accelerationY = accelerationY - sideGrav;
    
    upGrav = alpha * upGrav + (1 - alpha) * accelerationZ;
    accelerationZ = accelerationZ - upGrav;
    getImpactStatus();
}




char getImpactStatus()
{
    //INFO: Ring Buffers In use
    //For the arrays the 0th element means X and 1st element means Y axis
    if(accelerationX > 0)
    {
        if((HighBoundAverage[0] + IMPACT_DEADZONE_H) < accelerationX)
        {
//            LATGbits.LATG7 = 0;
//            __delay_ms(300);
//            LATGbits.LATG7 = 1;
            return 1;
        }
        
        highBoundX[highHead[0]] = accelerationX;
        highHead[0]++;
        HighBoundAverage[0] /= IMPACT_RING_BUFF_SIZE;
        if(highHead[0] >= IMPACT_RING_BUFF_SIZE)
        {
            highHead[0] = 0;
        }
    }
    else if(accelerationX < 0)
    {  
        if((LowBoundAverage[0] - IMPACT_DEADZONE_L) > accelerationX)
        {
//            int i;
//            for( i = 0; i < 2; i++)
//            {
//                LATGbits.LATG7 = 0;
//                __delay_ms(300);
//                LATGbits.LATG7 = 1;
//                __delay_ms(300);
//            };
//            return 2;
        }
        
        LowBoundX[lowHead[0]] = accelerationX;
        lowHead[0]++;
        LowBoundAverage[0] /= IMPACT_RING_BUFF_SIZE;
        if(lowHead[0] >= IMPACT_RING_BUFF_SIZE)
            {
            lowHead[0] = 0;
        } 
    }
    if(accelerationY > 0)
    {
        if((HighBoundAverage[1] + IMPACT_DEADZONE_H) < accelerationY)
        {
//            int i;
//             for(i = 0; i < 3; i++)
//            {
//                    LATGbits.LATG7 = 0;
//                    __delay_ms(300);
//                    LATGbits.LATG7 = 1;
//                    __delay_ms(300);
//            }
            return 3;
        }
        highBoundY[highHead[1]] = accelerationY;
        highHead[1]++;
        HighBoundAverage[0] /= IMPACT_RING_BUFF_SIZE;
        if(highHead[1] >= IMPACT_RING_BUFF_SIZE)
        {
            highHead[1] = 0;
        }
    }
    else if(accelerationY < 0)
    {
        if((LowBoundAverage[1] - IMPACT_DEADZONE_L) > accelerationY)
        {
//            int i;
//            for(i = 0; i < 4; i++)
//            {
//                    LATGbits.LATG7 = 0;
//                    __delay_ms(300);
//                    LATGbits.LATG7 = 1;
//                    __delay_ms(300);
//            }
//        
            return 4;
        }
        LowBoundX[lowHead[1]] = accelerationY;
        lowHead[1]++;
        LowBoundAverage[1] /= IMPACT_RING_BUFF_SIZE;
        if(lowHead[1] >= IMPACT_RING_BUFF_SIZE)
        {
            lowHead[1] = 0;
        }            

    }
    
    return false;
}
void get_BaseMotion()
{
    int AverageNumber = 0;
    bool wasOn = SOFTWARE_OFFSET;
    //Averaging the values of the gyro to find the offset to use in finding the angle of rotation
    SOFTWARE_OFFSET = true;
    while(AverageNumber < 15)
    {
        //get_Movement_6();
        if(getGData())
        {
             AverageNumber++;
        
            //Adding the next 15 data points to base_x_gyro 

            base_x_gyro += mocGX;   //IMPORTANT: if the orientation of the gyro vectors is changed with the forwardAxis variable these variables will not change accordingly
            base_y_gyro += mocGY;   //if not addressed may give bad angle data
            base_z_gyro += mocGZ;
        }
        
    }
      base_x_gyro /= 15;
      base_y_gyro /= 15;
      base_z_gyro /= 15;
      if(wasOn)SOFTWARE_OFFSET = true;
}




void get_Movement_6()
{
    //read raw accel/gyro measurements from device
    //Changing the values of ax, ay and az according to the orientation of the Accel and gyro chip&myData.reg
    
    if(ReceiveI2Cone(gyroAddress, MPU6050_RA_ACCEL_XOUT_H,DataIn , 6))
    {
        //LATEbits.LATE6 ^= 1;
        //__delay_ms(20);
        //while(get_dataIndex() < 6);
        time=0;
        while(!(StatusI2Cone() == SUCCESS || StatusI2Cone() == FAILED) && time<20);
        //bit shifting the dataIn[0] over 8 places and appending the data in DataIn[1]
        //tmpData[0] = myData.value.gyroInData;
        rawAX = (DataIn[0] << 8) | DataIn[1];   //AX
        rawAY = (DataIn[2] << 8) | DataIn[3];   //AY
        rawAZ = (DataIn[4] << 8) | DataIn[5];   //AZ
        
        if(SOFTWARE_OFFSET == true && RawValuesOffset == true )
        {
            rawAX += ax_offset;   //AXs
            rawAY += ay_offset;   //AY
            rawAZ += az_offset;   //AZ
        }
    }
    time=0;
    while(!(StatusI2Cone() == SUCCESS || StatusI2Cone() == FAILED) && time<20);
    if(ReceiveI2Cone(gyroAddress, MPU6050_GYRO_XOUT_H,DataIn , 6))
    {
        
        time=0;
        //Waiting till the status of the I2C bus is available
        while(!(StatusI2Cone() == SUCCESS || StatusI2Cone() == FAILED)&& time<20);
        //bit shifting the dataIn[0] over 8 places and appending the data in DataIn[1]
        rawGX = (DataIn[0] << 8) | DataIn[1];   //GX
        rawGY = (DataIn[2] << 8) | DataIn[3];   //GY  
        rawGZ = (DataIn[4] << 8) | DataIn[5];   //GZ
        
        if(SOFTWARE_OFFSET == true && RawValuesOffset == true )
        {
            rawGX += ax_offset;   //AXs
            rawGY += ay_offset;   //AY
            rawGZ += az_offset;   //AZ
        }
            
    }
    


 
    //TODO: Change the values of the axis variable to accommodate the orientation of the chips
    
 switch (forwardAxis) {
      case 0:
        //accelgyro.getMotion6(&rawAX, &ay, &az, &gx, &gy, &gz);
          //TODO: EXP
        if(SOFTWARE_OFFSET == true )
        {
            mocAX = rawAX + ax_offset;
            mocAY = rawAY + ay_offset;
            mocAZ = rawAZ + az_offset;
            mocGX = rawGX + gx_offset;
            mocGY = rawGY + gy_offset;
            mocGZ = rawGZ + gz_offset;
        }
        else
        {
            mocAX = rawAX;
            mocAY = rawAY;
            mocAZ = rawAZ;
            mocGX = rawGX;
            mocGY = rawGY;
            mocGZ = rawGZ;
        }
        break;
      
      case 1:
        //accelgyro.getMotion6(&ax, &az, &ay, &gx, &gz, &gy);
        if(SOFTWARE_OFFSET == true )
        {
            mocAX = rawAX + ax_offset;
            mocAY = rawAZ + az_offset;
            mocAZ = rawAY + ay_offset;
            mocGX = rawGX + gx_offset;
            mocGY = rawGZ + gz_offset;
            mocGZ = rawGY + gy_offset;
        }
        else
        {
            mocAX = rawAX;
            mocAY = rawAZ;
            mocAZ = rawAY;
            mocGX = rawGX;
            mocGY = rawGZ;
            mocGZ = rawGY;
        }

        break;
      case 2:
         //accelgyro.getMotion6(&ay, &ax, &az, &gy, &gx, &gz);
        if(SOFTWARE_OFFSET == true )
        {
            mocAX = rawAY + ay_offset;
            mocAY = rawAX + ax_offset;
            mocAZ = rawAZ + az_offset;
            mocGX = rawGY + gy_offset;
            mocGY = rawGX + gx_offset;
            mocGZ = rawGZ + gz_offset;
        }
        else
        {
            mocAX = rawAY;
            mocAY = rawAX;
            mocAZ = rawAZ;
            mocGX = rawGY;
            mocGY = rawGX;
            mocGZ = rawGZ;
            
        }
          
        
        break;
      case 3:
         //accelgyro.getMotion6(&ay, &az, &ax, &gy, &gz, &gx);
         if(SOFTWARE_OFFSET == true )
        {
            mocAX = rawAY + ay_offset;
            mocAY = rawAZ + az_offset;
            mocAZ = rawAX + ax_offset;
            mocGX = rawGY + gy_offset;
            mocGY = rawGZ + gz_offset;
            mocGZ = rawGX + gx_offset;
        }
        else
        {
            mocAX = rawAY;
            mocAY = rawAZ;
            mocAZ = rawAX;
            mocGX = rawGY;
            mocGY = rawGZ;
            mocGZ = rawGX;
        }
        break;  
      case 4:
         //accelgyro.getMotion6(&az, &ay, &ax, &gz, &gy, &gx);
        if(SOFTWARE_OFFSET == true )
        {
            mocAX = rawAZ + az_offset;
            mocAY = rawAY + ay_offset;
            mocAZ = rawAX + ax_offset;
            mocGX = rawGZ + gz_offset;
            mocGY = rawGY + gy_offset;
            mocGZ = rawGX + gx_offset;
        }
        else
        {
            mocAX = rawAZ;
            mocAY = rawAY;
            mocAZ = rawAX;
            mocGX = rawGZ;
            mocGY = rawGY;
            mocGZ = rawGX;
        }
        break; 
      case 5:
         //accelgyro.getMotion6(&az, &ax, &ay, &gz, &gx, &gy);
        if(SOFTWARE_OFFSET == true )
        {
            mocAX = rawAZ + az_offset;
            mocAY = rawAX + ax_offset;
            mocAZ = rawAY + ay_offset;
            mocGX = rawGZ + gz_offset;
            mocGY = rawGX + gx_offset;
            mocGZ = rawGY + gy_offset;
        }
        else
        {
            mocAX = rawAZ;
            mocAY = rawAX;
            mocAZ = rawAY;
            mocGX = rawGZ;
            mocGY = rawGX;
            mocGZ = rawGY;
        }
        break;   
     }
  }
unsigned char* offsetRegisterSet(int AxisOff)
{   
    
    offSet[0] = AxisOff >> 8;
    offSet[1] = AxisOff & 0xFF;
    return offSet;
}
void setAcclOffset(int AXoffset, int AYoffset, int AZoffset)
{
    bool XPass = true, YPass = true, ZPass = true;
   
  
    unsigned char *AccelOffSet;
    
    __delay_ms(20);
    AccelOffSet = offsetRegisterSet(AXoffset);
    if( SendI2Cone(gyroAddress, MPU6050_RA_XA_OFFS_H, AccelOffSet , 2) )
    {
        XPass = true;
    }
    __delay_ms(10);
    AccelOffSet = offsetRegisterSet(AYoffset);
    if( SendI2Cone(gyroAddress, MPU6050_RA_YA_OFFS_H, AccelOffSet , 2))
    {
        YPass = true;
    }
    __delay_ms(10);
    AccelOffSet = offsetRegisterSet(AZoffset);
    if( SendI2Cone(gyroAddress, MPU6050_RA_ZA_OFFS_H, AccelOffSet, 2))
    {
        ZPass = true;
    }
    __delay_ms(20);
    
    //TODO:IF THIS FAILS MORE THEN 3 time?
   //check if the data was sent and received by the slave not just filled the buffer and the I2c line was open
   // }   
   
}
void setGyroOffset(int GXoffset, int GYoffset, int GZoffset)
{
    bool XPass = true, YPass = true, ZPass = true;

    unsigned char *GyroOffSet;

    __delay_ms(20);
    
    GyroOffSet = offsetRegisterSet(GXoffset);
    if( SendI2Cone(gyroAddress, MPU6050_RA_XG_OFFS_USRH, GyroOffSet , 2))
    {
        XPass = true;
    }
    __delay_ms(10);
     GyroOffSet = offsetRegisterSet(GXoffset);
    if( SendI2Cone(gyroAddress, MPU6050_RA_YG_OFFS_USRH, GyroOffSet , 2))
    {
        YPass = true;
    }
    __delay_ms(10);
     GyroOffSet = offsetRegisterSet(GXoffset);
    if( SendI2Cone(gyroAddress, MPU6050_RA_ZG_OFFS_USRH, GyroOffSet , 2))
    {
        ZPass = true;
    }
    __delay_ms(20);

    //TODO:IF THIS FAILS MORE THEN 3 time?
    //check if the data was sent and received by the slave not just filled the buffer and the I2c line was open

}




void setFullScaleAccelRange(int data)
{
    SendI2Cone(gyroAddress, MPU6050_RA_ACCEL_CONFIG,(unsigned char *) &data, 1);
}

void setFullScaleGyroRange(int data)
{
   
    SendI2Cone(gyroAddress, MPU6050_RA_GYRO_CONFIG,(unsigned char *) &data, 1);
}

void SetSleepEnabled(bool isEnabled)
{
//    writeBit(GYROACCEL_1_ADDRESS_L,  MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, isEnabled);
    static unsigned char b = 0b000000001; 
    SendI2Cone(gyroAddress, MPU6050_RA_PWR_MGMT_1,(unsigned char *) &b, 1);
}

void setClockSource(char source) {
    writeBits(gyroAddress, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
}
void ResetGyro() {
    static unsigned char a = 0x68; 
    static unsigned char b = 0b00000110; 
    SendI2Cone(gyroAddress, 0x68,(unsigned char *) &b, 1);
}
void setSampleRate(char Rate)
{
    unsigned char val = 0b01;
    SendI2Cone(0x1A, 19,(unsigned char *) &val, 1);
    while(StatusI2Cone() == PENDING);
    SendI2Cone(gyroAddress, 19,(unsigned char *) &Rate, 1);
}

//********************************
//    Miscellaneous Functions    *
//********************************
//Math function that finds the absolute  value of a number   

int abs(int givenNum)    
{
    if (givenNum > 0)
        return givenNum;
    else 
        return -givenNum;
    
}
  


