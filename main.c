#include <stdbool.h>
#include <xc.h>
#include "main.h"
#include "interruptHandler.h"
#include "CAN_Lib/CANFastTransfer.h"
#include "SPI_lib.h"
#include "I2C_API_GYRO.h"
#include "FastTransfer.h"
#include "Initialize.h"
#include "gyro.h"
#include "UART_handler.h"
#include "comDefs.h"
#include "CAN_Lib/motor.h"
#include "macros.h"


int OperationIndex;
bool processData;

// DSPIC33EP512GP504 Configuration Bit Settings

// 'C' source line config statements

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25           // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = OFF             // PLL Lock Enable bit (Clock switch will not wait for the PLL lock signal.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Mode Select bits (HS Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)
#define FCY 60000000UL

#include <libpic30.h>
//extern struct UART_ring_buff GYRO_buffer;
//int receiveArray[20];
 //static int x = 0;
 //static int y = 0;

void testCAN(void);
void testCANMotor(void);
int main(void)
{
    __delay_ms(50);
    Start_Initialization();
    LED1 = LED_OFF;
    __delay_ms(100);
    LED2 = LED_OFF;
    __delay_ms(100);
    InitI2Cone();           //initializes the I2c
    LED3 = LED_OFF;
    __delay_ms(100);
    LED4 = LED_OFF;
    __delay_ms(100);
    GYRO_initialize(GYROACCEL_1_ADDRESS_H);
    LED5 = LED_OFF;
    __delay_ms(100);
    LED6 = LED_OFF;
    __delay_ms(100);
    get_BaseMotion();           //getting the average values from the gyro and accel
    LED7 = LED_OFF;
    __delay_ms(100);
    LED8 = LED_OFF;
    __delay_ms(100);
    getGData();
    LED1 = LED_ON;
    LED2 = LED_ON;
    LED3 = LED_ON;
    LED4 = LED_ON;
    LED5 = LED_ON;
    LED6 = LED_ON;
    LED7 = LED_ON;
    LED8 = LED_ON;
   
    while(1)
    {
//        static int x = 0;
//        count++;
        getGData();

  
 
        
        checkCANFTdata();
        if(getPerformNavigationCommand()>0)
        {
            
            doNavigationCommand();
        }        
        else
        {
           stopNavigationCommand();
           //StopMotorCommand();)
        }
         
        __delay_ms(1);
        if(processData == true)
        {
            processDataRequest();
            processData = false;
        }

    }
}
volatile bool RUN_G_DATA;
bool getGData()
{
    if(RUN_G_DATA == true)
    {
        get_Movement_6();       //getting the data points from the GYRO
        getAcceleration();      //Calculating the acceleration vectors
        getAngles();            //Calculating the angles on the z, y and x axis
        RUN_G_DATA = false;
        //LED8 ^= 1;
        return true;
    }
    return false;
}
void getMouseData()
{
    static int x = 0;
    static int y = 0;
    
    //x = ConvertFromTwos(Read_MouseSensor(0x03)) + x;
    y = ConvertFromTwos(Read_MouseSensor(0x02)) + y;
    //U1TXREG = x;
}

void delay_ms(int _Val)
{
    __delay_ms(_Val);
}

void delay_us(int _Val)
{
    __delay_us(_Val);
}

char ConvertFromTwos(char x)
{
 if(x > 127) {
  return 0xFF00 + x; 
 }
  else {
   return x;
  } 
}


void setOperationIndex(int *receiveArrayAddress)
{
    //OperationIndex = receiveArrayAddress[RECEIVE_OPERATOIN_INDEX];
}
void processDataRequest()
{
    //for these transmission we will send them at incremented values of time
    switch (OperationIndex)
    {
        case 0:
            //do nothing
            break;
        case 1:
            //send the Rotation Angle
            ToSend(TRANS_Z_ANGLE,angleZ[0],&RingBuff);
            break;
        case 2:
            //send the elapsed distance traveled
            break;
        case 3: 
            //send both the rotation angle and distance traveled
            ToSend(TRANS_Z_ANGLE,angleZ[0],&RingBuff);
            break;
        
    }
    sendData(FASTTRANS_HOST_ADDRESS, &RingBuff);
}

void testCAN(void)
{
   ToSendCAN(0,1);
   ToSendCAN(1,0);
   ToSendCAN(2,2);
   sendDataCAN(ROUTER_ADDRESS);
    
}
void delayMS(int val)
{
    __delay_ms(val);
}