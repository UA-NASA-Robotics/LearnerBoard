#include "macros.h"
#include <stdbool.h>
#include <stdlib.h>
#include "STD_Methods.h"
#include "CAN_Lib/motor.h"
#include "comDefs.h"
#include "gyro.h"
#include "CAN_Lib/CANFastTransfer.h"
#include "PID.h"
#include "main.h"
#include "interruptHandler.h"
#include "UART_handler.h"
#include "CAN_Lib/motor.h"
#include "../Common/CommsDefenition.h"



//this is the value that will be used to compare the desired turn radius 
//with the angle aquired from the gyros
#define ROTATION_ANGLE_TOLERANCE 2
#define DRIVE_DIST_ROTATIONAL_TOLERANCE 5

#define Straight            0
#define Backwards           1
#define Right               2
#define Left                3

#define MOTOR_COM_TURN       4
#define MOTOR_COM_DRIVE      6
#define MOTOR_SQUARE_DRIVE   7

//PID definitions
#define MOTOR_ROTATION_kp    1
#define MOTOR_ROTATION_ki    0
#define MOTOR_ROTATION_kd    0

#define MOTOR_MAX_SPEED 500
#define MOTOR_MIN_SPEED 20

bool navigationDone             = true;
int performNavigationCommand    = 0;
int navigationCommandData       = 0;
int PauseData                   = 0;
bool PauseDataUpdated           = false;
bool macroRunning               = false;
int macroAngle                  = 0;
int lastDir                     = 0;
int lastSpeed                   = 0;
float StartingAngle;
PID_Struct_t motorPID;
float localAngle                = 0;
float localAngleTwo             = 0;
float lastAngle                 = 0;
float lastAngleTwo              = 0;
unsigned long lastMillis        = 0;
void turnDegrees(float _degrees)
{
    localAngle = 0;
    lastAngle = 0;
    LED8 = 1;
    
    lastDir = 0;
    lastSpeed = 0;
    lastMillis = 0;
   

    lastAngle = get_ZAngle();
    INIT_PID(&motorPID, _degrees,MOTOR_ROTATION_kp, MOTOR_ROTATION_ki, MOTOR_ROTATION_kd);
    while(isInRange(localAngle,_degrees,ROTATION_ANGLE_TOLERANCE) == false && getPerformNavigationCommand() != 0)
    {
        
        if((millis()-lastMillis) >= 1)
        {
           // MessageParsser(0,0,localAngle);
           
            moveMotors(_degrees,localAngle);
            updateMacroCommunications();
            
            lastMillis = millis();
        }
        //update the gyro data
        if(getGData() && lastAngle != get_ZAngle())
        {
            localAngle += get_ZAngle() - lastAngle;
            lastAngle = get_ZAngle();
        }
        
    }
    setMotor_Vel(0,0);
    //if we are not running the motor macro we will want to stop the motors and 
    if(getPerformNavigationCommand() != MOTOR_COM_DRIVE)
    {
        setPerformNavigationCommand(0);
        MacroModeComplete();
        //LED7 ^= 1;
    } 
    
    LED8 = 0;
    //TODO: we need to keep track of the angles that are discarded as a result of the tolerance values(we are not turning back to zero and need to tally those values)
}
//here dist will need to be in mills
void travelDistance(void)
{
    LED7 = 1;
    StartMotorEncoderMode();
    localAngleTwo  = 0;
    lastAngleTwo  = 0;
    lastMillis = 0;
    //We are going to let the bumper board take this command and we will only interrupt 
    StartingAngle = get_ZAngle();
    lastAngleTwo  = get_ZAngle();
    
    while(getPerformNavigationCommand() == MOTOR_COM_DRIVE)
    { 
        if((millis()-lastMillis) >= 10)
        {
            updateMacroCommunications();
            //Get the Encoder data
            lastMillis = millis(); 
        } 
        //update the gyro data
        getGData();
        localAngleTwo  += get_ZAngle() - lastAngleTwo ;
        lastAngleTwo  = get_ZAngle();

            //if we are veering off in one direction
        if(!isInRange(localAngleTwo ,0,DRIVE_DIST_ROTATIONAL_TOLERANCE))
        {
            //TODO: the localAngle is not being updated after returning from the turnDegrees function. FIX!!!!
            LED1 ^= 1;
            //pause the motor 
            CAN_PauseMotor();
            //while we wait for the robot to rotate to correct the drift
            delayMS(100);
            stopNavigationCommand();
            turnDegrees(-localAngleTwo );
            delayMS(100);
            int dist = CAN_getAVGDistTraveled();
            delayMS(10);
            CAN_ResumeMotor(dist);
        }
        
        
    }
    
    stopNavigationCommand();
    StopMotorCommand();
    MacroModeComplete();
    LED7 = 0;
}
void StartMotorEncoderMode()
{
    //Start the Right motors
    ToSendCAN(MOTOR_COMMAND_MODE_DATA_ADDRESS, ENCODER_DISTANCE_MODE);
    ToSendCAN(MOTOR_ENCODER_DISTANCE_DATA_ADDRESS, (unsigned int)getNavigationCommandData()); 
    sendDataCAN(BumperSwitchRight);
    delayMS(1);
    //Start the Left motors
    ToSendCAN(MOTOR_COMMAND_MODE_DATA_ADDRESS, ENCODER_DISTANCE_MODE);
    ToSendCAN(MOTOR_ENCODER_DISTANCE_DATA_ADDRESS, (unsigned int)getNavigationCommandData());
    sendDataCAN(BumperSwitchLeft);
    
}
void StopMotorCommand(void)
{
    //Stop the Right motors(drag of any macro they may be in)
    ToSendCAN(MOTOR_COMMAND_MODE_DATA_ADDRESS, 0);
    sendDataCAN(BumperSwitchRight);
    delayMS(1);
    //Stop the Left motors
    ToSendCAN(MOTOR_COMMAND_MODE_DATA_ADDRESS, 0);
    sendDataCAN(BumperSwitchLeft);
    
}
int CAN_getAVGDistTraveled()
{
    int leftDist,rightDist;
    PauseDataUpdated = false;
    ToSendCAN(MOTOR_PAUSE_COMMAND_ADDRESS, REQUEST_ENCODER_VAL);
    sendDataCAN( BumperSwitchRight);
    while(PauseDataUpdated == false)
    {
        delayMS(1);
        checkCANFTdata();
        
    }
    rightDist = getPauseData();
    PauseDataUpdated = false;
    ToSendCAN(MOTOR_PAUSE_COMMAND_ADDRESS, REQUEST_ENCODER_VAL);
    sendDataCAN( BumperSwitchLeft);
    while(PauseDataUpdated == false)
    {
        delayMS(1);
        checkCANFTdata();
        
    }
    leftDist = getPauseData();
    double AVGdist = ((double)leftDist + (double)rightDist) / 2.0;
    return AVGdist;
    
}
void MacroModeComplete()
{
    ToSendCAN(MACRO_FINALIZED_COMMAND_INDEX, 0);
    sendDataCAN(SensorAddress);
}

void CAN_PauseMotor()
{  
    ToSendCAN(MOTOR_COMMAND_MODE_DATA_ADDRESS, ENCODER_DISTANCE_MODE);
    ToSendCAN(MOTOR_PAUSE_COMMAND_ADDRESS, PAUSE);
    sendDataCAN( BumperSwitchRight);
    delayMS(1);
    ToSendCAN(MOTOR_COMMAND_MODE_DATA_ADDRESS, ENCODER_DISTANCE_MODE);
    ToSendCAN(MOTOR_PAUSE_COMMAND_ADDRESS, PAUSE);
    sendDataCAN(BumperSwitchLeft);
}
void CAN_ResumeMotor(int _newDist)
{  
   
    ToSendCAN(MOTOR_COMMAND_MODE_DATA_ADDRESS, ENCODER_DISTANCE_MODE);
    ToSendCAN(MOTOR_PAUSE_COMMAND_ADDRESS, RESUME);
    ToSendCAN(MOTOR_PAUSE_DATA_ADDRESS, _newDist);
    sendDataCAN( BumperSwitchRight);
    delayMS(1);
    ToSendCAN(MOTOR_COMMAND_MODE_DATA_ADDRESS, ENCODER_DISTANCE_MODE);
    ToSendCAN(MOTOR_PAUSE_COMMAND_ADDRESS, RESUME);
    ToSendCAN(MOTOR_PAUSE_DATA_ADDRESS, _newDist);
    sendDataCAN(BumperSwitchLeft);
}

int getPerformNavigationCommand(void)
{
    return performNavigationCommand;
}
int getNavigationCommandData(void)
{
    return navigationCommandData;
}
int getPauseData()
{
    return PauseData;
}

void setPerformNavigationCommand(int c)
{
    performNavigationCommand=c;
}
 
void setNavigationCommandData(int d)
{
    navigationCommandData = d;
}
void setPauseData(int d)
{
    PauseData = d;
    PauseDataUpdated = true;
}

void finishNavigationCommand(void)
{
    
}

void updateMotors(int _Dir,float updatedAngle)
{
    int speed = 0;
    speed = updateOutput(&motorPID, updatedAngle);   
    
    if(abs(speed) < MOTOR_MIN_SPEED)
        speed = (speed > 0 ? 1: -1)*MOTOR_MIN_SPEED;
    if(abs(speed) > MOTOR_MAX_SPEED)
        speed = (speed > 0 ? 1: -1)*MOTOR_MAX_SPEED;

//        lastDir = _Dir;
//        lastSpeed = speed;
        setMotor_Vel(-speed,speed);

}
void moveMotors(float _degrees,float currentAngle)
{
    if(_degrees > get_ZAngle())
    {
       updateMotors(Left,currentAngle); 
    }
    else
    {
        updateMotors(Right,currentAngle); 
    }
}

void doNavigationCommand(void)
{
   LED5 = 0;
   if(!macroRunning)
   {
       switch(getPerformNavigationCommand())
       {
            case MOTOR_COM_TURN:
                if(getNavigationCommandData() != 0)
                {
                    macroRunning=true;
                    turnDegrees((int)getNavigationCommandData());
                }
                break;
            case MOTOR_COM_DRIVE:
                macroRunning=true;
                travelDistance();
                break;
        }
   }
}

void stopNavigationCommand(void)
{
   LED5=1;
   if(macroRunning)
   {
        macroRunning=false;
        StopMotorCommand();
   }
}

void updateMacroCommunications(void)
{
    //Check communications from the router for a cancel command
    checkCANFTdata();
    if(getPerformNavigationCommand()==0)
    {
       stopNavigationCommand();
    }
}


void stopMotors(void)
{
    setMotor_Vel(0,0);  
}
 