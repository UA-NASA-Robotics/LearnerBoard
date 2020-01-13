/*
 * IncFile1.h
 *
 * Created: 4/4/2016 8:24:18 PM
 *  Author: reed
 */ 

#ifndef Motor_H_
#define Motor_H_
#include <xc.h>
#include <stdbool.h>
#include "can.h"

typedef struct
{
	unsigned char NodeID;	
	unsigned int ObjIndx;
	unsigned char SubIndx;	
	unsigned long Data; 
}
SDO_packet;

//typedef struct circular_buffer
//{
//	void *buffer;     // data buffer
//	void *buffer_end; // end of data buffer
//	int capacity;  // maximum number of items in the buffer
//	int count;     // number of items in the buffer
//	int sz;        // size of each item in the buffer
//	void *head;       // pointer to head
//	void *tail;       // pointer to tail
//} circular_buffer;

void initMotors(void);
//communication functions to protect comms to motors. 
bool SendandVerify(SDO_packet Packet, circular_buffer* b, int MotorNumber);
bool ReadandVerify(SDO_packet Packet, circular_buffer* b, int MotorNumber, void *item);
long ArrayToLong(char temp[8]);

bool LeftMotor_Status();
bool RightMotor_Status();
bool ConveyorMotor_Status();
bool BucketMotor_Status();
//motor controller interface functions
//initialize functions
//configures controllers, creates over current and over speed protections
void Motor_Init(int motorID); 
void LeftMotor_Init(); 
void RightMotor_Init(); 
void ConveyorMotor_Init(); 
void BucketMotor_Init(); 
//mode functions
//set the type of mode for different motors
//velocity modes
void Motor_VelMode(int motorID);
void LeftMotor_VelMode();
void RightMotor_VelMode();

void ConveyorMotor_VelMode();
void BucketMotor_VelMode();  
//position modes
void Motor_PosMode(int motorID);
void BucketMotor_PosMode(); 
//motor output functions
//sets desired velocity for motors in RPM

void setMotor_Vel(int R_Vel,int L_Vel);
void SetMotorRight_Vel(int vel);
void SetMotorLeft_Vel(int vel);
/*
void Motor_SetVel(int motorID, int Vel);
void LeftMotor_SetVel(int Vel); 
void LeftMotor_SetVelNoCommsSafety(int Vel);
void RightMotor_SetVel(int Vel);
void RightMotor_SetVelNoCommsSafety(int Vel);
 */
void ConveyorMotor_SetVel(int Vel); 
void ConveyorMotor_SetVelNoCommsSafety(int Vel);
void BucketMotor_SetVel(int Vel); 
void BucketMotor_SetVelNoCommsSafety(int Vel);
//move the motors a fixed distance. 
void Motor_MoveCounts(int motorID, int Counts);
void BucketMotor_MoveCounts(long Counts); 
void BucketMotor_MoveCountsNoCommsSafety( long Counts);
//set limits for motors with limit switches attached. 
void BucketMotor_SetLimit(); 
//request data from controllers. 
char LeftMotor_GetTemperature();
char RightMotor_GetTemperature();
char ConveyorMotor_GetTemperature();
char BucketMotor_GetTemperature();
char LeftMotor_GetVoltage(); 
char RightMotor_GetVoltage(); 
char ConveyorMotor_GetVoltage(); 
char BucketMotor_GetVoltage(); 
long LeftMotor_GetPos();
long RightMotor_GetPos();
long ConveyorMotor_GetPos();
long BucketMotor_GetPos();
char BucketMotor_Inputs(); 
void LeftMotor_ReEstablishComms();
void RightMotor_ReEstablishComms();
void ConveyorMotor_ReEstablishComms();
void BucketMotor_ReEstablishComms();


void SDOWritePacketFull(unsigned int NodeID, unsigned int ObjIndx, char SubIndx, unsigned long Data); 
bool SDOWritePacket(SDO_packet SDOpacket);
bool SDOReadRequest(SDO_packet SDOpacket); 
bool SDOVerifyReply(char SDOreply[], SDO_packet SDOsent); 
bool SDOVerifyRead(char SDOreply[], SDO_packet SDOsent);
bool SDO_CAN_tx(CAN_packet * p);


#endif /* Motor_H_ */