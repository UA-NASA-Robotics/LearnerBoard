
/*
 * Motor.c
 *
 * Created: 4/4/2016 8:24:43 PM
 *  Author: Zac
 */
#include "motor.h"
#include "../comDefs.h"
#include "assert.h"
#include "CANFastTransfer.h"
#include "../../Common/CommsDefenition.h"



#define CAN_PACKET_SIZE 8
#define RECEIVE_MASK 0x7ff

#define BUCKETMOTORID 0x7F
#define BUCKETMOTOR_MOB 3
#define LEFTMOTORID 0x7D
#define LEFTMOTOR_MOB 0
#define RIGHTMOTORID 0x7E
#define RIGHTMOTOR_MOB 1
#define CONVEYORMOTORID 0x7C
#define CONVEYORMOTOR_MOB 2

//safety constants for motor controllers
//strictly speaking the motor controllers should
//self protect, but better safe than sorry.
const int MAXRPM = 3080; //3080 is safe value printed on motors, can probably push this up to 4000 without worry if needed. 
#define ACCEL_CONST 100000 //in rev/min^2, value should be between 100k and 10k *this value will probably have to be changed under load. 

//maximum current allowed through BG65x25
#define MAXCURRENTBG65 20000  //seting current limit to 10A, max is actually 20A. 

//motor status tracking
int MOTORSTATUS = 0xff; //assume all motors are innocent until proven guilty
//from msb to lsb, LeftMotor, RightMotor, ConveyorMotor, BucketMotor
#define  LEFTMOTOR_STATUS 7
#define  RIGHTMOTOR_STATUS 6
#define  CONVEYORMOTOR_STATUS 5
#define  BUCKETMOTOR_STATUS 4


circular_buffer LeftMotor_Buffer;
#define LeftMotor_Buffer_Size 10
circular_buffer RightMotor_Buffer;
#define RightMotor_Buffer_Size 10
circular_buffer ConveyorMotor_Buffer;
#define ConveyorMotor_Buffer_Size 10
circular_buffer BucketMotor_Buffer;
#define BucketMotor_Buffer_Size 10

#define COMMSTIMEOUT 100 //ms, longest delay I have seen between message and reply is 68ms
#define COMMSCHECKTIME 1 //ms, how long between checking comms during timeout period

void initMotors(void)
{
//	LeftMotor_Init();
//	RightMotor_Init();
//	ConveyorMotor_Init();
//	BucketMotor_Init();
//	
//	LeftMotor_VelMode();
//	RightMotor_VelMode();
//	ConveyorMotor_VelMode();
//	//BucketMotor_VelMode();
//	BucketMotor_SetLimit();
//	BucketMotor_PosMode();
}

//communication functions to verify motors are working as intended. 
bool SendandVerify(SDO_packet Packet, circular_buffer* b, int MotorNumber) {
    
	bool status = false;
	//write data to motor
	while(!SDOWritePacket(Packet));

	return status; 
}

//bool ReadandVerify(SDO_packet Packet, circular_buffer* b, int MotorNumber, void *item) {
//	bool status = false;
//	char temp[8];
//	//if motor is already in error skip comms
//	if((MOTORSTATUS & 1<<MotorNumber) == 0) {
//		return false;
//	}
//	//clear buffer of any already received messages (such as power up message).
//	while(cb_size(buffer)>0)
//	{
//		cb_pop_front(buffer, temp);
//	}
//	//write data to motor
//	while(!SDOReadRequest(Packet));
//	//check repeatedly for reply from motor controller
//	for(int i = 0; i<COMMSTIMEOUT/COMMSCHECKTIME; i++) {
//		if(cb_size(buffer)!=0) {
//			
//			cb_pop_front(buffer, temp);
//			if(SDOVerifyRead(temp, Packet)){
//				status = true;
//				memcpy(item, temp, sizeof(temp));
//				break;
//			}
//		}
//		_delay_ms(COMMSCHECKTIME);
//	}
//	//if nothing received before comms timeout declare motor bad.
//	if (!status)
//	{
//		MOTORSTATUS &= ~(1<<MotorNumber);
//	}
//	return status;
//}

long ArrayToLong(char temp[8]) {
	
	long result = temp[4] + ((long)temp[5]<<8) + ((long)temp[6] << 16) + ((long)temp[7] << 24); //turn LSB format into a long
	return result;
}

bool LeftMotor_Status() {
	if((MOTORSTATUS & 1<<LEFTMOTOR_STATUS) == 0) {
		return false;
	} 
	else {
		return true; 
	}
}

bool RightMotor_Status() {
	if((MOTORSTATUS & 1<<RIGHTMOTOR_STATUS) == 0) {
		return false;
	}
	else {
		return true;
	}
}
bool ConveyorMotor_Status() {
	if((MOTORSTATUS & 1<<CONVEYORMOTOR_STATUS) == 0) {
		return false;
	}
	else {
		return true;
	}
}
bool BucketMotor_Status() {
	if((MOTORSTATUS & 1<<BUCKETMOTOR_STATUS) == 0) {
		return false;
	}
	else {
		return true;
	}
}
//
////functions to control motors
//void ReceiveMotor1( CAN_packet *p, unsigned char mob) // interrupt callback
//{
//	cb_push_back(&LeftMotor_Buffer, &p->data);
//
//}
//
//void LeftMotor_Init() {
//	SDO_packet ClearErrors = { LEFTMOTORID, 0x3000, 0x00, 0x01};
//	SDO_packet SetMotor = { LEFTMOTORID, 0x3900, 0x00, 0x01};
//	SDO_packet SetPoles = { LEFTMOTORID, 0x3910, 0x00, 10};
//	SDO_packet MotorPolarity = { LEFTMOTORID, 0x3911, 0x00, 0x02};
//	SDO_packet SetFeedBack = {LEFTMOTORID, 0x3350, 0x00, 2410}; //2410 for encoder feedback, 2378 for hall feedback
//	SDO_packet SetSVELFeedBack = {LEFTMOTORID, 0x3550, 0x00, 2410}; 
//	SDO_packet SetFeedBackResolution = {LEFTMOTORID, 0x3962, 0x00, 2000}; 
//	SDO_packet CurrentLimitPos = {LEFTMOTORID, 0x3221, 0, MAXCURRENTBG65};
//	SDO_packet CurrentLimitNeg = {LEFTMOTORID, 0x3223, 0, MAXCURRENTBG65};
//	SDO_packet VelocityLimitPos = {LEFTMOTORID, 0x3321, 0x00, MAXRPM};
//	SDO_packet VelocityLimitNeg = {LEFTMOTORID, 0x3323, 0x00, MAXRPM}; 
//	SDO_packet PowerEnable = { LEFTMOTORID, 0x3004, 0x00, 0x01};
//	SDO_packet PowerDisable = { LEFTMOTORID, 0x3004, 0x00, 0x00};
//	//initialize circular buffer for left motor
//	cb_init(&LeftMotor_Buffer, LeftMotor_Buffer_Size, CAN_PACKET_SIZE);
//	//prepare RX receiving mob for motor. 
//	bool ret;
//	ret=prepare_rx( LEFTMOTOR_MOB, 0x580 + LEFTMOTORID, RECEIVE_MASK, ReceiveMotor1); //all 0s forces comparison
//
//	//wipe errors. This first attempt at communication also verifies the controller is in the system.
//	SendandVerify(ClearErrors,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	//disable controller so some commands actually work (Feed back commands)
//	SendandVerify(PowerDisable,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	//set resolution of encoder (500 count encoder * 4 for how the controllers work)
//	SendandVerify(SetFeedBack,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	SendandVerify(SetSVELFeedBack,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	SendandVerify(SetFeedBackResolution,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	//configure motor, number of magnet poles and motor type. 
//	SendandVerify(SetMotor,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	SendandVerify(SetPoles,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	//set limits on current on velocity to protect motor
//	SendandVerify(CurrentLimitPos,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	SendandVerify(CurrentLimitNeg,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	SendandVerify(VelocityLimitPos,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	SendandVerify(VelocityLimitNeg,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	//actually critical to get right if you are using encoders 
//	SendandVerify(MotorPolarity,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	//re-enable motor controller. 
//	SendandVerify(PowerEnable,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//}
//
//
//void ReceiveMotor2( CAN_packet *p, unsigned char mob) // interrupt callback
//{
//	cb_push_back(&RightMotor_Buffer, p->data);
//}
//
//void RightMotor_Init() {
//	SDO_packet ClearErrors = { RIGHTMOTORID, 0x3000, 0x00, 0x01};
//	SDO_packet SetMotor = { RIGHTMOTORID, 0x3900, 0x00, 0x01};
//	SDO_packet SetPoles = { RIGHTMOTORID, 0x3910, 0x00, 10};
//	SDO_packet MotorPolarity = { RIGHTMOTORID, 0x3911, 0x00, 0x02};
//	SDO_packet SetFeedBack = {RIGHTMOTORID, 0x3350, 0x00, 2410}; //2410 for encoder feedback, 2378 for hall feedback
//	SDO_packet SetSVELFeedBack = {RIGHTMOTORID, 0x3550, 0x00, 2410};
//	SDO_packet SetFeedBackResolution = {RIGHTMOTORID, 0x3962, 0x00, 2000};
//	SDO_packet CurrentLimitPos = {RIGHTMOTORID, 0x3221, 0, MAXCURRENTBG65};
//	SDO_packet CurrentLimitNeg = {RIGHTMOTORID, 0x3223, 0, MAXCURRENTBG65};
//	SDO_packet VelocityLimitPos = {RIGHTMOTORID, 0x3321, 0x00, MAXRPM};
//	SDO_packet VelocityLimitNeg = {RIGHTMOTORID, 0x3323, 0x00, MAXRPM};
//	SDO_packet PowerEnable = { RIGHTMOTORID, 0x3004, 0x00, 0x01};
//	SDO_packet PowerDisable = { RIGHTMOTORID, 0x3004, 0x00, 0x00};
//	//initialize circular buffer for right motor
//	cb_init(&RightMotor_Buffer, RightMotor_Buffer_Size, CAN_PACKET_SIZE);
//	//prepare RX receiving mob for motor.
//	bool ret;
//	ret=prepare_rx( RIGHTMOTOR_MOB, 0x580 + RIGHTMOTORID, RECEIVE_MASK, ReceiveMotor2); //all 0s forces comparison
//
//	//wipe errors. This first attempt at communication also verifies the controller is in the system.
//	SendandVerify(ClearErrors,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//	//disable controller so some commands actually work (Feed back commands)
//	SendandVerify(PowerDisable,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//	//set resolution of encoder (500 count encoder * 4 for how the controllers work)
//	SendandVerify(SetFeedBack,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//	SendandVerify(SetSVELFeedBack,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//	SendandVerify(SetFeedBackResolution,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//	//configure motor, number of magnet poles and motor type.
//	SendandVerify(SetMotor,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//	SendandVerify(SetPoles,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//	//set limits on current on velocity to protect motor
//	SendandVerify(CurrentLimitPos,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//	SendandVerify(CurrentLimitNeg,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//	SendandVerify(VelocityLimitPos,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//	SendandVerify(VelocityLimitNeg,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//	//actually critical to get right if you are using encoders
//	SendandVerify(MotorPolarity,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//	//re-enable motor controller.
//	SendandVerify(PowerEnable,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//
//}
//
//void ReceiveMotor3( CAN_packet *p, unsigned char mob) // interrupt callback
//{
//	cb_push_back(&ConveyorMotor_Buffer, p->data);
//}
//
//void ConveyorMotor_Init() {
//	SDO_packet ClearErrors = {CONVEYORMOTORID, 0x3000, 0x00, 0x01};
//	SDO_packet SetMotor = {CONVEYORMOTORID, 0x3900, 0x00, 0x00}; //0 for brushed, 1 for brushless
//	SDO_packet CurrentLimitPos = {CONVEYORMOTORID, 0x3221, 0, 10000};
//	SDO_packet CurrentLimitNeg = {CONVEYORMOTORID, 0x3223, 0, 10000};
//	//SDO_packet RatedVoltage = {CONVEYORMOTORID, 0x3902, 0x00, 24000}; //not used, since default is 24000. 
//	SDO_packet RatedSpeed = {CONVEYORMOTORID, 0x3901, 0x00, 129};
//	SDO_packet PowerEnable = {CONVEYORMOTORID, 0x3004, 0x00, 0x01};
//
//	cb_init(&ConveyorMotor_Buffer, ConveyorMotor_Buffer_Size, CAN_PACKET_SIZE);
//
//	bool ret;
//	ret=prepare_rx( CONVEYORMOTOR_MOB, 0x580 + CONVEYORMOTORID, RECEIVE_MASK, ReceiveMotor3); //all 0s forces comparison
//
//
//	SendandVerify(ClearErrors,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
//	SendandVerify(SetMotor,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
//	SendandVerify(CurrentLimitNeg,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
//	SendandVerify(CurrentLimitPos,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
//	SendandVerify(RatedSpeed,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
//	SendandVerify(PowerEnable,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
//	
//	//while(!SDOWritePacket(MotorPolarity));
//	//while(cb_size(&ConveyorMotor_Buffer)==0);
//	//cb_pop_front(&ConveyorMotor_Buffer,temp);
//}
//
//void ReceiveMotor4( CAN_packet *p, unsigned char mob) // interrupt callback
//{
//	cb_push_back(&BucketMotor_Buffer, p->data);
//}
//
//void BucketMotor_Init() {
//	SDO_packet ClearErrors = {BUCKETMOTORID, 0x3000, 0x00, 0x01};
//	SDO_packet SetMotor = {BUCKETMOTORID, 0x3900, 0x00, 1};
//	SDO_packet SetPoles = {BUCKETMOTORID, 0x3910, 0x00, 8};
//	SDO_packet CurrentLimitPos = {BUCKETMOTORID, 0x3221, 0, 10000}; //10A
//	SDO_packet CurrentLimitNeg = {BUCKETMOTORID, 0x3223, 0, 10000}; //10A
//	SDO_packet VelocityLimitPos = {BUCKETMOTORID, 0x3321, 0x00, MAXRPM};
//	SDO_packet VelocityLimitNeg = {BUCKETMOTORID, 0x3323, 0x00, MAXRPM};
//	//SDO_packet MotorPolarity = {BUCKETMOTORID, 0x3911, 0x00, 0x01};
//	SDO_packet SetFeedBack = {BUCKETMOTORID, 0x3350, 0x00, 2410}; //encoder feedback
//	SDO_packet SetEncoderResolution = {BUCKETMOTORID, 0x3962, 0x00, 4096}; //bg75pi has a resolution of 4096 counts/rev
//	SDO_packet PowerEnable = {BUCKETMOTORID, 0x3004, 0x00, 0x01};
//
//	cb_init(&BucketMotor_Buffer, BucketMotor_Buffer_Size, CAN_PACKET_SIZE);
//
//	bool ret;
//	ret=prepare_rx( BUCKETMOTOR_MOB, 0x580 + BUCKETMOTORID, RECEIVE_MASK, ReceiveMotor4); //all 0s forces comparison
//
//
//	SendandVerify(ClearErrors,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(SetMotor,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(SetPoles,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(CurrentLimitNeg,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(CurrentLimitPos,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(VelocityLimitPos,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(VelocityLimitNeg,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(SetFeedBack,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(SetEncoderResolution,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(PowerEnable,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//
//	//while(!SDOWritePacket(MotorPolarity));
//	//while(cb_size(&BucketMotor_Buffer)==0);
//	//cb_pop_front(&BucketMotor_Buffer,temp);
//}
//
//void LeftMotor_VelMode() {
//	SDO_packet ModeVel	 = {LEFTMOTORID, 0x3003, 0x00, 0x3};
//	SDO_packet VEL_Acc = {LEFTMOTORID, 0x3380, 0x00, ACCEL_CONST}; 
//	SDO_packet VEL_Dec = {LEFTMOTORID, 0x3381, 0x00, ACCEL_CONST};
//		
//	SendandVerify(ModeVel,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	SendandVerify(VEL_Acc,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	SendandVerify(VEL_Dec,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//
//}
//
//void RightMotor_VelMode() {
//	SDO_packet ModeVel	 = {RIGHTMOTORID, 0x3003, 0x00, 0x3};
//	SDO_packet VEL_Acc = {RIGHTMOTORID, 0x3380, 0x00, ACCEL_CONST}; //value should be between 100k and 10k
//	SDO_packet VEL_Dec = {RIGHTMOTORID, 0x3381, 0x00, ACCEL_CONST};
//
//	
//	SendandVerify(ModeVel,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//	SendandVerify(VEL_Acc,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//	SendandVerify(VEL_Dec,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
//}
//
//void ConveyorMotor_VelMode() {
//	SDO_packet ModeVel	 = {CONVEYORMOTORID, 0x3003, 0x00, 0x3};
//	SDO_packet VEL_Acc = {CONVEYORMOTORID, 0x3380, 0x00, 10000}; //value should be between 100k and 10k
//	SDO_packet VEL_Dec = {CONVEYORMOTORID, 0x3381, 0x00, 10000};
//
//	SendandVerify(ModeVel,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
//	SendandVerify(VEL_Acc,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
//	SendandVerify(VEL_Dec,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
//}
//
//void BucketMotor_VelMode() {
//	SDO_packet ModeVel	 = {BUCKETMOTORID, 0x3003, 0x00, 0x3};
//	SDO_packet VEL_Acc = {BUCKETMOTORID, 0x3380, 0x00, ACCEL_CONST}; //value should be between 100k and 10k
//	SDO_packet VEL_Dec = {BUCKETMOTORID, 0x3381, 0x00, ACCEL_CONST};
//	
//
//	SendandVerify(ModeVel,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(VEL_Acc,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(VEL_Dec,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//}
//
//void BucketMotor_PosMode() {
//	SDO_packet ModePos = {BUCKETMOTORID, 0x3003, 0x00, 0x7};
//	SDO_packet SetPositionWindow = {BUCKETMOTORID, 0x373A, 0x00, 1000};
//	SDO_packet DesiredVelocity1000 = {BUCKETMOTORID, 0x3300, 0x0, 1000};
//	SDO_packet ResetPosition = {BUCKETMOTORID, 0x3762, 0x00, 0x00};
//	
//	
//	SendandVerify(ModePos,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(DesiredVelocity1000,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(SetPositionWindow,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(ResetPosition,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//}


void setMotor_Vel(int R_Vel,int L_Vel)
{
       //the only time we are setting the motor velocity is when we are rotating
    SetMotorRight_Vel(R_Vel);
    
    SetMotorLeft_Vel(L_Vel);
}
void SetMotorRight_Vel(int Vel)
{
    if (Vel > MAXRPM)
	{
		Vel = MAXRPM;
	}
    if(Vel < -1*MAXRPM)
	{
		Vel = -1*MAXRPM;
	}
    ToSendCAN(MOTOR_COMMAND_MODE_DATA_ADDRESS, VELOCITY_MODE);
    ToSendCAN(MOTOR_VELOCITY_DATA_ADDRESS, (unsigned int)Vel);
    sendDataCAN(BumperSwitchRight);
}
void SetMotorLeft_Vel(int Vel)
{
    if (Vel > MAXRPM)
	{
		Vel = MAXRPM;
	}
    if(Vel < -1*MAXRPM)
	{
		Vel = -1*MAXRPM;
	}
    ToSendCAN(MOTOR_COMMAND_MODE_DATA_ADDRESS, VELOCITY_MODE);
    ToSendCAN(MOTOR_VELOCITY_DATA_ADDRESS, (unsigned int)Vel);
    sendDataCAN(BumperSwitchLeft);
}

void LeftMotor_SetVel( int Vel) {

	if (Vel > MAXRPM)
	{
		Vel = MAXRPM;
	}
	if(Vel < -1*MAXRPM)
	{
		Vel = -1*MAXRPM;
	}

	SDO_packet DesiredVelocity = {LEFTMOTORID, 0x3300, 0x0, Vel};
		
	SendandVerify(DesiredVelocity,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
}

void LeftMotor_SetVelNoCommsSafety( int Vel) {

	if (Vel > MAXRPM)
	{
		Vel = MAXRPM;
	}
	if(Vel < -MAXRPM)
	{
		Vel = -MAXRPM;
	}
	char temp[8];
	SDO_packet DesiredVelocity = {LEFTMOTORID, 0x3300, 0x0, Vel};
//	while(cb_size(&LeftMotor_Buffer)>0)
//	{
//		cb_pop_front(&LeftMotor_Buffer, temp);
//	}
	//write data to motor
	while(!SDOWritePacket(DesiredVelocity));

}

void RightMotor_SetVel( int Vel) {

	if (Vel > MAXRPM)
	{
		Vel = MAXRPM;
	}
	if(Vel < -MAXRPM)
	{
		Vel = -MAXRPM;
	}

	SDO_packet DesiredVelocity = {RIGHTMOTORID, 0x3300, 0x0, Vel};

	SendandVerify(DesiredVelocity,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
}

void RightMotor_SetVelNoCommsSafety( int Vel) {

	if (Vel > MAXRPM)
	{
		Vel = MAXRPM;
	}
	if(Vel < -MAXRPM)
	{
		Vel = -MAXRPM;
	}
	char temp[8];
	SDO_packet DesiredVelocity = {RIGHTMOTORID, 0x3300, 0x0, Vel};
//	while(cb_size(&RightMotor_Buffer)>0)
//	{
//		cb_pop_front(&RightMotor_Buffer, temp);
//	}
	//write data to motor
	while(!SDOWritePacket(DesiredVelocity));

}

void ConveyorMotor_SetVel( int Vel) {

	if (Vel > MAXRPM)
	{
		Vel = MAXRPM;
	}
	if(Vel < -MAXRPM)
	{
		Vel = -MAXRPM;
	}

	SDO_packet DesiredVelocity = {CONVEYORMOTORID, 0x3300, 0x0, Vel};

	SendandVerify(DesiredVelocity,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
}

void ConveyorMotor_SetVelNoCommsSafety( int Vel) {

	if (Vel > MAXRPM)
	{
		Vel = MAXRPM;
	}
	if(Vel < -MAXRPM)
	{
		Vel = -MAXRPM;
	}
	char temp[8];
	SDO_packet DesiredVelocity = {CONVEYORMOTORID, 0x3300, 0x0, Vel};
//	while(cb_size(&ConveyorMotor_Buffer)>0)
//	{
//		cb_pop_front(&ConveyorMotor_Buffer, temp);
//	}
	//write data to motor
	while(!SDOWritePacket(DesiredVelocity));

}

void BucketMotor_SetVel( int Vel) {
	if (Vel > MAXRPM)
	{
		Vel = MAXRPM;
	}
	if(Vel < -MAXRPM)
	{
		Vel = -MAXRPM;
	}

	SDO_packet Continue = {BUCKETMOTORID, 0x3000, 0x00, 4};
	SDO_packet DesiredVelocity = {BUCKETMOTORID, 0x3300, 0x0, Vel};

	SendandVerify(Continue,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(DesiredVelocity,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
}

void BucketMotor_SetVelNoCommsSafety( int Vel) {

	if (Vel > MAXRPM)
	{
		Vel = MAXRPM;
	}
	if(Vel < -MAXRPM)
	{
		Vel = -MAXRPM;
	}
	char temp[8];
	SDO_packet Continue = {BUCKETMOTORID, 0x3000, 0x00, 4};
	SDO_packet DesiredVelocity = {BUCKETMOTORID, 0x3300, 0x0, Vel};
//	while(cb_size(&BucketMotor_Buffer)>0)
//	{
//		cb_pop_front(&BucketMotor_Buffer, temp);
//	}
	//write data to motor
	while(!SDOWritePacket(Continue));
	while(!SDOWritePacket(DesiredVelocity));

}

void BucketMotor_MoveCounts( long Counts) {
	SDO_packet Move = {BUCKETMOTORID, 0x3791, 0x00, Counts};
	
	SendandVerify(Move,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
}
void BucketMotor_MoveCountsNoCommsSafety( long Counts) {
	SDO_packet Move = {BUCKETMOTORID, 0x3791, 0x00, Counts};
//	char temp[8];
//	while(cb_size(&BucketMotor_Buffer)>0)
//	{
//		cb_pop_front(&BucketMotor_Buffer, temp);
//	}
	//write data to motor
	while(!SDOWritePacket(Move));
}

void BucketMotor_SetLimit(void) {
	SDO_packet PositiveLimit = {BUCKETMOTORID, 0x3055, 0x00, 0x131}; //activates digital input 0 as positive limit switch high active
	SDO_packet NegativeLimit = {BUCKETMOTORID, 0x3056, 0x00, 0x130}; //activates digital input 1 as negative limit switch high active
	//SDO_packet Home_Method = {BUCKETMOTORID, 0x37B2, 0x00, 2}; //homes by turning CW to positive limit, should be what we want.
	
	SendandVerify(PositiveLimit,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(NegativeLimit,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	//SendandVerify(Home_Method,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
}
////returns temp of controller to nearest degree C
//char LeftMotor_GetTemperature() {
//	char temp[8];
//	SDO_packet ReadTemperature = {LEFTMOTORID, 0x3114, 0x00, 0x00};
//
//	if(ReadandVerify(ReadTemperature, &LeftMotor_Buffer, LEFTMOTOR_STATUS, temp)) {
//	long result = ArrayToLong(temp);
//	return result/10;
//	}
//	else {
//		return 0; 
//	}
//}
//char RightMotor_GetTemperature() {
//	char temp[8];
//	SDO_packet ReadTemperature = {RIGHTMOTORID, 0x3114, 0x00, 0x00};
//
//	if(ReadandVerify(ReadTemperature, &RightMotor_Buffer, RIGHTMOTOR_STATUS, temp)) {
//		long result = ArrayToLong(temp);
//		return result/10;
//	}
//	else {
//		return 0;
//	}
//}
//char ConveyorMotor_GetTemperature() {
//	char temp[8];
//	SDO_packet ReadTemperature = {CONVEYORMOTORID, 0x3114, 0x00, 0x00};
//
//	if(ReadandVerify(ReadTemperature, &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS, temp)) {
//		long result = ArrayToLong(temp);
//		return result/10;
//	}
//	else {
//		return 0;
//	}
//}
//char BucketMotor_GetTemperature() {
//	char temp[8]; 
//	SDO_packet ReadTemperature = {BUCKETMOTORID, 0x3114, 0x00, 0x00};
//
//	if(ReadandVerify(ReadTemperature, &BucketMotor_Buffer, BUCKETMOTOR_STATUS, temp)) {
//		long result = ArrayToLong(temp);
//		return result/10;
//	}
//	else {
//		return 0;
//	}
//}
////returns voltage to nearest volt
//char LeftMotor_GetVoltage() {
//	char temp[8];
//	SDO_packet ReadVoltage = {LEFTMOTORID, 0x3110, 0x00, 0x00};
//
//	if(ReadandVerify(ReadVoltage, &LeftMotor_Buffer, LEFTMOTOR_STATUS, temp)) {
//		long result = ArrayToLong(temp);
//		return (result/1000 +1); //get voltage in mV from controller, add 1v to compensate for suspected diode drop to input. 
//	}
//	else {
//		return 0;
//	}
//}
//char RightMotor_GetVoltage() {
//	char temp[8];
//	SDO_packet ReadVoltage = {RIGHTMOTORID, 0x3110, 0x00, 0x00};
//
//	if(ReadandVerify(ReadVoltage, &RightMotor_Buffer, RIGHTMOTOR_STATUS, temp)) {
//		long result = ArrayToLong(temp);
//		return (result/1000 +1); //get voltage in mV from controller, add 1v to compensate for suspected diode drop to input.
//	}
//	else {
//		return 0;
//	}
//}
//char ConveyorMotor_GetVoltage() {
//	char temp[8];
//	SDO_packet ReadVoltage = {CONVEYORMOTORID, 0x3110, 0x00, 0x00};
//
//	if(ReadandVerify(ReadVoltage, &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS, temp)) {
//		long result = ArrayToLong(temp);
//		return (result/1000 +1); //get voltage in mV from controller, add 1v to compensate for suspected diode drop to input.
//	}
//	else {
//		return 0;
//	}
//}
//char BucketMotor_GetVoltage() {
//	char temp[8];
//	SDO_packet ReadVoltage = {BUCKETMOTORID, 0x3110, 0x00, 0x00};
//
//	if(ReadandVerify(ReadVoltage, &BucketMotor_Buffer, BUCKETMOTOR_STATUS, temp)) {
//		long result = ArrayToLong(temp);
//		return (result/1000 +1); //get voltage in mV from controller, add 1v to compensate for suspected diode drop to input.
//	}
//	else {
//		return 0;
//	}
//}
//
//long LeftMotor_GetPos() {
//	char temp[8];
//	SDO_packet ReadVoltage = {LEFTMOTORID, 0x396A, 0x00, 0x00};
//
//	if(ReadandVerify(ReadVoltage, &LeftMotor_Buffer, LEFTMOTOR_STATUS, temp)) {
//		long result = ArrayToLong(temp);
//		return (result/4); //returns enocer counts
//	}
//	else {
//		return 0;
//	}
//}
//long RightMotor_GetPos() {
//	char temp[8];
//	SDO_packet ReadVoltage = {RIGHTMOTORID, 0x396A, 0x00, 0x00};
//
//	if(ReadandVerify(ReadVoltage, &RightMotor_Buffer, RIGHTMOTOR_STATUS, temp)) {
//		long result = ArrayToLong(temp);
//		return (result/4); //returns enocer counts
//	}
//	else {
//		return 0;
//	}
//}
//long ConveyorMotor_GetPos() {
//	char temp[8];
//	SDO_packet ReadVoltage = {CONVEYORMOTORID, 0x396A, 0x00, 0x00};
//
//	if(ReadandVerify(ReadVoltage, &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS, temp)) {
//		long result = ArrayToLong(temp);
//		return (result/4); //returns enocer counts
//	}
//	else {
//		return 0;
//	}
//}
//long BucketMotor_GetPos() {
//	char temp[8];
//	SDO_packet ReadVoltage = {BUCKETMOTORID, 0x396A, 0x00, 0x00};
//
//	if(ReadandVerify(ReadVoltage, &BucketMotor_Buffer, BUCKETMOTOR_STATUS, temp)) {
//		long result = ArrayToLong(temp);
//		return (result/4); //returns enocer counts
//	}
//	else {
//		return 0;
//	}
//}
//
//char BucketMotor_Inputs()
//{
//	char temp[8];
//	SDO_packet ReadPort0 = {BUCKETMOTORID, 0x3120, 0x00, 0x00};
//
//	ReadandVerify(ReadPort0, &BucketMotor_Buffer, BUCKETMOTOR_STATUS, temp);
//	long result = ArrayToLong(temp);
//	return result;
//}
//
//void LeftMotor_ReEstablishComms() {
//	SDO_packet ClearErrors = { LEFTMOTORID, 0x3000, 0x00, 0x01};
//	SDO_packet SetMotor = { LEFTMOTORID, 0x3900, 0x00, 0x01};
//	SDO_packet SetPoles = { LEFTMOTORID, 0x3910, 0x00, 10};
//	SDO_packet MotorPolarity = { LEFTMOTORID, 0x3911, 0x00, 0x02};
//	SDO_packet SetFeedBack = {LEFTMOTORID, 0x3350, 0x00, 2410}; //2410 for encoder feedback, 2378 for hall feedback
//	SDO_packet SetSVELFeedBack = {LEFTMOTORID, 0x3550, 0x00, 2410};
//	SDO_packet SetFeedBackResolution = {LEFTMOTORID, 0x3962, 0x00, 2000};
//	SDO_packet CurrentLimitPos = {LEFTMOTORID, 0x3221, 0, 2000};
//	SDO_packet CurrentLimitNeg = {LEFTMOTORID, 0x3223, 0, 2000};
//	SDO_packet VelocityLimitPos = {LEFTMOTORID, 0x3321, 0x00, MAXRPM};
//	SDO_packet VelocityLimitNeg = {LEFTMOTORID, 0x3323, 0x00, MAXRPM};
//	SDO_packet PowerEnable = { LEFTMOTORID, 0x3004, 0x00, 0x01};
//	SDO_packet PowerDisable = { LEFTMOTORID, 0x3004, 0x00, 0x00};
//
//	MOTORSTATUS |= (1<<LEFTMOTOR_STATUS);
//	//wipe errors. This first attempt at communication also verifies the controller is in the system.
//	SendandVerify(ClearErrors,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	//disable controller so some commands actually work (Feed back commands)
//	SendandVerify(PowerDisable,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	//set resolution of encoder (500 count encoder * 4 for how the controllers work)
//	SendandVerify(SetFeedBack,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	SendandVerify(SetSVELFeedBack,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	SendandVerify(SetFeedBackResolution,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	//configure motor, number of magnet poles and motor type.
//	SendandVerify(SetMotor,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	SendandVerify(SetPoles,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	//set limits on current on velocity to protect motor
//	_delay_ms(2); //not sure why this is needed, but was having a problem communicating at start up.
//	SendandVerify(CurrentLimitPos,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	SendandVerify(CurrentLimitNeg,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	SendandVerify(VelocityLimitPos,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	SendandVerify(VelocityLimitNeg,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	//actually critical to get right if you are using encoders
//	SendandVerify(MotorPolarity,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//	//re-enable motor controller.
//	SendandVerify(PowerEnable,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
//}
//
//
//void RightMotor_ReEstablishComms() {
//	SDO_packet ClearErrors = { RIGHTMOTORID, 0x3000, 0x00, 0x01};
//	SDO_packet SetMotor = { RIGHTMOTORID, 0x3900, 0x00, 0x01};
//	SDO_packet SetPoles = { RIGHTMOTORID, 0x3910, 0x00, 10};
//	SDO_packet MotorPolarity = { RIGHTMOTORID, 0x3911, 0x00, 0x02};
//	SDO_packet SetFeedBack = {RIGHTMOTORID, 0x3350, 0x00, 2410}; //2410 for encoder feedback, 2378 for hall feedback
//	SDO_packet SetSVELFeedBack = {RIGHTMOTORID, 0x3550, 0x00, 2410};
//	SDO_packet SetFeedBackResolution = {RIGHTMOTORID, 0x3962, 0x00, 2000};
//	SDO_packet CurrentLimitPos = {RIGHTMOTORID, 0x3221, 0, 2000};
//	SDO_packet CurrentLimitNeg = {RIGHTMOTORID, 0x3223, 0, 2000};
//	SDO_packet VelocityLimitPos = {RIGHTMOTORID, 0x3321, 0x00, MAXRPM};
//	SDO_packet VelocityLimitNeg = {RIGHTMOTORID, 0x3323, 0x00, MAXRPM};
//	SDO_packet PowerEnable = { RIGHTMOTORID, 0x3004, 0x00, 0x01};
//	SDO_packet PowerDisable = { RIGHTMOTORID, 0x3004, 0x00, 0x00};
//	
//	MOTORSTATUS |= (1<<RIGHTMOTOR_STATUS);
//
//	//wipe errors. This first attempt at communication also verifies the controller is in the system.
//	SendandVerify(ClearErrors,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
//	//disable controller so some commands actually work (Feed back commands)
//	SendandVerify(PowerDisable,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
//	//set resolution of encoder (500 count encoder * 4 for how the controllers work)
//	SendandVerify(SetFeedBack,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
//	SendandVerify(SetSVELFeedBack,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
//	SendandVerify(SetFeedBackResolution,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
//	//configure motor, number of magnet poles and motor type.
//	SendandVerify(SetMotor,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
//	SendandVerify(SetPoles,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
//	//set limits on current on velocity to protect motor
//	SendandVerify(CurrentLimitPos,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
//	SendandVerify(CurrentLimitNeg,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
//	SendandVerify(VelocityLimitPos,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
//	SendandVerify(VelocityLimitNeg,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
//	//actually critical to get right if you are using encoders
//	SendandVerify(MotorPolarity,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
//	//re-enable motor controller.
//	SendandVerify(PowerEnable,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
//
//}
//
//
//void ConveyorMotor_ReEstablishComms() {
//	SDO_packet ClearErrors = {CONVEYORMOTORID, 0x3000, 0x00, 0x01};
//	SDO_packet SetMotor = {CONVEYORMOTORID, 0x3900, 0x00, 0x00}; //0 for brushed, 1 for brushless
//	SDO_packet CurrentLimitPos = {CONVEYORMOTORID, 0x3221, 0, 10000};
//	SDO_packet CurrentLimitNeg = {CONVEYORMOTORID, 0x3223, 0, 10000};
//	//SDO_packet RatedVoltage = {CONVEYORMOTORID, 0x3902, 0x00, 24000}; //not used, since default is 24000.
//	SDO_packet RatedSpeed = {CONVEYORMOTORID, 0x3901, 0x00, 129};
//	SDO_packet PowerEnable = {CONVEYORMOTORID, 0x3004, 0x00, 0x01};
//
//	MOTORSTATUS |= (1<<CONVEYORMOTOR_STATUS);
//
//
//	SendandVerify(ClearErrors,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
//	SendandVerify(SetMotor,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
//	SendandVerify(CurrentLimitNeg,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
//	SendandVerify(CurrentLimitPos,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
//	SendandVerify(RatedSpeed,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
//	SendandVerify(PowerEnable,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
//}
//
//void BucketMotor_ReEstablishComms() {
//	SDO_packet ClearErrors = {BUCKETMOTORID, 0x3000, 0x00, 0x01};
//	SDO_packet SetMotor = {BUCKETMOTORID, 0x3900, 0x00, 1};
//	SDO_packet SetPoles = {BUCKETMOTORID, 0x3910, 0x00, 8};
//	SDO_packet CurrentLimitPos = {BUCKETMOTORID, 0x3221, 0, 10000}; //10A
//	SDO_packet CurrentLimitNeg = {BUCKETMOTORID, 0x3223, 0, 10000}; //10A
//	SDO_packet VelocityLimitPos = {BUCKETMOTORID, 0x3321, 0x00, MAXRPM};
//	SDO_packet VelocityLimitNeg = {BUCKETMOTORID, 0x3323, 0x00, MAXRPM};
//	//SDO_packet MotorPolarity = {BUCKETMOTORID, 0x3911, 0x00, 0x01};
//	SDO_packet SetFeedBack = {BUCKETMOTORID, 0x3350, 0x00, 2410}; //encoder feedback
//	SDO_packet SetEncoderResolution = {BUCKETMOTORID, 0x3962, 0x00, 4096}; //bg75pi has a resolution of 4096 counts/rev
//	SDO_packet PowerEnable = {BUCKETMOTORID, 0x3004, 0x00, 0x01};
//
//	MOTORSTATUS |= (1<<BUCKETMOTOR_STATUS);
//
//
//	SendandVerify(ClearErrors,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(SetMotor,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(SetPoles,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(CurrentLimitNeg,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(CurrentLimitPos,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(VelocityLimitPos,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(VelocityLimitNeg,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(SetFeedBack,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(SetEncoderResolution,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//	SendandVerify(PowerEnable,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
//}


bool SDOReadRequest( SDO_packet SDOpacket)
{
	CAN_packet SDO = {0x600+SDOpacket.NodeID, 8, "01234567"};
	SDO.data[0] = 0x40;
	SDO.data[1] = SDOpacket.ObjIndx;
	SDO.data[2] = SDOpacket.ObjIndx>>8;
	SDO.data[3] = SDOpacket.SubIndx;
	SDO.data[4] = 0x00;
	SDO.data[5] = 0x00;
	SDO.data[6] = 0x00;
	SDO.data[7] = 0x00;
	
	return can_tx(&SDO);
}

void SDOWritePacketFUll(unsigned int NodeID, unsigned int ObjIndx, char SubIndx, unsigned long Data){
	CAN_packet SDO = {0x600+NodeID, 8, "01234567"};
	SDO.data[0] = 0x23;
	SDO.data[1] = ObjIndx;
	SDO.data[2] = ObjIndx>>8; 	
	SDO.data[3] = SubIndx;
	SDO.data[4] = Data; 
	SDO.data[5] = Data>>8; 
	SDO.data[6] = Data>>16;
	SDO.data[7] = Data>>24; 
	
	can_tx(&SDO);
}

bool SDOWritePacket(SDO_packet SDOpacket){

    //LED3^=1;
	CAN_packet SDO = {0x600+SDOpacket.NodeID, 8, "01234567"};
	SDO.data[0] = 0x23;
	SDO.data[1] = SDOpacket.ObjIndx;
	SDO.data[2] = SDOpacket.ObjIndx>>8;
	SDO.data[3] = SDOpacket.SubIndx;
	SDO.data[4] = SDOpacket.Data;
	SDO.data[5] = SDOpacket.Data>>8;
	SDO.data[6] = SDOpacket.Data>>16;
	SDO.data[7] = SDOpacket.Data>>24;
	
	return can_tx(&SDO);
}

bool SDOVerifyReply(char SDOreply[], SDO_packet SDOsent) {
	//reply has 0x60 in the first byte, and the same data in the other overhead bytes
	//Ignore data bytes (4-7)
	if(SDOreply[0] != 0x60) {
		return false;
	}
	if(SDOreply[1] != (0xFF & SDOsent.ObjIndx)) {
		return false;
	}
	if(SDOreply[2] != SDOsent.ObjIndx>>8) {
		return false;
	}
	if(SDOreply[3] != SDOsent.SubIndx) {
		return false;
	}
	return true;
}

bool SDOVerifyRead(char SDOreply[], SDO_packet SDOsent) {
	//reply has 0x60 in the first byte, and the same data in the other overhead bytes
	//Ignore data bytes (4-7)
	if((SDOreply[0] != 0x42) && (SDOreply[0] != 0x43) && (SDOreply[0] != 0x4B) && (SDOreply[0] != 0x4F)) { //could be 0x43 for exactly 4 bytes, 0x4B for 2 bytes, or 0x4F for 1 byte
		return false;
	}
	if(SDOreply[1] != (0xFF & SDOsent.ObjIndx)) {
		return false;
	}
	if(SDOreply[2] != SDOsent.ObjIndx>>8) {
		return false;
	}
	if(SDOreply[3] != SDOsent.SubIndx) {
		return false;
	}
	return true;
}

bool SDO_CAN_tx(CAN_packet * p)
{
    
            /* Writing the message for Transmission
Ecan1WriteTxMsgBufId(unsigned int buf, long txIdentifier, unsigned int ide, unsigned int remoteTransmit);
Ecan1WriteTxMsgBufData(unsigned int buf, unsigned int dataLength, unsigned int data1, unsigned int data2, unsigned int data3, unsigned int data4);

buf -> Transmit Buffer number

txIdentifier -> SID<10:0> : EID<17:0>

ide = 0 -> Message will transmit standard identifier
ide = 1 -> Message will transmit extended identifier

remoteTransmit = 0 -> Normal message
remoteTransmit = 1 -> Message will request remote transmission

dataLength -> Data length can be from 0 to 8 bytes

data1, data2, data3, data4 -> Data words (2 bytes) each
*/
    int status = 1; //1: transmit success, 0: transmit buffers full. 
    if(C1TR01CONbits.TXREQ0 == 0){
        Ecan1WriteTxMsgBufId( 0, p->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 0, p->length, 
            (p->data[1]<<8) + p->data[0], (p->data[3]<<8) + p->data[2], (p->data[5]<<8) + p->data[4], (p->data[7]<<8) + p->data[6]);
        C1TR01CONbits.TXREQ0 = 1;
    }
    else if(C1TR01CONbits.TXREQ1 == 0){
        Ecan1WriteTxMsgBufId( 1, p->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 1, p->length, 
            (p->data[1]<<8) + p->data[0], (p->data[3]<<8) + p->data[2], (p->data[5]<<8) + p->data[4], (p->data[7]<<8) + p->data[6]);
        C1TR01CONbits.TXREQ1 = 1;
    }
    else if(C1TR23CONbits.TXREQ2 == 0){
        Ecan1WriteTxMsgBufId( 2, p->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 2, p->length, 
            (p->data[1]<<8) + p->data[0], (p->data[3]<<8) + p->data[2], (p->data[5]<<8) + p->data[4], (p->data[7]<<8) + p->data[6]);
        C1TR23CONbits.TXREQ2 = 1;
    }
    else if(C1TR23CONbits.TXREQ3 == 0){
        Ecan1WriteTxMsgBufId( 3, p->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 3, p->length, 
            (p->data[1]<<8) + p->data[0], (p->data[3]<<8) + p->data[2], (p->data[5]<<8) + p->data[4], (p->data[7]<<8) + p->data[6]);
        C1TR23CONbits.TXREQ3 = 1;
    }
    else if(C1TR45CONbits.TXREQ4 == 0){
        Ecan1WriteTxMsgBufId( 4, p->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 4, p->length, 
            (p->data[1]<<8) + p->data[0], (p->data[3]<<8) + p->data[2], (p->data[5]<<8) + p->data[4], (p->data[7]<<8) + p->data[6]);
        C1TR45CONbits.TXREQ4 = 1;
    }
    else if(C1TR45CONbits.TXREQ5 == 0){
        Ecan1WriteTxMsgBufId( 5, p->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 5, p->length, 
            (p->data[1]<<8) + p->data[0], (p->data[3]<<8) + p->data[2], (p->data[5]<<8) + p->data[4], (p->data[7]<<8) + p->data[6]);
        C1TR45CONbits.TXREQ5 = 1;
    }
    else if(C1TR67CONbits.TXREQ6 == 0){
        Ecan1WriteTxMsgBufId( 6, p->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 6, p->length, 
            (p->data[1]<<8) + p->data[0], (p->data[3]<<8) + p->data[2], (p->data[5]<<8) + p->data[4], (p->data[7]<<8) + p->data[6]);
        C1TR67CONbits.TXREQ6 = 1;
    }
    else if(C1TR67CONbits.TXREQ7 == 0){
        Ecan1WriteTxMsgBufId( 7, p->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 7, p->length, 
            (p->data[1]<<8) + p->data[0], (p->data[3]<<8) + p->data[2], (p->data[5]<<8) + p->data[4], (p->data[7]<<8) + p->data[6]);
        C1TR67CONbits.TXREQ7 = 1;
    }
    else {
         
        TX_cb_push_back(*p); 
        status = 0; //all buffers are full 
    }
    return status;
}

