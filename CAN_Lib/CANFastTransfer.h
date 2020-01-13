/*
 * CANFastTransfer.h
 *
 * Created: 10/26/2016 10:55:15 PM
 *  Author: reed
 */ 


#ifndef CANFASTTRANSFER_H_
#define CANFASTTRANSFER_H_

#define CANFAST_MOB 4
#define TRANSMITMOB 14
#define BUFFER_SIZECAN 500

#include <xc.h>
#include "can.h" 

void checkCANFTdata(void);
//Init function
void beginCANFast(volatile int * ptr, unsigned int maxSize, unsigned char givenAddress); 

//RX functions
void SetReceiveMode(int input); 
int  ReceiveDataCAN(void); 
void ReceiveCANFast( CAN_packet *p);

//TX functions
void ToSendCAN( unsigned int where, unsigned int what);
void sendDataCAN( unsigned int whereToSend);
int GetTransmitErrorCount(void); 




//Circular buffer stuff
struct ringBufSCAN { // this is where the send data is stored before sending
	int buf[BUFFER_SIZECAN];
	int head;
	int tail;
	int count;
};


void Send_buffer_put(struct ringBufSCAN *_this, const unsigned int towhere, const unsigned int towhat);
unsigned int Send_buffer_get(struct ringBufSCAN* _this);
void Send_buffer_flush(struct ringBufSCAN* _this, const int clearBuffer);
unsigned int Send_buffer_modulo_inc(const unsigned int value, const unsigned int modulus);
unsigned int Send_buffer_GetCount(struct ringBufSCAN* _this); 


#endif /* CANFASTTRANSFER_H_ */

