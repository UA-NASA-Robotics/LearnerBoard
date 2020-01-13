

/*
 * CANFastTransfer.c
 *
 * Created: 10/26/2016 11:23:36 PM
 *  Author: reed
 */ 
#include "xc.h"
#include "CANFastTransfer.h"
#include "can.h"
//#include "assert.h"

#include "../../Common/CommsDefenition.h"
//#include "test.h"
#include <stdlib.h>
#include "../macros.h"
#include "../comDefs.h"
#define Instant  0
#define WhenReceiveCall  1

//global variables
int receiveMode = 0; 
int ReceivedData = 0; 
int MaxIndex = 0; 
int TransmitPairMissMatch = 0;
volatile int * receiveArrayAddressCAN; // this is where the data will go when it is received
unsigned char moduleAddressCAN; // the address of this module
struct ringBufSCAN ring_buffer_CAN;
struct ringBufSCAN ReceiveBuffer; 

// Define ECAN Message Buffers
    __eds__ ECAN1MSGBUF ecan1msgBuf  __attribute__( (eds, space(xmemory), aligned(ECAN1_MSG_BUF_LENGTH * 16)) );

//Start Receive Functions

void checkCANFTdata(void)
{
    if(ReceiveDataCAN())
    {
        
        LED2^=1;
       
        setPerformNavigationCommand(receiveArrayAddressCAN[1]);
        setNavigationCommandData(receiveArrayAddressCAN[2]); 
        setPauseData(receiveArrayAddressCAN[MOTOR_PAUSE_DATA_ADDRESS]);
        receiveArrayAddressCAN[1] = 0;
        receiveArrayAddressCAN[2] = 0;
        receiveArrayAddressCAN[MOTOR_PAUSE_DATA_ADDRESS] = 0;
        if(receiveArrayAddressCAN[2] != 0)
        {
             LED3^=1;
        }

    }
}
void ReceiveCANFast( CAN_packet *p) // interrupt callback
{
	//Possibly two modes one that requires calling receive data like old Fast Transfer for compatibility purposes
	// Then the second that automatically updates the array, and is more efficient.
	if(receiveMode == WhenReceiveCall)
	{
        if(p->length >= 8) //Check number of bytes, if 8 read in two ints
		{
            if((p->data[0]<<8) +(p->data[1]) < MaxIndex) {
				Send_buffer_put(&ReceiveBuffer, (p->data[0]<<8) +(p->data[1]), (p->data[2]<<8) +(p->data[3]));
				ReceivedData = 1;
			}
			if((p->data[4]<<8) +(p->data[5]) < MaxIndex) {
				Send_buffer_put(&ReceiveBuffer, (p->data[4]<<8) +(p->data[5]), (p->data[6]<<8) +(p->data[7]));
				ReceivedData = 1;
			}
			Send_buffer_put(&ReceiveBuffer, LastBoardReceived, (p->id & 0b11111));
		}
		else //else read in one int
		{
			if((p->data[0]<<8) +(p->data[1]) < MaxIndex) {
				Send_buffer_put(&ReceiveBuffer, (p->data[0]<<8) +(p->data[1]), (p->data[2]<<8) +(p->data[3]));
				ReceivedData = 1; 
			}
			Send_buffer_put(&ReceiveBuffer, LastBoardReceived, (p->id & 0b11111));
		}
		 
	} //end wait receive mode
	else //instant
	{
		receiveArrayAddressCAN[LastBoardReceived] = (p->id & 0b11111); //set last board received
		if(p->length >= 8) //Check number of bytes, if 8 read in two ints
		{
			if((p->data[0]<<8) +(p->data[1]) < MaxIndex) {
				receiveArrayAddressCAN[(p->data[0]<<8) +(p->data[1])] = (p->data[2]<<8) +(p->data[3]);
				ReceivedData = 1;
			}
			if((p->data[4]<<8) +(p->data[5]) < MaxIndex) {
				receiveArrayAddressCAN[(p->data[4]<<8) +(p->data[5])] = (p->data[6]<<8) +(p->data[7]);
				ReceivedData = 1;
			}
			 
		}
		else //else read in one int
		{
			if((p->data[0]<<8) +(p->data[1]) < MaxIndex) {
				receiveArrayAddressCAN[(p->data[0]<<8) +(p->data[1])] = (p->data[2]<<8) +(p->data[3]);
				ReceivedData = 1;
			}
		}
        
	} //end default receive mode
    
    
	ReceivedData = 1; 
}



void beginCANFast(volatile int * ptr, unsigned int maxSize, unsigned char givenAddress){
	receiveArrayAddressCAN = ptr;
	moduleAddressCAN = givenAddress;
    MaxIndex = maxSize;
	can_init(&ReceiveCANFast);
	Send_buffer_flush(&ring_buffer_CAN,1);
    Send_buffer_flush(&ReceiveBuffer, 1); 
}

void SetReceiveMode(int input) {
	if(input == Instant || input == WhenReceiveCall ) 
	{
		receiveMode = input; 
	}
}

int ReceiveDataCAN(void) {
    if(receiveMode == Instant) {
		if(ReceivedData) {
            ReceivedData=0;
			return 1; 
		}
		else {
			return 0; 
		}
	}
	if(ReceivedData) {
        //LED1^=1;
		ReceivedData = 0;
        int i = Send_buffer_GetCount(&ReceiveBuffer);
        if(i) //this better be true
        {
            for(;i>0; i=i-2) 
            {
                int address = Send_buffer_get(&ReceiveBuffer);
                receiveArrayAddressCAN[address] = Send_buffer_get(&ReceiveBuffer); 
            }
           return 1; 
        }
        else 
        {
            //error (how was ReceiveData true if no data available)
            return 0; 
        }
		 
	}
	else
	 return 0; 
}

//End Receive Functions

//Start Transmit Functions

void ToSendCAN(unsigned int where, unsigned int what)
{
	Send_buffer_put(&ring_buffer_CAN, where, what);
}

void sendDataCAN( unsigned int whereToSend)
{
	int Count = Send_buffer_GetCount(&ring_buffer_CAN);
    CAN_packet t; 
    int temp; 
    t.id = (whereToSend<<6) + MOUSE_GYRO_ADDRESS; //address is the same for all following packets
	for(;Count>0;) {
        if(Count>=4) {
            //send 2 data and 2 indexes
            t.length = 8;
            temp = Send_buffer_get(&ring_buffer_CAN); 
            t.data[0] = temp>>8;
            t.data[1] = temp;
            temp = Send_buffer_get(&ring_buffer_CAN);
            t.data[2] = temp>>8; 
            t.data[3] = temp;
            temp = Send_buffer_get(&ring_buffer_CAN);
            t.data[4] = temp>>8; 
            t.data[5] = temp;
            temp = Send_buffer_get(&ring_buffer_CAN);
            t.data[6] = temp>>8;
            t.data[7] = temp; 
            can_tx(&t); 
            Count = Count -4; 
        }
        else if(Count >=2 ) {
            //send 1 data and 1 index
            t.length = 4; 
            temp = Send_buffer_get(&ring_buffer_CAN); 
            t.data[0] = temp>>8;
            t.data[1] = temp;
            temp = Send_buffer_get(&ring_buffer_CAN);
            t.data[2] = temp>>8; 
            t.data[3] = temp;
            can_tx(&t); 
            Count = Count -2; 
        }
        else if(Count == 1) {
            //error
            TransmitPairMissMatch++; 
        }
    }
	
}

int GetTransmitErrorCount(void) {
	return TransmitPairMissMatch; 
}

//End Transmit Functions

// disassembles the data and places it in a buffer to be sent

void Send_buffer_put(struct ringBufSCAN *_this, unsigned int towhere, unsigned int towhat) {


	if (_this->count < (BUFFER_SIZECAN - 3)) {
		_this->buf[_this->head] = towhere;
		_this->head = Send_buffer_modulo_inc(_this->head, BUFFER_SIZECAN);
		++_this->count;
		_this->buf[_this->head] = towhat;
		_this->head = Send_buffer_modulo_inc(_this->head, BUFFER_SIZECAN);
		++_this->count;
	}

}


//pulls info out of the send buffer in a first in first out fashion

unsigned int Send_buffer_get(struct ringBufSCAN* _this) {
	unsigned int c;
	if (_this->count > 0) {
		c = _this->buf[_this->tail];
		_this->tail = Send_buffer_modulo_inc(_this->tail, BUFFER_SIZECAN);
		--_this->count;
		} else {
		c = 0;
	}
	return (c);
}

void *memset(void *s, int c, size_t n);

//flushes the send buffer to get it ready for new data

void Send_buffer_flush(struct ringBufSCAN* _this, const int clearBuffer) {
	_this->count = 0;
	_this->head = 0;
	_this->tail = 0;
	if (clearBuffer) {
		memset(_this->buf, 0, sizeof (_this->buf));
	}
}

 
// increments counters for the buffer functions

unsigned int Send_buffer_modulo_inc(const unsigned int value, const unsigned int modulus) {
	unsigned int my_value = value + 1;
	if (my_value >= modulus) {
		my_value = 0;
	}
	return (my_value);
}

//getter for send circular buffer. 
unsigned int Send_buffer_GetCount(struct ringBufSCAN* _this) {
	return _this->count; 
	
}