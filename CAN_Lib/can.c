/*
 * File:   can.c
 * Author: reed
 *
 * Created on August 13, 2016, 11:32 AM
 */


#include "xc.h"
#include "can.h"
#include <stdlib.h>
#include "ecan1_config.h"
#include "../comDefs.h"
#include "CANFastTransfer.h"

circular_buffer CAN1_TX_Buffer;
#define CAN1_TX_Buffer_Size 256
circular_buffer CAN1_RX_Buffer; 
#define CAN1_RX_Buffer_Size 256

CAN_packet TX_Buff[CAN1_TX_Buffer_Size]; 
CAN_packet RX_Buff[CAN1_RX_Buffer_Size];

typedef void (* CAN_cbf)( CAN_packet *p);

void (*RXPUT) (CAN_packet *p) = NULL;

void can_init(void *RXfunction) {
     /* ECAN1 Initialisation         
   Configure DMA Channel 0 for ECAN1 Transmit
   Configure DMA Channel 2 for ECAN1 Receive */
    Ecan1Init();
   DMA0Init();
    DMA2Init();

    /* Enable ECAN1 Interrupt */
    IEC2bits.C1IE = 1;
    //IEC0bits.DMA0IE = 1; 
    //IEC1bits.DMA2IE = 1; 
    C1INTEbits.TBIE = 1;
    C1INTEbits.RBIE = 1;
    
    //setup for Receive CAN fast transfer
    RXPUT = RXfunction;
 
    TX_cb_init(); 
    RX_cb_init(); 
}


bool can_tx(CAN_packet *packet) {
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
    bool status = true; //1: transmit success, 0: transmit buffers full. 
    //CiTRmnCON register for information about the transmit system registers....
    if(C1TR01CONbits.TXREQ0 == 0){
        Ecan1WriteTxMsgBufId( 0, packet->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 0, packet->length, 
            (packet->data[1]<<8) + packet->data[0], (packet->data[3]<<8) + packet->data[2], (packet->data[5]<<8) + packet->data[4], (packet->data[7]<<8) + packet->data[6]);
        //Request Message Send
        C1TR01CONbits.TXREQ0 = 1;
    }
    else if(C1TR01CONbits.TXREQ1 == 0){
        Ecan1WriteTxMsgBufId( 1, packet->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 1, packet->length, 
            (packet->data[1]<<8) + packet->data[0], (packet->data[3]<<8) + packet->data[2], (packet->data[5]<<8) + packet->data[4], (packet->data[7]<<8) + packet->data[6]);
        //Request Message Send
        C1TR01CONbits.TXREQ1 = 1;
    }
    else if(C1TR23CONbits.TXREQ2 == 0){
        Ecan1WriteTxMsgBufId( 2, packet->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 2, packet->length, 
            (packet->data[1]<<8) + packet->data[0], (packet->data[3]<<8) + packet->data[2], (packet->data[5]<<8) + packet->data[4], (packet->data[7]<<8) + packet->data[6]);
        //Request Message Send
        C1TR23CONbits.TXREQ2 = 1;
    }
    else if(C1TR23CONbits.TXREQ3 == 0){
        Ecan1WriteTxMsgBufId( 3, packet->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 3, packet->length, 
            (packet->data[1]<<8) + packet->data[0], (packet->data[3]<<8) + packet->data[2], (packet->data[5]<<8) + packet->data[4], (packet->data[7]<<8) + packet->data[6]);
        //Request Message Send
        C1TR23CONbits.TXREQ3 = 1;
    }
    else if(C1TR45CONbits.TXREQ4 == 0){
        Ecan1WriteTxMsgBufId( 4, packet->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 4, packet->length, 
            (packet->data[1]<<8) + packet->data[0], (packet->data[3]<<8) + packet->data[2], (packet->data[5]<<8) + packet->data[4], (packet->data[7]<<8) + packet->data[6]);
        //Request Message Send
        C1TR45CONbits.TXREQ4 = 1;
    }
    else if(C1TR45CONbits.TXREQ5 == 0){
        Ecan1WriteTxMsgBufId( 5, packet->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 5, packet->length, 
            (packet->data[1]<<8) + packet->data[0], (packet->data[3]<<8) + packet->data[2], (packet->data[5]<<8) + packet->data[4], (packet->data[7]<<8) + packet->data[6]);
        //Request Message Send
        C1TR45CONbits.TXREQ5 = 1;
    }
    else if(C1TR67CONbits.TXREQ6 == 0){
        Ecan1WriteTxMsgBufId( 6, packet->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 6, packet->length, 
            (packet->data[1]<<8) + packet->data[0], (packet->data[3]<<8) + packet->data[2], (packet->data[5]<<8) + packet->data[4], (packet->data[7]<<8) + packet->data[6]);
        //Request Message Send
        C1TR67CONbits.TXREQ6 = 1;
    }
    else if(C1TR67CONbits.TXREQ7 == 0){
        Ecan1WriteTxMsgBufId( 7, packet->id, 0, 0 );
        Ecan1WriteTxMsgBufData( 7, packet->length, 
            (packet->data[1]<<8) + packet->data[0], (packet->data[3]<<8) + packet->data[2], (packet->data[5]<<8) + packet->data[4], (packet->data[7]<<8) + packet->data[6]);
        //Request Message Send
        C1TR67CONbits.TXREQ7 = 1;
    }
    else {
        TX_cb_push_back(*packet); 
        status = false; //all buffers are full 
    }
    return status; 
}

/******************************************************************************
 * Function:      void __attribute__((interrupt, no_auto_psv))_C1Interrupt(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      Interrupt service routine to handle CAN1 Transmit and
 *                receive interrupt.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _C1Interrupt( void )
{
    IFS2bits.C1IF = 0;      // clear interrupt flag
    //interrupt caused by completed transmit
    if( C1INTFbits.TBIF )
    { 
        //LED1^=1;
        //if data left in TX buffer, send it
        if(TX_cb_count()) {
            static CAN_packet tempStruct1;
            tempStruct1 = TX_cb_pop_front(); 
            can_tx(&tempStruct1); 
        }
        C1INTFbits.TBIF = 0;
    }
    //interrupt caused by received packet
    if( C1INTFbits.RBIF )
    {
        //LED6^=1;
        static int i = 0; 
      
        //received immediately. 
        //Fast transfer library will store it in circular buffer or 
        //immediately update receive array. 
        int bufferTriggered= C1VECbits.ICODE;
        CAN_packet tempStruct; 
        tempStruct = can_get_FIFO(bufferTriggered);
        ReceiveCANFast(&tempStruct);  
        
        C1INTFbits.RBIF = 0;
    }
}


CAN_packet can_get_FIFO(int buf)
{
   
    
    static CAN_packet temp; 
    static int Buffer=8;
    
    //Buffer = C1FIFO & 0x00FF; 
    temp.id      = ecan1msgBuf[Buffer][0]>>2; //removes SRR and IDE
    temp.length  = ecan1msgBuf[Buffer][2] & 0xF; //only last 4 bits are length 
    temp.data[0] = ecan1msgBuf[Buffer][3] & 0xFF; //LSB
    temp.data[1] = ecan1msgBuf[Buffer][3] >> 8; //MSB
    temp.data[2] = ecan1msgBuf[Buffer][4] & 0xFF; //LSB
    temp.data[3] = ecan1msgBuf[Buffer][4] >> 8; //MSB
    temp.data[4] = ecan1msgBuf[Buffer][5] & 0xFF; //LSB
    temp.data[5] = ecan1msgBuf[Buffer][5] >> 8; //MSB
    temp.data[6] = ecan1msgBuf[Buffer][6] & 0xFF; //LSB
    temp.data[7] = ecan1msgBuf[Buffer][6] >> 8; //MSB
    
    //mark buffer as read
    if(Buffer < 16) 
    {
        C1RXFUL1 = C1RXFUL1 ^ (1<<(Buffer)); 
    }
    else 
    {
        C1RXFUL2 = C1RXFUL2 ^ (1<<((Buffer)-16));
    }
    Buffer++;
    if(Buffer>31)
    {
        Buffer=8;
    }
    return temp; 
}


void TX_cb_init()
{
    CAN1_TX_Buffer.capacity = CAN1_TX_Buffer_Size;
    CAN1_TX_Buffer.count = 0; 
    CAN1_TX_Buffer.head = 0; 
    CAN1_TX_Buffer.tail = 0; 
}
void RX_cb_init()
{
    CAN1_RX_Buffer.capacity = CAN1_RX_Buffer_Size;
    CAN1_RX_Buffer.count = 0; 
    CAN1_RX_Buffer.head = 0; 
    CAN1_RX_Buffer.tail = 0; 
}

void TX_cb_push_back( CAN_packet item)
{
    if(CAN1_TX_Buffer.count == CAN1_TX_Buffer.capacity) {
        return;
    }
    TX_Buff[CAN1_TX_Buffer.head] = item;
    CAN1_TX_Buffer.head++; 
    if(CAN1_TX_Buffer.head == CAN1_TX_Buffer.capacity){
        CAN1_TX_Buffer.head = 0; 
    }
    CAN1_TX_Buffer.count++; 

}
void RX_cb_push_back( CAN_packet item)
{
    if(CAN1_RX_Buffer.count == CAN1_RX_Buffer.capacity) {
        return;
    }
    RX_Buff[CAN1_RX_Buffer.head] = item;
    CAN1_RX_Buffer.head++; 
    if(CAN1_RX_Buffer.head == CAN1_RX_Buffer.capacity){
        CAN1_RX_Buffer.head = 0; 
    }
    CAN1_RX_Buffer.count++; 

}


CAN_packet TX_cb_pop_front()
{   
    static CAN_packet temp; 
    temp.id = 0;
    temp.length = 0; 
    temp.data[0] = 0; 
    if(CAN1_TX_Buffer.count == 0) {
        return temp;
    }
    temp = TX_Buff[CAN1_TX_Buffer.tail]; 
    CAN1_TX_Buffer.tail++; 
    if(CAN1_TX_Buffer.tail == CAN1_TX_Buffer.capacity) {
        CAN1_TX_Buffer.tail = 0; 
    }
    CAN1_TX_Buffer.count--; 
    return temp;
    
}
CAN_packet RX_cb_pop_front()
{   
    static CAN_packet temp; 
    temp.id = 0;
    temp.length = 0; 
    temp.data[0] = 0; 
    if(CAN1_RX_Buffer.count == 0) {
        return temp;
    }
    temp = RX_Buff[CAN1_RX_Buffer.tail]; 
    CAN1_RX_Buffer.tail++; 
    if(CAN1_RX_Buffer.tail == CAN1_RX_Buffer.capacity) {
        CAN1_RX_Buffer.tail = 0; 
    }
    CAN1_RX_Buffer.count--; 
    return temp;
    
}


int TX_cb_count() {
    return CAN1_TX_Buffer.count;
}
int RX_cb_count() {
    return CAN1_RX_Buffer.count;
}