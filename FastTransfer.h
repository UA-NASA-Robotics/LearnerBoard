/* 
 * File:   FastTransfer.h
 * Author: Igor
 *
 * Created on March 23, 2015, 1:21 PM
 */

#ifndef FASTTRANSFER_H
#define	FASTTRANSFER_H

//the capital D is so there is no interference with the lower case d of EasyTransfer
#define Details(name) (int*)&name,sizeof(name)
//int receiveArray[20];
extern void (*serial_write)(unsigned char);
extern unsigned char (*serial_read)(void);
extern int (*serial_available)(void);
extern unsigned char (*serial_peek)(void);
extern unsigned char rx_buffer[255]; //address for temporary storage and parsing buffer
extern unsigned char rx_array_inx; //index for RX parsing buffer
extern unsigned char rx_len; //RX packet length according to the packet
extern unsigned char calc_CS; //calculated Checksum
extern unsigned char moduleAddress; // the address of this module
extern unsigned char returnAddress; //the address to send the crc back to
extern unsigned char maxDataAddress; //max address allowable
extern int * receiveArrayAddress; // this is where the data will go when it is received
extern unsigned char * sendStructAddress; // this is where the data will be sent from
extern bool AKNAKsend; // turns the acknowledged or not acknowledged on/off
extern unsigned int alignErrorCounter; //counts the align errors
extern unsigned int crcErrorCounter; // counts any failed crcs
extern unsigned int addressErrorCounter; // counts every time a wrong address is received
extern unsigned int dataAdressErrorCounter; // counts if the received data fall outside of the receive array
extern unsigned char rx_address; //RX address received
#define polynomial 0x8C  //polynomial used to calculate crc
#define BUFFER_SIZE 200 //ring buffer size
#define CRC_COUNT 5 // how many AKNAKs are stored
#define CRC_DEPTH 3  // how many pieces of data are stored with each CRC send event
#define CRC_BUFFER_SIZE (CRC_COUNT * CRC_DEPTH) //crc buffer size 5 deep and 3 bytes an entry

//extern struct ringBufS
//{ // this is where the send data is stored before sending
//    unsigned char buf[BUFFER_SIZE];
//    int head;
//    int tail;
//    int count;
//};
extern struct ringBufS debugRingBuff;
extern struct ringBufS OutGoing_DataTransBuff;  //this holds the output data for the fastTrans

extern struct ringBufS RingBuff;

//extern union stuff
//{ // this union is used to join and disassemble integers
//    unsigned char parts[2];
//    unsigned int integer;
//};
extern union stuff group;

//struct crcBufS
//{ // this is where the address where sent to, the sent crc, the status of the AKNAK
//    unsigned char buf[CRC_BUFFER_SIZE];
//    int head;
//};
extern struct crcBufS crc_buffer;

extern unsigned char CRC8(const unsigned char * data, unsigned char len);
extern void FastTransfer_buffer_put(struct ringBufS *_this, const unsigned char towhere, const unsigned int towhat);
extern unsigned char FastTransfer_buffer_get(struct ringBufS* _this);
extern void FastTransfer_buffer_flush(struct ringBufS* _this, const int clearBuffer);
extern unsigned int FastTransfer_buffer_modulo_inc(const unsigned int value, const unsigned int modulus);
extern void crcBufS_put(struct crcBufS* _this, unsigned char address, unsigned char oldCRC, unsigned char status);
extern void crcBufS_status_put(struct crcBufS* _this, unsigned char time, unsigned char status);
extern unsigned char crcBufS_get(struct crcBufS* _this, unsigned char time, unsigned char space);
extern void CRCcheck(void);

extern void begin(int * ptr, unsigned char maxSize, unsigned char givenAddress, bool error, void (*stufftosend)(unsigned char), unsigned char (*stufftoreceive)(void),int (*stuffavailable)(void), unsigned char (*stuffpeek)(void));
extern bool sendData_debug(unsigned char whereToSend, struct ringBufS *_ringBuff);
extern bool sendData(unsigned char whereToSend, struct ringBufS *_ringBuff);
extern bool receiveData();
extern void ToSend(const unsigned char where, const unsigned int what, struct ringBufS *_ringBuff);
extern unsigned char AKNAK(unsigned char module);
extern unsigned int alignError(void);
extern unsigned int CRCError(void);
extern unsigned int addressError(void);
extern unsigned int dataAddressError(void);




#endif	/* FASTTRANSFER_H */

