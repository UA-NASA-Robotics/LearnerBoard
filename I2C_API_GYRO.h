/* 
 * File:   I2C_API_GYRO.h
 * Author: Igor(modified by Seth Carpenter)
 *
 * Created on December 14, 2014, 8:04 PM(modified: April 27, 2016)
 */

#ifndef I2C_API_GYRO_H
#define	I2C_API_GYRO_H

#define PENDING 0
#define SUCCESS 1
#define FAILED 2
#define TRANSMIT 1
#define RECEIVE 2

int get_dataIndex();


void InitI2Cone(void);
///void InitI2Ctwo(void);

bool SendI2Cone(unsigned char s_address, unsigned char d_address, unsigned char * dat, unsigned char how_much);
//bool SendI2Ctwo(unsigned char s_address,  unsigned char d_address, unsigned char * dat, unsigned char how_much);

bool ReceiveI2Cone(unsigned char s_address, unsigned char d_address, unsigned char * dat, unsigned char how_much);
//bool ReceiveI2Ctwo(unsigned char s_address, unsigned char d_address, unsigned char * dat, unsigned char how_much);

void SendSlaveAddressI2Cone(void);
//void SendSlaveAddressI2Ctwo(void);

void SendDataAddressI2Cone(void);
//void SendDataAddressI2Ctwo(void);

void SendRestartI2Cone(void);
//void SendRestartI2Ctwo(void);

void SendStartI2Cone(void);
//void SendStartI2Ctwo(void);

void SendDataI2Cone(void);
//void SendDataI2Ctwo(void);

void StopFunctionI2Cone(void);
//void StopFunctionI2Ctwo(void);

void FailFunctionI2Cone(void);
//void FailFunctionI2Ctwo(void);

void SuccessFunctionI2Cone(void);
//void SuccessFunctionI2Ctwo(void);

void EnableReceiveI2Cone(void);
//void EnableReceiveI2Ctwo(void);

void SendReadRequestI2Cone(void);
//void SendReadRequestI2Ctwo(void);

void FirstReceiveI2Cone(void);
//void FirstReceiveI2Ctwo(void);

void ReceiveByteI2Cone(void);
//void ReceiveByteI2Ctwo(void);

unsigned char StatusI2Cone(void);
unsigned char StatusI2Ctwo(void);

void NACKFollowUpI2Cone(void);
//void NACKFollowUpI2Ctwo(void);

bool writeBits(char devAddr, char regAddr, char bitStart, char length, char data);
bool writeBit(char devAddr, char regAddr, char bitNum, char data);

#endif	/* I2C_API_GYROH */

