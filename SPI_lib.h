/* 
 * File:   SPI_lib.h
 * Author: John
 *
 * Created on March 6, 2017, 1:16 PM
 */

#ifndef SPI_LIB_H
#define	SPI_LIB_H

void SPI_Transfer(char data);
int Read_MouseSensor(unsigned char _address);
void Write_MouseSensor(unsigned char _address, unsigned char _data);

#endif	/* SPI_LIB_H */

