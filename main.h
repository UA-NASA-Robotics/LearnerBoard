/* 
 * File:   main.h
 * Author: Seth Carpenter
 *
 * Created on February 22, 2017, 7:28 AM
 */

#ifndef MAIN_H
#define	MAIN_H
#include <stdbool.h>
//volatile unsigned int time;
int main(void);
void delay_ms(int _Val);
void delay_us(int _Val);
char ConvertFromTwos(char x);
void getMouseData();
void setOperationIndex(int *receiveArrayAddress);
void processDataRequest();
bool getGData();
void delayMS(int val);
#endif	/* MAIN_H */

