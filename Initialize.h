/* 
 * File:   initialize.h
 * Author: John
 *
 * Created on February 22, 2017, 7:30 AM
 */

#ifndef INITIALIZE_H
#define	INITIALIZE_H

#define OFF 0
#define ON 1

#define INPUT 1
#define OUTPUT 0

//END OF LED DEFINITIONS



//bool Initialize();
void Start_Initialization();
//switches the clock from the FRC to the external HS oscillator
void oscillator(void);
void timerOne(void);
void timerTwo(void);

void init_SPI();


#endif	/* INITIALIZE_H */

