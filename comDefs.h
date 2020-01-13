/* 
 * File:   comDefs.h
 * Author: John
 *
 * Created on February 28, 2017, 3:28 PM
 */

#ifndef COMDEFS_H
#define	COMDEFS_H


#define LED_ON 0
#define LED_OFF 1

//LED DEFINITIONS
#define LED1 LATBbits.LATB15
#define LED2 LATBbits.LATB14
#define LED3 LATAbits.LATA7
#define LED4 LATAbits.LATA10
#define LED5 LATBbits.LATB13
#define LED6 LATBbits.LATB12
#define LED7 LATBbits.LATB11
#define LED8 LATBbits.LATB10

#define FastTransferAddress 6
#define MOUSE_GYRO_ADDRESS      7
#define SENSOR_ADDRESS          6

#define ROUTER_ADDRESS          2
#define FASTTRANS_HOST_ADDRESS  3
#define RECEIVE_OPERATOIN_INDEX 0
#define TRANS_Z_ANGLE 1
#define TRANS_MOUSE_Y_DISPLACEMENT 2

//CAN FASTTRANS ADDRESSING VALUES
#define CAN_BUMPERBOARD_ADDRESS 9
//GOING TO NEED ANOTHER ADDRESS AS WE HAVE BOTH LEFT AND RIGHT MOTOR BUMPER BOARDS

#define CAN_COMMAND_INDEX 1
#define CAN_COMMAND_DATA_INDEX 2

#define CAN_COMMAND_ROTATION 1  //rotate
#define CAN_COMMAND_ENCODER 2   //drive straight/backwards
#define CAN_COMMAND_PAUSE 3     //Pause the cammand that is running



#endif	/* COMDEFS_H */

