/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */
/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef CAN_H
#define	CAN_H


#include <xc.h> // include processor files - each processor file is guarded. 
#include "ecan1_config.h" 
#include "definitions.h"
#include <stdbool.h>
//#include "test.h"



//circular buffer functions
typedef struct circular_buffer
{
	int capacity;  // maximum number of items in the buffer
	int count;     // number of items in the buffer
	int head;       // pointer to head
	int tail;       // pointer to tail
} circular_buffer;


typedef struct CAN_packet {
    unsigned int id;
    unsigned char length;
    unsigned char data[8];
} CAN_packet ;

void can_init(void *RXfunction); 
bool can_tx(CAN_packet *packet); 
CAN_packet can_get_FIFO(int buf);
//receiving handled in interrupt
//repeated sending handled in interrupt



void TX_cb_init();
void TX_cb_push_back( CAN_packet item);
CAN_packet TX_cb_pop_front();
int TX_cb_count();
 
void RX_cb_init();
void RX_cb_push_back( CAN_packet item);
CAN_packet RX_cb_pop_front();
int RX_cb_count();
#endif	/* XC_HEADER_TEMPLATE_H */

