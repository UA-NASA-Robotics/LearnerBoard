/* 
 * File:   UART_handler.h
 * Author: Igor
 *
 * Created on March 23, 2015, 11:15 AM
 */

#ifndef UART_HANDLER_H
#define	UART_HANDLER_H

#include "main.h"
#define UART_BUFFER_SIZE 200
#define GYRO_BUFFER_SIZE 50
#define SPACE               32
#define CARRIAGE_RETURN     13
#define NEGATION            45
#define BAUD_RATE (((60000000/115200)/16)-1)

void *memset(void *s, int c, int n);

struct UART_ring_buff {
    unsigned char buf[UART_BUFFER_SIZE];
    int head;
    int tail;
    int count;
};
struct DEBUG_ring_buff {
    unsigned char buf[GYRO_BUFFER_SIZE];
    int head;
    int tail;
    int count;
};

//struct UART_ring_buff input_buffer;
//struct UART_ring_buff output_buffer;
//
//bool Transmit_stall = true;


void UART_init(void);
void UART_buff_init(struct UART_ring_buff* _this);
void UART_buff_put(struct UART_ring_buff* _this, const unsigned char c);
unsigned char UART_buff_get(struct UART_ring_buff* _this);
void UART_buff_flush(struct UART_ring_buff* _this, const int clearBuffer);
int UART_buff_size(struct UART_ring_buff* _this);
unsigned int UART_buff_modulo_inc(const unsigned int value, const unsigned int modulus);
unsigned char UART_buff_peek(struct UART_ring_buff* _this);


unsigned char Receive_peek(void);
int Receive_available(void);
unsigned char Receive_get(void);
void Send_put(unsigned char _data);
void Send();

//FUNCTIONS FOR GYRO DEBUG MODE
void UartInit(void);
void MessageParsser(int,int,int);
int absol(int); 
void maxedHead();
void isNegative(int *);
void loadHundreds(int *);
void loadTens(int *);
void loadUnits(int *);

#endif	/* UART_HANDLER_H */

