#include <xc.h>
#include <stdbool.h>
#include <stdlib.h>
#include "UART_handler.h"
#include "comDefs.h"

#define ON         0
#define OFF        1
#define INDICATOR1 LATEbits.LATE5
#define INDICATOR2 LATEbits.LATE6
#define INDICATOR3 LATGbits.LATG7
#define INDICATOR4 LATGbits.LATG8
#define WATCHDOG   LATEbits.LATE7



struct UART_ring_buff input_buffer;
struct UART_ring_buff output_buffer;

struct UART_ring_buff GYRO_buffer;

bool Transmit_stall = true;
bool Transmit_stall_gyro = true;

void UART_init(void)
{
   
        //****************************UART*************************
        RPINR18bits.U1RXR = 0b0100100; // RP36 for RX
        RPOR1bits.RP37R = 1; //configures the output TX to configurable pin RP65

        U1MODEbits.STSEL = 0; // 1-stop bit
        U1MODEbits.PDSEL = 0; // No parity, 8-data bits
        U1MODEbits.ABAUD = 0; // Auto-baud disabled
        U1BRG = BAUD_RATE; // Baud Rate setting for 57600
        U1STAbits.URXISEL = 0b01; // Interrupt after all TX character transmitted
        IFS0bits.U1TXIF = 0; // Clear TX interrupt flag
        IEC0bits.U1TXIE = 1; // Enable TX interrupt
        U1MODEbits.UARTEN = 1; // Enable UART
        U1STAbits.UTXEN = 1; // Enable UART TX

        UART_buff_init(&input_buffer);
        UART_buff_init(&output_buffer);
    
}

void UART_buff_init(struct UART_ring_buff* _this)
{
    /*****
      The following clears:
        -> buf
        -> head
        -> tail
        -> count
      and sets head = tail
     ***/
    memset(_this, 0, sizeof (*_this));
}

void UART_buff_put(struct UART_ring_buff* _this, const unsigned char c)
{
    if (_this->count < UART_BUFFER_SIZE)
    {
        _this->buf[_this->head] = c;
        _this->head = UART_buff_modulo_inc(_this->head, UART_BUFFER_SIZE);
        ++_this->count;
    } else
    {
        _this->buf[_this->head] = c;
        _this->head = UART_buff_modulo_inc(_this->head, UART_BUFFER_SIZE);
        _this->tail = UART_buff_modulo_inc(_this->tail, UART_BUFFER_SIZE);

    }
}

unsigned char UART_buff_get(struct UART_ring_buff* _this)
{
    unsigned char c;
    if (_this->count > 0)
    {
        c = _this->buf[_this->tail];
        _this->tail = UART_buff_modulo_inc(_this->tail, UART_BUFFER_SIZE);
        --_this->count;
    } else
    {
        c = 0;
    }
    return (c);
}

void UART_buff_flush(struct UART_ring_buff* _this, const int clearBuffer)
{
    _this->count = 0;
    _this->head = 0;
    _this->tail = 0;
    if (clearBuffer)
    {
        memset(_this->buf, 0, sizeof (_this->buf));
    }
}

int UART_buff_size(struct UART_ring_buff* _this)
{
    return (_this->count);
}

unsigned int UART_buff_modulo_inc(const unsigned int value, const unsigned int modulus)
{
    unsigned int my_value = value + 1;
    if (my_value >= modulus)
    {
        my_value = 0;
    }
    return (my_value);
}

unsigned char UART_buff_peek(struct UART_ring_buff* _this)
{
    return _this->buf[_this->tail];
}

unsigned char Receive_peek(void)
{
    return UART_buff_peek(&input_buffer);
}

int Receive_available(void)
{
    return UART_buff_size(&input_buffer);
}

unsigned char Receive_get(void)
{
    return UART_buff_get(&input_buffer);
}

void Send_put(unsigned char _data)
{
    //LED1 ^=1;
    UART_buff_put(&output_buffer, _data);
    if(Transmit_stall == true)
    {
        Transmit_stall = false;
        U1TXREG = UART_buff_get(&output_buffer);
    }
}
void Send()
{
    if(Transmit_stall == true)
    {
        Transmit_stall = false;
        U1TXREG = UART_buff_get(&output_buffer);
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
    
   //LED2 ^=1;
    unsigned char data = U1RXREG;
    UART_buff_put(&input_buffer, data);
    IFS0bits.U1RXIF = 0; // Clear RX interrupt flag
    //INDICATOR1=OFF;
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void)
{
   
        if(GYRO_buffer.count > 0)
        {
            U1TXREG = GYRO_buffer.buf[GYRO_buffer.tail];
            GYRO_buffer.tail++;
            if(GYRO_buffer.tail >= GYRO_BUFFER_SIZE)GYRO_buffer.tail = 0;
            GYRO_buffer.count--;
        }
        
        else
        {
            Transmit_stall_gyro = true;
        }
    
        //IFS0bits.U1TXIF = 0; // Clear TX interrupt flag
  
        
    IFS0bits.U1TXIF = 0; // Clear TX interrupt flag
}


//****************************************************
//GYRO DEBUGING 
int absol(int givenNum)    
{
    //if you don't know then DON'T TOUCH!!
    if (givenNum > 0)
        return givenNum;
    else 
        return -givenNum;
    
}
void MessageParsser(int inputNumX,int inputNumY,int inputNumZ)
{
    
    
    
//    isNegative(&inputNumX);
//    loadHundreds(&inputNumX);
//    loadTens(&inputNumX);
//    loadUnits(&inputNumX);
//    isNegative(&inputNumY);
//    loadHundreds(&inputNumY);
//    loadTens(&inputNumY);
//    loadUnits(&inputNumY);
    isNegative(&inputNumZ);
    loadHundreds(&inputNumZ);
    loadTens(&inputNumZ);
    loadUnits(&inputNumZ);

    GYRO_buffer.buf[GYRO_buffer.head] = CARRIAGE_RETURN;
    maxedHead();

    if(Transmit_stall_gyro == true)
    {
        U1TXREG = GYRO_buffer.buf[GYRO_buffer.tail];
        GYRO_buffer.tail++;
        if(GYRO_buffer.tail >= GYRO_BUFFER_SIZE)GYRO_buffer.tail = 0;
        GYRO_buffer.count--;
        
        Transmit_stall_gyro = false;
    }
  
}
void isNegative(int *passNum)
{
    if(*passNum < 0)
    {
        GYRO_buffer.buf[GYRO_buffer.head] = NEGATION;
        GYRO_buffer.head++;
        GYRO_buffer.count++;
        *passNum = absol(*passNum);
        
        //add Negative to the buffer ring
        //then take the abs value
    } 
}
void loadHundreds(int *passNum)
{
    
    int hundreds = 0;
    while(*passNum > 100)    //if the angle is larger then one-Hundred we can find how many hundreds and push the asci value into the ring buffer
    {
        *passNum -= 100;
        hundreds++;
    }
    GYRO_buffer.buf[GYRO_buffer.head] = hundreds + 48;
    maxedHead();
   
    
}
void loadTens(int *passNum)
{
    int tens = 0;
    while(*passNum > 10)    //if the angle is larger then one-Hundred we can find how many hundreds and push the asci value into the ring buffer
    {
        *passNum -= 10;
        tens++;
    }
    GYRO_buffer.buf[GYRO_buffer.head] = tens + 48;
    maxedHead();
    
    
}
void loadUnits(int *passNum)
{
    int units = 0;
    while(*passNum > 1)    //if the angle is larger then one-Hundred we can find how many hundreds and push the asci value into the ring buffer
    {
        *passNum -= 1;
        units++;
    }
    GYRO_buffer.buf[GYRO_buffer.head] = units + 48;
    maxedHead();
    GYRO_buffer.buf[GYRO_buffer.head] = SPACE;
    maxedHead();
    
}
void maxedHead()
{
    GYRO_buffer.head++;
    GYRO_buffer.count++;
    if (GYRO_buffer.head >= GYRO_BUFFER_SIZE)
    {
        GYRO_buffer.head = 0;
    }
   
}
