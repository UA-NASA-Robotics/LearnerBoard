#include <p33EP512GP504.h>
#include "SPI_lib.h"
#include "main.h"
#include "comDefs.h"

void SPI_Transfer(char data)
{
    int dummy = SPI1BUF;    //Dummy var to clear the value
    SPI1BUF = (0x00FF & data);
    while(!SPI1BUF);        //wait for the data to be sent out
    //LED1 ^= 1;
    
}
int Read_MouseSensor(unsigned char _address)
{
    //LED2 ^= 1;
    unsigned int data; 
    // take the CS pin low to select the chip: 
    //digitalWrite(nCS,LOW); 
    //  send in the address and value via SPI: 
    delay_us(100); 
    SPI_Transfer(_address); 

    SPI1CON1bits.DISSDO = 1;
    TRISAbits.TRISA4 = 1; 
    delay_us(100); 
    SPI_Transfer(0x00); 
    SPI1CON1bits.DISSDO = 0;
    TRISAbits.TRISA4 = 0; 
    delay_us(100); 
    return(SPI1BUF);
}
void Write_MouseSensor(unsigned char _address, unsigned char _data)
{
 
    delay_us(100); 
    SPI_Transfer(_address); 
    delay_us(100); 
    SPI_Transfer(_data); 


    
}
    
