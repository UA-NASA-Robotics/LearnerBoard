#include <p33EP512GP504.h>
#include <stdbool.h>
#include "FastTransfer.h"
#include "Initialize.h"
#include "comDefs.h"
#include "UART_handler.h"
#include "CAN_Lib/CANFastTransfer.h"

#include "SPI_lib.h"

#define pinModeLED1 TRISBbits.TRISB15
#define pinModeLED2 TRISBbits.TRISB14
#define pinModeLED3 TRISAbits.TRISA7
#define pinModeLED4 TRISAbits.TRISA10
#define pinModeLED5 TRISBbits.TRISB13
#define pinModeLED6 TRISBbits.TRISB12
#define pinModeLED7 TRISBbits.TRISB11
#define pinModeLED8 TRISBbits.TRISB10

#define GLOBAL_INTERRUPTS  INTCON2bits.GIE
int receiveArray[20];
void Start_Initialization()
{
    INTCON1bits.NSTDIS=1;
    GLOBAL_INTERRUPTS = OFF;
    //Initialization Function Calls go in here<GLOBAL_INTERRUPTS(OFF)/> to <GLOBAL_INTERRUPTS(ON)>
    oscillator();
    pinModeLED1 = OUTPUT;
    pinModeLED2 = OUTPUT;
    pinModeLED3 = OUTPUT;
    pinModeLED4 = OUTPUT;
    pinModeLED5 = OUTPUT;
    pinModeLED6 = OUTPUT;
    pinModeLED7 = OUTPUT;
    pinModeLED8 = OUTPUT;
    //timerOne();
    timerTwo();
    UART_init();
    init_SPI();
    //begin(receiveArray, sizeof(receiveArray),FastTransferAddress,false, Send_put,Receive_get,Receive_available,Receive_peek);
    beginCANFast(receiveArray, sizeof(receiveArray), MOUSE_GYRO_ADDRESS);
    GLOBAL_INTERRUPTS = ON;
}


void oscillator(void)
{
    // 120MHz 60MIPS
    // Configure PLL prescaler, PLL postscaler, PLL divisor
    PLLFBD = 58; // M=60
    CLKDIVbits.PLLPOST = 0; // N2=2
    CLKDIVbits.PLLPRE = 4; // N1=6

    // Initiate Clock Switch to PRI oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONH(0x03);
    __builtin_write_OSCCONL(OSCCON | 0x01);

    // Wait for Clock switch to occur
    while (OSCCONbits.COSC != 0b011);
    //    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1);
}
void timerOne(void)
{

    T1CONbits.TCKPS = 0b10; // 64 divider
    PR1 = 938;              // 0.001s timer
    IPC0bits.T1IP = 1;      // interrupt priority level 1
    IFS0bits.T1IF = 0;      // clear interrupt flag
    IEC0bits.T1IE = 1;      // enable timer 1 interrupt
    T1CONbits.TON = 1;      // turn on timer
}
void timerTwo(void)
{
    T2CONbits.TCKPS = 0b11; // 256 divider
    PR2 = 2350; // 0.01s timer
    IPC1bits.T2IP = 1; // interrupt priority level 1
    IFS0bits.T2IF = 0; // clear interrupt flag
    IEC0bits.T2IE = 1; // enable timer 2 interrupt
    T2CONbits.TON = 1; // turn on timer
    
}
void init_SPI()
{
    SPI1STATbits.SPIEN = 0;     //disables the module and configures SCK1, SDO1, SDI1, and SS1 as serial port pins
    SPI1CON1bits.MODE16 = 0;    //Communication is byte-wide (8bits))
    SPI1CON1bits.DISSDO = 0;    //SDO1 pin is controlled by the module
    SPI1CON1bits.CKE = 0;       //Serial output data changes on transition from active clock state to Idle clock state (refer to bit 6)
    SPI1CON1bits.CKP = 1;       // Idle state for clock is a high level; active state is a low level
    SPI1CON1bits.SSEN = 0;      //SS1 pin is not used by the module; pin is controlled by port function
    SPI1CON1bits.SPRE = 0;      //Secondary Prescale
    SPI1CON1bits.PPRE = 2;      //Primary Prescale
    SPI1CON1bits.MSTEN = 1;     //Master Mode bit Enabled
    SPI1CON2bits.FRMEN = 0;     // FRAMEN = 1, SPIFSD = 0,  //make the clock line continously oscillate when set to 1
    SPI1CON2bits.SPIFSD = 0;    //0 = Frame Sync pulse output (master)                 
    SPI1CON2bits.FRMPOL = 1;    //Polarity of Frame sync pulse
   // SPI1STATbits.SISEL = 0b101; //Interrupt when the last bit is shifted out of SPI1SR and the transmit is complete          //interrupt bits
   // IFS0bits.SPI1IF = 0;        //Setting the interrupt flag
    SPI1STATbits.SPIEN = 1;     //Enables the module and configures SCK1, SDO1, SDI1, and SS1 as serial port pins
}
