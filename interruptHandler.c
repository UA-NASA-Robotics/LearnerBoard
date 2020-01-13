#include "interruptHandler.h"
#include "main.h"
#include "comDefs.h"
#include "gyro.h"
#include <xc.h>
#include <stdbool.h>
extern volatile unsigned int time;
extern volatile bool RUN_G_DATA;
volatile unsigned long milliCount = 0;
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    time++;
   
    IFS0bits.T1IF = 0; // clear interrupt flag
}
unsigned long millis(void)
{
    return milliCount;
}

//10ms timer
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
//    get_Movement_6();       //getting the data points from the GYRO
//    getAcceleration();      //Calculating the acceleration vectors
//    getAngles();            //Calculating the angles on the z, y and x axis
    RUN_G_DATA = true;
     milliCount++;
    //this will transmit the data that was request to be sent 
   // processData = true;
   // LED8 ^=1;
    IFS0bits.T2IF = 0; // clear interrupt flag
}

//------------------------------------------------------------------------------
//    DMA interrupt handlers
//------------------------------------------------------------------------------
/******************************************************************************
 * Function:   void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      Interrupt service routine to handle DMA0interrupt
 *****************************************************************************/
int dmacount=0;
void __attribute__ ( (interrupt, no_auto_psv) ) _DMA0Interrupt( void )
{
    if(dmacount<500)
        dmacount++;
    else
    {
        dmacount=0;
        LED4^=1;
    }
    IFS0bits.DMA0IF = 0;    // Clear the DMA0 Interrupt Flag;
}

/******************************************************************************
 * Function:   void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      Interrupt service routine to handle DMA1interrupt
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _DMA1Interrupt( void )
{
    IFS0bits.DMA1IF = 0;    // Clear the DMA1 Interrupt Flag;
}

/******************************************************************************
 * Function:   void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      Interrupt service routine to handle DMA2interrupt
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _DMA2Interrupt( void )
{
   // LED5^=1;
    //SetDBG3(1); 
    IFS1bits.DMA2IF = 0;    // Clear the DMA2 Interrupt Flag;
}

/******************************************************************************
 * Function:   void __attribute__((interrupt, no_auto_psv)) _DMA3Interrupt(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      Interrupt service routine to handle DMA3interrupt
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _DMA3Interrupt( void )
{
    IFS2bits.DMA3IF = 0;    // Clear the DMA3 Interrupt Flag;
}