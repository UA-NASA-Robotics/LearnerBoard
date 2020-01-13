//#define BAUD_1000K
#define BAUD_500K

#include <xc.h>
#include "ecan1_config.h"
#include "..\..\..\tracee\Common\CommsDefenition.h"
#include "definitions.h"
#include "../comDefs.h"
/******************************************************************************
 * Function:      void DMA0Init(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      DMA0 initialization/configuration function.
 *                Direction: Read from RAM and write to the C1TXD register
 *                AMODE: Register indirect with post increment
 *                MODE: Continuous, Ping-Pong Mode
 *                IRQ: ECAN1 Transmit Interrupt
 *****************************************************************************/
void DMA0Init( void )
{
    
    DMA0CON = 0x2020;
    DMA0PAD = ( int ) &C1TXD;   /* ECAN 1 (C1TXD) */
    DMA0CNT = 0x0007;
    DMA0REQ = 0x0046;           /* ECAN 1 Transmit */

    #ifdef _HAS_DMA_
    DMA0STAL = __builtin_dmaoffset( ecan1msgBuf );
    DMA0STAH = __builtin_dmapage( ecan1msgBuf );
    #else
    DMA0STAL = (uint16_t)(int_least24_t)(&ecan1msgBuf);
    DMA0STAH = 0;
    #endif
    DMA0CONbits.CHEN = 1;
}

/******************************************************************************
 * Function:      void DMA2Init(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      DMA0 initialization/configuration function.
 *                Direction: Read from RAM and write to the C1RXD register
 *                AMODE: Register indirect with post increment
 *                MODE: Continuous, Ping-Pong Mode
 *                IRQ: ECAN1 Transmit Interrupt
 *****************************************************************************/
void DMA2Init( void )
{
    //     DMACS0=0;
    //  DMAPWC =0;
    // DMARQC =0;
    DMA2CON = 0x0020;
    DMA2PAD = ( int ) &C1RXD;   /* ECAN 1 (C1RXD) */
    DMA2CNT = 0x0007;
    DMA2REQ = 0x0022;           /* ECAN 1 Receive */

    #ifdef _HAS_DMA_
    DMA2STAL = __builtin_dmaoffset( ecan1msgBuf );
    DMA2STAH = __builtin_dmapage( ecan1msgBuf );
    #else
    DMA2STAL = (uint16_t)(int_least24_t)(&ecan1msgBuf);
    DMA2STAH = 0;
    //DMA2STAL = (unsigned int) (&ecan1msgBuf);  //clean
    //DMA2STAH = (unsigned int) (&ecan1msgBuf);
    #endif
    DMA2CONbits.CHEN = 1;
}

/******************************************************************************
 * Function:      void Ecan1ClkInit(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      ECAN1 clock initialization function
 *                This function is used to configure the clock used for the
 *                ECAN1 module during transmission/reception.
 *****************************************************************************/
void Ecan1ClkInit( void )
{
    //#define FP 40000000
    /* FCAN is selected to be FCY
   FCAN = FCY = 40MHz */
       // C1CTRL1bits.CANCKS = 0x1;
    /*
Bit Time = (Sync Segment + Propagation Delay + Phase Segment 1 + Phase Segment 2)=20*TQ
Phase Segment 1 = 8TQ
Phase Segment 2 = 6Tq
Propagation Delay = 5Tq
Sync Segment = 1TQ
CiCFG1<BRP> =(FCAN /(2 ×N×FBAUD))– 1
     * Total Tq = Sync + Prop + phase1 + phase2
     *       10 = 1    +  2   +   5    +   2 
     * 
     * 
*/
    /* Synchronization Jump Width set to 1 TQ */
    C1CFG1bits.SJW = 0x3;
    
    
    /* Baud Rate Prescaler */
#ifdef BAUD_1000K
    
    C1CFG1bits.BRP = 1; 
    
    /* Phase Segment 1 time is 8 TQ */   //Setting to 5
    C1CFG2bits.SEG1PH = 0x4;

    /* Phase Segment 2 time is set to be programmable */
    C1CFG2bits.SEG2PHTS = 0x1;

    /* Phase Segment 2 time is 6 TQ */  //Setting to 6
    C1CFG2bits.SEG2PH = 0x5;

    /* Propagation Segment time is 5 TQ */  //Setting to 7
    C1CFG2bits.PRSEG = 0x6;

    /* Bus line is sampled three times at the sample point */
    C1CFG2bits.SAM = 0x1;
#elif defined BAUD_500K
    
    C1CFG1bits.BRP = 2;  
    
    /* Phase Segment 1 time is 8 TQ */   //Setting to 5
    C1CFG2bits.SEG1PH = 0x4;

    /* Phase Segment 2 time is set to be programmable */
    C1CFG2bits.SEG2PHTS = 0x1;

    /* Phase Segment 2 time is 6 TQ */  //Setting to 2
    C1CFG2bits.SEG2PH = 0x7;

    /* Propagation Segment time is 5 TQ */  //Setting to 7
    C1CFG2bits.PRSEG = 0x5;

    /* Bus line is sampled three times at the sample point */
    C1CFG2bits.SAM = 0x1;
#endif
  
}

/******************************************************************************
 * Function:     void Ecan1Init(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      ECAN1 initialization function.This function is used to
 *                initialize the ECAN1 module by configuring the message
 *                buffers, and the acceptance filters and
 *                setting appropriate masks for the same.
 *****************************************************************************/
void Ecan1Init( void )
{
    /* Request Configuration Mode */
    C1CTRL1bits.REQOP = 4;
    while( C1CTRL1bits.OPMODE != 4 );

    Ecan1ClkInit();

    C1FCTRLbits.FSA = 0b01000;     /* FIFO Starts at Message Buffer 8 */
    C1FCTRLbits.DMABS = 0b110;     /* 32 CAN Message Buffers in DMA RAM */

    /*	Filter Configuration

    Ecan1WriteRxAcptFilter(int n, long identifier, unsigned int exide,unsigned int bufPnt,unsigned int maskSel)

    n = 0 to 15 -> Filter number

    identifier -> SID <10:0> : EID <17:0> 
     * 
     * 

    exide = 0 -> Match messages with standard identifier addresses 
    exide = 1 -> Match messages with extended identifier addresses 

    bufPnt = 0 to 14  -> RX Buffer 0 to 14
    bufPnt = 15 -> RX FIFO Buffer

    maskSel = 0    ->    Acceptance Mask 0 register contains mask
    maskSel = 1    ->    Acceptance Mask 1 register contains mask
    maskSel = 2    ->    Acceptance Mask 2 register contains mask
    maskSel = 3    ->    No Mask Selection
    
*/
    Ecan1WriteRxAcptFilter( 0, MOUSE_GYRO_ADDRESS << 6, 0, 15, 0 );
    
    /*    Mask Configuration

    Ecan1WriteRxAcptMask(int m, long identifierMask, unsigned int mide, unsigned int exide)

    m = 0 to 2 -> Mask Number

    identifier -> SID <10:0> : EID <17:0> 

    mide = 0 -> Match either standard or extended address message if filters match 
    mide = 1 -> Match only message types that correpond to 'exide' bit in filter

    exide = 0 -> Match messages with standard identifier addresses 
    exide = 1 -> Match messages with extended identifier addresses
    
*/
    Ecan1WriteRxAcptMask( 0, 0b11111100000, 1, 0); 
    /* Enter Normal Mode */
    C1CTRL1bits.REQOP = 0;
    while( C1CTRL1bits.OPMODE != 0 );

    /* ECAN transmit/receive message control */
    C1RXFUL1 = C1RXFUL2 = C1RXOVF1 = C1RXOVF2 = 0x0000;
    C1TR01CONbits.TXEN0 = 1;        /* ECAN1, Buffer 0 is a Transmit Buffer */
    C1TR01CONbits.TXEN1 = 1;        /* ECAN1, Buffer 1 is a Transmit Buffer */
    C1TR23CONbits.TXEN2 = 1;        /* ECAN1, Buffer 2 is a Transmit Buffer */
    C1TR23CONbits.TXEN3 = 1;        /* ECAN1, Buffer 3 is a Transmit Buffer */
    C1TR45CONbits.TXEN4 = 1;        /* ECAN1, Buffer 4 is a Transmit Buffer */
    C1TR45CONbits.TXEN5 = 1;        /* ECAN1, Buffer 5 is a Transmit Buffer */
    C1TR67CONbits.TXEN6 = 1;        /* ECAN1, Buffer 6 is a Transmit Buffer */
    C1TR67CONbits.TXEN7 = 1;        /* ECAN1, Buffer 7 is a Transmit Buffer */

    C1TR01CONbits.TX0PRI = 0b11;   /* Message Buffer 0 Priority Level */
    C1TR01CONbits.TX1PRI = 0b11;   /* Message Buffer 1 Priority Level */
    C1TR23CONbits.TX2PRI = 0b11;   /* Message Buffer 2 Priority Level */
    C1TR23CONbits.TX3PRI = 0b11;   /* Message Buffer 3 Priority Level */
    C1TR45CONbits.TX4PRI = 0b11;   /* Message Buffer 4 Priority Level */
    C1TR45CONbits.TX5PRI = 0b11;   /* Message Buffer 5 Priority Level */
    C1TR67CONbits.TX6PRI = 0b11;   /* Message Buffer 6 Priority Level */
    C1TR67CONbits.TX7PRI = 0b11;   /* Message Buffer 7 Priority Level */

    /* Setup I/O pins */
    // TRISF = 1;
    
    // The PPS configuration varies from device to device. Refer the datasheet of the device being used and
    // use the appropriate values for the RPINR/RPOR registers.
    TRISCbits.TRISC6=1;
    TRISCbits.TRISC7=1;
    RPINR26 = 0;                    //clear register
    RPINR26bits.C1RXR = 55;         //set CAN1 RX to RP96    (87)
    RPOR5bits.RP54R = 14;           //RPOR7bits.RP97R = 14; set CAN1TX to RP97        (88)
}

/*******************************************************************************
 End of File
*/
