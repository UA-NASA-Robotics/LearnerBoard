#include <xc.h>
#include <stdbool.h>
//#include "Initialize.h"
#include "comDefs.h"

#define FOSC    (72000000ULL)
#define FCY     (FOSC/2)
#include <libpic30.h>

#include "I2C_API_GYRO.h"


// general struct for storing settings

struct operator_values
{
    unsigned char slave_address;
    unsigned char data_address;
    unsigned char * data;
    unsigned char how_much_data;
    unsigned char data_index;
    unsigned char direction;
    unsigned char status;
};

// initialize the setting struct
static struct operator_values I2C_1_values = {0, 0, 0, 0, 0, 0, 1};

// function pointer for transition functions
void (*FunctionI2Cone)(void);

void InitI2Cone(void)
{
    I2C1BRG = 500.8;//591*4; // set baud rate (edited back FROM 591*4)
    IPC4bits.MI2C1IP = 2; // priority level 2
    IFS1bits.MI2C1IF = 0; // clear flag
    IEC1bits.MI2C1IE = 1; // enable interrupt flag
    I2C1CONbits.I2CEN = 1; // enable i2c one
}





// initiates a send of an array containing a set number of data

bool SendI2Cone(unsigned char s_address, unsigned char d_address, unsigned char * dat, unsigned char how_much)
{
    //LED3 ^= 1;
    // see if a transmit or receive is in prograss
    if ((I2C_1_values.status == SUCCESS) || (I2C_1_values.status == FAILED))
    {
        //populate struct with needed data
        I2C_1_values.slave_address = s_address << 1;
        I2C_1_values.data_address = d_address;
        I2C_1_values.data = dat;
        I2C_1_values.how_much_data = how_much;
        I2C_1_values.data_index = 0;
        I2C_1_values.direction = TRANSMIT;
        I2C_1_values.status = PENDING;
        FunctionI2Cone = &SendSlaveAddressI2Cone; // load the send slave address function
        I2C1CONbits.SEN = 1; // send start condition
        return true; // return successful
    }
    else
    {
        return false; // return failed if an i2c request is already running
    }
}

// initiate a receive moving data to an array of a set number of data

bool ReceiveI2Cone(unsigned char s_address, unsigned char d_address, unsigned char * dat, unsigned char how_much)
{
   
    //see if a transmit or receive is in prograss
    if ((I2C_1_values.status == SUCCESS) || (I2C_1_values.status == FAILED))
    {
        //LATEbits.LATE5 ^= 1;
        // __delay_ms(100);
        //populate struct with needed data
        I2C_1_values.slave_address = s_address << 1;
        I2C_1_values.data_address = d_address;
        I2C_1_values.data = dat;
        I2C_1_values.how_much_data = how_much;
        I2C_1_values.data_index = 0;
        I2C_1_values.direction = RECEIVE;
        I2C_1_values.status = PENDING;
        FunctionI2Cone = &SendSlaveAddressI2Cone; // load the send slave address function
        I2C1CONbits.SEN = 1; // send start condition
        return true; // return successful
    }
    else
    {           
        return false; // return failed if an i2c request is already running
    }
}

// send the slave address

void SendSlaveAddressI2Cone(void)
{
    if(I2C_1_values.direction == RECEIVE)
    {
        I2C1TRN = I2C_1_values.slave_address | 0x00; // load slave address into buffer
    }
    else
    {
        I2C1TRN = I2C_1_values.slave_address | 0x00; // load slave address into buffer
    }
    
    FunctionI2Cone = &SendDataAddressI2Cone; // load the send data address function
    
}

// send data address if receiving or send files byte if sending

void SendDataAddressI2Cone(void)
{
    // if ack is recieved then slave responded
    if (I2C1STATbits.ACKSTAT == 0) //ack received
    {
      
        // check the direction sending or receiving
        if (I2C_1_values.direction == RECEIVE) // receiving
        {
            I2C1TRN = I2C_1_values.data_address; // load data address value
            FunctionI2Cone = &SendRestartI2Cone; // load send restart function
        }
        else if (I2C_1_values.direction == TRANSMIT) // transmitting
        {
            I2C1TRN = I2C_1_values.data_address; // load data address value
            FunctionI2Cone = &SendDataI2Cone; // load function that will continue sending
        }
        else //neither transmit or receive (just in case)
        {
            StopFunctionI2Cone(); // initiate stop
            FunctionI2Cone = &FailFunctionI2Cone; // load fail function
        }
       
    }
    else //nack received
    {
        StopFunctionI2Cone(); // since nack redeived stop the buss
        FunctionI2Cone = &FailFunctionI2Cone; // load fail function
        // LATGbits.LATG8 ^= 1;
    }
}

void SendDataI2Cone(void)
{
    if (I2C1STATbits.ACKSTAT == 0) //ack received
    {
        //if index is less than how much data, send data and increment index
        if (I2C_1_values.data_index < I2C_1_values.how_much_data)
        {
            I2C1TRN = I2C_1_values.data[I2C_1_values.data_index]; // load data into buffer
            I2C_1_values.data_index++; // increment index
        }
        else //all data has been sent
        {
            StopFunctionI2Cone(); // since all data hase been sent initiate stop
            FunctionI2Cone = &SuccessFunctionI2Cone; // load sucess function
        }
    }
    else //nack received
    {
        StopFunctionI2Cone(); // since nack redeived stop the buss
        FunctionI2Cone = &FailFunctionI2Cone; // load fail function
    }
}

// send a stop to then later send start

void SendRestartI2Cone(void)
{
    I2C1CONbits.PEN = 1; //send stop
    FunctionI2Cone = &SendStartI2Cone; // load start function
}

// send start as a followup to the restart

void SendStartI2Cone(void)
{
    I2C1CONbits.SEN = 1; // send start condition
    FunctionI2Cone = &SendReadRequestI2Cone; // load send read request function
}


// send read request

void SendReadRequestI2Cone(void)
{
    I2C1TRN = (I2C_1_values.slave_address + 1); // send slave address plus 1
    FunctionI2Cone = &FirstReceiveI2Cone; // load first receive function
}

void FirstReceiveI2Cone(void)
{
    
    if (I2C1STATbits.ACKSTAT == 0) //ack received
    {
        I2C1CONbits.RCEN = 1; // enable receive
        FunctionI2Cone = &ReceiveByteI2Cone;
    }
    else //nack received
    {
        StopFunctionI2Cone();
        FunctionI2Cone = &FailFunctionI2Cone;
    }
}

void ReceiveByteI2Cone(void)
{
    I2C_1_values.data[I2C_1_values.data_index] = I2C1RCV;
    I2C_1_values.data_index++;
    if (I2C_1_values.data_index < I2C_1_values.how_much_data)
    {
        I2C1CONbits.ACKDT = 0; //Setup ACK (EDITED()
        I2C1CONbits.ACKEN = 1; // send ACK
        FunctionI2Cone = &EnableReceiveI2Cone;
    }
    else
    {
         I2C1CONbits.ACKDT = 1; //Setup NACK (EDITED()
        I2C1CONbits.ACKEN = 1; // send NACK
        FunctionI2Cone = &NACKFollowUpI2Cone;
    }
}

void EnableReceiveI2Cone(void)
{
    I2C1CONbits.RCEN = 1; // enable receive
    FunctionI2Cone = &ReceiveByteI2Cone;
}
void NACKFollowUpI2Cone(void)
{
    StopFunctionI2Cone();
    FunctionI2Cone = &SuccessFunctionI2Cone;
}
void StopFunctionI2Cone(void)
{
    I2C1CONbits.PEN = 1; //send stop
}

void FailFunctionI2Cone(void)
{
    
    I2C_1_values.status = FAILED;
   
}

void SuccessFunctionI2Cone(void)
{
    I2C_1_values.status = SUCCESS;
    
}

unsigned char StatusI2Cone(void)
{
    return I2C_1_values.status;
}

void __attribute__((interrupt, no_auto_psv)) _MI2C1Interrupt(void)
{
    //LED2 ^= 1;
    if (I2C1STATbits.BCL == 1)
    {
        StopFunctionI2Cone();
        FunctionI2Cone = &FailFunctionI2Cone;
        I2C1STATbits.BCL = 0;
         
    }
    else
    {
        FunctionI2Cone();
        //LATEbits.LATE7 ^= 1;
    }
    IFS1bits.MI2C1IF = 0; // clear interrupt flag
}

/******************************************************************************/
//
//void InitI2Ctwo(void)
//{
//    I2C2BRG = 591; // set baud rate 
//    IPC12bits.MI2C2IP = 2; // priority level 2
//    IFS3bits.MI2C2IF = 0; // clear flag
//    IEC3bits.MI2C2IE = 1; // enable interrupt flag
//    I2C2CONbits.I2CEN = 1; // enable i2c one
//}
//
//static struct operator_values I2C_2_values = {0, 0, 0, 0, 0, 0, 1};
//
//void (*FunctionI2Ctwo)(void);
//
//
//bool SendI2Ctwo(unsigned char s_address,  unsigned char d_address, unsigned char * dat, unsigned char how_much)
//{
//    //see if a transmit or receive is in prograss
//    if ((I2C_2_values.status == SUCCESS) || (I2C_2_values.status == FAILED))
//    {
//        //populate struct with needed data
//        I2C_2_values.slave_address = s_address << 1;
//        I2C_2_values.slave_address = d_address;
//        I2C_2_values.data = dat;
//        I2C_2_values.how_much_data = how_much;
//        I2C_2_values.data_index = 0;
//        I2C_2_values.direction = TRANSMIT;
//        I2C_2_values.status = PENDING;
//        FunctionI2Ctwo = &SendSlaveAddressI2Ctwo;
//        I2C2CONbits.SEN = 1; // send start condition
//        return true;
//    }
//    else
//    {
//        return false; //operation already in progress
//    }
//}
//
//bool ReceiveI2Ctwo(unsigned char s_address, unsigned char d_address, unsigned char * dat, unsigned char how_much)
//{
//    //see if a transmit or receive is in prograss
//    if ((I2C_2_values.status == SUCCESS) || (I2C_2_values.status == FAILED))
//    {
//        //populate struct with needed data
//        I2C_2_values.slave_address = s_address << 1;
//        I2C_2_values.data_address = d_address;
//        I2C_2_values.data = dat;
//        I2C_2_values.how_much_data = how_much;
//        I2C_2_values.data_index = 0;
//        I2C_2_values.direction = RECEIVE;
//        I2C_2_values.status = PENDING;
//        FunctionI2Ctwo = &SendSlaveAddressI2Ctwo;
//        I2C2CONbits.SEN = 1; // send start condition
//        return true;
//    }
//    else
//    {
//        return false; //operation already in progress
//    }
//}
//
//void SendSlaveAddressI2Ctwo(void)
//{
//    I2C2TRN = I2C_2_values.slave_address | 0x00;
//    FunctionI2Ctwo = &SendDataAddressI2Ctwo;
//}
//
//void SendDataAddressI2Ctwo(void)
//{
//    if (I2C2STATbits.ACKSTAT == 0) //ack received
//    {
//
//        if (I2C_2_values.direction == RECEIVE)
//        {
//            I2C2TRN = I2C_2_values.data_address;
//            FunctionI2Ctwo = &SendRestartI2Ctwo;
//        }
//        else if (I2C_2_values.direction == TRANSMIT)
//        {
//            I2C2TRN = I2C_2_values.data_address; // load data address value
//            FunctionI2Ctwo = &SendDataI2Ctwo; // load function that will continue sending
//        }
//        else //neither transmit or receive
//        {
//            StopFunctionI2Ctwo();
//            FunctionI2Ctwo = &FailFunctionI2Ctwo;
//        }
//    }
//    else //nack received
//    {
//        StopFunctionI2Ctwo();
//        FunctionI2Ctwo = &FailFunctionI2Ctwo;
//    }
//}
//
//void SendDataI2Ctwo(void)
//{
//    if (I2C2STATbits.ACKSTAT == 0) //ack received
//    {
//        //if index is less than how much data, send data and increment index
//        if (I2C_2_values.data_index < I2C_2_values.how_much_data)
//        {
//            I2C2TRN = I2C_2_values.data[I2C_2_values.data_index];
//            I2C_2_values.data_index++;
//        }
//        else //all data has been sent
//        {
//            StopFunctionI2Ctwo();
//            FunctionI2Ctwo = &SuccessFunctionI2Ctwo;
//        }
//    }
//    else //nack received
//    {
//        StopFunctionI2Ctwo();
//        FunctionI2Ctwo = &FailFunctionI2Ctwo;
//    }
//}
//
//void SendRestartI2Ctwo(void)
//{
//    I2C2CONbits.PEN = 1; //send stop
//    FunctionI2Ctwo = &SendStartI2Ctwo; // load start function
//}
//
//void SendStartI2Ctwo(void)
//{
//    I2C2CONbits.SEN = 1; // send start condition
//    FunctionI2Ctwo = &SendReadRequestI2Ctwo;
//}
//
//
//
//void SendReadRequestI2Ctwo(void)
//{
//    I2C2TRN = (I2C_2_values.slave_address + 1);
//    FunctionI2Ctwo = &FirstReceiveI2Ctwo;
//}
//
//void FirstReceiveI2Ctwo(void)
//{
//    if (I2C2STATbits.ACKSTAT == 0) //ack received
//    {
//        I2C2CONbits.RCEN = 1; // enable receive
//        FunctionI2Ctwo = &ReceiveByteI2Ctwo;
//    }
//    else //nack received
//    {
//        StopFunctionI2Ctwo();
//        FunctionI2Ctwo = &FailFunctionI2Ctwo;
//    }
//}
//
//void ReceiveByteI2Ctwo(void)
//{
//    I2C_2_values.data[I2C_2_values.data_index] = I2C2RCV;
//    I2C_2_values.data_index++;
//    if (I2C_2_values.data_index < I2C_2_values.how_much_data)
//    {
//        I2C2CONbits.ACKDT = 0; //Setup ACK
//        I2C2CONbits.ACKEN = 1; // send ACK
//        FunctionI2Ctwo = &EnableReceiveI2Ctwo;
//    }
//    else
//    {
//        I2C2CONbits.ACKDT = 1; //Setup ACK (EDITED()
//        I2C2CONbits.ACKEN = 1; // send ACK
//        FunctionI2Ctwo= &NACKFollowUpI2Ctwo;
//        //StopFunctionI2Ctwo();
//        //FunctionI2Ctwo = &SuccessFunctionI2Ctwo;
//    }
//}
//
//void EnableReceiveI2Ctwo(void)
//{
//    I2C2CONbits.RCEN = 1; // enable receive
//    FunctionI2Ctwo = &ReceiveByteI2Ctwo;
//}
//void NACKFollowUpI2Ctwo(void)
//{
//    StopFunctionI2Ctwo();
//    FunctionI2Cone = &SuccessFunctionI2Ctwo;
//}
//
//void StopFunctionI2Ctwo(void)
//{
//    I2C2CONbits.PEN = 1; //send stop
//}
//
//void FailFunctionI2Ctwo(void)
//{
//    I2C_2_values.status = FAILED;
//}
//
//void SuccessFunctionI2Ctwo(void)
//{
//    I2C_2_values.status = SUCCESS;
//}
//
//unsigned char StatusI2Ctwo(void)
//{
//    return I2C_2_values.status;
//}
//
//void __attribute__((interrupt, no_auto_psv)) _MI2C2Interrupt(void)
//{
//    if (I2C2STATbits.BCL == 1)
//    {
//        StopFunctionI2Ctwo();
//        FunctionI2Ctwo = &FailFunctionI2Ctwo;
//        I2C2STATbits.BCL = 0;
//    }
//    else
//    {
//        FunctionI2Ctwo();
//    }
//    IFS3bits.MI2C2IF = 0; // clear interrupt flag
//}

bool writeBits(char devAddr, char regAddr, char bitStart, char length, char data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    int b;
    if (ReceiveI2Cone(devAddr, regAddr,(unsigned char *) &b, 1) != 0) {
        char mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return SendI2Cone(devAddr, regAddr,(unsigned char *) &b,1);
    } else {
        return false;
    }
}

bool writeBit(char devAddr, char regAddr, char bitNum, char data) {
    char b;
    
//    ReceiveI2Cone(devAddr, regAddr,(unsigned char *) &b, 1);
//    while(StatusI2Cone() == PENDING);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return SendI2Cone(devAddr, regAddr,(unsigned char *) &b, 1);
}
