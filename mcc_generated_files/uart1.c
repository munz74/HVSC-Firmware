/**
  UART1 Generated Driver File 

  @Company
    Microchip Technology Inc.

  @File Name
    uart1.c

  @Summary 
    This is the generated source file for the UART1 driver using MPLAB(c) Code Configurator

  @Description
    This source file provides APIs for driver for UART1. 
    Generation Information : 
        Product Revision  :  MPLAB(c) Code Configurator - pic24-dspic-pic32mm : v1.26
        Device            :  dsPIC33EV256GM002
    The generated drivers are tested against the following:
        Compiler          :  XC16 1.30
        MPLAB 	          :  MPLAB X 3.45
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/

#include "uart1.h"

/**
  Section: Data Type Definitions
*/

/** UART Driver Queue Status

  @Summary
    Defines the object required for the status of the queue.
*/

typedef union
{
    struct
    {
            uint8_t full:1;
            uint8_t empty:1;
            uint8_t reserved:6;
    }s;
    uint8_t status;
}

UART_BYTEQ_STATUS;

/** UART Driver Hardware Instance Object

  @Summary
    Defines the object required for the maintenance of the hardware instance.

*/
typedef struct
{
    /* RX Byte Q */
    uint8_t                                      *rxTail ;

    uint8_t                                      *rxHead ;

    /* TX Byte Q */
    uint8_t                                      *txTail ;

    uint8_t                                      *txHead ;

    UART_BYTEQ_STATUS                        rxStatus ;

    UART_BYTEQ_STATUS                        txStatus ;

} UART_OBJECT ;

static UART_OBJECT uart1_obj ;

/** UART Driver Queue Length

  @Summary
    Defines the length of the Transmit and Receive Buffers

*/

#define UART1_CONFIG_TX_BYTEQ_LENGTH 8
#define UART1_CONFIG_RX_BYTEQ_LENGTH 8

/** UART Driver Queue

  @Summary
    Defines the Transmit and Receive Buffers

*/

static uint8_t uart1_txByteQ[UART1_CONFIG_TX_BYTEQ_LENGTH] ;
static uint8_t uart1_rxByteQ[UART1_CONFIG_RX_BYTEQ_LENGTH] ;

/**
  Section: Driver Interface
*/

void UART1_Initialize(void)
{
    // Set the UART1 module to the options selected in the user interface.

    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; URXINV disabled; UEN TX_RX; 
    U1MODE = (0x8008 & ~(1<<15));  // disabling UARTEN bit   
    // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
    // DMA설정  :  For the UART receiver to generate an RX interrupt for each character received, 
    //            Receive Interrupt Mode Selection bits (URXISEL<1:0>) must be set to ‘00’ or ‘01’ in the Status 
    //            and Control register (UxSTA).  DMA설정을 위해서 필요한것 70348C.pdf 23쪽 확인 
    U1STA = 0x0;
    // BaudRate = 230400; Frequency = 24000000 Hz; BRG 25; 
    U1BRG = 0x19;
   IEC0bits.U1RXIE = 1;

   //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
  
   U1MODEbits.UARTEN = 1;  // enabling UARTEN bit
   U1STAbits.UTXEN = 1; 
 
/*    
   uart1_obj.txHead = uart1_txByteQ;
   uart1_obj.txTail = uart1_txByteQ;
   uart1_obj.rxHead = uart1_rxByteQ;
   uart1_obj.rxTail = uart1_rxByteQ;
   uart1_obj.rxStatus.s.empty = true;
   uart1_obj.txStatus.s.empty = true;
   uart1_obj.txStatus.s.full = false;
   uart1_obj.rxStatus.s.full = false;
*/

}




/**
    Maintains the driver's transmitter state machine and implements its ISR
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1TXInterrupt ( void )  //  DMA 사용시 아무것도 하지 않는다. 
{ 
  
    IFS0bits.U1TXIF = false;  // Clear interrupt
    /*
    
    if(uart1_obj.txStatus.s.empty)
    {
        IEC0bits.U1TXIE = false;
        return;
    }



    while(!(U1STAbits.UTXBF == 1))
    {

        U1TXREG = *uart1_obj.txHead;

        uart1_obj.txHead++;

        if(uart1_obj.txHead == (uart1_txByteQ + UART1_CONFIG_TX_BYTEQ_LENGTH))
        {
            uart1_obj.txHead = uart1_txByteQ;
        }

        uart1_obj.txStatus.s.full = false;

        if(uart1_obj.txHead == uart1_obj.txTail)
        {
            uart1_obj.txStatus.s.empty = true;
            break;
        }
    }   */
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1RXInterrupt( void )  //  DMA 사용시 아무것도 하지 않는다. 
{
   /*
    
    while((U1STAbits.URXDA == 1))
    {

        *uart1_obj.rxTail = U1RXREG;

        uart1_obj.rxTail++;

        if(uart1_obj.rxTail == (uart1_rxByteQ + UART1_CONFIG_RX_BYTEQ_LENGTH))
        {
            uart1_obj.rxTail = uart1_rxByteQ;
        }

        uart1_obj.rxStatus.s.empty = false;
        
        if(uart1_obj.rxTail == uart1_obj.rxHead)
        {
            //Sets the flag RX full
            uart1_obj.rxStatus.s.full = true;
            break;
        }
        
    }*/

    IFS0bits.U1RXIF = false;  // Clear interrupt
   
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1ErrInterrupt ( void )
{
    if ((U1STAbits.OERR == 1))
    {
        U1STAbits.OERR = 0;
    }

    UART1_Initialize();  // 다시 켠다
    
    DMA1CONbits.CHEN = 1;
    
    IFS4bits.U1EIF = false;
}

/**
  Section: UART Driver Client Routines
*/

uint8_t UART1_Read( void)
{
    uint8_t data = 0;

    data = *uart1_obj.rxHead;

    uart1_obj.rxHead++;

    if (uart1_obj.rxHead == (uart1_rxByteQ + UART1_CONFIG_RX_BYTEQ_LENGTH))
    {
        uart1_obj.rxHead = uart1_rxByteQ;
    }

    if (uart1_obj.rxHead == uart1_obj.rxTail)
    {
        uart1_obj.rxStatus.s.empty = true;
    }

    uart1_obj.rxStatus.s.full = false;

    return data;
}


unsigned int UART1_ReadBuffer( uint8_t *buffer, const unsigned int bufLen)
{
    unsigned int numBytesRead = 0 ;
    while ( numBytesRead < ( bufLen ))
    {
        if( uart1_obj.rxStatus.s.empty)
        {
            break;
        }
        else
        {
            buffer[numBytesRead++] = UART1_Read () ;
        }
    }

    return numBytesRead ;
}



void UART1_Write( const uint8_t byte)
{
    IEC0bits.U1TXIE = false;
    
    *uart1_obj.txTail = byte;

    uart1_obj.txTail++;
    
    if (uart1_obj.txTail == (uart1_txByteQ + UART1_CONFIG_TX_BYTEQ_LENGTH))
    {
        uart1_obj.txTail = uart1_txByteQ;
    }

    uart1_obj.txStatus.s.empty = false;

    if (uart1_obj.txHead == uart1_obj.txTail)
    {
        uart1_obj.txStatus.s.full = true;
    }

    IEC0bits.U1TXIE = true ;
	
}


unsigned int UART1_WriteBuffer( const uint8_t *buffer , const unsigned int bufLen )
{
    unsigned int numBytesWritten = 0 ;

    while ( numBytesWritten < ( bufLen ))
    {
        if((uart1_obj.txStatus.s.full))
        {
            break;
        }
        else
        {
            UART1_Write (buffer[numBytesWritten++] ) ;
        }
    }

    return numBytesWritten ;

}


UART1_TRANSFER_STATUS UART1_TransferStatusGet (void )
{
    UART1_TRANSFER_STATUS status = 0;

    if(uart1_obj.txStatus.s.full)
    {
        status |= UART1_TRANSFER_STATUS_TX_FULL;
    }

    if(uart1_obj.txStatus.s.empty)
    {
        status |= UART1_TRANSFER_STATUS_TX_EMPTY;
    }

    if(uart1_obj.rxStatus.s.full)
    {
        status |= UART1_TRANSFER_STATUS_RX_FULL;
    }

    if(uart1_obj.rxStatus.s.empty)
    {
        status |= UART1_TRANSFER_STATUS_RX_EMPTY;
    }
    else
    {
        status |= UART1_TRANSFER_STATUS_RX_DATA_PRESENT;
    }
    return status;
}


uint8_t UART1_Peek(uint16_t offset)
{
    if( (uart1_obj.rxHead + offset) > (uart1_rxByteQ + UART1_CONFIG_RX_BYTEQ_LENGTH))
    {
      return uart1_rxByteQ[offset - (uart1_rxByteQ + UART1_CONFIG_RX_BYTEQ_LENGTH - uart1_obj.rxHead)];
    }
    else
    {
      return *(uart1_obj.rxHead + offset);
    }
}


unsigned int UART1_ReceiveBufferSizeGet(void)
{
    if(!uart1_obj.rxStatus.s.full)
    {
        if(uart1_obj.rxHead > uart1_obj.rxTail)
        {
            return(uart1_obj.rxHead - uart1_obj.rxTail);
        }
        else
        {
            return(UART1_CONFIG_RX_BYTEQ_LENGTH - (uart1_obj.rxTail - uart1_obj.rxHead));
        } 
    }
    return 0;
}


unsigned int UART1_TransmitBufferSizeGet(void)
{
    if(!uart1_obj.txStatus.s.full)
    { 
        if(uart1_obj.txHead > uart1_obj.txTail)
        {
            return(uart1_obj.txHead - uart1_obj.txTail);
        }
        else
        {
            return(UART1_CONFIG_TX_BYTEQ_LENGTH - (uart1_obj.txTail - uart1_obj.txHead));
        }
    }
    return 0;
}


bool UART1_ReceiveBufferIsEmpty (void)
{
    return(uart1_obj.rxStatus.s.empty);
}


bool UART1_TransmitBufferIsFull(void)
{
    return(uart1_obj.txStatus.s.full);
}


UART1_STATUS UART1_StatusGet (void)
{
    return U1STA;
}



/**
  End of File
*/
