/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB(c) Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - pic24-dspic-pic32mm : v1.26
        Device            :  dsPIC33EV256GM002
    The generated drivers are tested against the following:
        Compiler          :  XC16 1.30
        MPLAB             :  MPLAB X 3.45
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

#include "mcc_generated_files/mcc.h"
#include "Communication.h"
#include "AD7175.h"
#include <libpic30.h>
#include <stdio.h>

/*
 * Clock를 24MHz --> 48MHz로 변경 
 * ADC read를 IC의 Interrupt에서  EXT-interrupt로 변경 
 * 통신 TX DATA update 를 RX DATA read의 DMA interrupt에서 진행 
 *                          Main application
 */

volatile int32_t capture1=0; 
//volatile int32_t ic_interval=0; 
volatile int32_t capture2=0; 

volatile unsigned int Buffer1A[4] ;//
volatile unsigned int BufferA[4] ;
//volatile unsigned int uart_plag1=0 ;//

volatile unsigned char *bufferA_8bit_pointer = (unsigned char *)BufferA;
volatile unsigned char *capture2_1byte_pointer = (unsigned char *)&capture2;   //ADC data를 byte 단위로 읽기 위해서 pointer 선언


int spi_starting_test(void){

    uint8_t trans_buffer1[3]={0x47,SPI1_DUMMY_DATA,SPI1_DUMMY_DATA};
    uint8_t  read_buffer1[3];

   
    uint8_t  reset1[10]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
    uint8_t  dummy_buffer[10];
            
            
    uint8_t spi_read_add1=0b01000111; 
    uint8_t spi_read1,spi_read2; 
//    uint16_t read1;

    int32_t pData1;
    int32_t ret;

    
    
    SPI1_Exchange8bitBuffer(reset1, 10, dummy_buffer);      // reset

    __delay32(150000);
    
    SPI1_Exchange8bit( spi_read_add1 );
    spi_read1=SPI1_Exchange8bit( spi_read_add1 );
    spi_read2=SPI1_Exchange8bit( spi_read_add1 );
    

    printf("AD7175 ID1:  %X, %X\n", spi_read1, spi_read2);

    SPI1_Exchange8bitBuffer(trans_buffer1, 3, read_buffer1);    
        
    //read1 = (((unsigned int)read_buffer1[1])<<8) | (read_buffer1[0]);
    
    printf("AD7175 ID2:  %X, %X\n",  read_buffer1[1], read_buffer1[2]);
    
  
    /* Read the value of the Status Register */
    ret = AD7175_ReadRegister(&AD7175_regs[ID_AD7175]);

    pData1 = AD7175_regs[ID_AD7175].value;

    printf("AD7175 ID3: %ld  %lX  \n", ret, pData1 );

        __delay32(150000);


        ret = AD7175_Setup();

        __delay32(1000);
        
    if(ret < 0)  {   
            printf("AD7175 Err \n");
        }
    else{
            printf("AD7175 OK ,%ld \n", ret);
        } 
 
        
        return ret;
}
    
    
    
    
int main(void)
{
    // initialize the device
    
    unsigned int roll=0;//, uart_status1;
    long ret = 0;
 

//    unsigned int *capture2_2byte_pointer = (unsigned int *)&capture2;     //ADC data를 2 byte 단위로 읽기 위해서 pointer 선언
//    unsigned int *pwm_2byte_pointer = (unsigned int *)&transfer1[1];  //ADC data의 앞 16bit를 읽기 위해서 pointer 선언
    

    BufferA[0] =0x3231;
    BufferA[1] =0x3433;
    BufferA[2] =0x0A0D;
    BufferA[3] =0x0A0D;//,0x3635,0x0D37,0x3738,0x3738,0x3738,0x3738};
    
    
    SYSTEM_Initialize();
    
    // When using interrupts, you need to set the Global Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts


    // Disable the Global Interrupts
    INTERRUPT_GlobalDisable();

    
    IO_RB0_SetLow();
        
    ret= spi_starting_test();
     
    IO_RB0_SetHigh();

        
    DMA0PAD = (volatile unsigned int) &U1TXREG;
    DMA_PeripheralAddressSet(DMA_CHANNEL_0, &U1TXREG);
    DMA0STAL = ( unsigned int) BufferA;

    DMA1PAD = (volatile unsigned int) &U1RXREG;
    DMA_PeripheralAddressSet(DMA_CHANNEL_1, &U1RXREG);
    
    DMA1STAL= (volatile unsigned int) Buffer1A;
    DMA1STAH= (volatile unsigned int) Buffer1A;
    
    IEC4bits.U1EIE=0x01; // UART ERROR interrupt. 
    
 //   TMR2_Start();
 //   TMR3_Start();

    OC1_Tasks();
    OC1_Start();
    
    INTERRUPT_GlobalEnable();

    IO_RB0_SetLow();  // AD7175 start
    
    __delay32(100000);

    while (1)
    {
       
       if(capture2!=capture1){  //ADC 값이 바뀌었을때 값을 update 한다. 
           
            capture2=capture1;
           
            roll=(unsigned int)((0x00FFFF00&capture2)>>8);
            OC1_PrimaryValueSet(roll);   // PWM Output
            
       }
    
    }
    return 0xFF;

}
/**
 End of File
*/