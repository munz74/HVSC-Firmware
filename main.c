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
volatile int32_t capture2=0; 

volatile unsigned int Buffer1A[4]={0x0A0D,0x0A0D,0x3433,0x3231}; // receiver
volatile unsigned int BufferA[4]= {0x0A0D,0x0A0D,0x3433,0x3231};//,0x0D37,0x3738,0x3738,0x3738,0x3738};  transmit

volatile unsigned char mode_plug=0xF0 ;// A Normal reading , B test signal application, C wire break monitor, 0xF0 =초기값 ADC setup확인 
volatile unsigned char comm_error=0b00001000 ;// Communication Error 3rd bit 

volatile unsigned char range=0 ; // 1: 1V, 2: 10V, 3: 30V, 4: 600V range, 0 은 setting이 안된상태

volatile unsigned char *bufferA_8bit_pointer = (unsigned char *)BufferA;
volatile unsigned char *capture2_1byte_pointer = (unsigned char *)&capture2;   //ADC data를 byte 단위로 읽기 위해서 pointer 선언


int spi_starting_test(void){

    uint8_t trans_buffer1[3]={0x47,SPI1_DUMMY_DATA,SPI1_DUMMY_DATA};   //#define SPI1_DUMMY_DATA 0x0
    uint8_t  read_buffer1[3];
  
    uint8_t  reset1[10]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
    uint8_t  dummy_buffer[10];
            
    uint8_t spi_read_add1=0b01000111; // = 0x47 = 1 000111  read command ID register
    uint8_t spi_read1,spi_read2; 

    int32_t pData1;
    int32_t ret;
    
    
    SPI1_Exchange8bitBuffer(reset1, 10, dummy_buffer);      // reset

    __delay32(150000);
    
//              SPI1_Exchange8bit( spi_read_add1 );  // output SPI1_Exchange8bit( uint8_t input )
    spi_read1=SPI1_Exchange8bit( spi_read_add1 );   // ID 확인 하기 
    spi_read2=SPI1_Exchange8bit( spi_read_add1 );   // ID 확인 하기
 
    printf("AD7175 ID1:  %X, %X\n", spi_read1, spi_read2);

    
    SPI1_Exchange8bitBuffer(trans_buffer1, 3, read_buffer1);    //  uint16_t SPI1_Exchange8bitBuffer(uint8_t *dataTransmitted, uint16_t byteCount, uint8_t *dataReceived)
        
    printf("AD7175 ID2:  %X, %X\n",  read_buffer1[1], read_buffer1[2]);
    
  
    /* Read the value of the Status Register */
    ret = AD7175_ReadRegister(&AD7175_regs1[ID_AD7175]);

    pData1 = AD7175_regs1[ID_AD7175].value;

    printf("AD7175 ID3: %ld  %lX  \n", ret, pData1 );

        
    if(ret < 0)  {   
            printf("AD7175 Err \n");
        }
    else{
            printf("AD7175 OK ,%ld \n", ret);
        } 
 
        
        return ret;
}
    


unsigned char range_reading(void){ 
       
    unsigned char selector1[5];//, uart_status1;
    
    IO_RB12_SetDigitalInput(); // selector read setting
    IO_RB13_SetDigitalInput();
    IO_RB14_SetDigitalInput();
    IO_RB15_SetDigitalInput();
    
    selector1[0]=IO_RB12_GetValue();
    selector1[1]=IO_RB13_GetValue();
    selector1[2]=IO_RB14_GetValue();
    selector1[3]=IO_RB15_GetValue();    
    
    selector1[4]=selector1[0]+(selector1[1]<1)+(selector1[2]<2)+(selector1[3]<3);
    
    switch(selector1[4]){  // range read and range plug setting 
 
        case 1 : return(1); break;    //600V
        case 2 : return(2); break;    //30V 
        case 4 : return(3); break;    //10V
        case 8 : return(4); break;    // 2V
        default : return(0); break;   // range read error 
  }       
    
    
}   
    
unsigned char ADC_setup_normal(unsigned char range){

    long ret = 0;
    
    if(mode_plug!=0xA0){  // 초기값 0xF0가 반영된다. 
    
        mode_plug=0xA0 ;// A Normal reading , B test signal application, C wire break monitor
    
            INTERRUPT_GlobalDisable();
            __delay32(1000);
        
            if(range ==0x04){
                ret = AD7175_Setup(AD7175_regs2);  // 2.048V ADC reading
            }else{
                ret = AD7175_Setup(AD7175_regs1);  // 5V ADC reading
            }
            __delay32(100);
            INTERRUPT_GlobalEnable();

            IO_RA0_SetHigh();    // Sine OFF
            IO_RA1_SetHigh();    // wirebreak OFF

    }
    
    return(ret);
}    

unsigned char ADC_setup_signal(unsigned char range){

    long ret = 0;
    
    if(mode_plug==0xA0){  

        mode_plug=0xB0 ;// A Normal reading , B test signal application, C wire break monitor
        
        IO_RA0_SetLow();   // Sine ON    
        IO_RA1_SetLow();   // wirebreak ON  
    }    
    return(ret);
}    

unsigned char ADC_setup_wirebreak(unsigned char range){

    long ret = 0;
    
    if(mode_plug==0xB0){

        mode_plug=0xC0 ;// A Normal reading , B test signal application, C wire break monitor

            INTERRUPT_GlobalDisable();
            __delay32(1000);
            ret = AD7175_Setup(AD7175_regs3);  // AN8 AN9 Reading 
            __delay32(100);
            INTERRUPT_GlobalEnable();
    }        

    return(ret);
}    


int main(void)
{
    // initialize the device
    
    unsigned int roll=0;
    long ret = 0;

    int64_t *command= (int64_t *)Buffer1A, command_plug=0x0;//
 
    SYSTEM_Initialize();
    
    // When using interrupts, you need to set the Global Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts


    // Disable the Global Interrupts
    INTERRUPT_GlobalDisable();

 

//DMA memory address allocation         
    DMA0PAD = (volatile unsigned int) &U1TXREG;
    DMA_PeripheralAddressSet(DMA_CHANNEL_0, &U1TXREG);
    DMA0STAL = ( unsigned int) BufferA;

    DMA1PAD = (volatile unsigned int) &U1RXREG;
    DMA_PeripheralAddressSet(DMA_CHANNEL_1, &U1RXREG);
    
    DMA1STAL= (volatile unsigned int) Buffer1A;
    DMA1STAH= (volatile unsigned int) Buffer1A;
    
    IEC4bits.U1EIE=0x01; // UART ERROR interrupt. 
 
// SPI 작동 여부 확인     
    IO_RB0_SetLow(); // AD7175 enable
    __delay32(1000);
    ret= spi_starting_test();
    __delay32(1000);     
    IO_RB0_SetHigh(); // AD7175 stop
    __delay32(1000);    

//GPIO setting 

    
    IO_RA0_SetDigitalOutput();    
    IO_RA0_SetHigh();    // Sine OFF
//    IO_RA0_SetLow();   // Sine ON

    IO_RA1_SetDigitalOutput();    
    IO_RA1_SetHigh();    // wirebreak OFF
//    IO_RA1_SetLow();   // wirebreak ON  
    
    
// Selector Range reading    
    range = range_reading();

    ret=ADC_setup_normal(range);
    INTERRUPT_GlobalDisable();
    IO_RB0_SetHigh();  // AD7175 off
    __delay32(10000);
    
    // PWM DAC start
    OC1_Tasks();  
    OC1_Start();
    
    INTERRUPT_GlobalEnable();

    IO_RB0_SetLow();  // AD7175 start
    __delay32(10000);

    while (1)
    {
        if(*command!=command_plug){
            
            command_plug=*command;
            
            switch(command_plug){
                    case 0x000200520044000D : comm_error=0b00000000; break; // RD
                    case 0x000200570054000D : ADC_setup_wirebreak(range); break; // WT ADC Setting function1
                    case 0x000200540054000D : ADC_setup_signal(range); break; // ST ADC Setting function2
                    case 0x000200430054000D : ADC_setup_normal(range); break; // CT ADC initial Setting function0
                    default : comm_error=0b00001000; break;
            }

        }
        
        
        
        
        
       
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