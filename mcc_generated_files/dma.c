/*******************************************************************************
  DMA Generated Driver File

Company:
Microchip Technology Inc.

File Name:
    dma.c

Summary:
This is the generated driver implementation file for the DMA driver using MPLAB(c) Code Configurator

Description:
This source file provides implementations for driver APIs for DMA.
Generation Information :
Product Revision  :  MPLAB(c) Code Configurator - pic24-dspic-pic32mm : v1.26
Device            :  dsPIC33EV256GM002
The generated drivers are tested against the following:
Compiler          :  XC16 1.30
MPLAB             :  MPLAB X 3.45
*******************************************************************************/

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

#include <xc.h>
#include "dma.h"
#include "interrupt_manager.h"
#include "uart1.h"

extern volatile unsigned int BufferA[4];//
extern volatile unsigned int Buffer1A[6] ;//


extern volatile unsigned char *bufferA_8bit_pointer;
extern volatile unsigned char range;
extern volatile unsigned char mode_plug;
extern volatile unsigned char comm_error;// =0b00001000 ;// Communication Error 3rd bit 

extern volatile int32_t capture2; 
extern volatile unsigned char *capture2_1byte_pointer; 



void DMA_Initialize(void) 
{ 
    // Initialize channels which are enabled 
    // AMODE Register Indirect with Post-Increment mode; CHEN enabled; DIR Reads from RAM address, writes to peripheral address; HALF Initiates interrupt when all of the data has been moved; SIZE 8 bit; NULLW disabled; MODE One-Shot, Ping-Pong modes are disabled; 
    DMA0CON= 0x6001 & 0x7FFF; //Enable DMA Channel later;
    // FORCE disabled; IRQSEL UART1 TX; 
    DMA0REQ= 0xC;     // REQ  UART1 TX 
    // CNT 3; 
    DMA0CNT= 0x4;     //총 5개를 내 보낸다. 
    // STA 4096; 
    DMA0STAL= 0x1000;
    // STA 0; 
    DMA0STAH= 0x0;
    // Clearing Channel 0 Interrupt Flag;
    IFS0bits.DMA0IF = false;
    // Enabling Channel 0 Interrupt
    IEC0bits.DMA0IE = 1;
    // AMODE Register Indirect with Post-Increment mode; CHEN enabled; SIZE 16 bit; DIR Reads from peripheral address, writes to RAM address; NULLW disabled; HALF Initiates interrupt when all of the data has been moved; MODE Continuous, Ping-Pong modes are disabled; 
    DMA1CON= 0x0001 & 0x7FFF; //Enable DMA Channel later;  0x0001  ; ping pong mode 필요 없음, 연속으로 받을 때만 필요 
    // FORCE disabled; IRQSEL UART1 RX; 
    DMA1REQ= 0xB;      // REQ UART RX
    // CNT 3; 
    DMA1CNT= 0x3;    // 총4개를 받은 후에 DMA interrupt  발생 
    // STA 4096; 
    DMA1STAL= 0x1000;
    // STA 0; 
    DMA1STAH= 0x0;
    
    DMA1STBL= 0x1000;
    // STA 0; 
    DMA1STBH= 0x0;
    // Clearing Channel 1 Interrupt Flag;
    IFS0bits.DMA1IF = false;
    // Enabling Channel 1 Interrupt
    IEC0bits.DMA1IE = 1;

    //Enable DMA Channel 0
    
    DMA0CONbits.CHEN = 0;
    //Enable DMA Channel 1
    
    DMA1CONbits.CHEN = 1;
}
void __attribute__ ( ( interrupt, no_auto_psv ) ) _DMA0Interrupt( void )   //5개가 모두 TX로 출력된 뒤 interrupt
{
    
    DMA_ChannelEnable(DMA_CHANNEL_1);     // RX DMA 동작 
    DMA1CONbits.CHEN = 1;
    
    U1STAbits.OERR = 0;
 
    IFS0bits.DMA0IF = 0;  // Clear DMA interrupt
}
void __attribute__ ( ( interrupt, no_auto_psv ) ) _DMA1Interrupt( void )   // DMA에 4개가 모인후 발생된 interrupt
{
    
    unsigned char transfer1[2] ={0xA1,0x0D};//
      
    
       //***User Area Begin
    INTERRUPT_GlobalDisable();
        
    
        if((Buffer1A[0]==0x0002)&&(Buffer1A[3]==0x000D)){
            
            transfer1[0]=mode_plug|range|comm_error;
 
        }else{  // 정상 Protocol이 아님
            
           transfer1[0]=0xF0|range;

           IEC0bits.U1RXIE = 0;  // interrupt 
           U1MODEbits.UARTEN = 0;// 순서가 중요. 먼저 끄고 
           UART1_Initialize();  // 다시 켠다
   
        }
        
              transfer1[1]=~(transfer1[0]+capture2_1byte_pointer[0]+capture2_1byte_pointer[1]+capture2_1byte_pointer[2]);  // checksum one's complements
    
    
                bufferA_8bit_pointer[0] = transfer1[0];
                bufferA_8bit_pointer[1] = capture2_1byte_pointer[2];
                bufferA_8bit_pointer[2] = capture2_1byte_pointer[1];
                bufferA_8bit_pointer[3] = capture2_1byte_pointer[0];
                bufferA_8bit_pointer[4] = transfer1[1];   
        
        
        DMA_ChannelEnable(DMA_CHANNEL_0);  // 4byte 입력이 들어 오면 무조건 DATA를 보낸다. // ERROR는 추후에 교정 
        DMA_SoftwareTriggerEnable(DMA_CHANNEL_0); // 위와 같은 행위 UART TX 구동 


    IFS0bits.DMA1IF = 0;  // Clear DMA interrupt
    
    INTERRUPT_GlobalEnable();
    
}

/**
  End of File
*/
