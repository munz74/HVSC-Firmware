/**
  EXT_INT Generated Driver File 

  @Company:
    Microchip Technology Inc.

  @File Name:
    ext_int.c

  @Summary
    This is the generated driver implementation file for the EXT_INT 
    driver using MPLAB(c) Code Configurator

  @Description:
    This source file provides implementations for driver APIs for EXT_INT. 
    Generation Information : 
        Product Revision  :  MPLAB(c) Code Configurator - pic24-dspic-pic32mm : v1.26
        Device            :  dsPIC33EV256GM002
    The generated drivers are tested against the following:
        Compiler          :  XC16 1.30
        MPLAB             :  MPLAB X 3.45
*/

/**
   Section: Includes
 */
#include <xc.h>
#include "ext_int.h"
//***User Area Begin->code: Add External Interrupt handler specific headers 
#include "../AD7175.h"

extern volatile int32_t capture1;

//***User Area End->code: Add External Interrupt handler specific headers

/**
   Section: External Interrupt Handlers
 */
/**
  Interrupt Handler for EX_INT1 - INT1
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _INT1Interrupt(void)  // int0 negative edge에서 ADC data 읽어 오기
{
    //***User Area Begin->code: External Interrupt 1***
        long ret = 0;
    
        ret = AD7175_ReadData(&capture1);

    //***User Area End->code: External Interrupt 1***
    EX_INT1_InterruptFlagClear();
}
/**
    Section: External Interrupt Initializers
 */
/**
    void EXT_INT_Initialize(void)

    Initializer for the following external interrupts
    INT1
*/
void EXT_INT_Initialize(void)
{
    /*******
     * INT1
     * Clear the interrupt flag
     * Set the external interrupt edge detect
     * Enable the interrupt, if enabled in the UI. 
     ********/
    EX_INT1_InterruptFlagClear();   
    EX_INT1_NegativeEdgeSet();
    EX_INT1_InterruptEnable();
}
