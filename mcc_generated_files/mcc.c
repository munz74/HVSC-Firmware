/**
  @Generated MPLAB(c) Code Configurator Source File

  @Company:
    Microchip Technology Inc.

  @File Name:
    mcc.c

  @Summary:
    This is the mcc.c file generated using MPLAB(c) Code Configurator

  @Description:
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

// Configuration bits: selected in the GUI

// FSEC
#pragma config BWRP = OFF    // Boot Segment Write-Protect Bit->Boot Segment may be written
#pragma config BSS = DISABLED    // Boot Segment Code-Protect Level bits->No Protection (other than BWRP)
#pragma config BSS2 = OFF    // Boot Segment Control Bit->No Boot Segment
#pragma config GWRP = OFF    // General Segment Write-Protect Bit->General Segment may be written
#pragma config GSS = DISABLED    // General Segment Code-Protect Level bits->No Protection (other than GWRP)
#pragma config CWRP = OFF    // Configuration Segment Write-Protect Bit->Configuration Segment may be written
#pragma config CSS = DISABLED    // Configuration Segment Code-Protect Level bits->No Protection (other than CWRP)
#pragma config AIVTDIS = DISABLE    // Alternate Interrupt Vector Table Disable Bit ->Disable Alternate Vector Table

// FOSCSEL
#pragma config FNOSC = PRIPLL    // Initial oscillator Source Selection Bits->Primary Oscillator with PLL module (XT + PLL, HS + PLL, EC + PLL)
#pragma config IESO = ON    // Two Speed Oscillator Start-Up Bit->Start up device with FRC,then automatically switch to user selected oscillator source

// FOSC
#pragma config POSCMD = HS    // Primary Oscillator Mode Select Bits->HS Crystal Oscillator mode
#pragma config OSCIOFNC = ON    // OSC2 Pin I/O Function Enable Bit->OSC2 is general purpose digital I/O pin
#pragma config IOL1WAY = ON    // Peripheral Pin Select Configuration Bit->Allow Only One reconfiguration
#pragma config FCKSM = CSDCMD    // Clock Switching Mode Bits->Both Clock Switching and Fail-safe Clock Monitor are disabled
#pragma config PLLKEN = ON    // PLL Lock Enable Bit->Clock switch to PLL source will wait until the PLL lock signal is valid

// FWDT
#pragma config WDTPOST = PS32768    // Watchdog Timer Postscaler Bits->1:32768
#pragma config WDTPRE = PR128    // Watchdog Timer Prescaler Bit->1:128
#pragma config FWDTEN = OFF    // Watchdog Timer Enable Bits->WDT and SWDTEN Disabled
#pragma config WINDIS = OFF    // Watchdog Timer Window Enable Bit->Watchdog timer in Non-Window Mode
#pragma config WDTWIN = WIN25    // Watchdog Window Select Bits->WDT Window is 25% of WDT period

// FPOR
#pragma config BOREN0 = ON    // Brown Out Reset Detection Bit->BOR is Enabled

// FICD
#pragma config ICS = PGD1    // ICD Communication Channel Select Bits->Communicate on PGEC1 and PGED1

// FDMTINTVL
#pragma config DMTIVTL = 0    // Lower 16 Bits of 32 Bit DMT Window Interval->

// FDMTINTVH
#pragma config DMTIVTH = 0    // Upper 16 Bits of 32 Bit DMT Window Interval->

// FDMTCNTL
#pragma config DMTCNTL = 0    // Lower 16 Bits of 32 Bit DMT Instruction Count Time-Out Value->

// FDMTCNTH
#pragma config DMTCNTH = 0    // Upper 16 Bits of 32 Bit DMT Instruction Count Time-Out Value->

// FDMT
#pragma config DMTEN = DISABLE    // Dead Man Timer Enable Bit->Dead Man Timer is Disabled and can be enabled by software

// FDEVOPT
#pragma config PWMLOCK = ON    // PWM Lock Enable Bit->Certain PWM registers may only be written after key sequence
#pragma config ALTI2C1 = OFF    // Alternate I2C1 Pins Selection Bit->I2C1 mapped to SDA1/SCL1 pins

// FALTREG
#pragma config CTXT1 = NONE    // Interrupt Priority Level (IPL) Selection Bits For Alternate Working Register Set 1->Not Assigned
#pragma config CTXT2 = NONE    // Interrupt Priority Level (IPL) Selection Bits For Alternate Working Register Set 2->Not Assigned

#include "mcc.h"

/**
 Section: Local Variables
*/

/**
 Section: Function prototypes
*/
bool SYSTEM_ResetCauseFromSoftware(uint16_t resetCause);
bool SYSTEM_ResetCauseFromWatchdogTimer(uint16_t resetCause);
bool SYSTEM_ResetCauseFromConfigurationMismatch(uint16_t resetCause);
bool SYSTEM_ResetCauseFromIllegalOpcode(uint16_t resetCause);
bool SYSTEM_ResetCauseFromExternal(uint16_t resetCause);
bool SYSTEM_ResetCauseFromTrap(uint16_t resetCause);
void SYSTEM_ResetCauseClear(RESET_MASKS resetFlagMask);

/**
* a private place to store the error code if we run into a severe error
*/

void SYSTEM_Initialize(void)
{
    PIN_MANAGER_Initialize();
    OSCILLATOR_Initialize();
    INTERRUPT_Initialize();
    SPI1_Initialize();
    UART1_Initialize();
    OC1_Initialize();
    TMR2_Initialize();
    EXT_INT_Initialize();
    DMA_Initialize();
    INTERRUPT_GlobalEnable();
    CORCON_ModeOperatingSet(CORCON_MODE_PORVALUES);
}

void OSCILLATOR_Initialize(void)
{
    // CF no clock failure; NOSC PRIPLL; CLKLOCK unlocked; OSWEN Switch is Complete; 
    __builtin_write_OSCCONL((uint8_t) (0x300 & 0x00FF));
    // FRCDIV FRC/2; PLLPRE 4; DOZE 1:8; PLLPOST 1:4; DOZEN disabled; ROI disabled; 
    CLKDIV = 0x3142;
    // TUN Center frequency; 
    OSCTUN = 0x0;
    // ROON disabled; ROSEL disabled; RODIV Base clock value; ROSSLP disabled; 
    REFOCON = 0x0;
    // PLLDIV 94; 
    PLLFBD = 0x5E;
    // RND disabled; SATB disabled; SATA disabled; ACCSAT disabled; 
	CORCONbits.RND = 0;
	CORCONbits.SATB = 0;
	CORCONbits.SATA = 0;
	CORCONbits.ACCSAT = 0;
}

uint16_t SYSTEM_GetResetCause(void)
{
    return RCON;
}

void __attribute__ ((weak)) SYSTEM_ResetCauseHandler(void)
{
    uint16_t resetCause = SYSTEM_GetResetCause();
    if(SYSTEM_ResetCauseFromTrap(resetCause))
    { 
      SYSTEM_ResetCauseClear(RESET_MASK_TRAPR); 
      //Do something 
    }
    if(SYSTEM_ResetCauseFromIllegalOpcode(resetCause))
    { 
      SYSTEM_ResetCauseClear(RESET_MASK_IOPUWR); 
      //Do something 
    }
    if(SYSTEM_ResetCauseFromConfigurationMismatch(resetCause))
    { 
      SYSTEM_ResetCauseClear(RESET_MASK_CM); 
      //Do something 
    }
    if(SYSTEM_ResetCauseFromExternal(resetCause))
    { 
      SYSTEM_ResetCauseClear(RESET_MASK_EXTR); 
      //Do something 
    }
    if(SYSTEM_ResetCauseFromSoftware(resetCause))
    { 
      SYSTEM_ResetCauseClear(RESET_MASK_SWR); 
      //Do something 
    }
    if(SYSTEM_ResetCauseFromWatchdogTimer(resetCause))
    { 
      SYSTEM_ResetCauseClear(RESET_MASK_WDTO); 
      //Do something 
    }
}

bool SYSTEM_ResetCauseFromTrap(uint16_t resetCause)
{
    bool resetStatus = false;
    if(resetCause & RESET_MASK_TRAPR)
    { 
      resetStatus = true; 
    }
    return resetStatus;
}

bool SYSTEM_ResetCauseFromIllegalOpcode(uint16_t resetCause)
{
    bool resetStatus = false;
    if(resetCause & RESET_MASK_IOPUWR)
    { 
      resetStatus = true; 
    }
    return resetStatus;
}

bool SYSTEM_ResetCauseFromConfigurationMismatch(uint16_t resetCause)
{
    bool resetStatus = false;
    if(resetCause & RESET_MASK_CM)
    { 
      resetStatus = true; 
    }
    return resetStatus;
}

bool SYSTEM_ResetCauseFromExternal(uint16_t resetCause)
{
    bool resetStatus = false;
    if(resetCause & RESET_MASK_EXTR)
    { 
      resetStatus = true; 
    }
    return resetStatus;
}

bool SYSTEM_ResetCauseFromSoftware(uint16_t resetCause)
{
    bool resetStatus = false;
    if(resetCause & RESET_MASK_SWR)
    { 
      resetStatus = true; 
    }
    return resetStatus;
}

bool SYSTEM_ResetCauseFromWatchdogTimer(uint16_t resetCause)
{
    bool resetStatus = false;
    if(resetCause & RESET_MASK_WDTO)
    { 
      resetStatus = true;
    }
    return resetStatus;
}

void SYSTEM_ResetCauseClear(RESET_MASKS resetFlagMask)
{ 
     RCON = RCON & (~resetFlagMask); 
} 

void SYSTEM_ResetCauseClearAll()
{ 
    RCON = 0x00; 
}
/**
 End of File
*/