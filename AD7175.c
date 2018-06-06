/**************************************************************************//**
*   @file   AD7175.c
*   @brief  AD7175 implementation file.
*   @author acozma (andrei.cozma@analog.com)
*
*******************************************************************************
* Copyright 2011(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************
*   SVN Revision: 0
******************************************************************************/

#define AD7175_INIT

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include "Communication.h"
#include "AD7175.h"
#include"mcc_generated_files/spi1.h"

/******************************************************************************/
/************************ Local variables and types ***************************/
/******************************************************************************/
struct AD7175_state
{
    uint8_t useCRC;
}AD7175_st;

/**************************************************************************//**
* @brief Reads the value of the specified register
*
* @param pReg - Pointer to the register structure holding info about the 
*               register to be read. The read value is stored inside the 
*               register structure.
*
* @return Returns 0 for success or negative error code.
******************************************************************************/
int32_t AD7175_ReadRegister(st_reg* pReg)
{
    int32_t ret       = 0;
    uint8_t buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t i         = 0;
    uint8_t crc       = 0;

    /* Build the Command word */
    buffer[0] = COMM_REG_WEN | COMM_REG_RD | pReg->addr;
    
    /* Read data from the device */
    ret = SPI_Read(AD7175_SLAVE_ID, 
                   buffer, 
                   (AD7175_st.useCRC ? pReg->size + 1 : pReg->size) + 1);
    if(ret < 0)
        return ret;

    /* Check the CRC */
    if(AD7175_st.useCRC)
    {
        crc = AD7175_ComputeCRC(&buffer[1], pReg->size + 1);
        if(crc != AD7175_CRC_CHECK_CODE)
            return -1;
    }

    /* Build the result */
    pReg->value = 0;
    for(i = 1; i < pReg->size + 1; i++)
    {
        pReg->value <<= 8;
        pReg->value += buffer[i];
    }

    return ret;
}

/**************************************************************************//**
* @brief Writes the value of the specified register
*
* @param reg - Register structure holding info about the register to be written
*
* @return Returns 0 for success or negative error code.
******************************************************************************/
//ret = AD7175_WriteRegister(AD7175_regs[ADC_Mode_Register]);
int32_t AD7175_WriteRegister(st_reg reg)
{
    int32_t ret      = 0;
    int32_t regValue = 0;
    uint8_t wrBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t i        = 0;
    uint8_t crc      = 0;
    
    /* Build the Command word */
    wrBuf[0] = COMM_REG_WEN | COMM_REG_WR | reg.addr;
    
    /* Fill the write buffer */
    regValue = reg.value;
    for(i = 0; i < reg.size; i++)
    {
        wrBuf[reg.size - i] = regValue & 0xFF;
        regValue >>= 8;
    }

    /* Compute the CRC */
    if(AD7175_st.useCRC)
    {
        crc = AD7175_ComputeCRC(wrBuf, reg.size+1);
        wrBuf[reg.size + 1] = ~crc;
    }

    /* Write data to the device */
    ret = SPI_Write(AD7175_SLAVE_ID,
                    wrBuf,
                    AD7175_st.useCRC ? reg.size + 2 : reg.size + 1);
    
    return ret;
}

/**************************************************************************//**
* @brief Waits until a new conversion result is available
*
* @param timeout - Count representing the number of polls to be done until the
*                  function returns if no new data is available. 
*
* @return Returns 0 for success or negative error code.
******************************************************************************/
int32_t AD7175_WaitForReady(uint32_t timeout)
{
    int32_t ret;
    int8_t ready = 0;

    while(!ready && --timeout)
    {
        /* Read the value of the Status Register */
        ret = AD7175_ReadRegister(&AD7175_regs[Status_Register]);
        if(ret < 0)
            return ret;

        /* Check the RDY bit in the Status Register */
        ready = (AD7175_regs[Status_Register].value & STATUS_REG_RDY) != 0;
    }

    return timeout ? 0 : -1; 
}

/**************************************************************************//**
* @brief Reads the conversion result from the device.
*
* @param pData - Pointer to store the read data.
*
* @return Returns 0 for success or negative error code.
******************************************************************************/
int32_t AD7175_ReadData(volatile int32_t* pData)
{
    volatile int32_t ret;

    /* Read the value of the Status Register */
    ret = AD7175_ReadRegister(&AD7175_regs[Data_Register]);

    /* Get the read result */
    *pData = AD7175_regs[Data_Register].value;

    return ret;
}

/**************************************************************************//**
* @brief Computes the CRC for a data buffer
*
* @param pBuf - Data buffer
* @param bufSize - Data buffer size in bytes
*
* @return Returns the computed CRC
******************************************************************************/
uint8_t AD7175_ComputeCRC(uint8_t* pBuf, uint8_t bufSize)
{
    uint8_t i = 0;
    uint8_t crc = 0xFF;

	while(bufSize--)
	{
		crc ^= *pBuf++;
		for(i = 0; i < 8; i++)
		{
			if(crc & 0x80)
				crc = (crc << 1) ^ AD7175_CRC_POLYNOMIAL;
            else
				crc <<= 1;
		}
	}
	return crc;  //여기서 수리
   // return 0xF3;
}

/**************************************************************************//**
* @brief Initializes the AD7175 
*
* @return Returns 0 for success or negative error code.
******************************************************************************/
int32_t AD7175_Setup(void)
{
    int32_t ret;

    /* Initialize the SPI communication */
    
  //  SPI_Init(0, 8000000, 1, 0);
    
    
    uint8_t  reset1[10]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
    uint8_t  dummy_buffer[10];
    
    
    SPI1_Exchange8bitBuffer(reset1, 10, dummy_buffer);      // reset
    
    /* Initialize ADC mode register */
    
    ret = AD7175_WriteRegister(AD7175_regs[ADC_Mode_Register]);
    if(ret < 0)
        return ret;
    
    /* Initialize Interface mode register */
    ret = AD7175_WriteRegister(AD7175_regs[Interface_Mode_Register]);
    if(ret < 0)
        return ret;
    AD7175_st.useCRC = INTF_MODE_REG_CRC_STAT(AD7175_regs[Interface_Mode_Register].value);
    
    /* Initialize GPIO configuration register */
    ret = AD7175_WriteRegister(AD7175_regs[GPIOCON]);
    if(ret < 0)
        return ret;   
    
    /* Initialize Channel Map registers */
    ret = AD7175_WriteRegister(AD7175_regs[CH_Map_1]);
    if(ret < 0)
        return ret;   
    ret = AD7175_WriteRegister(AD7175_regs[CH_Map_2]);
    if(ret < 0)
        return ret;   
    ret = AD7175_WriteRegister(AD7175_regs[CH_Map_3]);
    if(ret < 0)
        return ret;    
    ret = AD7175_WriteRegister(AD7175_regs[CH_Map_4]);
    if(ret < 0)
        return ret;
    
    /* Initialize Setup Configuration registers */
    ret = AD7175_WriteRegister(AD7175_regs[Setup_Config_1]);
    if(ret < 0)
        return ret;
    ret = AD7175_WriteRegister(AD7175_regs[Setup_Config_2]);
    if(ret < 0)
        return ret;
    ret = AD7175_WriteRegister(AD7175_regs[Setup_Config_3]);
    if(ret < 0)
        return ret;
    ret = AD7175_WriteRegister(AD7175_regs[Setup_Config_4]);
    if(ret < 0)
        return ret;

    /* Initialize Filter Configuration registers */
    ret = AD7175_WriteRegister(AD7175_regs[Filter_Config_1]);
    if(ret < 0)
        return ret;
    ret = AD7175_WriteRegister(AD7175_regs[Filter_Config_2]);
    if(ret < 0)
        return ret;
    ret = AD7175_WriteRegister(AD7175_regs[Filter_Config_3]);
    if(ret < 0)
        return ret;
    ret = AD7175_WriteRegister(AD7175_regs[Filter_Config_4]);
    if(ret < 0)
        return ret;

    return ret;
}
