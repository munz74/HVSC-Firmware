/**************************************************************************//**
*   @file   AD7175.h
*   @brief  AD7175 header file.
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

#ifndef __AD7175_H__
#define __AD7175_H__

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include <stdint.h>
#include "AD7175_regs.h"

/*****************************************************************************/
/******************* AD7175 Constants ****************************************/
/*****************************************************************************/
#define AD7175_CRC_POLYNOMIAL 0x07 // x^8 + x^2 + x +1 (MSB first)
#define AD7175_CRC_CHECK_CODE 0xF3 

/*****************************************************************************/
/************************ Functions Declarations *****************************/
/*****************************************************************************/
int32_t AD7175_ReadRegister(st_reg* pReg);
int32_t AD7175_WriteRegister(st_reg reg);
//int32_t AD7175_WaitForReady(uint32_t timeout);
int32_t AD7175_WaitForReady(uint32_t timeout,st_reg *AD7175_regs);
//int32_t AD7175_ReadData(volatile int32_t* pData);
int32_t AD7175_ReadData(volatile int32_t* pData,st_reg *AD7175_regs); 
uint8_t AD7175_ComputeCRC(uint8_t* pBuf, uint8_t bufSize);
//int32_t AD7175_Setup(void);
int32_t AD7175_Setup(st_reg *AD7175_regs);


#endif // __AD7175_H__
