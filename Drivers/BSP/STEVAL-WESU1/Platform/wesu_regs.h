/** 
  ******************************************************************************
  * @file    wesu_regs.h
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Header for wesu.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WESU_REGS_H
#define __WESU_REGS_H

#ifdef __cplusplus
 extern "C" {
#endif
 
   
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

#include <stdio.h>
#include <stdint.h>   
#include <stdlib.h>
#include <string.h>  
 
#include "algorithms.h"

/** @addtogroup BSP             BSP
  * @{
  */

/** @addtogroup STEVAL-WESU1    STEVAL-WESU1
  * @{
  */


/** @addtogroup PLATFORM        PLATFORM
  * @{
  */

/** @addtogroup WeSU_LOW_LEVEL  WeSU_LOW_LEVEL 
  * @{
  */    
   
   
/** @defgroup WeSU_LOW_LEVEL_Private_Types WeSU_LOW_LEVEL_Private_Types
  * @{
  */


/** 
  * @brief Registers array structure
  */ 
typedef struct _sRegsArray
{
  uint16_t nRegisters[255];         //!<  see \ref PERS_SESS_DESC_REG_0x00
}RegsStructArray_t;

/**
  * @}
  */ 

/** 
  * @brief Registers structure
  */ 
typedef struct _sRegs
{
  uint16_t nRegister00;         //!<  see \ref PERS_SESS_DESC_REG_0x00
  uint16_t nRegister01;         //!<  see \ref PERS_SESS_DESC_REG_0x01
  uint16_t nRegister02;         //!<  see \ref PERS_SESS_DESC_REG_0x02
  uint16_t nRegister03;         //!<  see \ref PERS_SESS_DESC_REG_0x03
  uint16_t nRegister04;         //!<  see \ref PERS_SESS_DESC_REG_0x04
  uint16_t nRegister05;         //!<  see \ref PERS_SESS_DESC_REG_0x05
  uint16_t nRegister06;         //!<  see \ref PERS_SESS_DESC_REG_0x06
  uint16_t nRegister07;         //!<  see \ref PERS_SESS_DESC_REG_0x07
  uint16_t nRegister08;         //!<  see \ref PERS_SESS_DESC_REG_0x08
  uint16_t nRegister09;         //!<  see \ref PERS_SESS_DESC_REG_0x09
  uint16_t nRegister0A;         //!<  see \ref PERS_SESS_DESC_REG_0x0A
  uint16_t nRegister0B;         //!<  see \ref PERS_SESS_DESC_REG_0x0B
  uint16_t nRegister0C;         //!<  see \ref PERS_SESS_DESC_REG_0x0C
  uint16_t nRegister0D;         //!<  see \ref PERS_SESS_DESC_REG_0x0D
  uint16_t nRegister0E;         //!<  see \ref PERS_SESS_DESC_REG_0x0E
  uint16_t nRegister0F;         //!<  see \ref PERS_SESS_DESC_REG_0x0F
  uint16_t nRegister10;         //!<  see \ref PERS_SESS_DESC_REG_0x10
  uint16_t nRegister11;         //!<  see \ref PERS_SESS_DESC_REG_0x11
  uint16_t nRegister12;         //!<  see \ref PERS_SESS_DESC_REG_0x12
  uint16_t nRegister13;         //!<  see \ref PERS_SESS_DESC_REG_0x13
  uint16_t nRegister14;         //!<  see \ref PERS_SESS_DESC_REG_0x14
  uint16_t nRegister15;         //!<  see \ref PERS_SESS_DESC_REG_0x15
  uint16_t nRegister16;         //!<  see \ref PERS_SESS_DESC_REG_0x16
  uint16_t nRegister17;         //!<  see \ref PERS_SESS_DESC_REG_0x17
  uint16_t nRegister18;         //!<  see \ref PERS_SESS_DESC_REG_0x18
  uint16_t nRegister19;         //!<  see \ref PERS_SESS_DESC_REG_0x19
  uint16_t nRegister1A;         //!<  see \ref PERS_SESS_DESC_REG_0x1A
  uint16_t nRegister1B;         //!<  see \ref PERS_SESS_DESC_REG_0x1B
  uint16_t nRegister1C;         //!<  see \ref PERS_SESS_DESC_REG_0x1C
  uint16_t nRegister1D;         //!<  see \ref PERS_SESS_DESC_REG_0x1D
  uint16_t nRegister1E;         //!<  see \ref PERS_SESS_DESC_REG_0x1E
  uint16_t nRegister1F;         //!<  see \ref PERS_SESS_DESC_REG_0x1F
  uint16_t nRegister20;         //!<  see \ref PERS_SESS_DESC_REG_0x20
  uint16_t nRegister21;         //!<  see \ref PERS_SESS_DESC_REG_0x21
  uint16_t nRegister22;         //!<  see \ref PERS_SESS_DESC_REG_0x22
  uint16_t nRegister23;         //!<  see \ref PERS_SESS_DESC_REG_0x23
  uint16_t nRegister24;         //!<  see \ref PERS_SESS_DESC_REG_0x24
  uint16_t nRegister25;         //!<  see \ref PERS_SESS_DESC_REG_0x25
  uint16_t nRegister26;         //!<  see \ref PERS_SESS_DESC_REG_0x26
  uint16_t nRegister27;         //!<  see \ref PERS_SESS_DESC_REG_0x27
  uint16_t nRegister28;         //!<  see \ref PERS_SESS_DESC_REG_0x28
  uint16_t nRegister29;         //!<  see \ref PERS_SESS_DESC_REG_0x29
  uint16_t nRegister2A;         //!<  see \ref PERS_SESS_DESC_REG_0x2A
  uint16_t nRegister2B;         //!<  see \ref PERS_SESS_DESC_REG_0x2B
  uint16_t nRegister2C;         //!<  see \ref PERS_SESS_DESC_REG_0x2C
  uint16_t nRegister2D;         //!<  see \ref PERS_SESS_DESC_REG_0x2D
  uint16_t nRegister2E;         //!<  see \ref PERS_SESS_DESC_REG_0x2E
  uint16_t nRegister2F;         //!<  see \ref PERS_SESS_DESC_REG_0x2F
  uint16_t nRegister30;         //!<  see \ref PERS_SESS_DESC_REG_0x30
  uint16_t nRegister31;         //!<  see \ref PERS_SESS_DESC_REG_0x31
  uint16_t nRegister32;         //!<  see \ref PERS_SESS_DESC_REG_0x32
  uint16_t nRegister33;         //!<  see \ref PERS_SESS_DESC_REG_0x33
  uint16_t nRegister34;         //!<  see \ref PERS_SESS_DESC_REG_0x34
  uint16_t nRegister35;         //!<  see \ref PERS_SESS_DESC_REG_0x35
  uint16_t nRegister36;         //!<  see \ref PERS_SESS_DESC_REG_0x36
  uint16_t nRegister37;         //!<  see \ref PERS_SESS_DESC_REG_0x37
  uint16_t nRegister38;         //!<  see \ref PERS_SESS_DESC_REG_0x38
  uint16_t nRegister39;         //!<  see \ref PERS_SESS_DESC_REG_0x39
  uint16_t nRegister3A;         //!<  see \ref PERS_SESS_DESC_REG_0x3A
  uint16_t nRegister3B;         //!<  see \ref PERS_SESS_DESC_REG_0x3B
  uint16_t nRegister3C;         //!<  see \ref PERS_SESS_DESC_REG_0x3C
  uint16_t nRegister3D;         //!<  see \ref PERS_SESS_DESC_REG_0x3D
  uint16_t nRegister3E;         //!<  see \ref PERS_SESS_DESC_REG_0x3E
  uint16_t nRegister3F;         //!<  see \ref PERS_SESS_DESC_REG_0x3F
  uint16_t nRegister40;         //!<  see \ref PERS_SESS_DESC_REG_0x40
  uint16_t nRegister41;         //!<  see \ref PERS_SESS_DESC_REG_0x41
  uint16_t nRegister42;         //!<  see \ref PERS_SESS_DESC_REG_0x42
  uint16_t nRegister43;         //!<  see \ref PERS_SESS_DESC_REG_0x43
  uint16_t nRegister44;         //!<  see \ref PERS_SESS_DESC_REG_0x44
  uint16_t nRegister45;         //!<  see \ref PERS_SESS_DESC_REG_0x45
  uint16_t nRegister46;         //!<  see \ref PERS_SESS_DESC_REG_0x46
  uint16_t nRegister47;         //!<  see \ref PERS_SESS_DESC_REG_0x47
  uint16_t nRegister48;         //!<  see \ref PERS_SESS_DESC_REG_0x48
  uint16_t nRegister49;         //!<  see \ref PERS_SESS_DESC_REG_0x49
  uint16_t nRegister4A;         //!<  see \ref PERS_SESS_DESC_REG_0x4A
  uint16_t nRegister4B;         //!<  see \ref PERS_SESS_DESC_REG_0x4B
  uint16_t nRegister4C;         //!<  see \ref PERS_SESS_DESC_REG_0x4C
  uint16_t nRegister4D;         //!<  see \ref PERS_SESS_DESC_REG_0x4D
  uint16_t nRegister4E;         //!<  see \ref PERS_SESS_DESC_REG_0x4E
  uint16_t nRegister4F;         //!<  see \ref PERS_SESS_DESC_REG_0x4F
  uint16_t nRegister50;         //!<  see \ref PERS_SESS_DESC_REG_0x50
  uint16_t nRegister51;         //!<  see \ref PERS_SESS_DESC_REG_0x51
  uint16_t nRegister52;         //!<  see \ref PERS_SESS_DESC_REG_0x52
  uint16_t nRegister53;         //!<  see \ref PERS_SESS_DESC_REG_0x53
  uint16_t nRegister54;         //!<  see \ref PERS_SESS_DESC_REG_0x54
  uint16_t nRegister55;         //!<  see \ref PERS_SESS_DESC_REG_0x55
  uint16_t nRegister56;         //!<  see \ref PERS_SESS_DESC_REG_0x56
  uint16_t nRegister57;         //!<  see \ref PERS_SESS_DESC_REG_0x57
  uint16_t nRegister58;         //!<  see \ref PERS_SESS_DESC_REG_0x58
  uint16_t nRegister59;         //!<  see \ref PERS_SESS_DESC_REG_0x59
  uint16_t nRegister5A;         //!<  see \ref PERS_SESS_DESC_REG_0x5A
  uint16_t nRegister5B;         //!<  see \ref PERS_SESS_DESC_REG_0x5B
  uint16_t nRegister5C;         //!<  see \ref PERS_SESS_DESC_REG_0x5C
  uint16_t nRegister5D;         //!<  see \ref PERS_SESS_DESC_REG_0x5D
  uint16_t nRegister5E;         //!<  see \ref PERS_SESS_DESC_REG_0x5E
  uint16_t nRegister5F;         //!<  see \ref PERS_SESS_DESC_REG_0x5F
  uint16_t nRegister60;         //!<  see \ref PERS_SESS_DESC_REG_0x60
  uint16_t nRegister61;         //!<  see \ref PERS_SESS_DESC_REG_0x61
  uint16_t nRegister62;         //!<  see \ref PERS_SESS_DESC_REG_0x62
  uint16_t nRegister63;         //!<  see \ref PERS_SESS_DESC_REG_0x63
  uint16_t nRegister64;         //!<  see \ref PERS_SESS_DESC_REG_0x64
  uint16_t nRegister65;         //!<  see \ref PERS_SESS_DESC_REG_0x65
  uint16_t nRegister66;         //!<  see \ref PERS_SESS_DESC_REG_0x66
  uint16_t nRegister67;         //!<  see \ref PERS_SESS_DESC_REG_0x67
  uint16_t nRegister68;         //!<  see \ref PERS_SESS_DESC_REG_0x68
  uint16_t nRegister69;         //!<  see \ref PERS_SESS_DESC_REG_0x69
  uint16_t nRegister6A;         //!<  see \ref PERS_SESS_DESC_REG_0x6A
  uint16_t nRegister6B;         //!<  see \ref PERS_SESS_DESC_REG_0x6B
  uint16_t nRegister6C;         //!<  see \ref PERS_SESS_DESC_REG_0x6C
  uint16_t nRegister6D;         //!<  see \ref PERS_SESS_DESC_REG_0x6D
  uint16_t nRegister6E;         //!<  see \ref PERS_SESS_DESC_REG_0x6E
  uint16_t nRegister6F;         //!<  see \ref PERS_SESS_DESC_REG_0x6F
  uint16_t nRegister70;         //!<  see \ref PERS_SESS_DESC_REG_0x70
  uint16_t nRegister71;         //!<  see \ref PERS_SESS_DESC_REG_0x71
  uint16_t nRegister72;         //!<  see \ref PERS_SESS_DESC_REG_0x72
  uint16_t nRegister73;         //!<  see \ref PERS_SESS_DESC_REG_0x73
  uint16_t nRegister74;         //!<  see \ref PERS_SESS_DESC_REG_0x74
  uint16_t nRegister75;         //!<  see \ref PERS_SESS_DESC_REG_0x75
  uint16_t nRegister76;         //!<  see \ref PERS_SESS_DESC_REG_0x76
  uint16_t nRegister77;         //!<  see \ref PERS_SESS_DESC_REG_0x77
  uint16_t nRegister78;         //!<  see \ref PERS_SESS_DESC_REG_0x78
  uint16_t nRegister79;         //!<  see \ref PERS_SESS_DESC_REG_0x79
  uint16_t nRegister7A;         //!<  see \ref PERS_SESS_DESC_REG_0x7A
  uint16_t nRegister7B;         //!<  see \ref PERS_SESS_DESC_REG_0x7B
  uint16_t nRegister7C;         //!<  see \ref PERS_SESS_DESC_REG_0x7C
  uint16_t nRegister7D;         //!<  see \ref PERS_SESS_DESC_REG_0x7D
  uint16_t nRegister7E;         //!<  see \ref PERS_SESS_DESC_REG_0x7E
  uint16_t nRegister7F;         //!<  see \ref PERS_SESS_DESC_REG_0x7F
  uint16_t nRegister80;         //!<  see \ref PERS_SESS_DESC_REG_0x80
  uint16_t nRegister81;         //!<  see \ref PERS_SESS_DESC_REG_0x81
  uint16_t nRegister82;         //!<  see \ref PERS_SESS_DESC_REG_0x82
  uint16_t nRegister83;         //!<  see \ref PERS_SESS_DESC_REG_0x83
  uint16_t nRegister84;         //!<  see \ref PERS_SESS_DESC_REG_0x84
  uint16_t nRegister85;         //!<  see \ref PERS_SESS_DESC_REG_0x85
  uint16_t nRegister86;         //!<  see \ref PERS_SESS_DESC_REG_0x86
  uint16_t nRegister87;         //!<  see \ref PERS_SESS_DESC_REG_0x87
  uint16_t nRegister88;         //!<  see \ref PERS_SESS_DESC_REG_0x88
  uint16_t nRegister89;         //!<  see \ref PERS_SESS_DESC_REG_0x89
  uint16_t nRegister8A;         //!<  see \ref PERS_SESS_DESC_REG_0x8A
  uint16_t nRegister8B;         //!<  see \ref PERS_SESS_DESC_REG_0x8B
  uint16_t nRegister8C;         //!<  see \ref PERS_SESS_DESC_REG_0x8C
  uint16_t nRegister8D;         //!<  see \ref PERS_SESS_DESC_REG_0x8D
  uint16_t nRegister8E;         //!<  see \ref PERS_SESS_DESC_REG_0x8E
  uint16_t nRegister8F;         //!<  see \ref PERS_SESS_DESC_REG_0x8F
  uint16_t nRegister90;         //!<  see \ref PERS_SESS_DESC_REG_0x90
  uint16_t nRegister91;         //!<  see \ref PERS_SESS_DESC_REG_0x91
  uint16_t nRegister92;         //!<  see \ref PERS_SESS_DESC_REG_0x92
  uint16_t nRegister93;         //!<  see \ref PERS_SESS_DESC_REG_0x93
  uint16_t nRegister94;         //!<  see \ref PERS_SESS_DESC_REG_0x94
  uint16_t nRegister95;         //!<  see \ref PERS_SESS_DESC_REG_0x95
  uint16_t nRegister96;         //!<  see \ref PERS_SESS_DESC_REG_0x96
  uint16_t nRegister97;         //!<  see \ref PERS_SESS_DESC_REG_0x97
  uint16_t nRegister98;         //!<  see \ref PERS_SESS_DESC_REG_0x98
  uint16_t nRegister99;         //!<  see \ref PERS_SESS_DESC_REG_0x99
  uint16_t nRegister9A;         //!<  see \ref PERS_SESS_DESC_REG_0x9A
  uint16_t nRegister9B;         //!<  see \ref PERS_SESS_DESC_REG_0x9B
  uint16_t nRegister9C;         //!<  see \ref PERS_SESS_DESC_REG_0x9C
  uint16_t nRegister9D;         //!<  see \ref PERS_SESS_DESC_REG_0x9D
  uint16_t nRegister9E;         //!<  see \ref PERS_SESS_DESC_REG_0x9E
  uint16_t nRegister9F;         //!<  see \ref PERS_SESS_DESC_REG_0x9F
  uint16_t nRegisterA0;         //!<  see \ref PERS_SESS_DESC_REG_0xA0
  uint16_t nRegisterA1;         //!<  see \ref PERS_SESS_DESC_REG_0xA1
  uint16_t nRegisterA2;         //!<  see \ref PERS_SESS_DESC_REG_0xA2
  uint16_t nRegisterA3;         //!<  see \ref PERS_SESS_DESC_REG_0xA3
  uint16_t nRegisterA4;         //!<  see \ref PERS_SESS_DESC_REG_0xA4
  uint16_t nRegisterA5;         //!<  see \ref PERS_SESS_DESC_REG_0xA5
  uint16_t nRegisterA6;         //!<  see \ref PERS_SESS_DESC_REG_0xA6
  uint16_t nRegisterA7;         //!<  see \ref PERS_SESS_DESC_REG_0xA7
  uint16_t nRegisterA8;         //!<  see \ref PERS_SESS_DESC_REG_0xA8
  uint16_t nRegisterA9;         //!<  see \ref PERS_SESS_DESC_REG_0xA9
  uint16_t nRegisterAA;         //!<  see \ref PERS_SESS_DESC_REG_0xAA
  uint16_t nRegisterAB;         //!<  see \ref PERS_SESS_DESC_REG_0xAB
  uint16_t nRegisterAC;         //!<  see \ref PERS_SESS_DESC_REG_0xAC
  uint16_t nRegisterAD;         //!<  see \ref PERS_SESS_DESC_REG_0xAD
  uint16_t nRegisterAE;         //!<  see \ref PERS_SESS_DESC_REG_0xAE
  uint16_t nRegisterAF;         //!<  see \ref PERS_SESS_DESC_REG_0xAF
  uint16_t nRegisterB0;         //!<  see \ref PERS_SESS_DESC_REG_0xB0
  uint16_t nRegisterB1;         //!<  see \ref PERS_SESS_DESC_REG_0xB1
  uint16_t nRegisterB2;         //!<  see \ref PERS_SESS_DESC_REG_0xB2
  uint16_t nRegisterB3;         //!<  see \ref PERS_SESS_DESC_REG_0xB3
  uint16_t nRegisterB4;         //!<  see \ref PERS_SESS_DESC_REG_0xB4
  uint16_t nRegisterB5;         //!<  see \ref PERS_SESS_DESC_REG_0xB5
  uint16_t nRegisterB6;         //!<  see \ref PERS_SESS_DESC_REG_0xB6
  uint16_t nRegisterB7;         //!<  see \ref PERS_SESS_DESC_REG_0xB7
  uint16_t nRegisterB8;         //!<  see \ref PERS_SESS_DESC_REG_0xB8
  uint16_t nRegisterB9;         //!<  see \ref PERS_SESS_DESC_REG_0xB9
  uint16_t nRegisterBA;         //!<  see \ref PERS_SESS_DESC_REG_0xBA
  uint16_t nRegisterBB;         //!<  see \ref PERS_SESS_DESC_REG_0xBB
  uint16_t nRegisterBC;         //!<  see \ref PERS_SESS_DESC_REG_0xBC
  uint16_t nRegisterBD;         //!<  see \ref PERS_SESS_DESC_REG_0xBD
  uint16_t nRegisterBE;         //!<  see \ref PERS_SESS_DESC_REG_0xBE
  uint16_t nRegisterBF;         //!<  see \ref PERS_SESS_DESC_REG_0xBF
  uint16_t nRegisterC0;         //!<  see \ref PERS_SESS_DESC_REG_0xC0
  uint16_t nRegisterC1;         //!<  see \ref PERS_SESS_DESC_REG_0xC1
  uint16_t nRegisterC2;         //!<  see \ref PERS_SESS_DESC_REG_0xC2
  uint16_t nRegisterC3;         //!<  see \ref PERS_SESS_DESC_REG_0xC3
  uint16_t nRegisterC4;         //!<  see \ref PERS_SESS_DESC_REG_0xC4
  uint16_t nRegisterC5;         //!<  see \ref PERS_SESS_DESC_REG_0xC5
  uint16_t nRegisterC6;         //!<  see \ref PERS_SESS_DESC_REG_0xC6
  uint16_t nRegisterC7;         //!<  see \ref PERS_SESS_DESC_REG_0xC7
  uint16_t nRegisterC8;         //!<  see \ref PERS_SESS_DESC_REG_0xC8
  uint16_t nRegisterC9;         //!<  see \ref PERS_SESS_DESC_REG_0xC9
  uint16_t nRegisterCA;         //!<  see \ref PERS_SESS_DESC_REG_0xCA
  uint16_t nRegisterCB;         //!<  see \ref PERS_SESS_DESC_REG_0xCB
  uint16_t nRegisterCC;         //!<  see \ref PERS_SESS_DESC_REG_0xCC
  uint16_t nRegisterCD;         //!<  see \ref PERS_SESS_DESC_REG_0xCD
  uint16_t nRegisterCE;         //!<  see \ref PERS_SESS_DESC_REG_0xCE
  uint16_t nRegisterCF;         //!<  see \ref PERS_SESS_DESC_REG_0xCF
  uint16_t nRegisterD0;         //!<  see \ref PERS_SESS_DESC_REG_0xD0
  uint16_t nRegisterD1;         //!<  see \ref PERS_SESS_DESC_REG_0xD1
  uint16_t nRegisterD2;         //!<  see \ref PERS_SESS_DESC_REG_0xD2
  uint16_t nRegisterD3;         //!<  see \ref PERS_SESS_DESC_REG_0xD3
  uint16_t nRegisterD4;         //!<  see \ref PERS_SESS_DESC_REG_0xD4
  uint16_t nRegisterD5;         //!<  see \ref PERS_SESS_DESC_REG_0xD5
  uint16_t nRegisterD6;         //!<  see \ref PERS_SESS_DESC_REG_0xD6
  uint16_t nRegisterD7;         //!<  see \ref PERS_SESS_DESC_REG_0xD7
  uint16_t nRegisterD8;         //!<  see \ref PERS_SESS_DESC_REG_0xD8
  uint16_t nRegisterD9;         //!<  see \ref PERS_SESS_DESC_REG_0xD9
  uint16_t nRegisterDA;         //!<  see \ref PERS_SESS_DESC_REG_0xDA
  uint16_t nRegisterDB;         //!<  see \ref PERS_SESS_DESC_REG_0xDB
  uint16_t nRegisterDC;         //!<  see \ref PERS_SESS_DESC_REG_0xDC
  uint16_t nRegisterDD;         //!<  see \ref PERS_SESS_DESC_REG_0xDD
  uint16_t nRegisterDE;         //!<  see \ref PERS_SESS_DESC_REG_0xDE
  uint16_t nRegisterDF;         //!<  see \ref PERS_SESS_DESC_REG_0xDF
  uint16_t nRegisterE0;         //!<  see \ref PERS_SESS_DESC_REG_0xE0
  uint16_t nRegisterE1;         //!<  see \ref PERS_SESS_DESC_REG_0xE1
  uint16_t nRegisterE2;         //!<  see \ref PERS_SESS_DESC_REG_0xE2
  uint16_t nRegisterE3;         //!<  see \ref PERS_SESS_DESC_REG_0xE3
  uint16_t nRegisterE4;         //!<  see \ref PERS_SESS_DESC_REG_0xE4
  uint16_t nRegisterE5;         //!<  see \ref PERS_SESS_DESC_REG_0xE5
  uint16_t nRegisterE6;         //!<  see \ref PERS_SESS_DESC_REG_0xE6
  uint16_t nRegisterE7;         //!<  see \ref PERS_SESS_DESC_REG_0xE7
  uint16_t nRegisterE8;         //!<  see \ref PERS_SESS_DESC_REG_0xE8
  uint16_t nRegisterE9;         //!<  see \ref PERS_SESS_DESC_REG_0xE9
  uint16_t nRegisterEA;         //!<  see \ref PERS_SESS_DESC_REG_0xEA
  uint16_t nRegisterEB;         //!<  see \ref PERS_SESS_DESC_REG_0xEB
  uint16_t nRegisterEC;         //!<  see \ref PERS_SESS_DESC_REG_0xEC
  uint16_t nRegisterED;         //!<  see \ref PERS_SESS_DESC_REG_0xED
  uint16_t nRegisterEE;         //!<  see \ref PERS_SESS_DESC_REG_0xEE
  uint16_t nRegisterEF;         //!<  see \ref PERS_SESS_DESC_REG_0xEF
  uint16_t nRegisterF0;         //!<  see \ref PERS_SESS_DESC_REG_0xF0
  uint16_t nRegisterF1;         //!<  see \ref PERS_SESS_DESC_REG_0xF1
  uint16_t nRegisterF2;         //!<  see \ref PERS_SESS_DESC_REG_0xF2
  uint16_t nRegisterF3;         //!<  see \ref PERS_SESS_DESC_REG_0xF3
  uint16_t nRegisterF4;         //!<  see \ref PERS_SESS_DESC_REG_0xF4
  uint16_t nRegisterF5;         //!<  see \ref PERS_SESS_DESC_REG_0xF5
  uint16_t nRegisterF6;         //!<  see \ref PERS_SESS_DESC_REG_0xF6
  uint16_t nRegisterF7;         //!<  see \ref PERS_SESS_DESC_REG_0xF7
  uint16_t nRegisterF8;         //!<  see \ref PERS_SESS_DESC_REG_0xF8
  uint16_t nRegisterF9;         //!<  see \ref PERS_SESS_DESC_REG_0xF9
  uint16_t nRegisterFA;         //!<  see \ref PERS_SESS_DESC_REG_0xFA
  uint16_t nRegisterFB;         //!<  see \ref PERS_SESS_DESC_REG_0xFB
  uint16_t nRegisterFC;         //!<  see \ref PERS_SESS_DESC_REG_0xFC
  uint16_t nRegisterFD;         //!<  see \ref PERS_SESS_DESC_REG_0xFD
  uint16_t nRegisterFE;         //!<  see \ref PERS_SESS_DESC_REG_0xFE
  uint16_t nRegisterFF;         //!<  see \ref PERS_SESS_DESC_REG_0xFF
}RegsStruct_t;

/**
  * @}
  */ 

/** @defgroup WeSU_LOW_LEVEL_Exported_Defines WeSU_LOW_LEVEL_Exported_Defines
  * @{
  */

#define PERSISTENT_DEFAULT_REG_0x00               FIRMWARE_VERSION                                                        //!< persistent register 00 default value
#define PERSISTENT_DEFAULT_REG_0x01               FIRMWARE_VERSION                                                        //!< persistent register 01 default value
#define PERSISTENT_DEFAULT_REG_0x02               (LED_CONFIG_APP_CONTROLLED_MASK | (LED_CONFIG_BRIGHT_MAX&0xFF00))       //!< persistent register 02 default value
#define PERSISTENT_DEFAULT_REG_0x03               STORE_U8_LE(0x09,'W')                                                   //!< persistent register 03 default value
#define PERSISTENT_DEFAULT_REG_0x04               STORE_U8_LE('e','S')                                                    //!< persistent register 04 default value
#define PERSISTENT_DEFAULT_REG_0x05               STORE_U8_LE('U','\0')                                                   //!< persistent register 05 default value
#define PERSISTENT_DEFAULT_REG_0x06               0x0000                                                                  //!< persistent register 06 default value
#define PERSISTENT_DEFAULT_REG_0x07               0x0000                                                                  //!< persistent register 07 default value
#define PERSISTENT_DEFAULT_REG_0x08               0x0000                                                                  //!< persistent register 08 default value
#define PERSISTENT_DEFAULT_REG_0x09               0x0000                                                                  //!< persistent register 09 default value
#define PERSISTENT_DEFAULT_REG_0x0A               0x0000                                                                  //!< persistent register 0A default value
#define PERSISTENT_DEFAULT_REG_0x0B               STORE_U8_LE(BLE_PUB_ADDR_DEFAULT_0,BLE_PUB_ADDR_DEFAULT_1)              //!< persistent register 0B default value
#define PERSISTENT_DEFAULT_REG_0x0C               STORE_U8_LE(BLE_PUB_ADDR_DEFAULT_2,BLE_PUB_ADDR_DEFAULT_3)              //!< persistent register 0C default value
#define PERSISTENT_DEFAULT_REG_0x0D               STORE_U8_LE(BLE_PUB_ADDR_DEFAULT_4,BLE_PUB_ADDR_DEFAULT_5)              //!< persistent register 0D default value
#define PERSISTENT_DEFAULT_REG_0x0E               0x0000                                                                  //!< persistent register 0E default value
#define PERSISTENT_DEFAULT_REG_0x0F               0x0000                                                                  //!< persistent register 0F default value
#define PERSISTENT_DEFAULT_REG_0x10               0x0000                                                                  //!< persistent register 10 default value
#define PERSISTENT_DEFAULT_REG_0x11               0x0000                                                                  //!< persistent register 11 default value
#define PERSISTENT_DEFAULT_REG_0x12               0x0000                                                                  //!< persistent register 12 default value
#define PERSISTENT_DEFAULT_REG_0x13               0x0000                                                                  //!< persistent register 13 default value
#define PERSISTENT_DEFAULT_REG_0x14               0x0000                                                                  //!< persistent register 14 default value
#define PERSISTENT_DEFAULT_REG_0x15               0x0000                                                                  //!< persistent register 15 default value
#define PERSISTENT_DEFAULT_REG_0x16               0x0000                                                                  //!< persistent register 16 default value
#define PERSISTENT_DEFAULT_REG_0x17               0x0000                                                                  //!< persistent register 17 default value
#define PERSISTENT_DEFAULT_REG_0x18               0x0000                                                                  //!< persistent register 18 default value
#define PERSISTENT_DEFAULT_REG_0x19               0x0000                                                                  //!< persistent register 19 default value
#define PERSISTENT_DEFAULT_REG_0x1A               0x0000                                                                  //!< persistent register 1A default value
#define PERSISTENT_DEFAULT_REG_0x1B               0x0000                                                                  //!< persistent register 1B default value
#define PERSISTENT_DEFAULT_REG_0x1C               0x0000                                                                  //!< persistent register 1C default value
#define PERSISTENT_DEFAULT_REG_0x1D               0x0000                                                                  //!< persistent register 1D default value
#define PERSISTENT_DEFAULT_REG_0x1E               WeSU_IWDG_DEFAULT_RELOAD_VALUE                                          //!< persistent register 1E default value
#define PERSISTENT_DEFAULT_REG_0x1F               WeSU_DEBUG_DEFAULT_VALUE                                                //!< persistent register 1F default value
#define PERSISTENT_DEFAULT_REG_0x20               STORE_U8_LE(DEFAULT_BLE_TX_POWER_VALID_MASK|DEFAULT_BLE_TX_POWER_LEVEL, DEFAULT_BLE_TX_POWER_VALID_MASK|DEFAULT_BLE_TX_HIGH_POWER)    //!< persistent register 20 default value
#define PERSISTENT_DEFAULT_REG_0x21               DEFAULT_TIMER_FREQUENCY                                                 //!< persistent register 21 default value
#define PERSISTENT_DEFAULT_REG_0x22               WESU_SYS_POWER_FULLRUN_WFI                                              //!< persistent register 22 default value
#define PERSISTENT_DEFAULT_REG_0x23               WESU_L_V2_HW_CAPABILITIES                                               //!< persistent register 23 default value
#define PERSISTENT_DEFAULT_REG_0x24               0x0000                                                                  //!< persistent register 24 default value
#define PERSISTENT_DEFAULT_REG_0x25               STORE_U8_LE(PWR_FEATURE_MASK_CHANNEL_DEFAULT_VALUE,PWR_FEATURE_MASK_SUBSAMPL_DEFAULT_VALUE)     //!< persistent register 25 default value
#define PERSISTENT_DEFAULT_REG_0x26               STORE_U8_LE(TEMPERATURE_FEATURE_MASK_CHANNEL_DEFAULT_VALUE,TEMPERATURE_FEATURE_MASK_SUBSAMPL_DEFAULT_VALUE)    //!< persistent register 26 default value
#define PERSISTENT_DEFAULT_REG_0x27               0x0000                                                                  //!< persistent register 27 default value
#define PERSISTENT_DEFAULT_REG_0x28               STORE_U8_LE(PRESSURE_FEATURE_MASK_CHANNEL_DEFAULT_VALUE,PRESSURE_FEATURE_MASK_SUBSAMPL_DEFAULT_VALUE)           //!< persistent register 28 default value
#define PERSISTENT_DEFAULT_REG_0x29               0x0000                                                                  //!< persistent register 29 default value
#define PERSISTENT_DEFAULT_REG_0x2A               0x0000                                                                  //!< persistent register 2A default value
#define PERSISTENT_DEFAULT_REG_0x2B               STORE_U8_LE(RAWMOTION_FEATURE_MASK_CHANNEL_DEFAULT_VALUE,RAWMOTION_FEATURE_MASK_SUBSAMPL_DEFAULT_VALUE)         //!< persistent register 2B default value
#define PERSISTENT_DEFAULT_REG_0x2C               0x0000                                                                  //!< persistent register 2C default value
#define PERSISTENT_DEFAULT_REG_0x2D               0x0000                                                                  //!< persistent register 2D default value
#define PERSISTENT_DEFAULT_REG_0x2E               0x0000                                                                  //!< persistent register 2E default value
#define PERSISTENT_DEFAULT_REG_0x2F               0x0000                                                                  //!< persistent register 2F default value
#define PERSISTENT_DEFAULT_REG_0x30               0x0000                                                                  //!< persistent register 30 default value
#define PERSISTENT_DEFAULT_REG_0x31               0x0000                                                                  //!< persistent register 31 default value
#define PERSISTENT_DEFAULT_REG_0x32               0x0000                                                                  //!< persistent register 32 default value
#define PERSISTENT_DEFAULT_REG_0x33               0x0000                                                                  //!< persistent register 33 default value
#define PERSISTENT_DEFAULT_REG_0x34               WESU_L_V2_FW_CAPABILITIES                                               //!< persistent register 34 default value
#define PERSISTENT_DEFAULT_REG_0x35               0x0000                                                                  //!< persistent register 35 default value */
#define PERSISTENT_DEFAULT_REG_0x36               0x0000                                                                  //!< persistent register 36 default value
#define PERSISTENT_DEFAULT_REG_0x37               0x0000                                                                  //!< persistent register 37 default value
#define PERSISTENT_DEFAULT_REG_0x38               STORE_U8_LE(MOTION_CP_CONTROL_MASK_CHANNEL_DEFAULT_VALUE,MOTION_CP_CONTROL_MASK_SUBSAMPL_DEFAULT_VALUE) //!< persistent register 38 default value
#define PERSISTENT_DEFAULT_REG_0x39               STORE_U8_LE(MOTION_AR_CONTROL_MASK_CHANNEL_DEFAULT_VALUE,MOTION_AR_CONTROL_MASK_SUBSAMPL_DEFAULT_VALUE) //!< persistent register 39 default value
#define PERSISTENT_DEFAULT_REG_0x3A               0x0000                                                                  //!< persistent register 3A default value
#define PERSISTENT_DEFAULT_REG_0x3B               0x0000                                                                  //!< persistent register 3B default value
#define PERSISTENT_DEFAULT_REG_0x3C               0x0000                                                                  //!< persistent register 3C default value
#define PERSISTENT_DEFAULT_REG_0x3D               STORE_U8_LE(MOTION_FX_CONTROL_MASK_CHANNEL_DEFAULT_VALUE,MOTION_FX_CONTROL_MASK_SUBSAMPL_DEFAULT_VALUE)               //!< persistent register 3D default value
#define PERSISTENT_DEFAULT_REG_0x3E               0x0000                                                                  //!< persistent register 3E default value
#define PERSISTENT_DEFAULT_REG_0x3F               0X0000                                                                  //!< persistent register 3F default value
#define PERSISTENT_DEFAULT_REG_0x40               STORE_U8_LE(ACCEVENT_CONTROL_MASK_CHANNEL_DEFAULT_VALUE,ACCEVENT_CONTROL_MASK_SUBSAMPL_DEFAULT_VALUE)                 //!< persistent register 40 default value
#define PERSISTENT_DEFAULT_REG_0x41               0x0000                                                                  //!< persistent register 41 default value
#define PERSISTENT_DEFAULT_REG_0x42               0x0000                                                                  //!< persistent register 42 default value
#define PERSISTENT_DEFAULT_REG_0x43               0x0000                                                                  //!< persistent register 43 default value
#define PERSISTENT_DEFAULT_REG_0x44               0x0000                                                                  //!< persistent register 44 default value
#define PERSISTENT_DEFAULT_REG_0x45               BLE_DEBUG_CONFIG_DEFAULT_VALUE                                          //!< persistent register 45 default value
#define PERSISTENT_DEFAULT_REG_0x46               USART_DEBUG_CONFIG_DEFAULT_VALUE                                        //!< persistent register 46 default value
#define PERSISTENT_DEFAULT_REG_0x47               WeSU_DEBUG_SEVERITY_INFO                                                //!< persistent register 47 default value
#define PERSISTENT_DEFAULT_REG_0x48               STORE_U8_LE(GP_ALGORITHM_CONTROL_MASK_CHANNEL_DEFAULT_VALUE,GP_ALGORITHM_CONTROL_MASK_SUBSAMPL_DEFAULT_VALUE)   //!< persistent register 44 default value
#define PERSISTENT_DEFAULT_REG_0x49               HW_OPERATING_CAPABILITIES_DEFAULT_REG_VALUE                             //!< persistent register 49 default value
#define PERSISTENT_DEFAULT_REG_0x4A               FW_OPERATING_CAPABILITIES_DEFAULT_REG_VALUE                             //!< persistent register 4A default value
#define PERSISTENT_DEFAULT_REG_0x4B               BLE_CON_INTV_DEFAULT_REG_VALUE                                          //!< persistent register 4B default value
#define PERSISTENT_DEFAULT_REG_0x4C               LED_INTERVAL_DISCONNECTED_DEFAULT_VALUE                                 //!< persistent register 4C default value
#define PERSISTENT_DEFAULT_REG_0x4D               LED_INTERVAL_DISCONNECTED_ON_DEFAULT_VALUE                              //!< persistent register 4D default value
#define PERSISTENT_DEFAULT_REG_0x4E               LED_INTERVAL_CONNECTED_DEFAULT_VALUE                                    //!< persistent register 4E default value
#define PERSISTENT_DEFAULT_REG_0x4F               LED_INTERVAL_CONNECTED_ON_DEFAULT_VALUE                                 //!< persistent register 4F default value
#define PERSISTENT_DEFAULT_REG_0x50               0x0000                                                                  //!< persistent register 50 default value
#define PERSISTENT_DEFAULT_REG_0x51               0x0000                                                                  //!< persistent register 51 default value
#define PERSISTENT_DEFAULT_REG_0x52               0x0000                                                                  //!< persistent register 52 default value
#define PERSISTENT_DEFAULT_REG_0x53               0x0000                                                                  //!< persistent register 53 default value
#define PERSISTENT_DEFAULT_REG_0x54               0x0000                                                                  //!< persistent register 54 default value
#define PERSISTENT_DEFAULT_REG_0x55               AUTOSLEEP_TIME_DEFAULT_REG_VALUE                                        //!< persistent register 55 default value
#define PERSISTENT_DEFAULT_REG_0x56               0x0000                                                                  //!< persistent register 56 default value
#define PERSISTENT_DEFAULT_REG_0x57               0x0000                                                                  //!< persistent register 57 default value
#define PERSISTENT_DEFAULT_REG_0x58               0x0000                                                                  //!< persistent register 58 default value
#define PERSISTENT_DEFAULT_REG_0x59               BUTTON_PRESS_MODE_DEFAULT_VALUE                                         //!< persistent register 59 default value
#define PERSISTENT_DEFAULT_REG_0x5A               CONN_ONLY_MODE_DEFAULT_VALUE                                            //!< persistent register 5A default value
#define PERSISTENT_DEFAULT_REG_0x5B               AUTOSLEEP_MODE_DEFAULT_VALUE                                            //!< persistent register 5B default value
#define PERSISTENT_DEFAULT_REG_0x5C               0x0000                                                                  //!< persistent register 5C default value
#define PERSISTENT_DEFAULT_REG_0x5D               0x0000                                                                  //!< persistent register 5D default value
#define PERSISTENT_DEFAULT_REG_0x5E               0x0000                                                                  //!< persistent register 5E default value
#define PERSISTENT_DEFAULT_REG_0x5F               0x0000                                                                  //!< persistent register 5F default value
#define PERSISTENT_DEFAULT_REG_0x60               DEFAULT_CALIBRATION                                                     //!< persistent register 60 default value
#define PERSISTENT_DEFAULT_REG_0x61               0x0000                                                                  //!< persistent register 61 default value
#define PERSISTENT_DEFAULT_REG_0x62               0x0000                                                                  //!< persistent register 62 default value
#define PERSISTENT_DEFAULT_REG_0x63               0x0000                                                                  //!< persistent register 63 default value
#define PERSISTENT_DEFAULT_REG_0x64               0x0000                                                                  //!< persistent register 64 default value
#define PERSISTENT_DEFAULT_REG_0x65               0x0000                                                                  //!< persistent register 65 default value
#define PERSISTENT_DEFAULT_REG_0x66               0x0000                                                                  //!< persistent register 66 default value
#define PERSISTENT_DEFAULT_REG_0x67               0x0000                                                                  //!< persistent register 67 default value
#define PERSISTENT_DEFAULT_REG_0x68               0x0000                                                                  //!< persistent register 68 default value
#define PERSISTENT_DEFAULT_REG_0x69               0x0000                                                                  //!< persistent register 69 default value
#define PERSISTENT_DEFAULT_REG_0x6A               0x0000                                                                  //!< persistent register 6A default value
#define PERSISTENT_DEFAULT_REG_0x6B               0x0000                                                                  //!< persistent register 6B default value
#define PERSISTENT_DEFAULT_REG_0x6C               0x0000                                                                  //!< persistent register 6C default value
#define PERSISTENT_DEFAULT_REG_0x6D               0x0000                                                                  //!< persistent register 6D default value
#define PERSISTENT_DEFAULT_REG_0x6E               CALIBRATION_HW_DEFAULT_REG_VALUE                                        //!< persistent register 6E default value
#define PERSISTENT_DEFAULT_REG_0x6F               CALIBRATION_FW_DEFAULT_REG_VALUE                                        //!< persistent register 6F default value
#define PERSISTENT_DEFAULT_REG_0x70               BLE_ADV_INTV_RUN_DEFAULT                                                //!< persistent register 70 default value
#define PERSISTENT_DEFAULT_REG_0x71               BLE_ADV_INTV_SLEEP_DEFAULT                                              //!< persistent register 71 default value
#define PERSISTENT_DEFAULT_REG_0x72               0x0000                                                                  //!< persistent register 72 default value
#define PERSISTENT_DEFAULT_REG_0x73               0x0000                                                                  //!< persistent register 73 default value
#define PERSISTENT_DEFAULT_REG_0x74               ACC_FullScale_DEFAULT                                                   //!< persistent register 74 default value
#define PERSISTENT_DEFAULT_REG_0x75               ACC_OutputDataRate_DEFAULT                                              //!< persistent register 75 default value
#define PERSISTENT_DEFAULT_REG_0x76               GYRO_FullScale_DEFAULT                                                  //!< persistent register 76 default value
#define PERSISTENT_DEFAULT_REG_0x77               GYRO_OutputDataRate_DEFAULT                                             //!< persistent register 77 default value
#define PERSISTENT_DEFAULT_REG_0x78               MAG_FullScale_DEFAULT                                                   //!< persistent register 78 default value
#define PERSISTENT_DEFAULT_REG_0x79               MAG_OutputDataRate_DEFAULT                                              //!< persistent register 79 default value
#define PERSISTENT_DEFAULT_REG_0x7A               ACCEVENT_CONF_DEFAULT                                                   //!< persistent register 7A default value
#define PERSISTENT_DEFAULT_REG_0x7B               PRESS_TEMP_OutputDataRate_DEFAULT                                       //!< persistent register 7B default value
#define PERSISTENT_DEFAULT_REG_0x7C               0x0000                                                                  //!< persistent register 7C default value
#define PERSISTENT_DEFAULT_REG_0x7D               0x0000                                                                  //!< persistent register 7D default value
#define PERSISTENT_DEFAULT_REG_0x7E               0x0000                                                                  //!< persistent register 7E default value
#define PERSISTENT_DEFAULT_REG_0x7F               0x0000                                                                  //!< persistent register 7F default value
#define PERSISTENT_DEFAULT_REG_0x80               GP_ALGORITHM_THS1_DEFAULT                                               //!< persistent register 80 default value
#define PERSISTENT_DEFAULT_REG_0x81               GP_ALGORITHM_THS2_DEFAULT                                               //!< persistent register 81 default value
#define PERSISTENT_DEFAULT_REG_0x82               0x0000                                                                  //!< persistent register 82 default value
#define PERSISTENT_DEFAULT_REG_0x83               0x0000                                                                  //!< persistent register 83 default value
#define PERSISTENT_DEFAULT_REG_0x84               0x0000                                                                  //!< persistent register 84 default value
#define PERSISTENT_DEFAULT_REG_0x85               0x0000                                                                  //!< persistent register 85 default value
#define PERSISTENT_DEFAULT_REG_0x86               0x0000                                                                  //!< persistent register 86 default value
#define PERSISTENT_DEFAULT_REG_0x87               0x0000                                                                  //!< persistent register 87 default value
#define PERSISTENT_DEFAULT_REG_0x88               0x0000                                                                  //!< persistent register 88 default value
#define PERSISTENT_DEFAULT_REG_0x89               0x0000                                                                  //!< persistent register 89 default value
#define PERSISTENT_DEFAULT_REG_0x8A               0x0000                                                                  //!< persistent register 8A default value
#define PERSISTENT_DEFAULT_REG_0x8B               0x0000                                                                  //!< persistent register 8B default value
#define PERSISTENT_DEFAULT_REG_0x8C               0x0000                                                                  //!< persistent register 8C default value
#define PERSISTENT_DEFAULT_REG_0x8D               0x0000                                                                  //!< persistent register 8D default value
#define PERSISTENT_DEFAULT_REG_0x8E               0x0000                                                                  //!< persistent register 8E default value
#define PERSISTENT_DEFAULT_REG_0x8F               0x0000                                                                  //!< persistent register 8F default value
#define PERSISTENT_DEFAULT_REG_0x90               STORE_U8_LE(HOURS_DEFAULT,MINUTES_DEFAULT)                              //!< persistent register 90 default value
#define PERSISTENT_DEFAULT_REG_0x91               STORE_U8_LE(SECONDS_DEFAULT,DAY_DEFAULT)                                //!< persistent register 91 default value
#define PERSISTENT_DEFAULT_REG_0x92               STORE_U8_LE(MONTH_DEFAULT, YEAR_DEFAULT)                                //!< persistent register 92 default value
#define PERSISTENT_DEFAULT_REG_0x93               STORE_U8_LE(WEEKDAY_DEFAULT, RTC_CONFIG_DEFAULT)                        //!< persistent register 93 default value
#define PERSISTENT_DEFAULT_REG_0x94               0x0000                                                                  //!< persistent register 94 default value
#define PERSISTENT_DEFAULT_REG_0x95               0x0000                                                                  //!< persistent register 95 default value
#define PERSISTENT_DEFAULT_REG_0x96               0x0000                                                                  //!< persistent register 96 default value
#define PERSISTENT_DEFAULT_REG_0x97               0x0000                                                                  //!< persistent register 97 default value
#define PERSISTENT_DEFAULT_REG_0x98               0x0000                                                                  //!< persistent register 98 default value
#define PERSISTENT_DEFAULT_REG_0x99               0x0000                                                                  //!< persistent register 99 default value
#define PERSISTENT_DEFAULT_REG_0x9A               0x0000                                                                  //!< persistent register 9A default value
#define PERSISTENT_DEFAULT_REG_0x9B               0x0000                                                                  //!< persistent register 9B default value
#define PERSISTENT_DEFAULT_REG_0x9C               0x0000                                                                  //!< persistent register 9C default value
#define PERSISTENT_DEFAULT_REG_0x9D               0x0000                                                                  //!< persistent register 9D default value
#define PERSISTENT_DEFAULT_REG_0x9E               0x0000                                                                  //!< persistent register 9E default value
#define PERSISTENT_DEFAULT_REG_0x9F               0x0000                                                                  //!< persistent register 9F default value
#define PERSISTENT_DEFAULT_REG_0xA0               0x0000                                                                  //!< persistent register A0 default value
#define PERSISTENT_DEFAULT_REG_0xA1               0x0000                                                                  //!< persistent register A1 default value
#define PERSISTENT_DEFAULT_REG_0xA2               0x0000                                                                  //!< persistent register A2 default value
#define PERSISTENT_DEFAULT_REG_0xA3               0x0000                                                                  //!< persistent register A3 default value
#define PERSISTENT_DEFAULT_REG_0xA4               0x0000                                                                  //!< persistent register A4 default value
#define PERSISTENT_DEFAULT_REG_0xA5               0x0000                                                                  //!< persistent register A5 default value
#define PERSISTENT_DEFAULT_REG_0xA6               0x0000                                                                  //!< persistent register A6 default value
#define PERSISTENT_DEFAULT_REG_0xA7               0x0000                                                                  //!< persistent register A7 default value
#define PERSISTENT_DEFAULT_REG_0xA8               0x0000                                                                  //!< persistent register A8 default value
#define PERSISTENT_DEFAULT_REG_0xA9               0x0000                                                                  //!< persistent register A9 default value
#define PERSISTENT_DEFAULT_REG_0xAA               0x0000                                                                  //!< persistent register AA default value
#define PERSISTENT_DEFAULT_REG_0xAB               0x0000                                                                  //!< persistent register AB default value
#define PERSISTENT_DEFAULT_REG_0xAC               0x0000                                                                  //!< persistent register AC default value
#define PERSISTENT_DEFAULT_REG_0xAD               0x0000                                                                  //!< persistent register AD default value
#define PERSISTENT_DEFAULT_REG_0xAE               0x0000                                                                  //!< persistent register AE default value
#define PERSISTENT_DEFAULT_REG_0xAF               0x0000                                                                  //!< persistent register AF default value
#define PERSISTENT_DEFAULT_REG_0xB0               0x0000                                                                  //!< persistent register B0 default value
#define PERSISTENT_DEFAULT_REG_0xB1               0x0000                                                                  //!< persistent register B1 default value
#define PERSISTENT_DEFAULT_REG_0xB2               0x0000                                                                  //!< persistent register B2 default value
#define PERSISTENT_DEFAULT_REG_0xB3               0x0000                                                                  //!< persistent register B3 default value
#define PERSISTENT_DEFAULT_REG_0xB4               0x0000                                                                  //!< persistent register B4 default value
#define PERSISTENT_DEFAULT_REG_0xB5               0x0000                                                                  //!< persistent register B5 default value
#define PERSISTENT_DEFAULT_REG_0xB6               0x0000                                                                  //!< persistent register B6 default value
#define PERSISTENT_DEFAULT_REG_0xB7               0x0000                                                                  //!< persistent register B7 default value
#define PERSISTENT_DEFAULT_REG_0xB8               0x0000                                                                  //!< persistent register B8 default value
#define PERSISTENT_DEFAULT_REG_0xB9               0x0000                                                                  //!< persistent register B9 default value
#define PERSISTENT_DEFAULT_REG_0xBA               0x0000                                                                  //!< persistent register BA default value
#define PERSISTENT_DEFAULT_REG_0xBB               0x0000                                                                  //!< persistent register BB default value
#define PERSISTENT_DEFAULT_REG_0xBC               0x0000                                                                  //!< persistent register BC default value
#define PERSISTENT_DEFAULT_REG_0xBD               0x0000                                                                  //!< persistent register BD default value
#define PERSISTENT_DEFAULT_REG_0xBE               0x0000                                                                  //!< persistent register BE default value
#define PERSISTENT_DEFAULT_REG_0xBF               0x0000                                                                  //!< persistent register BF default value
#define PERSISTENT_DEFAULT_REG_0xC0               0x0000                                                                  //!< persistent register C0 default value
#define PERSISTENT_DEFAULT_REG_0xC1               0x0000                                                                  //!< persistent register C1 default value
#define PERSISTENT_DEFAULT_REG_0xC2               0x0000                                                                  //!< persistent register C2 default value
#define PERSISTENT_DEFAULT_REG_0xC3               0x0000                                                                  //!< persistent register C3 default value
#define PERSISTENT_DEFAULT_REG_0xC4               0x0000                                                                  //!< persistent register C4 default value
#define PERSISTENT_DEFAULT_REG_0xC5               0x0000                                                                  //!< persistent register C5 default value
#define PERSISTENT_DEFAULT_REG_0xC6               0x0000                                                                  //!< persistent register C6 default value
#define PERSISTENT_DEFAULT_REG_0xC7               0x0000                                                                  //!< persistent register C7 default value
#define PERSISTENT_DEFAULT_REG_0xC8               0x0000                                                                  //!< persistent register C8 default value
#define PERSISTENT_DEFAULT_REG_0xC9               0x0000                                                                  //!< persistent register C9 default value
#define PERSISTENT_DEFAULT_REG_0xCA               0x0000                                                                  //!< persistent register CA default value
#define PERSISTENT_DEFAULT_REG_0xCB               0x0000                                                                  //!< persistent register CB default value
#define PERSISTENT_DEFAULT_REG_0xCC               0x0000                                                                  //!< persistent register CC default value
#define PERSISTENT_DEFAULT_REG_0xCD               0x0000                                                                  //!< persistent register CD default value
#define PERSISTENT_DEFAULT_REG_0xCE               0x0000                                                                  //!< persistent register CE default value
#define PERSISTENT_DEFAULT_REG_0xCF               0x0000                                                                  //!< persistent register CF default value
#define PERSISTENT_DEFAULT_REG_0xD0               0x0000                                                                  //!< persistent register D0 default value
#define PERSISTENT_DEFAULT_REG_0xD1               0x0000                                                                  //!< persistent register D1 default value
#define PERSISTENT_DEFAULT_REG_0xD2               0x0000                                                                  //!< persistent register D2 default value
#define PERSISTENT_DEFAULT_REG_0xD3               0x0000                                                                  //!< persistent register D3 default value
#define PERSISTENT_DEFAULT_REG_0xD4               0x0000                                                                  //!< persistent register D4 default value
#define PERSISTENT_DEFAULT_REG_0xD5               0x0000                                                                  //!< persistent register D5 default value
#define PERSISTENT_DEFAULT_REG_0xD6               0x0000                                                                  //!< persistent register D6 default value
#define PERSISTENT_DEFAULT_REG_0xD7               0x0000                                                                  //!< persistent register D7 default value
#define PERSISTENT_DEFAULT_REG_0xD8               0x0000                                                                  //!< persistent register D8 default value
#define PERSISTENT_DEFAULT_REG_0xD9               0x0000                                                                  //!< persistent register D9 default value
#define PERSISTENT_DEFAULT_REG_0xDA               0x0000                                                                  //!< persistent register DA default value
#define PERSISTENT_DEFAULT_REG_0xDB               0x0000                                                                  //!< persistent register DB default value
#define PERSISTENT_DEFAULT_REG_0xDC               0x0000                                                                  //!< persistent register DC default value
#define PERSISTENT_DEFAULT_REG_0xDD               0x0000                                                                  //!< persistent register DD default value
#define PERSISTENT_DEFAULT_REG_0xDE               0x0000                                                                  //!< persistent register DE default value
#define PERSISTENT_DEFAULT_REG_0xDF               0x0000                                                                  //!< persistent register DF default value
#define PERSISTENT_DEFAULT_REG_0xE0               0x0000                                                                  //!< persistent register E0 default value
#define PERSISTENT_DEFAULT_REG_0xE1               0x0000                                                                  //!< persistent register E1 default value
#define PERSISTENT_DEFAULT_REG_0xE2               0x0000                                                                  //!< persistent register E2 default value
#define PERSISTENT_DEFAULT_REG_0xE3               0x0000                                                                  //!< persistent register E3 default value
#define PERSISTENT_DEFAULT_REG_0xE4               0x0000                                                                  //!< persistent register E4 default value
#define PERSISTENT_DEFAULT_REG_0xE5               0x0000                                                                  //!< persistent register E5 default value
#define PERSISTENT_DEFAULT_REG_0xE6               0x0000                                                                  //!< persistent register E6 default value
#define PERSISTENT_DEFAULT_REG_0xE7               0x0000                                                                  //!< persistent register E7 default value
#define PERSISTENT_DEFAULT_REG_0xE8               0x0000                                                                  //!< persistent register E8 default value
#define PERSISTENT_DEFAULT_REG_0xE9               0x0000                                                                  //!< persistent register E9 default value
#define PERSISTENT_DEFAULT_REG_0xEA               0x0000                                                                  //!< persistent register EA default value
#define PERSISTENT_DEFAULT_REG_0xEB               0x0000                                                                  //!< persistent register EB default value
#define PERSISTENT_DEFAULT_REG_0xEC               0x0000                                                                  //!< persistent register EC default value
#define PERSISTENT_DEFAULT_REG_0xED               0x0000                                                                  //!< persistent register ED default value
#define PERSISTENT_DEFAULT_REG_0xEE               0x0000                                                                  //!< persistent register EE default value
#define PERSISTENT_DEFAULT_REG_0xEF               0x0000                                                                  //!< persistent register EF default value
#define PERSISTENT_DEFAULT_REG_0xF0               0x0000                                                                  //!< persistent register F0 default value
#define PERSISTENT_DEFAULT_REG_0xF1               0x0000                                                                  //!< persistent register F1 default value
#define PERSISTENT_DEFAULT_REG_0xF2               0x0000                                                                  //!< persistent register F2 default value
#define PERSISTENT_DEFAULT_REG_0xF3               0x0000                                                                  //!< persistent register F3 default value
#define PERSISTENT_DEFAULT_REG_0xF4               0x0000                                                                  //!< persistent register F4 default value
#define PERSISTENT_DEFAULT_REG_0xF5               0x0000                                                                  //!< persistent register F5 default value
#define PERSISTENT_DEFAULT_REG_0xF6               0x0000                                                                  //!< persistent register F6 default value
#define PERSISTENT_DEFAULT_REG_0xF7               0x0000                                                                  //!< persistent register F7 default value
#define PERSISTENT_DEFAULT_REG_0xF8               0x0000                                                                  //!< persistent register F8 default value
#define PERSISTENT_DEFAULT_REG_0xF9               0x0000                                                                  //!< persistent register F9 default value
#define PERSISTENT_DEFAULT_REG_0xFA               0x0000                                                                  //!< persistent register FA default value
#define PERSISTENT_DEFAULT_REG_0xFB               0x0000                                                                  //!< persistent register FB default value
#define PERSISTENT_DEFAULT_REG_0xFC               STORE_U8_LE('F','W')                                                    //!< persistent register FC default value
#define PERSISTENT_DEFAULT_REG_0xFD               FIRMWARE_VERSION                                                        //!< persistent register FD default value
#define PERSISTENT_DEFAULT_REG_0xFE               STORE_U8_LE('_','E')                                                    //!< persistent register FE default value
#define PERSISTENT_DEFAULT_REG_0xFF               STORE_U8_LE('N','D')                                                    //!< persistent register FF default value


#define SESSION_DEFAULT_REG_0x00               FIRMWARE_VERSION                                                        //!< session register 00 default value
#define SESSION_DEFAULT_REG_0x01               FIRMWARE_VERSION                                                        //!< session register 01 default value
#define SESSION_DEFAULT_REG_0x02               *LED_CONFIG_REGADDR                                                     //!< session register 02 default value
#define SESSION_DEFAULT_REG_0x03               0x0000                                                                  //!< session register 03 default value
#define SESSION_DEFAULT_REG_0x04               0x0000                                                                  //!< session register 04 default value
#define SESSION_DEFAULT_REG_0x05               0x0000                                                                  //!< session register 05 default value
#define SESSION_DEFAULT_REG_0x06               0x0000                                                                  //!< session register 06 default value
#define SESSION_DEFAULT_REG_0x07               0x0000                                                                  //!< session register 07 default value
#define SESSION_DEFAULT_REG_0x08               0x0000                                                                  //!< session register 08 default value
#define SESSION_DEFAULT_REG_0x09               0x0000                                                                  //!< session register 09 default value
#define SESSION_DEFAULT_REG_0x0A               0x0000                                                                  //!< session register 0A default value
#define SESSION_DEFAULT_REG_0x0B               0x0000                                                                  //!< session register 0B default value
#define SESSION_DEFAULT_REG_0x0C               0x0000                                                                  //!< session register 0C default value
#define SESSION_DEFAULT_REG_0x0D               0x0000                                                                  //!< session register 0D default value
#define SESSION_DEFAULT_REG_0x0E               0x0000                                                                  //!< session register 0E default value
#define SESSION_DEFAULT_REG_0x0F               0x0000                                                                  //!< session register 0F default value
#define SESSION_DEFAULT_REG_0x10               0x0000                                                                  //!< session register 10 default value
#define SESSION_DEFAULT_REG_0x11               0x0000                                                                  //!< session register 11 default value
#define SESSION_DEFAULT_REG_0x12               0x0000                                                                  //!< session register 12 default value
#define SESSION_DEFAULT_REG_0x13               0x0000                                                                  //!< session register 13 default value
#define SESSION_DEFAULT_REG_0x14               0x0000                                                                  //!< session register 14 default value
#define SESSION_DEFAULT_REG_0x15               0x0000                                                                  //!< session register 15 default value
#define SESSION_DEFAULT_REG_0x16               0x0000                                                                  //!< session register 16 default value
#define SESSION_DEFAULT_REG_0x17               0x0000                                                                  //!< session register 17 default value
#define SESSION_DEFAULT_REG_0x18               0x0000                                                                  //!< session register 18 default value
#define SESSION_DEFAULT_REG_0x19               0x0000                                                                  //!< session register 19 default value
#define SESSION_DEFAULT_REG_0x1A               0x0000                                                                  //!< session register 1A default value
#define SESSION_DEFAULT_REG_0x1B               0x0000                                                                  //!< session register 1B default value
#define SESSION_DEFAULT_REG_0x1C               0x0000                                                                  //!< session register 1C default value
#define SESSION_DEFAULT_REG_0x1D               0x0000                                                                  //!< session register 1D default value
#define SESSION_DEFAULT_REG_0x1E               *WeSU_IWDG_RELOAD_VALUE_REGADDR                                         //!< session register 1E default value
#define SESSION_DEFAULT_REG_0x1F               *WeSU_DEBUG_MASK_REGADDR                                                //!< session register 1F default value
#define SESSION_DEFAULT_REG_0x20               0x0000                                                                  //!< session register 20 default value
#define SESSION_DEFAULT_REG_0x21               *TIMER_FREQUENCY_REGADDR                                                //!< session register 21 default value
#define SESSION_DEFAULT_REG_0x22               0x0000                                                                  //!< session register 22 default value
#define SESSION_DEFAULT_REG_0x23               *HW_CAPABILITIES_REGADDR                                                //!< session register 23 default value
#define SESSION_DEFAULT_REG_0x24               *HW_FEATURE_0_MASK_REGADDR                                              //!< session register 24 default value
#define SESSION_DEFAULT_REG_0x25               *PWR_FEATURE_MASK_REGADDR                                               //!< session register 25 default value
#define SESSION_DEFAULT_REG_0x26               *TEMPERATURE_FEATURE_MASK_REGADDR                                       //!< session register 26 default value
#define SESSION_DEFAULT_REG_0x27               *HW_FEATURE_3_MASK_REGADDR                                              //!< session register 27 default value
#define SESSION_DEFAULT_REG_0x28               *PRESSURE_FEATURE_MASK_REGADDR                                          //!< session register 28 default value
#define SESSION_DEFAULT_REG_0x29               *HW_FEATURE_5_MASK_REGADDR                                              //!< session register 29 default value
#define SESSION_DEFAULT_REG_0x2A               *HW_FEATURE_6_MASK_REGADDR                                              //!< session register 2A default value
#define SESSION_DEFAULT_REG_0x2B               *RAWMOTION_FEATURE_7_MASK_REGADDR                                       //!< session register 2B default value
#define SESSION_DEFAULT_REG_0x2C               *HW_FEATURE_8_MASK_REGADDR                                              //!< session register 2C default value
#define SESSION_DEFAULT_REG_0x2D               *HW_FEATURE_9_MASK_REGADDR                                              //!< session register 2D default value
#define SESSION_DEFAULT_REG_0x2E               *HW_FEATURE_A_MASK_REGADDR                                              //!< session register 2E default value
#define SESSION_DEFAULT_REG_0x2F               *HW_FEATURE_B_MASK_REGADDR                                              //!< session register 2F default value
#define SESSION_DEFAULT_REG_0x30               *HW_FEATURE_C_MASK_REGADDR                                              //!< session register 30 default value
#define SESSION_DEFAULT_REG_0x31               *HW_FEATURE_D_MASK_REGADDR                                              //!< session register 31 default value
#define SESSION_DEFAULT_REG_0x32               *HW_FEATURE_E_MASK_REGADDR                                              //!< session register 32 default value
#define SESSION_DEFAULT_REG_0x33               *HW_FEATURE_F_MASK_REGADDR                                              //!< session register 33 default value
#define SESSION_DEFAULT_REG_0x34               *FW_CAPABILITIES_REGADDR                                                //!< session register 34 default value
#define SESSION_DEFAULT_REG_0x35               *PEDOMETER_CONTROL_MASK_REGADDR                                         //!< session register 35 default value
#define SESSION_DEFAULT_REG_0x36               *FW_FEATURE_1_MASK_REGADDR                                              //!< session register 36 default value
#define SESSION_DEFAULT_REG_0x37               *FW_FEATURE_2_MASK_REGADDR                                              //!< session register 37 default value
#define SESSION_DEFAULT_REG_0x38               *MOTION_CP_CONTROL_MASK_REGADDR                                         //!< session register 38 default value
#define SESSION_DEFAULT_REG_0x39               *MOTION_AR_CONTROL_MASK_REGADDR                                         //!< session register 39 default value
#define SESSION_DEFAULT_REG_0x3A               *FW_FEATURE_5_MASK_REGADDR                                              //!< session register 3A default value
#define SESSION_DEFAULT_REG_0x3B               *FW_FEATURE_6_MASK_REGADDR                                              //!< session register 3B default value
#define SESSION_DEFAULT_REG_0x3C               0x0000//*AHRS_CONTROL_MASK_REGADDR                                              //!< session register 3C default value
#define SESSION_DEFAULT_REG_0x3D               *MOTION_FX_CONTROL_MASK_REGADDR                                         //!< session register 3D default value
#define SESSION_DEFAULT_REG_0x3E               *FW_FEATURE_9_MASK_REGADDR                                              //!< session register 3E default value
#define SESSION_DEFAULT_REG_0x3F               *FREEFALL_CONTROL_MASK_REGADDR                                          //!< session register 3F default value
#define SESSION_DEFAULT_REG_0x40               *ACCEVENT_CONTROL_MASK_REGADDR                                          //!< session register 40 default value
#define SESSION_DEFAULT_REG_0x41               *FW_FEATURE_C_MASK_REGADDR                                              //!< session register 41 default value
#define SESSION_DEFAULT_REG_0x42               *FW_FEATURE_D_MASK_REGADDR                                              //!< session register 42 default value
#define SESSION_DEFAULT_REG_0x43               *FW_FEATURE_E_MASK_REGADDR                                              //!< session register 43 default value
#define SESSION_DEFAULT_REG_0x44               *FW_FEATURE_F_MASK_REGADDR                                              //!< session register 44 default value
#define SESSION_DEFAULT_REG_0x45               *BLE_DEBUG_CONFIG_REGADDR                                               //!< session register 45 default value
#define SESSION_DEFAULT_REG_0x46               *USART_DEBUG_CONFIG_REGADDR                                             //!< session register 46 default value
#define SESSION_DEFAULT_REG_0x47               *APP_DEBUG_CONFIG_REGADDR                                                //!< session register 47 default value
#define SESSION_DEFAULT_REG_0x48               *GP_ALGORITHM_FW_REGADDR                                                //!< session register 48 default value
#define SESSION_DEFAULT_REG_0x49               *HW_OPERATING_CAPABILITIES_REGADDR                                      //!< session register 49 default value
#define SESSION_DEFAULT_REG_0x4A               *FW_OPERATING_CAPABILITIES_REGADDR                                      //!< session register 4A default value
#define SESSION_DEFAULT_REG_0x4B               *BLE_CON_INTV_REGADDR                                                   //!< session register 4B default value
#define SESSION_DEFAULT_REG_0x4C               0x0000                                                                  //!< session register 4C default value
#define SESSION_DEFAULT_REG_0x4D               0x0000                                                                  //!< session register 4D default value
#define SESSION_DEFAULT_REG_0x4E               0x0000                                                                  //!< session register 4E default value
#define SESSION_DEFAULT_REG_0x4F               0x0000                                                                  //!< session register 4F default value
#define SESSION_DEFAULT_REG_0x50               0x0000                                                                  //!< session register 50 default value
#define SESSION_DEFAULT_REG_0x51               0x0000                                                                  //!< session register 51 default value
#define SESSION_DEFAULT_REG_0x52               0x0000                                                                  //!< session register 52 default value
#define SESSION_DEFAULT_REG_0x53               0x0000                                                                  //!< session register 53 default value
#define SESSION_DEFAULT_REG_0x54               0x0000                                                                  //!< session register 54 default value
#define SESSION_DEFAULT_REG_0x55               0x0000                                                                  //!< session register 55 default value
#define SESSION_DEFAULT_REG_0x56               0x0000                                                                  //!< session register 56 default value
#define SESSION_DEFAULT_REG_0x57               0x0000                                                                  //!< session register 57 default value
#define SESSION_DEFAULT_REG_0x58               0x0000                                                                  //!< session register 58 default value
#define SESSION_DEFAULT_REG_0x59               0x0000                                                                  //!< session register 59 default value
#define SESSION_DEFAULT_REG_0x5A               0x0000                                                                  //!< session register 5A default value
#define SESSION_DEFAULT_REG_0x5B               0x0000                                                                  //!< session register 5B default value
#define SESSION_DEFAULT_REG_0x5C               0x0000                                                                  //!< session register 5C default value
#define SESSION_DEFAULT_REG_0x5D               0x0000                                                                  //!< session register 5D default value
#define SESSION_DEFAULT_REG_0x5E               0x0000                                                                  //!< session register 5E default value
#define SESSION_DEFAULT_REG_0x5F               0x0000                                                                  //!< session register 5F default value
#define SESSION_DEFAULT_REG_0x60               0x0000                                                                  //!< session register 60 default value
#define SESSION_DEFAULT_REG_0x61               0x0000                                                                  //!< session register 61 default value
#define SESSION_DEFAULT_REG_0x62               0x0000                                                                  //!< session register 62 default value
#define SESSION_DEFAULT_REG_0x63               0x0000                                                                  //!< session register 63 default value
#define SESSION_DEFAULT_REG_0x64               0x0000                                                                  //!< session register 64 default value
#define SESSION_DEFAULT_REG_0x65               0x0000                                                                  //!< session register 65 default value
#define SESSION_DEFAULT_REG_0x66               0x0000                                                                  //!< session register 66 default value
#define SESSION_DEFAULT_REG_0x67               0x0000                                                                  //!< session register 67 default value
#define SESSION_DEFAULT_REG_0x68               0x0000                                                                  //!< session register 68 default value
#define SESSION_DEFAULT_REG_0x69               0x0000                                                                  //!< session register 69 default value
#define SESSION_DEFAULT_REG_0x6A               0x0000                                                                  //!< session register 6A default value
#define SESSION_DEFAULT_REG_0x6B               0x0000                                                                  //!< session register 6B default value
#define SESSION_DEFAULT_REG_0x6C               0x0000                                                                  //!< session register 6C default value
#define SESSION_DEFAULT_REG_0x6D               0x0000                                                                  //!< session register 6D default value
#define SESSION_DEFAULT_REG_0x6E               0x0000                                                                  //!< session register 6E default value
#define SESSION_DEFAULT_REG_0x6F               0x0000                                                                  //!< session register 6F default value
#define SESSION_DEFAULT_REG_0x70               0x0000                                                                  //!< session register 70 default value
#define SESSION_DEFAULT_REG_0x71               0x0000                                                                  //!< session register 71 default value
#define SESSION_DEFAULT_REG_0x72               0x0000                                                                  //!< session register 72 default value
#define SESSION_DEFAULT_REG_0x73               0x0000                                                                  //!< session register 73 default value
#define SESSION_DEFAULT_REG_0x74               0x0000                                                                  //!< session register 74 default value
#define SESSION_DEFAULT_REG_0x75               0x0000                                                                  //!< session register 75 default value
#define SESSION_DEFAULT_REG_0x76               0x0000                                                                  //!< session register 76 default value
#define SESSION_DEFAULT_REG_0x77               0x0000                                                                  //!< session register 77 default value
#define SESSION_DEFAULT_REG_0x78               0x0000                                                                  //!< session register 78 default value
#define SESSION_DEFAULT_REG_0x79               0x0000                                                                  //!< session register 79 default value
#define SESSION_DEFAULT_REG_0x7A               *ACCEVENT_CONF_REGADDR                                                 //!< session register 7A default value
#define SESSION_DEFAULT_REG_0x7B               0x0000                                                                  //!< session register 7B default value
#define SESSION_DEFAULT_REG_0x7C               0x0000                                                                  //!< session register 7C default value
#define SESSION_DEFAULT_REG_0x7D               0x0000                                                                  //!< session register 7D default value
#define SESSION_DEFAULT_REG_0x7E               0x0000                                                                  //!< session register 7E default value
#define SESSION_DEFAULT_REG_0x7F               0x0000                                                                  //!< session register 7F default value
#define SESSION_DEFAULT_REG_0x80               0x0000                                                                  //!< session register 80 default value
#define SESSION_DEFAULT_REG_0x81               0x0000                                                                  //!< session register 81 default value
#define SESSION_DEFAULT_REG_0x82               0x0000                                                                  //!< session register 82 default value
#define SESSION_DEFAULT_REG_0x83               0x0000                                                                  //!< session register 83 default value
#define SESSION_DEFAULT_REG_0x84               0x0000                                                                  //!< session register 84 default value
#define SESSION_DEFAULT_REG_0x85               0x0000                                                                  //!< session register 85 default value
#define SESSION_DEFAULT_REG_0x86               0x0000                                                                  //!< session register 86 default value
#define SESSION_DEFAULT_REG_0x87               0x0000                                                                  //!< session register 87 default value
#define SESSION_DEFAULT_REG_0x88               0x0000                                                                  //!< session register 88 default value
#define SESSION_DEFAULT_REG_0x89               0x0000                                                                  //!< session register 89 default value
#define SESSION_DEFAULT_REG_0x8A               0x0000                                                                  //!< session register 8A default value
#define SESSION_DEFAULT_REG_0x8B               0x0000                                                                  //!< session register 8B default value
#define SESSION_DEFAULT_REG_0x8C               0x0000                                                                  //!< session register 8C default value
#define SESSION_DEFAULT_REG_0x8D               0x0000                                                                  //!< session register 8D default value
#define SESSION_DEFAULT_REG_0x8E               0x0000                                                                  //!< session register 8E default value
#define SESSION_DEFAULT_REG_0x8F               0x0000                                                                  //!< session register 8F default value
#define SESSION_DEFAULT_REG_0x90               0x0000                                                                  //!< session register 90 default value
#define SESSION_DEFAULT_REG_0x91               0x0000                                                                  //!< session register 91 default value
#define SESSION_DEFAULT_REG_0x92               0x0000                                                                  //!< session register 92 default value
#define SESSION_DEFAULT_REG_0x93               0x0000                                                                  //!< session register 93 default value
#define SESSION_DEFAULT_REG_0x94               0x0000                                                                  //!< session register 94 default value
#define SESSION_DEFAULT_REG_0x95               0x0000                                                                  //!< session register 95 default value
#define SESSION_DEFAULT_REG_0x96               0x0000                                                                  //!< session register 96 default value
#define SESSION_DEFAULT_REG_0x97               0x0000                                                                  //!< session register 97 default value
#define SESSION_DEFAULT_REG_0x98               0x0000                                                                  //!< session register 98 default value
#define SESSION_DEFAULT_REG_0x99               0x0000                                                                  //!< session register 99 default value
#define SESSION_DEFAULT_REG_0x9A               0x0000                                                                  //!< session register 9A default value
#define SESSION_DEFAULT_REG_0x9B               0x0000                                                                  //!< session register 9B default value
#define SESSION_DEFAULT_REG_0x9C               0x0000                                                                  //!< session register 9C default value
#define SESSION_DEFAULT_REG_0x9D               0x0000                                                                  //!< session register 9D default value
#define SESSION_DEFAULT_REG_0x9E               0x0000                                                                  //!< session register 9E default value
#define SESSION_DEFAULT_REG_0x9F               0x0000                                                                  //!< session register 9F default value
#define SESSION_DEFAULT_REG_0xA0               0x0000                                                                  //!< session register A0 default value
#define SESSION_DEFAULT_REG_0xA1               0x0000                                                                  //!< session register A1 default value
#define SESSION_DEFAULT_REG_0xA2               0x0000                                                                  //!< session register A2 default value
#define SESSION_DEFAULT_REG_0xA3               0x0000                                                                  //!< session register A3 default value
#define SESSION_DEFAULT_REG_0xA4               0x0000                                                                  //!< session register A4 default value
#define SESSION_DEFAULT_REG_0xA5               0x0000                                                                  //!< session register A5 default value
#define SESSION_DEFAULT_REG_0xA6               0x0000                                                                  //!< session register A6 default value
#define SESSION_DEFAULT_REG_0xA7               0x0000                                                                  //!< session register A7 default value
#define SESSION_DEFAULT_REG_0xA8               0x0000                                                                  //!< session register A8 default value
#define SESSION_DEFAULT_REG_0xA9               0x0000                                                                  //!< session register A9 default value
#define SESSION_DEFAULT_REG_0xAA               0x0000                                                                  //!< session register AA default value
#define SESSION_DEFAULT_REG_0xAB               0x0000                                                                  //!< session register AB default value
#define SESSION_DEFAULT_REG_0xAC               0x0000                                                                  //!< session register AC default value
#define SESSION_DEFAULT_REG_0xAD               0x0000                                                                  //!< session register AD default value
#define SESSION_DEFAULT_REG_0xAE               0x0000                                                                  //!< session register AE default value
#define SESSION_DEFAULT_REG_0xAF               0x0000                                                                  //!< session register AF default value
#define SESSION_DEFAULT_REG_0xB0               0x0000                                                                  //!< session register B0 default value
#define SESSION_DEFAULT_REG_0xB1               0x0000                                                                  //!< session register B1 default value
#define SESSION_DEFAULT_REG_0xB2               0x0000                                                                  //!< session register B2 default value
#define SESSION_DEFAULT_REG_0xB3               0x0000                                                                  //!< session register B3 default value
#define SESSION_DEFAULT_REG_0xB4               0x0000                                                                  //!< session register B4 default value
#define SESSION_DEFAULT_REG_0xB5               0x0000                                                                  //!< session register B5 default value
#define SESSION_DEFAULT_REG_0xB6               0x0000                                                                  //!< session register B6 default value
#define SESSION_DEFAULT_REG_0xB7               0x0000                                                                  //!< session register B7 default value
#define SESSION_DEFAULT_REG_0xB8               0x0000                                                                  //!< session register B8 default value
#define SESSION_DEFAULT_REG_0xB9               0x0000                                                                  //!< session register B9 default value
#define SESSION_DEFAULT_REG_0xBA               0x0000                                                                  //!< session register BA default value
#define SESSION_DEFAULT_REG_0xBB               0x0000                                                                  //!< session register BB default value
#define SESSION_DEFAULT_REG_0xBC               0x0000                                                                  //!< session register BC default value
#define SESSION_DEFAULT_REG_0xBD               0x0000                                                                  //!< session register BD default value
#define SESSION_DEFAULT_REG_0xBE               0x0000                                                                  //!< session register BE default value
#define SESSION_DEFAULT_REG_0xBF               0x0000                                                                  //!< session register BF default value
#define SESSION_DEFAULT_REG_0xC0               0x0000                                                                  //!< session register C0 default value
#define SESSION_DEFAULT_REG_0xC1               0x0000                                                                  //!< session register C1 default value
#define SESSION_DEFAULT_REG_0xC2               0x0000                                                                  //!< session register C2 default value
#define SESSION_DEFAULT_REG_0xC3               0x0000                                                                  //!< session register C3 default value
#define SESSION_DEFAULT_REG_0xC4               0x0000                                                                  //!< session register C4 default value
#define SESSION_DEFAULT_REG_0xC5               0x0000                                                                  //!< session register C5 default value
#define SESSION_DEFAULT_REG_0xC6               0x0000                                                                  //!< session register C6 default value
#define SESSION_DEFAULT_REG_0xC7               0x0000                                                                  //!< session register C7 default value
#define SESSION_DEFAULT_REG_0xC8               0x0000                                                                  //!< session register C8 default value
#define SESSION_DEFAULT_REG_0xC9               0x0000                                                                  //!< session register C9 default value
#define SESSION_DEFAULT_REG_0xCA               0x0000                                                                  //!< session register CA default value
#define SESSION_DEFAULT_REG_0xCB               0x0000                                                                  //!< session register CB default value
#define SESSION_DEFAULT_REG_0xCC               0x0000                                                                  //!< session register CC default value
#define SESSION_DEFAULT_REG_0xCD               0x0000                                                                  //!< session register CD default value
#define SESSION_DEFAULT_REG_0xCE               0x0000                                                                  //!< session register CE default value
#define SESSION_DEFAULT_REG_0xCF               0x0000                                                                  //!< session register CF default value
#define SESSION_DEFAULT_REG_0xD0               0x0000                                                                  //!< session register D0 default value
#define SESSION_DEFAULT_REG_0xD1               0x0000                                                                  //!< session register D1 default value
#define SESSION_DEFAULT_REG_0xD2               0x0000                                                                  //!< session register D2 default value
#define SESSION_DEFAULT_REG_0xD3               0x0000                                                                  //!< session register D3 default value
#define SESSION_DEFAULT_REG_0xD4               0x0000                                                                  //!< session register D4 default value
#define SESSION_DEFAULT_REG_0xD5               0x0000                                                                  //!< session register D5 default value
#define SESSION_DEFAULT_REG_0xD6               0x0000                                                                  //!< session register D6 default value
#define SESSION_DEFAULT_REG_0xD7               0x0000                                                                  //!< session register D7 default value
#define SESSION_DEFAULT_REG_0xD8               0x0000                                                                  //!< session register D8 default value
#define SESSION_DEFAULT_REG_0xD9               0x0000                                                                  //!< session register D9 default value
#define SESSION_DEFAULT_REG_0xDA               0x0000                                                                  //!< session register DA default value
#define SESSION_DEFAULT_REG_0xDB               0x0000                                                                  //!< session register DB default value
#define SESSION_DEFAULT_REG_0xDC               0x0000                                                                  //!< session register DC default value
#define SESSION_DEFAULT_REG_0xDD               0x0000                                                                  //!< session register DD default value
#define SESSION_DEFAULT_REG_0xDE               0x0000                                                                  //!< session register DE default value
#define SESSION_DEFAULT_REG_0xDF               0x0000                                                                  //!< session register DF default value
#define SESSION_DEFAULT_REG_0xE0               0x0000                                                                  //!< session register E0 default value
#define SESSION_DEFAULT_REG_0xE1               0x0000                                                                  //!< session register E1 default value
#define SESSION_DEFAULT_REG_0xE2               0x0000                                                                  //!< session register E2 default value
#define SESSION_DEFAULT_REG_0xE3               0x0000                                                                  //!< session register E3 default value
#define SESSION_DEFAULT_REG_0xE4               0x0000                                                                  //!< session register E4 default value
#define SESSION_DEFAULT_REG_0xE5               0x0000                                                                  //!< session register E5 default value
#define SESSION_DEFAULT_REG_0xE6               0x0000                                                                  //!< session register E6 default value
#define SESSION_DEFAULT_REG_0xE7               0x0000                                                                  //!< session register E7 default value
#define SESSION_DEFAULT_REG_0xE8               0x0000                                                                  //!< session register E8 default value
#define SESSION_DEFAULT_REG_0xE9               0x0000                                                                  //!< session register E9 default value
#define SESSION_DEFAULT_REG_0xEA               0x0000                                                                  //!< session register EA default value
#define SESSION_DEFAULT_REG_0xEB               0x0000                                                                  //!< session register EB default value
#define SESSION_DEFAULT_REG_0xEC               0x0000                                                                  //!< session register EC default value
#define SESSION_DEFAULT_REG_0xED               0x0000                                                                  //!< session register ED default value
#define SESSION_DEFAULT_REG_0xEE               0x0000                                                                  //!< session register EE default value
#define SESSION_DEFAULT_REG_0xEF               0x0000                                                                  //!< session register EF default value
#define SESSION_DEFAULT_REG_0xF0               0x0000                                                                  //!< session register F0 default value
#define SESSION_DEFAULT_REG_0xF1               0x0000                                                                  //!< session register F1 default value
#define SESSION_DEFAULT_REG_0xF2               0x0000                                                                  //!< session register F2 default value
#define SESSION_DEFAULT_REG_0xF3               0x0000                                                                  //!< session register F3 default value
#define SESSION_DEFAULT_REG_0xF4               0x0000                                                                  //!< session register F4 default value
#define SESSION_DEFAULT_REG_0xF5               0x0000                                                                  //!< session register F5 default value
#define SESSION_DEFAULT_REG_0xF6               0x0000                                                                  //!< session register F6 default value
#define SESSION_DEFAULT_REG_0xF7               0x0000                                                                  //!< session register F7 default value
#define SESSION_DEFAULT_REG_0xF8               0x0000                                                                  //!< session register F8 default value
#define SESSION_DEFAULT_REG_0xF9               0x0000                                                                  //!< session register F9 default value
#define SESSION_DEFAULT_REG_0xFA               0x0000                                                                  //!< session register FA default value
#define SESSION_DEFAULT_REG_0xFB               0x0000                                                                  //!< session register FB default value
#define SESSION_DEFAULT_REG_0xFC               0x0000                                                                  //!< session register FC default value
#define SESSION_DEFAULT_REG_0xFD               0x0000                                                                  //!< session register FD default value
#define SESSION_DEFAULT_REG_0xFE               0x0000                                                                  //!< session register FE default value
#define SESSION_DEFAULT_REG_0xFF               0x0000                                                                  //!< session register FF default value


/**
  * @}
  */ 


/** @defgroup WeSU_LOW_LEVEL_Exported_Types WeSU_LOW_LEVEL_Exported_Types
  * @{
  */

/**
 * @brief Global Variables Structure: Session Copy Regs
 */
typedef struct _WesuBackupData_t
{
  uint8_t tm_hour;                      //!< Hours
  uint8_t tm_min;                       //!< Minutes
  uint8_t tm_sec;                       //!< Seconds
  uint8_t tm_day;                       //!< Day
  uint8_t tm_mon;                       //!< Month
  uint8_t tm_year;                      //!< Year
  uint8_t tm_wday;                      //!< Weekday
  uint8_t tm_rtcConf;                   //!< RtcConfiguration
  uint32_t nTicks;                      //!< System ticks
  uint32_t nSteps;                      //!< Step counter
  float fBatteryVolt;                   //!< Battery voltage
}WesuBackupData_t, *pWesuBackupData_t;  //!< WESU Backup Data


/**
 * @brief Global Variables Structure: Session Copy Regs
 */
typedef struct _gVars
{
  /* Mandatory */
  uint16_t nVersion;                    //!< see @ref PERS_SESS_DESC_REG_0x00 Firmware version
  uint16_t nVersion1;                   //!< see @ref PERS_SESS_DESC_REG_0x01 Firmware version
  uint16_t nLedControlMask;             //!< see @ref PERS_SESS_DESC_REG_0x02 Led control mask
  uint16_t nChargepercent;              //!< see @ref PERS_SESS_DESC_REG_0x03 Battery charge percentage (0.1%)
  float fBatteryVolt;                   //!< see @ref PERS_SESS_DESC_REG_0x04 Battery Voltage (reg04->reg05)
  float fCurrentuA;                     //!< see @ref PERS_SESS_DESC_REG_0x06 Battery Current (uA)(in >0 /out <0) (reg06->reg07)
  uint8_t nPowerMode;                   //!< see @ref PERS_SESS_DESC_REG_0x08 LSB: Battery power (charging/discharging/battery/low battery) MSB: RFU
  uint8_t nRsvd0x08_MSB;                //!< see @ref PERS_SESS_DESC_REG_0x08
  uint16_t nRsvd0x09;                   //!< see @ref PERS_SESS_DESC_REG_0x09 RFU 
  uint16_t nRsvd0x0A;                   //!< see @ref PERS_SESS_DESC_REG_0x0A RFU 
  uint16_t nRsvd0x0B;                   //!< see @ref PERS_SESS_DESC_REG_0x0B RFU 
  uint16_t nRsvd0x0C;                   //!< see @ref PERS_SESS_DESC_REG_0x0C RFU 
  uint16_t nRsvd0x0D;                   //!< see @ref PERS_SESS_DESC_REG_0x0D RFU 
  uint16_t nRsvd0x0E;                   //!< see @ref PERS_SESS_DESC_REG_0x0E RFU 
  uint16_t nRsvd0x0F;                   //!< see @ref PERS_SESS_DESC_REG_0x0F RFU 
  uint16_t nRsvd0x10;                   //!< see @ref PERS_SESS_DESC_REG_0x10 RFU 
  uint16_t nRsvd0x11;                   //!< see @ref PERS_SESS_DESC_REG_0x11 RFU 
  uint16_t nRsvd0x12;                   //!< see @ref PERS_SESS_DESC_REG_0x12 RFU 
  uint16_t nRsvd0x13;                   //!< see @ref PERS_SESS_DESC_REG_0x13 RFU 
  uint16_t nRsvd0x14;                   //!< see @ref PERS_SESS_DESC_REG_0x14 RFU 
  uint16_t nRsvd0x15;                   //!< see @ref PERS_SESS_DESC_REG_0x15 RFU 
  uint16_t nRsvd0x16;                   //!< see @ref PERS_SESS_DESC_REG_0x16 RFU 
  uint16_t nRsvd0x17;                   //!< see @ref PERS_SESS_DESC_REG_0x17 RFU 
  uint16_t nRsvd0x18;                   //!< see @ref PERS_SESS_DESC_REG_0x18 RFU 
  uint16_t nRsvd0x19;                   //!< see @ref PERS_SESS_DESC_REG_0x19 RFU 
  uint16_t nRsvd0x1A;                   //!< see @ref PERS_SESS_DESC_REG_0x1A RFU 
  uint16_t nRsvd0x1B;                   //!< see @ref PERS_SESS_DESC_REG_0x1B RFU 
  uint16_t nRsvd0x1C;                   //!< see @ref PERS_SESS_DESC_REG_0x1C RFU 
  uint16_t nRsvd0x1D;                   //!< see @ref PERS_SESS_DESC_REG_0x1D RFU 
  uint16_t WeSU_IWDG_RELOAD_VALUE;      //!< see @ref PERS_SESS_DESC_REG_0x1E IWDG Reload value
  uint16_t WeSU_DEBUG_MASK;             //!< see @ref PERS_SESS_DESC_REG_0x1F WeSU DEBUG enable mask for dedicated DBG_PRINTF (terminal, algorithms, power management, bluenrg)
  
  /* Optional Generic */
  uint16_t nRsvd0x20;                   //!< see @ref PERS_SESS_DESC_REG_0x20 RFU
  uint16_t nTimerFrequency;             //!< see @ref PERS_SESS_DESC_REG_0x21 Timer frequency
  uint16_t nLPFunction;                 //!< see @ref PERS_SESS_DESC_REG_0x22 Application power mode (full run, low power, permanent stop)
  uint16_t nRsvd0x23;                   //!< see @ref PERS_SESS_DESC_REG_0x23 RFU
  uint16_t nRsvd0x24;                   //!< see @ref PERS_SESS_DESC_REG_0x24 Reserved for HW Features
  uint8_t nPwrCtrlMask;                 //!< see @ref PERS_SESS_DESC_REG_0x25 LSB Power control mask; MSB: Power control Subsampling
  uint8_t nPwrSubSampl;                 //!< see @ref PERS_SESS_DESC_REG_0x25
  uint8_t nTemperatureCtrlMask;         //!< see @ref PERS_SESS_DESC_REG_0x26 LSB Temperature sensor control: mask (enable/disable, output channel); MSB: Temperature sensor control: Subsampling
  uint8_t nTemperatureSubSampl;         //!< see @ref PERS_SESS_DESC_REG_0x26
  uint16_t nRsvd0x27;                   //!< see @ref PERS_SESS_DESC_REG_0x27 Reserved for HW Features
  uint8_t nPressCtrlMask;               //!< see @ref PERS_SESS_DESC_REG_0x28 LSB Pressure sensor control: mask (enable/disable, output channel); MSB: Pressure sensor control: Subsampling
  uint8_t nPressSubSampl;               //!< see @ref PERS_SESS_DESC_REG_0x28
  uint16_t nRsvd0x29;                   //!< see @ref PERS_SESS_DESC_REG_0x29 Reserved for HW Features
  uint16_t nRsvd0x2A;                   //!< see @ref PERS_SESS_DESC_REG_0x2A Reserved for HW Features
  uint8_t nRawMotionCtrlMask;           //!< see @ref PERS_SESS_DESC_REG_0x2B LSB Raw motion control: mask ((enable/disable, output channel)); MSB: Raw motion SubSampling control
  uint8_t nRawMotionSubSampl;           //!< see @ref PERS_SESS_DESC_REG_0x2B
  uint16_t nRsvd0x2C;                   //!< see @ref PERS_SESS_DESC_REG_0x2C Reserved for HW Features
  uint16_t nRsvd0x2D;                   //!< see @ref PERS_SESS_DESC_REG_0x2D Reserved for HW Features
  uint16_t nRsvd0x2E;                   //!< see @ref PERS_SESS_DESC_REG_0x2E Reserved for HW Features
  uint16_t nRsvd0x2F;                   //!< see @ref PERS_SESS_DESC_REG_0x2F Reserved for HW Features
  uint16_t nRsvd0x30;                   //!< see @ref PERS_SESS_DESC_REG_0x30 Reserved for HW Features
  uint16_t nRsvd0x31;                   //!< see @ref PERS_SESS_DESC_REG_0x31 Reserved for HW Features
  uint16_t nRsvd0x32;                   //!< see @ref PERS_SESS_DESC_REG_0x32 Reserved for HW Features
  uint16_t nRsvd0x33;                   //!< see @ref PERS_SESS_DESC_REG_0x33 Reserved for HW Features
  uint16_t nRsvd0x34;                   //!< see @ref PERS_SESS_DESC_REG_0x34 RFU
  uint8_t nPedometerCtrlMask;           //!< see @ref PERS_SESS_DESC_REG_0x35 LSB Pedometer control: mask (enable/disable, output channel); MSB:  Pedometer control: Subsampling
  uint8_t nPedometerSubSampl;           //!< see @ref PERS_SESS_DESC_REG_0x35 
  uint16_t nRsvd0x36;                   //!< see @ref PERS_SESS_DESC_REG_0x36 Reserved for SW Features
  uint16_t nRsvd0x37;                   //!< see @ref PERS_SESS_DESC_REG_0x37 Reserved for SW Features
  uint8_t nMotionCpCtrlMask;            //!< see @ref PERS_SESS_DESC_REG_0x38 LSB MotionCp control: mask (enable/disable, output channel); MSB:  MotionCp control: Subsampling
  uint8_t nMotionCpSubSampl;            //!< see @ref PERS_SESS_DESC_REG_0x38 
  uint8_t nMotionArCtrlMask;            //!< see @ref PERS_SESS_DESC_REG_0x39 LSB MotionAr control: mask (enable/disable, output channel); MSB:  MotionAr control: Subsampling
  uint8_t nMotionArSubSampl;            //!< see @ref PERS_SESS_DESC_REG_0x39 
  uint16_t nRsvd0x3A;                   //!< see @ref PERS_SESS_DESC_REG_0x3A Reserved for SW Features
  uint16_t nRsvd0x3B;                   //!< see @ref PERS_SESS_DESC_REG_0x3B Reserved for SW Features
  uint16_t nRsvd0x3C;                   //!< see @ref PERS_SESS_DESC_REG_0x3D Reserved for SW Features 
  uint8_t nMotionFxCtrlMask;            //!< see @ref PERS_SESS_DESC_REG_0x3D LSB FX control: mask (enable/disable, output channel); MSB:  FX control: Subsampling
  uint8_t nMotionFxSubSampl;            //!< see @ref PERS_SESS_DESC_REG_0x3D 
  uint16_t nRsvd0x3E;                   //!< see @ref PERS_SESS_DESC_REG_0x3E Reserved for SW Features
  uint16_t nRsvd0x3F;                   //!< see @ref PERS_SESS_DESC_REG_0x3F Reserved for SW Features
  uint8_t nAccEventCtrlMask;            //!< see @ref PERS_SESS_DESC_REG_0x40 LSB: AccEvent control: mask (enable/disable, output channel); 
  uint8_t nAccEventSubSampl;            //!< see @ref PERS_SESS_DESC_REG_0x40 MSB: AccEvent control: Subsampling
  uint16_t nRsvd0x41;                   //!< see @ref PERS_SESS_DESC_REG_0x41 Reserved for SW Features
  uint16_t nRsvd0x42;                   //!< see @ref PERS_SESS_DESC_REG_0x42 Reserved for SW Features
  uint16_t nRsvd0x43;                   //!< see @ref PERS_SESS_DESC_REG_0x43 Reserved for SW Features
  uint16_t nRsvd0x44;                   //!< see @ref PERS_SESS_DESC_REG_0x44 Reserved for SW Features
  uint16_t WeSU_DEBUG_SEVERITY_ble;     //!< see @ref PERS_SESS_DESC_REG_0x45 Configured severity level for BLE
  uint16_t WeSU_DEBUG_SEVERITY_usart;   //!< see @ref PERS_SESS_DESC_REG_0x46 Configured severity level for USART
  uint16_t WeSU_APP_DEFAULT_SEVERITY;   //!< see @ref PERS_SESS_DESC_REG_0x47 WeSU application messages default severity
  uint8_t nGP_ALGORITHMCtrlMask;        //!< see @ref PERS_SESS_DESC_REG_0x48 LSB GP_ALGORITHMS control: mask (enable/disable, output channel); MSB:  GP_ALGORITHM control: Subsampling
  uint8_t nGP_ALGORITHMSubSampl;        //!< see @ref PERS_SESS_DESC_REG_0x48 
  uint16_t nReadSensorMask;             //!< see @ref PERS_SESS_DESC_REG_0x49 Read sensors mask
  uint16_t nProcessFwMask;              //!< see @ref PERS_SESS_DESC_REG_0x4A Process algorithms mask 
  uint16_t nBleConIntv;                 //!< see @ref PERS_SESS_DESC_REG_0x4B BlueNRG connection interval
  uint16_t xBNRG_HW_ver;                //!< see @ref PERS_SESS_DESC_REG_0x4C BlueNRG HW version
  uint16_t xBNRG_FW_ver;                //!< see @ref PERS_SESS_DESC_REG_0x4D BlueNRG FW version
  uint16_t nRsvd0x4E;                   //!< see @ref PERS_SESS_DESC_REG_0x4E RFU
  uint16_t nRsvd0x4F;                   //!< see @ref PERS_SESS_DESC_REG_0x4F RFU

  /* Optional Specific */
  uint16_t nRedLedControlMask;          //!< see @ref PERS_SESS_DESC_REG_0x50 Red Led control mask
  uint16_t nLedBlinkMode;               //!< see @ref PERS_SESS_DESC_REG_0x51 Led mode control
  uint32_t nLsiFreq;                    //!< see @ref PERS_SESS_DESC_REG_0x52  Low speed oscillator frequency
  uint32_t nTicks;                      //!< see @ref PERS_SESS_DESC_REG_0x54  Number of seconds after system boot (reg54->reg55)
  uint32_t nReadPower;                  //!< see @ref PERS_SESS_DESC_REG_0x56  number of power read (reg56->reg57)
  uint32_t nWakeSource;                 //!< see @ref PERS_SESS_DESC_REG_0x58  MCU wakeup source (reg58->reg59)
  uint32_t nLPMask;                     //!< see @ref PERS_SESS_DESC_REG_0x5A  MCU operations before low power mode (reg5A->reg5B)
  uint8_t bLP;                          //!< see @ref PERS_SESS_DESC_REG_0x5C  LSB MCU low power mode, MSB RFU
  uint8_t nRsvd0x5C_MSB;                //!< see @ref PERS_SESS_DESC_REG_0x5C
  uint16_t nTimerDrdyCtrlMask;          //!< see @ref PERS_SESS_DESC_REG_0x5D  Application Timer control mask: mcu timer or Accelerometer data-ready 
  uint16_t nRsvd0x5E;                   //!< see @ref PERS_SESS_DESC_REG_0x5E  RFU
  uint16_t nErrorsBlueNRGFX;            //!< see @ref PERS_SESS_DESC_REG_0x5F  BlueNRG error monitoring on Motion FX characteristic
  uint16_t nErrorsBlueNRGRwMot;         //!< see @ref PERS_SESS_DESC_REG_0x60  BlueNRG error monitoring on Motion characteristic
  uint16_t nErrorsBlueNRGPrs;           //!< see @ref PERS_SESS_DESC_REG_0x61  BlueNRG error monitoring on pressure characteristic
  uint16_t nErrorsBlueNRGPwr;           //!< see @ref PERS_SESS_DESC_REG_0x62  BlueNRG error monitoring on power characteristic
  uint16_t calibrationCounter;          //!< see @ref PERS_SESS_DESC_REG_0x63  Counter to manage magnetometer calibration
  int32_t pressure;                     //!< see @ref PERS_SESS_DESC_REG_0x64  Pressure output            (mbar) (reg64->reg65)
  AxesRawFloat_TypeDef acceleration;    //!< see @ref PERS_SESS_DESC_REG_0x66  Accelerometer output       (XXXXXX) (reg66->reg6B)
  AxesRawFloat_TypeDef angular_rate;    //!< see @ref PERS_SESS_DESC_REG_0x6C  Gyroscope output           (XXXXXX) (reg6C->reg71)
  AxesRawFloat_TypeDef magnetic_field;  //!< see @ref PERS_SESS_DESC_REG_0x72  Magnetometer output        (XXXXXX) (reg72->reg77)
  int32_t temperature;                  //!< see @ref PERS_SESS_DESC_REG_0x78  Temperature                (XXXXXX) (reg78->reg79)
  uint16_t nAccEventConf;               //!< see @ref PERS_SESS_DESC_REG_0x7A  Accelerometer Event Config
  uint8_t nAccEventStatus;              //!< see @ref PERS_SESS_DESC_REG_0x7B  LSB Accelerometer Event status, MSB GP Algorithms status
  uint8_t nGP_ALGORITHMStatus;          //!< see @ref PERS_SESS_DESC_REG_0x7B  
  uint32_t nPedoStepCounter;            //!< see @ref PERS_SESS_DESC_REG_0x7C  Step counter, output of pedometer algorithm (reg7C->reg7D)
  uint16_t nRsvd0x7E;                   //!< see @ref PERS_SESS_DESC_REG_0x7E  RFU
  uint16_t nRsvd0x7F;                   //!< see @ref PERS_SESS_DESC_REG_0x7F  RFU
  uint16_t nRsvd0x80;                   //!< see @ref PERS_SESS_DESC_REG_0x80  RFU
  uint16_t nRsvd0x81;                   //!< see @ref PERS_SESS_DESC_REG_0x81  RFU
  uint16_t nRsvd0x82;                   //!< see @ref PERS_SESS_DESC_REG_0x82  RFU
  uint16_t nRsvd0x83;                   //!< see @ref PERS_SESS_DESC_REG_0x83  RFU
  uint16_t nRsvd0x84;                   //!< see @ref PERS_SESS_DESC_REG_0x84  RFU
  uint16_t nRsvd0x85;                   //!< see @ref PERS_SESS_DESC_REG_0x85  RFU
  uint16_t nRsvd0x86;                   //!< see @ref PERS_SESS_DESC_REG_0x86  RFU
  uint16_t nRsvd0x87;                   //!< see @ref PERS_SESS_DESC_REG_0x87  RFU
  uint16_t nRsvd0x88;                   //!< see @ref PERS_SESS_DESC_REG_0x88  RFU
  uint16_t nRsvd0x89;                   //!< see @ref PERS_SESS_DESC_REG_0x89  RFU
  uint16_t nRsvd0x8A;                   //!< see @ref PERS_SESS_DESC_REG_0x8A  RFU
  uint16_t nRsvd0x8B;                   //!< see @ref PERS_SESS_DESC_REG_0x8B  RFU
  uint8_t xFXCalib_Status;              //!< see @ref PERS_SESS_DESC_REG_0x8C  LSB FX Calibration status, MSB FX license status
  uint8_t xMFX_Status;                  //!< see @ref PERS_SESS_DESC_REG_0x8C  
  uint8_t xMAR_Value;                   //!< see @ref PERS_SESS_DESC_REG_0x8D  LSB Motion Activity Recognition Value, MSB Motion Activity Recognition Status
  uint8_t xMAR_Status;                  //!< see @ref PERS_SESS_DESC_REG_0x8D
  uint8_t xMCP_Value;                   //!< see @ref PERS_SESS_DESC_REG_0x8E  LSB Motion Carry Position Value, MSB Motion Carry Position Status
  uint8_t xMCP_Status;                  //!< see @ref PERS_SESS_DESC_REG_0x8E
  uint16_t nRsvd0x8F;                   //!< see @ref PERS_SESS_DESC_REG_0x8F  RFU
  uint8_t tm_hour;                      //!< see @ref PERS_SESS_DESC_REG_0x90  LSB Hours, MSB Min
  uint8_t tm_min;                       //!< see @ref PERS_SESS_DESC_REG_0x90  
  uint8_t tm_sec;                       //!< see @ref PERS_SESS_DESC_REG_0x91  LSB Sec, MSB Day
  uint8_t tm_day;                       //!< see @ref PERS_SESS_DESC_REG_0x91  
  uint8_t tm_mon;                       //!< see @ref PERS_SESS_DESC_REG_0x92  LSB Month, MSB Year
  uint8_t tm_year;                      //!< see @ref PERS_SESS_DESC_REG_0x92  
  uint8_t tm_wday;                      //!< see @ref PERS_SESS_DESC_REG_0x93  LSB WDay, MSB RtcConfiguration
  uint8_t tm_rtcConf;                   //!< see @ref PERS_SESS_DESC_REG_0x93  
  uint16_t LowBatteryStatus;            //!< see @ref PERS_SESS_DESC_REG_0x94  RFU
  uint16_t UsbCableCondition;           //!< see @ref PERS_SESS_DESC_REG_0x95  RFU
  uint16_t bRebootSensors;              //!< see @ref PERS_SESS_DESC_REG_0x96  Bitmask to reboot sensors at system startup
  uint8_t bSetConnectable;              //!< see @ref PERS_SESS_DESC_REG_0x97  LSB BlueNRG flag for discoverable mode, BlueNRG error flag; MSB  Bluenrg error flag, RFU
  uint8_t bBlueNRGErrorFlag;            //!< see @ref PERS_SESS_DESC_REG_0x97
  uint32_t nUserProcessCount;           //!< see @ref PERS_SESS_DESC_REG_0x98  Processing count: number of calls to "User_Process()" function
  uint16_t nRsvd0x9A_EF[0xEF-0x9A+1];   //!< see @ref PERS_SESS_DESC_REG_0x9A -> 0xEF  RFU
  uint16_t nRsvdDummyRegStc3115;        //!< see @ref PERS_SESS_DESC_REG_0xF0  dummy registers, cannot be accessed by Communication Protocol register management
  uint16_t nRsvdDummyRegDebug;          //!< see @ref PERS_SESS_DESC_REG_0xF1  dummy registers, cannot be accessed by the app
  uint16_t nRsvdDummyRegsStdby;         //!< see @ref PERS_SESS_DESC_REG_0xF2
  uint16_t nRsvdDummyRegSystemFlag;     //!< see @ref PERS_SESS_DESC_REG_0xF3
  uint16_t nMfxInitState;               //!< see @ref PERS_SESS_DESC_REG_0xF4
  uint16_t nMarInitState;               //!< see @ref PERS_SESS_DESC_REG_0xF5
  uint16_t nMcpInitState;               //!< see @ref PERS_SESS_DESC_REG_0xF6
  uint16_t nRsvdDummyRegs0xF7;          //!< see @ref PERS_SESS_DESC_REG_0xF7
  uint16_t nRsvdDummyRegs0xF8;          //!< see @ref PERS_SESS_DESC_REG_0xF8
  uint16_t nRsvdDummyRegs0xF9;          //!< see @ref PERS_SESS_DESC_REG_0xF9
  uint16_t nRsvdDummyRegs0xFA;          //!< see @ref PERS_SESS_DESC_REG_0xFA
  uint16_t nRsvdDummyRegs0xFB;          //!< see @ref PERS_SESS_DESC_REG_0xFB
  uint16_t nRsvdDummyRegs0xFC;          //!< see @ref PERS_SESS_DESC_REG_0xFC
  uint16_t nRsvdDummyRegs0xFD;          //!< see @ref PERS_SESS_DESC_REG_0xFD
  uint16_t nRsvdDummyRegs0xFE;          //!< see @ref PERS_SESS_DESC_REG_0xFE
  uint16_t nRsvdDummyRegs0xFF;          //!< see @ref PERS_SESS_DESC_REG_0xFF

}GlobalVarsStruct_t, *pGlobalVarsStruct_t;      //!< Session Copy Regs STRUCT

/**
  * @}
  */ 

/** @defgroup WeSU_LOW_LEVEL_Exported_Variables WeSU_LOW_LEVEL_Exported_Variables
  * @{
  */

  extern pGlobalVarsStruct_t GlobalSessionStruct;         //!< Pointer to Session Registers structure

/**
  * @}
  */ 


/* Persistent registers */
/* Mandatory */

#define FW_VERSION_0_REG                                0x00                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x00
#define FW_VERSION_0_REG_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FW_VERSION_0_REG))                                  //!< Memory address for @ref PERS_SESS_DESC_REG_0x00 
#define FW_VERSION_0_REG_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(FW_VERSION_0_REG))                       //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x00 

#define FW_VERSION_1_REG                                0x01                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x01
#define FW_VERSION_1_REG_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FW_VERSION_1_REG))                                  //!< Memory address for @ref PERS_SESS_DESC_REG_0x01 
#define FW_VERSION_1_REG_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(FW_VERSION_1_REG))                       //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x01

#define LED_CONFIG_REG                                  0x02                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x02
#define LED_CONFIG_REGADDR                              (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(LED_CONFIG_REG))                                    //!< Memory address for @ref PERS_SESS_DESC_REG_0x02
#define LED_CONFIG_REGADDR_DEFAULT                      (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(LED_CONFIG_REG))                         //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x02

#define BLE_LOC_NAME_REG                                0x03                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x03
#define BLE_LOC_NAME_REGADDR                            (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(BLE_LOC_NAME_REG))                                  //!< Memory address for @ref PERS_SESS_DESC_REG_0x03
#define BLE_LOC_NAME_REGADDR_DEFAULT                    (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(BLE_LOC_NAME_REG))                       //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x03

#define BLE_PUB_ADDR_REG                                0x0B                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x0B
#define BLE_PUB_ADDR_REGADDR                            (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(BLE_PUB_ADDR_REG))                                //!< Memory address for @ref PERS_SESS_DESC_REG_0x0B
#define BLE_PUB_ADDR_REGADDR_DEFAULT                    (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(BLE_PUB_ADDR_REG))                     //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x0B     

#define WeSU_IWDG_RELOAD_VALUE_REG                      0x1E                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x1E
#define WeSU_IWDG_RELOAD_VALUE_REGADDR                  (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(WeSU_IWDG_RELOAD_VALUE_REG))                      //!< Memory address for @ref PERS_SESS_DESC_REG_0x1E
#define WeSU_IWDG_RELOAD_VALUE_REGADDR_DEFAULT          (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(WeSU_IWDG_RELOAD_VALUE_REG))           //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x1E

#define WeSU_DEBUG_MASK_REG                             0x1F                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x1F
#define WeSU_DEBUG_MASK_REGADDR                         (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(WeSU_DEBUG_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x1F
#define WeSU_DEBUG_MASK_REGADDR_DEFAULT                 (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(WeSU_DEBUG_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x1F

/* Optional (generic) */

#define TX_PWR_LVL_CONTROL_REG                          0x20                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x20
#define TX_PWR_LVL_CONTROL_REGADDR                      (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(TX_PWR_LVL_CONTROL_REG))                            //!< Memory address for @ref PERS_SESS_DESC_REG_0x20
#define TX_PWR_LVL_CONTROL_REGADDR_DEFAULT              (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(TX_PWR_LVL_CONTROL_REG))                 //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x20

#define TIMER_FREQUENCY_REG                             0x21                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x21
#define TIMER_FREQUENCY_REGADDR                         (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(TIMER_FREQUENCY_REG))                               //!< Memory address for @ref PERS_SESS_DESC_REG_0x21
#define TIMER_FREQUENCY_REGADDR_DEFAULT                 (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(TIMER_FREQUENCY_REG))                    //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x21

#define PWR_MODE_CONTROL_REG                            0x22                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x22
#define PWR_MODE_CONTROL_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PWR_MODE_CONTROL_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0x22
#define PWR_MODE_CONTROL_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PWR_MODE_CONTROL_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x22

#define HW_CAPABILITIES_REG                             0x23                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x23
#define HW_CAPABILITIES_REGADDR                         (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(HW_CAPABILITIES_REG))                               //!< Memory address for @ref PERS_SESS_DESC_REG_0x23
#define HW_CAPABILITIES_REGADDR_DEFAULT                 (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(HW_CAPABILITIES_REG))                    //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x23

#define HW_FEATURE_0_MASK_REG                           0x24                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x24
#define HW_FEATURE_0_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(HW_FEATURE_0_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x24
#define HW_FEATURE_0_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(HW_FEATURE_0_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x24

#define PWR_FEATURE_MASK_REG                           0x25                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x25
#define PWR_FEATURE_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PWR_FEATURE_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x25
#define PWR_FEATURE_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PWR_FEATURE_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x25

#define TEMPERATURE_FEATURE_MASK_REG                    0x26                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x26
#define TEMPERATURE_FEATURE_MASK_REGADDR                (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(TEMPERATURE_FEATURE_MASK_REG))                    //!< Memory address for @ref PERS_SESS_DESC_REG_0x26
#define TEMPERATURE_FEATURE_MASK_REGADDR_DEFAULT        (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(TEMPERATURE_FEATURE_MASK_REG))         //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x26

#define HW_FEATURE_3_MASK_REG                           0x27                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x27
#define HW_FEATURE_3_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(HW_FEATURE_3_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x27
#define HW_FEATURE_3_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(HW_FEATURE_3_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x27
  
#define PRESSURE_FEATURE_MASK_REG                       0x28                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x28
#define PRESSURE_FEATURE_MASK_REGADDR                   (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PRESSURE_FEATURE_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x28
#define PRESSURE_FEATURE_MASK_REGADDR_DEFAULT           (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PRESSURE_FEATURE_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x28
  
#define HW_FEATURE_5_MASK_REG                           0x29                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x29
#define HW_FEATURE_5_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(HW_FEATURE_5_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x29
#define HW_FEATURE_5_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(HW_FEATURE_5_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x29
  
#define HW_FEATURE_6_MASK_REG                           0x2A                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x2A
#define HW_FEATURE_6_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(HW_FEATURE_6_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x2A
#define HW_FEATURE_6_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(HW_FEATURE_6_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x2A
  
#define RAWMOTION_FEATURE_7_MASK_REG                    0x2B                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x2B
#define RAWMOTION_FEATURE_7_MASK_REGADDR                (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(RAWMOTION_FEATURE_7_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x2B
#define RAWMOTION_FEATURE_7_MASK_REGADDR_DEFAULT        (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(RAWMOTION_FEATURE_7_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x2B
  
#define HW_FEATURE_8_MASK_REG                           0x2C                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x2C
#define HW_FEATURE_8_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(HW_FEATURE_8_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x2C
#define HW_FEATURE_8_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(HW_FEATURE_8_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x2C
  
#define HW_FEATURE_9_MASK_REG                           0x2D                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x2D
#define HW_FEATURE_9_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(HW_FEATURE_9_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x2D
#define HW_FEATURE_9_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(HW_FEATURE_9_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x2D
  
#define HW_FEATURE_A_MASK_REG                           0x2E                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x2E
#define HW_FEATURE_A_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(HW_FEATURE_A_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x2E
#define HW_FEATURE_A_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(HW_FEATURE_A_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x2E
  
#define HW_FEATURE_B_MASK_REG                           0x2F                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x2F
#define HW_FEATURE_B_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(HW_FEATURE_B_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x2F
#define HW_FEATURE_B_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(HW_FEATURE_B_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x2F
  
#define HW_FEATURE_C_MASK_REG                           0x30                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x30
#define HW_FEATURE_C_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(HW_FEATURE_C_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x30
#define HW_FEATURE_C_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(HW_FEATURE_C_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x30
  
#define HW_FEATURE_D_MASK_REG                           0x31                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x31
#define HW_FEATURE_D_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(HW_FEATURE_D_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x31
#define HW_FEATURE_D_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(HW_FEATURE_D_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x31
  
#define HW_FEATURE_E_MASK_REG                           0x32                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x32
#define HW_FEATURE_E_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(HW_FEATURE_E_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x32
#define HW_FEATURE_E_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(HW_FEATURE_E_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x32
  
#define HW_FEATURE_F_MASK_REG                           0x33                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x33
#define HW_FEATURE_F_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(HW_FEATURE_F_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x33
#define HW_FEATURE_F_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(HW_FEATURE_F_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x33
  
#define FW_CAPABILITIES_REG                             0x34                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x34
#define FW_CAPABILITIES_REGADDR                         (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FW_CAPABILITIES_REG))                               //!< Memory address for @ref PERS_SESS_DESC_REG_0x34
#define FW_CAPABILITIES_REGADDR_DEFAULT                 (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(FW_CAPABILITIES_REG))                    //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x34

#define PEDOMETER_CONTROL_MASK_REG                      0x35                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x35
#define PEDOMETER_CONTROL_MASK_REGADDR                  (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDOMETER_CONTROL_MASK_REG))                        //!< Memory address for @ref PERS_SESS_DESC_REG_0x35
#define PEDOMETER_CONTROL_MASK_REGADDR_DEFAULT          (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDOMETER_CONTROL_MASK_REG))             //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x35

#define FW_FEATURE_1_MASK_REG                           0x36                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x36
#define FW_FEATURE_1_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FW_FEATURE_1_MASK_REG))                           //!< Memory address for @ref PERS_SESS_DESC_REG_0x36
#define FW_FEATURE_1_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(FW_FEATURE_1_MASK_REG))                //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x36

#define FW_FEATURE_2_MASK_REG                           0x37                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x37
#define FW_FEATURE_2_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FW_FEATURE_2_MASK_REG))                           //!< Memory address for @ref PERS_SESS_DESC_REG_0x37
#define FW_FEATURE_2_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(FW_FEATURE_2_MASK_REG))                //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x37

#define MOTION_CP_CONTROL_MASK_REG                       0x38                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x38
#define MOTION_CP_CONTROL_MASK_REGADDR                   (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(MOTION_CP_CONTROL_MASK_REG))                         //!< Memory address for @ref PERS_SESS_DESC_REG_0x38
#define MOTION_CP_CONTROL_MASK_REGADDR_DEFAULT           (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(MOTION_CP_CONTROL_MASK_REG))              //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x38

#define MOTION_AR_CONTROL_MASK_REG                       0x39                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x39
#define MOTION_AR_CONTROL_MASK_REGADDR                   (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(MOTION_AR_CONTROL_MASK_REG))                         //!< Memory address for @ref PERS_SESS_DESC_REG_0x39
#define MOTION_AR_CONTROL_MASK_REGADDR_DEFAULT           (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(MOTION_AR_CONTROL_MASK_REG))              //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x39

#define FW_FEATURE_5_MASK_REG                           0x3A                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x3A
#define FW_FEATURE_5_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FW_FEATURE_5_MASK_REG))                           //!< Memory address for @ref PERS_SESS_DESC_REG_0x3A
#define FW_FEATURE_5_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(FW_FEATURE_5_MASK_REG))                //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x3A

#define FW_FEATURE_6_MASK_REG                           0x3B                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x3B
#define FW_FEATURE_6_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FW_FEATURE_6_MASK_REG))                           //!< Memory address for @ref PERS_SESS_DESC_REG_0x3B
#define FW_FEATURE_6_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(FW_FEATURE_6_MASK_REG))                //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x3B

#define AHRS_CONTROL_MASK_REG                           0x3C                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x3C
#define AHRS_CONTROL_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(AHRS_CONTROL_MASK_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x3C
#define AHRS_CONTROL_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(AHRS_CONTROL_MASK_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x3C

#define MOTION_FX_CONTROL_MASK_REG                      0x3D                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x3D
#define MOTION_FX_CONTROL_MASK_REGADDR                  (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(MOTION_FX_CONTROL_MASK_REG))                      //!< Memory address for @ref PERS_SESS_DESC_REG_0x3D
#define MOTION_FX_CONTROL_MASK_REGADDR_DEFAULT          (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(MOTION_FX_CONTROL_MASK_REG))           //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x3D
  
#define FW_FEATURE_9_MASK_REG                           0x3E                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x3E
#define FW_FEATURE_9_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FW_FEATURE_9_MASK_REG))                           //!< Memory address for @ref PERS_SESS_DESC_REG_0x3E
#define FW_FEATURE_9_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(FW_FEATURE_9_MASK_REG))                //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x3E

#define FREEFALL_CONTROL_MASK_REG                       0x3F                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x3F
#define FREEFALL_CONTROL_MASK_REGADDR                   (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FREEFALL_CONTROL_MASK_REG))                         //!< Memory address for @ref PERS_SESS_DESC_REG_0x3F
#define FREEFALL_CONTROL_MASK_REGADDR_DEFAULT           (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(FREEFALL_CONTROL_MASK_REG))              //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x3F
  
#define ACCEVENT_CONTROL_MASK_REG                       0x40                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x40
#define ACCEVENT_CONTROL_MASK_REGADDR                   (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(ACCEVENT_CONTROL_MASK_REG))                           //!< Memory address for @ref PERS_SESS_DESC_REG_0x40
#define ACCEVENT_CONTROL_MASK_REGADDR_DEFAULT           (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(ACCEVENT_CONTROL_MASK_REG))                //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x40

#define FW_FEATURE_C_MASK_REG                           0x41                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x41
#define FW_FEATURE_C_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FW_FEATURE_C_MASK_REG))                           //!< Memory address for @ref PERS_SESS_DESC_REG_0x41
#define FW_FEATURE_C_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(FW_FEATURE_C_MASK_REG))                //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x41

#define FW_FEATURE_D_MASK_REG                           0x42                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x42
#define FW_FEATURE_D_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FW_FEATURE_D_MASK_REG))                           //!< Memory address for @ref PERS_SESS_DESC_REG_0x42
#define FW_FEATURE_D_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(FW_FEATURE_D_MASK_REG))                //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x42

#define FW_FEATURE_E_MASK_REG                           0x43                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x43
#define FW_FEATURE_E_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FW_FEATURE_E_MASK_REG))                           //!< Memory address for @ref PERS_SESS_DESC_REG_0x43
#define FW_FEATURE_E_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(FW_FEATURE_E_MASK_REG))                //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x43

#define FW_FEATURE_F_MASK_REG                           0x44                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x44
#define FW_FEATURE_F_MASK_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FW_FEATURE_F_MASK_REG))                           //!< Memory address for @ref PERS_SESS_DESC_REG_0x44
#define FW_FEATURE_F_MASK_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(FW_FEATURE_F_MASK_REG))                //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x44

#define BLE_DEBUG_CONFIG_REG                            0x45                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x45
#define BLE_DEBUG_CONFIG_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(BLE_DEBUG_CONFIG_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0x45
#define BLE_DEBUG_CONFIG_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(BLE_DEBUG_CONFIG_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x45

#define USART_DEBUG_CONFIG_REG                          0x46                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x46
#define USART_DEBUG_CONFIG_REGADDR                      (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(USART_DEBUG_CONFIG_REG))                            //!< Memory address for @ref PERS_SESS_DESC_REG_0x46
#define USART_DEBUG_CONFIG_REGADDR_DEFAULT              (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(USART_DEBUG_CONFIG_REG))                 //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x46

#define APP_DEBUG_CONFIG_REG                              0x47                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x47
#define APP_DEBUG_CONFIG_REGADDR                          (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(APP_DEBUG_CONFIG_REG))                                //!< Memory address for @ref PERS_SESS_DESC_REG_0x47
#define APP_DEBUG_CONFIG_REGADDR_DEFAULT                  (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(APP_DEBUG_CONFIG_REG))                     //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x47

#define GP_ALGORITHM_FW_REG                              0x48                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x48
#define GP_ALGORITHM_FW_REGADDR                          (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(GP_ALGORITHM_FW_REG))                                //!< Memory address for @ref PERS_SESS_DESC_REG_0x48
#define GP_ALGORITHM_FW_REGADDR_DEFAULT                  (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(GP_ALGORITHM_FW_REG))                     //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x48

#define HW_OPERATING_CAPABILITIES_REG                   0x49                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x49
#define HW_OPERATING_CAPABILITIES_REGADDR               (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(HW_OPERATING_CAPABILITIES_REG))                     //!< Memory address for @ref PERS_SESS_DESC_REG_0x49
#define HW_OPERATING_CAPABILITIES_REGADDR_DEFAULT       (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(HW_OPERATING_CAPABILITIES_REG))          //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x49

#define FW_OPERATING_CAPABILITIES_REG                   0x4A                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x4A
#define FW_OPERATING_CAPABILITIES_REGADDR               (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FW_OPERATING_CAPABILITIES_REG))                     //!< Memory address for @ref PERS_SESS_DESC_REG_0x4A
#define FW_OPERATING_CAPABILITIES_REGADDR_DEFAULT       (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(FW_OPERATING_CAPABILITIES_REG))          //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x4A

#define BLE_CON_INTV_REG                                0x4B                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x4B
#define BLE_CON_INTV_REGADDR                            (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(BLE_CON_INTV_REG))                                  //!< Memory address for @ref PERS_SESS_DESC_REG_0x4B
#define BLE_CON_INTV_REGADDR_DEFAULT                    (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(BLE_CON_INTV_REG))                       //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x4B

#define LED_INTERVAL_DISCONNECTED_REG                   0x4C                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x4C
#define LED_INTERVAL_DISCONNECTED_REGADDR               (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(LED_INTERVAL_DISCONNECTED_REG))                     //!< Memory address for @ref PERS_SESS_DESC_REG_0x4C
#define LED_INTERVAL_DISCONNECTED_REGADDR_DEFAULT       (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(LED_INTERVAL_DISCONNECTED_REG))          //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x4C

#define LED_INTERVAL_DISCONNECTED_ON_REG                0x4D                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x4D
#define LED_INTERVAL_DISCONNECTED_ON_REGADDR            (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(LED_INTERVAL_DISCONNECTED_ON_REG))                  //!< Memory address for @ref PERS_SESS_DESC_REG_0x4D
#define LED_INTERVAL_DISCONNECTED_ON_REGADDR_DEFAULT    (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(LED_INTERVAL_DISCONNECTED_ON_REG))       //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x4D

#define LED_INTERVAL_CONNECTED_REG                      0x4E                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x4E
#define LED_INTERVAL_CONNECTED_REGADDR                  (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(LED_INTERVAL_CONNECTED_REG))                        //!< Memory address for @ref PERS_SESS_DESC_REG_0x4E
#define LED_INTERVAL_CONNECTED_REGADDR_DEFAULT          (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(LED_INTERVAL_CONNECTED_REG))             //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x4E

#define LED_INTERVAL_CONNECTED_ON_REG                   0x4F                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x4F
#define LED_INTERVAL_CONNECTED_ON_REGADDR               (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(LED_INTERVAL_CONNECTED_ON_REG))                     //!< Memory address for @ref PERS_SESS_DESC_REG_0x4F
#define LED_INTERVAL_CONNECTED_ON_REGADDR_DEFAULT       (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(LED_INTERVAL_CONNECTED_ON_REG))          //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x4F

/* Optional Specific */
  
#define RED_LED_CONFIG_REG                              0x50                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x50
#define RED_LED_CONFIG_REGADDR                          (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(RED_LED_CONFIG_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0x50
#define RED_LED_CONFIG_REGADDR_DEFAULT                  (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(RED_LED_CONFIG_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x50

#define USB_PWR_DISC_REG                                0x52                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x52
#define USB_PWR_DISC_REGADDR                            (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(USB_PWR_DISC_REG))                                  //!< Memory address for @ref PERS_SESS_DESC_REG_0x52
#define USB_PWR_DISC_REGADDR_DEFAULT                    (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(USB_PWR_DISC_REG))                       //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x52

#define CHARGER_DISABLE_REG                             0x53                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x53
#define CHARGER_DISABLE_REGADDR                         (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(CHARGER_DISABLE_REG))                               //!< Memory address for @ref PERS_SESS_DESC_REG_0x53
#define CHARGER_DISABLE_REGADDR_DEFAULT                 (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(CHARGER_DISABLE_REG))                    //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x53

#define USB_CONNECT_REG                                 0x54                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x54
#define USB_CONNECT_REGADDR                             (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(USB_CONNECT_REG))                                   //!< Memory address for @ref PERS_SESS_DESC_REG_0x54
#define USB_CONNECT_REGADDR_DEFAULT                     (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(USB_CONNECT_REG))                        //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x54

#define AUTOSLEEP_TIME_REG                              0x55                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x55
#define AUTOSLEEP_TIME_REGADDR                          (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(AUTOSLEEP_TIME_REG))                                //!< Memory address for @ref PERS_SESS_DESC_REG_0x55
#define AUTOSLEEP_TIME_REGADDR_DEFAULT                  (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(AUTOSLEEP_TIME_REG))                     //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x55

#define USE_RANDOM_ADDRESS_REG                          0x56                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x56
#define USE_RANDOM_ADDRESS_REGADDR                      (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(USE_RANDOM_ADDRESS_REG))                            //!< Memory address for @ref PERS_SESS_DESC_REG_0x56
#define USE_RANDOM_ADDRESS_REGADDR_DEFAULT              (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(USE_RANDOM_ADDRESS_REG))                 //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x56

#define USB_WAKEUP_DISABLE_REG                          0x57                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x57
#define USB_WAKEUP_DISABLE_REGADDR                      (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(USB_WAKEUP_DISABLE_REG))                            //!< Memory address for @ref PERS_SESS_DESC_REG_0x57
#define USB_WAKEUP_DISABLE_REGADDR_DEFAULT              (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(USB_WAKEUP_DISABLE_REG))                 //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x57

#define GG_PWRDOWN_MODE_REG                             0x58                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x58
#define GG_PWRDOWN_MODE_REGADDR                         (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(GG_PWRDOWN_MODE_REG))                               //!< Memory address for @ref PERS_SESS_DESC_REG_0x58
#define GG_PWRDOWN_MODE_REGADDR_DEFAULT                 (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(GG_PWRDOWN_MODE_REG))                    //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x58

#define BUTTON_PRESS_MODE_REG                           0x59                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x59
#define BUTTON_PRESS_MODE_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(BUTTON_PRESS_MODE_REG))                           //!< Memory address for @ref PERS_SESS_DESC_REG_0x59
#define BUTTON_PRESS_MODE_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(BUTTON_PRESS_MODE_REG))                //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x59

#define CONN_ONLY_MODE_REG                              0x5A                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x5A
#define CONN_ONLY_MODE_REGADDR                          (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(CONN_ONLY_MODE_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0x5A
#define CONN_ONLY_MODE_REGADDR_DEFAULT                  (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(CONN_ONLY_MODE_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x5A

#define AUTOSLEEP_MODE_REG                              0x5B                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x5B
#define AUTOSLEEP_MODE_REGADDR                          (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(AUTOSLEEP_MODE_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0x5B
#define AUTOSLEEP_MODE_REGADDR_DEFAULT                  (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(AUTOSLEEP_MODE_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x5B

#define MAG_CALIB_CONTROL_REG                           0x60                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x60
#define MAG_CALIB_CONTROL_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(MAG_CALIB_CONTROL_REG))                           //!< Memory address for @ref PERS_SESS_DESC_REG_0x60
#define MAG_CALIB_CONTROL_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(MAG_CALIB_CONTROL_REG))                //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x60

#define MAG_CALIB_OFFSET_0_REG                          0x62                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x61
#define MAG_CALIB_OFFSET_0_REGADDR                      (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(MAG_CALIB_OFFSET_0_REG))                            //!< Memory address for @ref PERS_SESS_DESC_REG_0x61

#define MAG_CALIB_OFFSET_1_REG                          0x64                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x63
#define MAG_CALIB_OFFSET_1_REGADDR                      (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(MAG_CALIB_OFFSET_1_REG))                            //!< Memory address for @ref PERS_SESS_DESC_REG_0x63

#define MAG_CALIB_OFFSET_2_REG                          0x66                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x65
#define MAG_CALIB_OFFSET_2_REGADDR                      (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(MAG_CALIB_OFFSET_2_REG))                            //!< Memory address for @ref PERS_SESS_DESC_REG_0x65

#define MAG_CALIB_GAIN_0_REG                            0x68                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x67
#define MAG_CALIB_GAIN_0_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(MAG_CALIB_GAIN_0_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0x67

#define MAG_CALIB_GAIN_1_REG                            0x6A                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x69
#define MAG_CALIB_GAIN_1_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(MAG_CALIB_GAIN_1_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0x69

#define MAG_CALIB_GAIN_2_REG                            0x6C                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x6B
#define MAG_CALIB_GAIN_2_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(MAG_CALIB_GAIN_2_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0x6B

#define CALIBRATION_HW_REG                              0x6E                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x6E
#define CALIBRATION_HW_REGADDR                          (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(CALIBRATION_HW_REG))                                //!< Memory address for @ref PERS_SESS_DESC_REG_0x6E
#define CALIBRATION_HW_REGADDR_DEFAULT                  (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(CALIBRATION_HW_REG))                     //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x6E

#define CALIBRATION_FW_REG                              0x6F                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x6F
#define CALIBRATION_FW_REGADDR                          (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(CALIBRATION_FW_REG))                                //!< Memory address for @ref PERS_SESS_DESC_REG_0x6F
#define CALIBRATION_FW_REGADDR_DEFAULT                  (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(CALIBRATION_FW_REG))                     //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x6F

#define BLE_ADV_INTV_RUN_REG                            0x70                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x70
#define BLE_ADV_INTV_RUN_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(BLE_ADV_INTV_RUN_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0x70
#define BLE_ADV_INTV_RUN_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(BLE_ADV_INTV_RUN_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x70

#define BLE_ADV_INTV_SLEEP_REG                          0x71                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x71
#define BLE_ADV_INTV_SLEEP_REGADDR                      (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(BLE_ADV_INTV_SLEEP_REG))                            //!< Memory address for @ref PERS_SESS_DESC_REG_0x71
#define BLE_ADV_INTV_SLEEP_REGADDR_DEFAULT              (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(BLE_ADV_INTV_SLEEP_REG))                 //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x71

#define ACC_FullScale_REG                               0x74                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x74
#define ACC_FullScale_REGADDR                           (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(ACC_FullScale_REG))                                 //!< Memory address for @ref PERS_SESS_DESC_REG_0x74
#define ACC_FullScale_REGADDR_DEFAULT                   (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(ACC_FullScale_REG))                      //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x74

#define ACC_OutputDataRate_REG                          0x75                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x75
#define ACC_OutputDataRate_REGADDR                      (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(ACC_OutputDataRate_REG))                            //!< Memory address for @ref PERS_SESS_DESC_REG_0x75
#define ACC_OutputDataRate_REGADDR_DEFAULT              (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(ACC_OutputDataRate_REG))                 //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x75

#define GYRO_FullScale_REG                              0x76                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x76
#define GYRO_FullScale_REGADDR                          (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(GYRO_FullScale_REG))                                //!< Memory address for @ref PERS_SESS_DESC_REG_0x76
#define GYRO_FullScale_REGADDR_DEFAULT                  (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(GYRO_FullScale_REG))                     //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x76

#define GYRO_OutputDataRate_REG                         0x77                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x77
#define GYRO_OutputDataRate_REGADDR                     (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(GYRO_OutputDataRate_REG))                           //!< Memory address for @ref PERS_SESS_DESC_REG_0x77
#define GYRO_OutputDataRate_REGADDR_DEFAULT             (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(GYRO_OutputDataRate_REG))                //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x77

#define MAG_FullScale_REG                               0x78                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x78
#define MAG_FullScale_REGADDR                           (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(MAG_FullScale_REG))                                 //!< Memory address for @ref PERS_SESS_DESC_REG_0x78
#define MAG_FullScale_REGADDR_DEFAULT                   (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(Mag_FullScale_REG))                      //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x78

#define MAG_OutputDataRate_REG                          0x79                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x79
#define MAG_OutputDataRate_REGADDR                      (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(MAG_OutputDataRate_REG))                            //!< Memory address for @ref PERS_SESS_DESC_REG_0x79
#define MAG_OutputDataRate_REGADDR_DEFAULT              (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(MAG_OutputDataRate_REG))                 //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x79

#define ACCEVENT_CONF_REG                   0x7A                                                                       //!< see @ref PERS_SESS_DESC_REG_0x7A
#define ACCEVENT_CONF_REGADDR               (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(ACCEVENT_CONF_REG))                 //!< Memory address for @ref PERS_SESS_DESC_REG_0x7A
#define ACCEVENT_CONF_REGADDR_DEFAULT       (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(ACCEVENT_CONF_REG))      //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x7A

#define PRESS_TEMP_OutputDataRate_REG                   0x7B                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x7B
#define PRESS_TEMP_OutputDataRate_REGADDR               (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PRESS_TEMP_OutputDataRate_REG))                     //!< Memory address for @ref PERS_SESS_DESC_REG_0x7B
#define PRESS_TEMP_OutputDataRate_REGADDR_DEFAULT       (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PRESS_TEMP_OutputDataRate_REG))          //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x7B

/* EEPROMREG_19 Application Timer control mask: mcu timer or Accelerometer data-ready  */
#define TIMER_DRDY_CONTROL_MASK_REG                   0x7D
#define TIMER_DRDY_CONTROL_MASK_REGADDR               (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(TIMER_DRDY_CONTROL_MASK_REG))
#define TIMER_DRDY_CONTROL_MASK_REGADDR_DEFAULT       (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(TIMER_DRDY_CONTROL_MASK_REG))

#define BATTERY_LOW_THRESHOLD_REG                   0x7E
#define BATTERY_LOW_THRESHOLD_REGADDR               (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(BATTERY_LOW_THRESHOLD_REG))
#define BATTERY_LOW_THRESHOLD_REGADDR_DEFAULT       (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(BATTERY_LOW_THRESHOLD_REG))
    
#define BATTERY_VERY_LOW_THRESHOLD_REG                   0x7F
#define BATTERY_VERY_LOW_THRESHOLD_REGADDR               (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(BATTERY_VERY_LOW_THRESHOLD_REG))
#define BATTERY_VERY_LOW_THRESHOLD_REGADDR_DEFAULT       (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(BATTERY_VERY_LOW_THRESHOLD_REG))
    
#define GP_ALGORITHM_THS1_REG                           0x80                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x80
#define GP_ALGORITHM_THS1_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(GP_ALGORITHM_THS1_REG))                     //!< Memory address for @ref PERS_SESS_DESC_REG_0x80
#define GP_ALGORITHM_THS1_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(GP_ALGORITHM_THS1_REG))          //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x80

#define GP_ALGORITHM_THS2_REG                           0x81                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x81
#define GP_ALGORITHM_THS2_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(GP_ALGORITHM_THS2_REG))                     //!< Memory address for @ref PERS_SESS_DESC_REG_0x81
#define GP_ALGORITHM_THS2_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(GP_ALGORITHM_THS2_REG))          //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x81

#define RTC_CONF_0_REG                                  0x90                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x90
#define RTC_CONF_0_REGADDR                              (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(RTC_CONF_0_REG))                                    //!< Memory address for @ref PERS_SESS_DESC_REG_0x90
#define RTC_CONF_0_REGADDR_DEFAULT                      (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(RTC_CONF_0_REG))                         //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x90

#define RTC_CONF_1_REG                                  0x91                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x91
#define RTC_CONF_1_REGADDR                              (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(RTC_CONF_1_REG))                                    //!< Memory address for @ref PERS_SESS_DESC_REG_0x91
#define RTC_CONF_1_REGADDR_DEFAULT                      (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(RTC_CONF_1_REG))                         //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x91

#define RTC_CONF_2_REG                                  0x92                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x92
#define RTC_CONF_2_REGADDR                              (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(RTC_CONF_2_REG))                                    //!< Memory address for @ref PERS_SESS_DESC_REG_0x92
#define RTC_CONF_2_REGADDR_DEFAULT                      (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(RTC_CONF_2_REG))                         //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x92

#define RTC_CONF_3_REG                                  0x93                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x93
#define RTC_CONF_3_REGADDR                              (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(RTC_CONF_3_REG))                                    //!< Memory address for @ref PERS_SESS_DESC_REG_0x93
#define RTC_CONF_3_REGADDR_DEFAULT                      (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(RTC_CONF_3_REG))                         //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x93

#define RTC_BACKUP_CONF_0_REG                           0x94                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x94
#define RTC_BACKUP_CONF_0_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(RTC_BACKUP_CONF_0_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x94
#define RTC_BACKUP_CONF_0_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(RTC_BACKUP_CONF_0_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x94

#define RTC_BACKUP_CONF_1_REG                           0x95                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x95
#define RTC_BACKUP_CONF_1_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(RTC_BACKUP_CONF_1_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x95
#define RTC_BACKUP_CONF_1_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(RTC_BACKUP_CONF_1_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x95

#define RTC_BACKUP_CONF_2_REG                           0x96                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x96
#define RTC_BACKUP_CONF_2_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(RTC_BACKUP_CONF_2_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x96
#define RTC_BACKUP_CONF_2_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(RTC_BACKUP_CONF_2_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x96

#define RTC_BACKUP_CONF_3_REG                           0x97                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x97
#define RTC_BACKUP_CONF_3_REGADDR                       (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(RTC_BACKUP_CONF_3_REG))                             //!< Memory address for @ref PERS_SESS_DESC_REG_0x97
#define RTC_BACKUP_CONF_3_REGADDR_DEFAULT               (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(RTC_BACKUP_CONF_3_REG))                  //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0x97

#define SYSTEM_TICKS_BACKUP_POWERON_REG                 0xB0                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xB0
#define SYSTEM_TICKS_BACKUP_POWERON_REGADDR             (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(SYSTEM_TICKS_BACKUP_POWERON_REG))                   //!< Memory address for @ref PERS_SESS_DESC_REG_0xB0
#define SYSTEM_TICKS_BACKUP_POWERON_REGADDR_DEFAULT     (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(SYSTEM_TICKS_BACKUP_POWERON_REG))        //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xB0
    
#define BATTERY_VOLTAGE_BACKUP_POWERON_REG              0xB2                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xB2
#define BATTERY_VOLTAGE_BACKUP_POWERON_REGADDR          (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(BATTERY_VOLTAGE_BACKUP_POWERON_REG))                //!< Memory address for @ref PERS_SESS_DESC_REG_0xB2
#define BATTERY_VOLTAGE_BACKUP_POWERON_REGADDR_DEFAULT  (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(BATTERY_VOLTAGE_BACKUP_POWERON_REG))     //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xB2

#define STEPS_BACKUP_POWERON_REG                        0xB4                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xB4
#define STEPS_BACKUP_POWERON_REGADDR                    (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(STEPS_BACKUP_POWERON_REG))                          //!< Memory address for @ref PERS_SESS_DESC_REG_0xB4
#define STEPS_BACKUP_POWERON_REGADDR_DEFAULT            (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(STEPS_BACKUP_POWERON_REG))               //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xB4

#define SYSTEM_TICKS_BACKUP_SESSION_REG                 0xC0                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xC0
#define SYSTEM_TICKS_BACKUP_SESSION_REGADDR             (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(SYSTEM_TICKS_BACKUP_SESSION_REG))                   //!< Memory address for @ref PERS_SESS_DESC_REG_0xC0
#define SYSTEM_TICKS_BACKUP_SESSION_REGADDR_DEFAULT     (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(SYSTEM_TICKS_BACKUP_SESSION_REG))        //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xC0
    
#define BATTERY_VOLTAGE_BACKUP_SESSION_REG              0xC2                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xC2
#define BATTERY_VOLTAGE_BACKUP_SESSION_REGADDR          (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(BATTERY_VOLTAGE_BACKUP_SESSION_REG))                //!< Memory address for @ref PERS_SESS_DESC_REG_0xC2
#define BATTERY_VOLTAGE_BACKUP_SESSION_REGADDR_DEFAULT  (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(BATTERY_VOLTAGE_BACKUP_SESSION_REG))     //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xC2

#define STEPS_BACKUP_SESSION_REG                        0xC4                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xC4
#define STEPS_BACKUP_SESSION_REGADDR                    (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(STEPS_BACKUP_SESSION_REG))                          //!< Memory address for @ref PERS_SESS_DESC_REG_0xC4
#define STEPS_BACKUP_SESSION_REGADDR_DEFAULT            (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(STEPS_BACKUP_SESSION_REG))               //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xC4

#define PEDO_SAVE_DATA_0_REG                            0xE0                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xE0
#define PEDO_SAVE_DATA_0_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_0_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xE0
#define PEDO_SAVE_DATA_0_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_0_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xE0

#define PEDO_SAVE_DATA_1_REG                            0xE1                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xE1
#define PEDO_SAVE_DATA_1_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_1_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xE1
#define PEDO_SAVE_DATA_1_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_1_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xE1

#define PEDO_SAVE_DATA_2_REG                            0xE2                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xE2
#define PEDO_SAVE_DATA_2_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_2_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xE2
#define PEDO_SAVE_DATA_2_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_2_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xE2

#define PEDO_SAVE_DATA_3_REG                            0xE3                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xE3
#define PEDO_SAVE_DATA_3_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_3_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xE3
#define PEDO_SAVE_DATA_3_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_3_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xE3

#define PEDO_SAVE_DATA_4_REG                            0xE4                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xE4
#define PEDO_SAVE_DATA_4_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_4_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xE4
#define PEDO_SAVE_DATA_4_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_4_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xE4

#define PEDO_SAVE_DATA_5_REG                            0xE5                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xE5
#define PEDO_SAVE_DATA_5_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_5_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xE5
#define PEDO_SAVE_DATA_5_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_5_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xE5

#define PEDO_SAVE_DATA_6_REG                            0xE6                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xE6
#define PEDO_SAVE_DATA_6_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_6_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xE6
#define PEDO_SAVE_DATA_6_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_6_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xE6

#define PEDO_SAVE_DATA_7_REG                            0xE7                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xE7
#define PEDO_SAVE_DATA_7_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_7_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xE7
#define PEDO_SAVE_DATA_7_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_7_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xE7

#define PEDO_SAVE_DATA_8_REG                            0xE8                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xE8
#define PEDO_SAVE_DATA_8_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_8_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xE8
#define PEDO_SAVE_DATA_8_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_8_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xE8

#define PEDO_SAVE_DATA_9_REG                            0xE9                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xE9
#define PEDO_SAVE_DATA_9_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_9_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xE9
#define PEDO_SAVE_DATA_9_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_9_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xE9

#define PEDO_SAVE_DATA_A_REG                            0xEA                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xEA
#define PEDO_SAVE_DATA_A_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_A_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xEA
#define PEDO_SAVE_DATA_A_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_A_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xEA

#define PEDO_SAVE_DATA_B_REG                            0xEB                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xEB
#define PEDO_SAVE_DATA_B_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_B_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xEB
#define PEDO_SAVE_DATA_B_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_B_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xEB

#define PEDO_SAVE_DATA_C_REG                            0xEC                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xEC
#define PEDO_SAVE_DATA_C_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_C_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xEC
#define PEDO_SAVE_DATA_C_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_C_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xEC

#define PEDO_SAVE_DATA_D_REG                            0xED                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xED
#define PEDO_SAVE_DATA_D_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_D_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xED
#define PEDO_SAVE_DATA_D_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_D_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xED

#define PEDO_SAVE_DATA_E_REG                            0xEE                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xEE
#define PEDO_SAVE_DATA_E_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_E_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xEE
#define PEDO_SAVE_DATA_E_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_E_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xEE

#define PEDO_SAVE_DATA_F_REG                            0xEF                                                                                    //!< see @ref PERS_SESS_DESC_REG_0xEF
#define PEDO_SAVE_DATA_F_REGADDR                        (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(PEDO_SAVE_DATA_F_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xEF
#define PEDO_SAVE_DATA_F_REGADDR_DEFAULT                (xRegType_t*)(GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(PEDO_SAVE_DATA_F_REG))                   //!< Memory address for Default Value @ref PERS_SESS_DESC_REG_0xEF

#define FX_MAG_CALIB_OFFSET_0_REG                        0x10                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x10
#define FX_MAG_CALIB_OFFSET_0_REGADDR                    (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FX_MAG_CALIB_OFFSET_0_REG))                            //!< Memory address for @ref PERS_SESS_DESC_REG_0xF0

#define FX_MAG_CALIB_OFFSET_1_REG                        0x11                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x11
#define FX_MAG_CALIB_OFFSET_1_REGADDR                    (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FX_MAG_CALIB_OFFSET_1_REG))                            //!< Memory address for @ref PERS_SESS_DESC_REG_0xF1

#define FX_MAG_CALIB_OFFSET_2_REG                        0x12                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x12
#define FX_MAG_CALIB_OFFSET_2_REGADDR                    (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FX_MAG_CALIB_OFFSET_2_REG))                            //!< Memory address for @ref PERS_SESS_DESC_REG_0xF2

#define FX_MAG_CALIB_GAIN_0_REG                          0x13                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x13
#define FX_MAG_CALIB_GAIN_0_REGADDR                      (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FX_MAG_CALIB_GAIN_0_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xF3

#define FX_MAG_CALIB_GAIN_1_REG                          0x15                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x15
#define FX_MAG_CALIB_GAIN_1_REGADDR                      (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FX_MAG_CALIB_GAIN_1_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xF5

#define FX_MAG_CALIB_GAIN_2_REG                          0x17                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x17
#define FX_MAG_CALIB_GAIN_2_REGADDR                      (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FX_MAG_CALIB_GAIN_2_REG))                              //!< Memory address for @ref PERS_SESS_DESC_REG_0xF7

#define FX_EXP_MAG_VECT_REG                              0x19                                                                                    //!< see @ref PERS_SESS_DESC_REG_0x19
#define FX_EXP_MAG_VECT_REGADDR                          (xRegType_t*)(PERMREG_STRUCT_GET_ADDR(FX_EXP_MAG_VECT_REG))                                  //!< Memory address for @ref PERS_SESS_DESC_REG_0xF9
  
/* Register type */
#define xRegType_t                      const uint16_t        //!< Default type for register data (16-bit unsigned)

/* Global configuration register size */
#define GLOBAL_CONF_REG_SIZE                  sizeof(xRegType_t)        //!< Register 'default value' size
#define PERMREG_REG_SIZE                      sizeof(xRegType_t)        //!< Register data size
#define N_REGS_FLOAT                          (sizeof(float)/sizeof(xRegType_t))        //!< 'float' size

#define PERMREG_STRUCT_GET_ADDR(R)            (PERMREG_STRUCT_START_ADDRESS + (R * PERMREG_REG_SIZE))       //!< Macro to get persistent register address
#define PERMREG_STRUCT_BCK_GET_ADDR(R)        (PERMREG_STRUCT_BCK_START_ADDRESS + (R * PERMREG_REG_SIZE))   //!< Macro to get persistent backup registers address
#define LICENSE_GET_ADDR(R)                   (LICENSE_START_ADDRESS + (R * LICENSE_SIZE))                  //!< Macro to get license address
                                                      

#define GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(R)   (uint16_t*)(nGlobalConfStruct + (R * GLOBAL_CONF_REG_SIZE)) //!< Macro to get register 'default value' address

#define LOAD_DEFAULT_EEPROM_REGS()                              do{uint8_t nVect[2] = {0,0};BSP_PERSREGS_WRITE(0xFF, (void*)nVect,1);BSP_PERSREGS_WRITE(0, (void*)GLOBAL_CONF_FLASH_PERMREG_GET_ADDR(0), 0x100);SESSION_REG(nVersion) = 0;SESSION_REG(nVersion1) = 0;}while(0) //!< Load default values for EEPROM registers

#define PERSISTENT_REGS_VALIDATION()                            ((*FW_VERSION_0_REG_REGADDR == FIRMWARE_VERSION) && (*FW_VERSION_1_REG_REGADDR == FIRMWARE_VERSION) && (*(char*)(BLE_LOC_NAME_REGADDR)==9) && (*(char*)(PERMREG_STRUCT_GET_ADDR(0xFF))!=0))  //!< Persistent registers validation

#define RAM_REGS_VALIDATION()                                   ((SESSION_REG(nVersion) == SESSION_REG(nVersion1)) && (SESSION_REG(nVersion) == FIRMWARE_VERSION))  //!< Session registers validation



/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */  
  
/**
  * @}
  */
  
/** @addtogroup WeSU_Demo       WeSU Demo      
  * @{
  */

/** @addtogroup WeSU_User               WeSU User
  * @{
  */

/** @addtogroup BlueST_PROTOCOL          BlueST PROTOCOL
  * @{
  * @brief API to provide the BlueST Protocol specs on WeSU
  */

/** @addtogroup Registers_description     Registers description 
 * @brief Persistent and session registers description and memory address; for complete memory mapping see \ref pageMemory
 * @{
 */
  
#define PERS_SESS_DESC_REG_0x00                   /*!< Persistent: Firmware version; Session: Firmware version 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x00  </TD> 
   <TD COLSPAN=2> Firmware version  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x00  </TD> 
   <TD> Address: 0x08081000 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref ReadOnly  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x00  </TD> 
   <TD COLSPAN=2> Firmware version  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x00  </TD> 
   <TD> Address: 0x20013000 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref ReadOnly  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x01                   /*!< Persistent: Firmware version; Session: Firmware version 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x01  </TD> 
   <TD COLSPAN=2> Firmware version and subversion; must be identical to @ref PERS_SESS_DESC_REG_0x00 in order to validate the content   </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x01  </TD> 
   <TD> Address: 0x08081002 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref ReadOnly  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x01  </TD> 
   <TD COLSPAN=2> Firmware version and subversion; must be identical to @ref PERS_SESS_DESC_REG_0x00 in order to validate the content   </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x01  </TD> 
   <TD> Address: 0x20013002 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref ReadOnly  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x02                   /*!< Persistent: Led configuration; Session: Led control mask 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x02  </TD> 
   <TD COLSPAN=2> Led configuration  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x02  </TD> 
   <TD> Address: 0x08081004 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref LedControlMaskChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x02  </TD> 
   <TD COLSPAN=2> Led configuration  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x02  </TD> 
   <TD> Address: 0x20013004 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref LedControlMaskChanged  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x03                   /*!< Persistent: BlueNRG local name; Session: Battery charge percentage 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x03  </TD> 
   <TD COLSPAN=2> BlueNRG local name, this register MUST start with char 0x09  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x03  </TD> 
   <TD> Address: 0x08081006 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x03  </TD> 
   <TD COLSPAN=2> Battery charge percentage (0.1%)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x03  </TD> 
   <TD> Address: 0x20013006 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x04                   /*!< Persistent: BlueNRG local name; Session: Battery Voltage 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x04  </TD> 
   <TD COLSPAN=2> BlueNRG local name  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x04  </TD> 
   <TD> Address: 0x08081008 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x04  </TD> 
   <TD COLSPAN=2> Battery Voltage (mV) (reg04->reg05)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x04  </TD> 
   <TD> Address: 0x20013008 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x05                   /*!< Persistent: BlueNRG local name; Session: Battery Voltage 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x05  </TD> 
   <TD COLSPAN=2> BlueNRG local name  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x05  </TD> 
   <TD> Address: 0x0808100A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x05  </TD> 
   <TD COLSPAN=2> Battery Voltage (mV) (reg04->reg05)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x05  </TD> 
   <TD> Address: 0x2001300A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x06                   /*!< Persistent: BlueNRG local name; Session: Battery Current 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x06  </TD> 
   <TD COLSPAN=2> BlueNRG local name  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x06  </TD> 
   <TD> Address: 0x0808100C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x06  </TD> 
   <TD COLSPAN=2> Battery Current (uA)(in >0 /out <0) (reg06->reg07)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x06  </TD> 
   <TD> Address: 0x2001300C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x07                   /*!< Persistent: BlueNRG local name; Session: Battery Current 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x07  </TD> 
   <TD COLSPAN=2> BlueNRG local name  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x07  </TD> 
   <TD> Address: 0x0808100E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x07  </TD> 
   <TD COLSPAN=2> Battery Current (uA)(in >0 /out <0) (reg06->reg07)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x07  </TD> 
   <TD> Address: 0x2001300E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x08                   /*!< Persistent: BlueNRG local name; Session: Battery power 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x08  </TD> 
   <TD COLSPAN=2> BlueNRG local name  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x08  </TD> 
   <TD> Address: 0x08081010 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x08  </TD> 
   <TD COLSPAN=2> Battery power (charging/discharging/battery/low battery)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x08  </TD> 
   <TD> Address: 0x20013010 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x09                   /*!< Persistent: BlueNRG local name; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x09  </TD> 
   <TD COLSPAN=2> BlueNRG local name  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x09  </TD> 
   <TD> Address: 0x08081012 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x09  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x09  </TD> 
   <TD> Address: 0x20013012 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x0A                   /*!< Persistent: BlueNRG local name; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x0A  </TD> 
   <TD COLSPAN=2> BlueNRG local name  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x0A  </TD> 
   <TD> Address: 0x08081014 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x0A  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x0A  </TD> 
   <TD> Address: 0x20013014 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x0B                   /*!< Persistent: BlueNRG public address; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x0B  </TD> 
   <TD COLSPAN=2> BlueNRG public address  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x0B  </TD> 
   <TD> Address: 0x08081016 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x0B  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x0B  </TD> 
   <TD> Address: 0x20013016 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x0C                   /*!< Persistent: BlueNRG public address; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x0C  </TD> 
   <TD COLSPAN=2> BlueNRG public address  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x0C  </TD> 
   <TD> Address: 0x08081018 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x0C  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x0C  </TD> 
   <TD> Address: 0x20013018 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x0D                   /*!< Persistent: BlueNRG public address; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x0D  </TD> 
   <TD COLSPAN=2> BlueNRG public address  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x0D  </TD> 
   <TD> Address: 0x0808101A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x0D  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x0D  </TD> 
   <TD> Address: 0x2001301A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x0E                   /*!< Persistent: BlueNRG public address; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x0E  </TD> 
   <TD COLSPAN=2> BlueNRG public address  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x0E  </TD> 
   <TD> Address: 0x0808101C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x0E  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x0E  </TD> 
   <TD> Address: 0x2001301C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x0F                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x0F  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x0F  </TD> 
   <TD> Address: 0x0808101E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x0F  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x0F  </TD> 
   <TD> Address: 0x2001301E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x10                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x10  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x10  </TD> 
   <TD> Address: 0x08081020 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x10  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x10  </TD> 
   <TD> Address: 0x20013020 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x11                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x11  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x11  </TD> 
   <TD> Address: 0x08081022 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x11  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x11  </TD> 
   <TD> Address: 0x20013022 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x12                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x12  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x12  </TD> 
   <TD> Address: 0x08081024 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x12  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x12  </TD> 
   <TD> Address: 0x20013024 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x13                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x13  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x13  </TD> 
   <TD> Address: 0x08081026 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x13  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x13  </TD> 
   <TD> Address: 0x20013026 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x14                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x14  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x14  </TD> 
   <TD> Address: 0x08081028 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x14  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x14  </TD> 
   <TD> Address: 0x20013028 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x15                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x15  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x15  </TD> 
   <TD> Address: 0x0808102A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x15  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x15  </TD> 
   <TD> Address: 0x2001302A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x16                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x16  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x16  </TD> 
   <TD> Address: 0x0808102C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x16  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x16  </TD> 
   <TD> Address: 0x2001302C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x17                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x17  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x17  </TD> 
   <TD> Address: 0x0808102E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x17  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x17  </TD> 
   <TD> Address: 0x2001302E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x18                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x18  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x18  </TD> 
   <TD> Address: 0x08081030 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x18  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x18  </TD> 
   <TD> Address: 0x20013030 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x19                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x19  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x19  </TD> 
   <TD> Address: 0x08081032 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x19  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x19  </TD> 
   <TD> Address: 0x20013032 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x1A                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x1A  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x1A  </TD> 
   <TD> Address: 0x08081034 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x1A  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x1A  </TD> 
   <TD> Address: 0x20013034 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x1B                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x1B  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x1B  </TD> 
   <TD> Address: 0x08081036 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x1B  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x1B  </TD> 
   <TD> Address: 0x20013036 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x1C                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x1C  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x1C  </TD> 
   <TD> Address: 0x08081038 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x1C  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x1C  </TD> 
   <TD> Address: 0x20013038 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x1D                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x1D  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x1D  </TD> 
   <TD> Address: 0x0808103A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x1D  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x1D  </TD> 
   <TD> Address: 0x2001303A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x1E                   /*!< Persistent: IWDG Reload value; Session: IWDG Reload value 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x1E  </TD> 
   <TD COLSPAN=2> IWDG Reload value  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x1E  </TD> 
   <TD> Address: 0x0808103C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x1E  </TD> 
   <TD COLSPAN=2> IWDG Reload value  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x1E  </TD> 
   <TD> Address: 0x2001303C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x1F                   /*!< Persistent: WeSU DEBUG enable mask ; Session: WeSU DEBUG enable mask  
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x1F  </TD> 
   <TD COLSPAN=2> WeSU DEBUG enable mask for dedicated DBG_PRINTF (terminal, algorithms, power management, bluenrg)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x1F  </TD> 
   <TD> Address: 0x0808103E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref CopyToSession  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x1F  </TD> 
   <TD COLSPAN=2> WeSU DEBUG enable mask for dedicated DBG_PRINTF (terminal, algorithms, power management, bluenrg)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x1F  </TD> 
   <TD> Address: 0x2001303E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x20                   /*!< Persistent: BlueNRG tx power level; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x20  </TD> 
   <TD COLSPAN=2> BlueNRG tx power level: LSB PA_Level (0 to 7) MSB En_High_Power level (0 or 1). The entire register value is ORed with 0x0808
<BR> 
 <TABLE BGCOLOR=white> 

  <TR> 
   <TD> Enable High Power  
   </TD><TD> PA Level  
   </TD><TD> Description  
   </TD><TD> Register value  </TD> 

  </TR><TR> 
   <TD> 1  
   </TD><TD> 0  
   </TD><TD> -14dBm  
   </TD><TD> 0x0908  </TD> 

  </TR><TR> 
   <TD> 1  
   </TD><TD> 1  
   </TD><TD> -11dBm  
   </TD><TD> 0x0909  </TD> 

  </TR><TR> 
   <TD> 1  
   </TD><TD> 2  
   </TD><TD> -8dBm  
   </TD><TD> 0x090A  </TD> 

  </TR><TR> 
   <TD> 1  
   </TD><TD> 3  
   </TD><TD> -5dBm  
   </TD><TD> 0x090B  </TD> 

  </TR><TR> 
   <TD> 1  
   </TD><TD> 4  
   </TD><TD> -2dBm  
   </TD><TD> 0x090C </TD> 

  </TR><TR> 
   <TD> 1  
   </TD><TD> 5  
   </TD><TD> +2dBm  
   </TD><TD> 0x090D  </TD> 

  </TR><TR> 
   <TD> 1  
   </TD><TD> 6  
   </TD><TD> +4dBm  
   </TD><TD> 0x090E  </TD> 

  </TR><TR> 
   <TD> 1  
   </TD><TD> 7  
   </TD><TD> +8dBm  
   </TD><TD> 0x090F  </TD> 

  </TR><TR> 
   <TD> 0  
   </TD><TD> 0  
   </TD><TD> -18dBm  
   </TD><TD> 0x0808  </TD> 

  </TR><TR> 
   <TD> 0  
   </TD><TD> 1  
   </TD><TD> -15dBm  
   </TD><TD> 0x0809  </TD> 

  </TR><TR> 
   <TD> 0  
   </TD><TD> 2  
   </TD><TD> -12dBm  
   </TD><TD> 0x080A  </TD> 

  </TR><TR> 
   <TD> 0  
   </TD><TD> 3  
   </TD><TD> -9dBm  
   </TD><TD> 0x080B  </TD> 

  </TR><TR> 
   <TD> 0  
   </TD><TD> 4  
   </TD><TD> -6dBm  
   </TD><TD> 0x080C  </TD> 

  </TR><TR> 
   <TD> 0  
   </TD><TD> 5  
   </TD><TD> -2dBm  
   </TD><TD> 0x080D  </TD> 

  </TR><TR> 
   <TD> 0  
   </TD><TD> 6  
   </TD><TD> 0dBm  
   </TD><TD> 0x080E  </TD> 

  </TR><TR> 
   <TD> 0  
   </TD><TD> 7  
   </TD><TD> +5dBm  
   </TD><TD> 0x080F  </TD> 

  </TR>

 </TABLE>  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x20  </TD> 
   <TD> Address: 0x08081040 <BR> see \ref eeprommemorylayout ""EEPROM mapping""  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref TxPwrLvlChange  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x20  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x20  </TD> 
   <TD> Address: 0x20013040 <BR> see \ref rammemorylayout ""RAM mapping""  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x21                   /*!< Persistent: Timer frequency; Session: Timer frequency 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x21  </TD> 
   <TD COLSPAN=2> Timer frequency  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x21  </TD> 
   <TD> Address: 0x08081042 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref TimerFreqChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x21  </TD> 
   <TD COLSPAN=2> Timer frequency  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x21  </TD> 
   <TD> Address: 0x20013042 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref TimerFreqChanged  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x22                   /*!< Persistent: Application power mode; Session: Application power mode 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x22  </TD> 
   <TD COLSPAN=2> Application power mode (full run, low power, permanent stop)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x22  </TD> 
   <TD> Address: 0x08081044 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref PwrModeControlChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x22  </TD> 
   <TD COLSPAN=2> Application power mode (full run, low power, permanent stop)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x22  </TD> 
   <TD> Address: 0x20013044 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref PwrModeControlChanged  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x23                   /*!< Persistent: Hw capabilities; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x23  </TD> 
   <TD COLSPAN=2> Hw capabilities  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x23  </TD> 
   <TD> Address: 0x08081046 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref ReadOnly  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x23  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x23  </TD> 
   <TD> Address: 0x20013046 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x24                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x24  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x24  </TD> 
   <TD> Address: 0x08081048 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x24  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x24  </TD> 
   <TD> Address: 0x20013048 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x25                   /*!< Persistent: GasGauge control; Session: GasGauge control 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x25  </TD> 
   <TD COLSPAN=2> GasGauge control: mask (enable/disable, output channel), Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x25  </TD> 
   <TD> Address: 0x0808104A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x25  </TD> 
   <TD COLSPAN=2> GasGauge control: mask (enable/disable, output channel), Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x25  </TD> 
   <TD> Address: 0x2001304A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x26                   /*!< Persistent: Temperature sensor control; Session: Temperature sensor control 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x26  </TD> 
   <TD COLSPAN=2> Temperature sensor control: mask (enable/disable, output channel), Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x26  </TD> 
   <TD> Address: 0x0808104C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x26  </TD> 
   <TD COLSPAN=2> Temperature sensor control: mask (enable/disable, output channel), Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x26  </TD> 
   <TD> Address: 0x2001304C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x27                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x27  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x27  </TD> 
   <TD> Address: 0x0808104E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x27  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x27  </TD> 
   <TD> Address: 0x2001304E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x28                   /*!< Persistent: Pressure sensor control; Session: Pressure sensor control 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x28  </TD> 
   <TD COLSPAN=2> Pressure sensor control: mask (enable/disable, output channel), Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x28  </TD> 
   <TD> Address: 0x08081050 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x28  </TD> 
   <TD COLSPAN=2> Pressure sensor control: mask (enable/disable, output channel), Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x28  </TD> 
   <TD> Address: 0x20013050 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x29                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x29  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x29  </TD> 
   <TD> Address: 0x08081052 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x29  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x29  </TD> 
   <TD> Address: 0x20013052 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x2A                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x2A  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x2A  </TD> 
   <TD> Address: 0x08081054 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x2A  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x2A  </TD> 
   <TD> Address: 0x20013054 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x2B                   /*!< Persistent: Motion MEMS control; Session: Motion MEMS control 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x2B  </TD> 
   <TD COLSPAN=2> Motion MEMS control: mask (enable/disable, output channel), Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x2B  </TD> 
   <TD> Address: 0x08081056 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x2B  </TD> 
   <TD COLSPAN=2> Motion MEMS control: mask (enable/disable, output channel), Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x2B  </TD> 
   <TD> Address: 0x20013056 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x2C                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x2C  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x2C  </TD> 
   <TD> Address: 0x08081058 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x2C  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x2C  </TD> 
   <TD> Address: 0x20013058 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x2D                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x2D  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x2D  </TD> 
   <TD> Address: 0x0808105A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x2D  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x2D  </TD> 
   <TD> Address: 0x2001305A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x2E                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x2E  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x2E  </TD> 
   <TD> Address: 0x0808105C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x2E  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x2E  </TD> 
   <TD> Address: 0x2001305C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x2F                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x2F  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x2F  </TD> 
   <TD> Address: 0x0808105E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x2F  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x2F  </TD> 
   <TD> Address: 0x2001305E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x30                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x30  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x30  </TD> 
   <TD> Address: 0x08081060 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x30  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x30  </TD> 
   <TD> Address: 0x20013060 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x31                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x31  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x31  </TD> 
   <TD> Address: 0x08081062 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x31  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x31  </TD> 
   <TD> Address: 0x20013062 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x32                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x32  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x32  </TD> 
   <TD> Address: 0x08081064 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x32  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x32  </TD> 
   <TD> Address: 0x20013064 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x33                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x33  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x33  </TD> 
   <TD> Address: 0x08081066 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x33  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x33  </TD> 
   <TD> Address: 0x20013066 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x34                   /*!< Persistent: Fw capabilities; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x34  </TD> 
   <TD COLSPAN=2> Fw capabilities  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x34  </TD> 
   <TD> Address: 0x08081068 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref ReadOnly  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x34  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x34  </TD> 
   <TD> Address: 0x20013068 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x35                   /*!< Persistent: Pedometer control; Session: Pedometer control 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x35  </TD> 
   <TD COLSPAN=2> Pedometer control: mask (enable/disable, output channel), Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x35  </TD> 
   <TD> Address: 0x0808106A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x35  </TD> 
   <TD COLSPAN=2> Pedometer control: mask (enable/disable, output channel), Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x35  </TD> 
   <TD> Address: 0x2001306A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref PedometerCtrlChanged  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x36                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x36  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x36  </TD> 
   <TD> Address: 0x0808106C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref PedometerCtrlChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x36  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x36  </TD> 
   <TD> Address: 0x2001306C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x37                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x37  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x37  </TD> 
   <TD> Address: 0x0808106E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x37  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x37  </TD> 
   <TD> Address: 0x2001306E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x38                   /*!< Persistent: MotionCP control mask; Session: MotionCP control mask 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x38  </TD> 
   <TD COLSPAN=2> LSB MotionCP control: mask (enable/disable, output channel); MSB:  MotionCP control: Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x38  </TD> 
   <TD> Address: 0x08081070 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x38  </TD> 
   <TD COLSPAN=2> LSB MotionCP control: mask (enable/disable, output channel); MSB:  MotionCP control: Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x38  </TD> 
   <TD> Address: 0x20013070 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x39                   /*!< Persistent: MotionAR control mask; Session: MotionAR control mask 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x39  </TD> 
   <TD COLSPAN=2> LSB MotionAR control: mask (enable/disable, output channel); MSB:  MotionCP control: Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x39  </TD> 
   <TD> Address: 0x08081072 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x39  </TD> 
   <TD COLSPAN=2> LSB MotionAR control: mask (enable/disable, output channel); MSB:  MotionCP control: Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x39  </TD> 
   <TD> Address: 0x20013072 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x3A                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x3A  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x3A  </TD> 
   <TD> Address: 0x08081074 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x3A  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x3A  </TD> 
   <TD> Address: 0x20013074 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x3B                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x3B  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x3B  </TD> 
   <TD> Address: 0x08081076 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x3B  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x3B  </TD> 
   <TD> Address: 0x20013076 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x3C                   /*!< Persistent: AHRS control mask; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x3C  </TD> 
   <TD COLSPAN=2> LSB AHRS control: mask (enable/disable, output channel); MSB:  AHRS control: Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x3C  </TD> 
   <TD> Address: 0x08081078 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x3C  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x3C  </TD> 
   <TD> Address: 0x20013078 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x3D                   /*!< Persistent: MotionFX control mask; Session: MotionFX control mask 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x3D  </TD> 
   <TD COLSPAN=2> LSB MotionFX control: mask (enable/disable, output channel); MSB:  MotionFX control: Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x3D  </TD> 
   <TD> Address: 0x0808107A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref MotionFxCtrlChanged   </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x3D  </TD> 
   <TD COLSPAN=2> LSB MotionFX control: mask (enable/disable, output channel); MSB:  MotionFX control: Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x3D  </TD> 
   <TD> Address: 0x2001307A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref MotionFxCtrlChanged  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x3E                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x3E  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x3E  </TD> 
   <TD> Address: 0x0808107C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x3E  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x3E  </TD> 
   <TD> Address: 0x2001307C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x3F                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x3F  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x3F  </TD> 
   <TD> Address: 0x0808107E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x3F  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x3F  </TD> 
   <TD> Address: 0x2001307E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x40                   /*!< Persistent: AccEvent control mask; Session: AccEvent control mask 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x40  </TD> 
   <TD COLSPAN=2> LSB AccEvent control: mask (enable/disable, output channel); MSB:  AccEvent control: Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x40  </TD> 
   <TD> Address: 0x08081080 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x40  </TD> 
   <TD COLSPAN=2> LSB AccEvent control: mask (enable/disable, output channel); MSB:  AccEvent control: Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x40  </TD> 
   <TD> Address: 0x20013080 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x41                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x41  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x41  </TD> 
   <TD> Address: 0x08081082 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x41  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x41  </TD> 
   <TD> Address: 0x20013082 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x42                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x42  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x42  </TD> 
   <TD> Address: 0x08081084 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x42  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x42  </TD> 
   <TD> Address: 0x20013084 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x43                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x43  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x43  </TD> 
   <TD> Address: 0x08081086 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x43  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x43  </TD> 
   <TD> Address: 0x20013086 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x44                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x44  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x44  </TD> 
   <TD> Address: 0x08081088 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x44  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x44  </TD> 
   <TD> Address: 0x20013088 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x45                   /*!< Persistent: BLE_DEBUG_CONFIG: mask; Session: Configured severity level for BLE 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x45  </TD> 
   <TD COLSPAN=2> BLE_DEBUG_CONFIG: mask  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x45  </TD> 
   <TD> Address: 0x0808108A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref CopyToSession  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x45  </TD> 
   <TD COLSPAN=2> Configured severity level for BLE  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x45  </TD> 
   <TD> Address: 0x2001308A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x46                   /*!< Persistent: USART_DEBUG_CONFIG: mask; Session: Configured severity level for USART 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x46  </TD> 
   <TD COLSPAN=2> USART_DEBUG_CONFIG: mask  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x46  </TD> 
   <TD> Address: 0x0808108C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref CopyToSession  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x46  </TD> 
   <TD COLSPAN=2> Configured severity level for USART  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x46  </TD> 
   <TD> Address: 0x2001308C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x47                   /*!< Persistent: Default severity; Session: Default severity 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x47  </TD> 
   <TD COLSPAN=2> WeSU application messages default severity, can be a value of \ref WeSU_Debug_Severity_t  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x47  </TD> 
   <TD> Address: 0x0808108E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref CopyToSession  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x47  </TD> 
   <TD COLSPAN=2> WeSU application messages default severity, can be a value of \ref WeSU_Debug_Severity_t  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x47  </TD> 
   <TD> Address: 0x2001308E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x48                   /*!< Persistent: GP Algorithms control mask; Session: GP Algorithms control mask 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x48  </TD> 
   <TD COLSPAN=2> LSB GP Algorithms control: mask (enable/disable, output channel); MSB:  GP Algorithms control: Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x48  </TD> 
   <TD> Address: 0x08081090 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x48  </TD> 
   <TD COLSPAN=2> LSB GP Algorithms control: mask (enable/disable, output channel); MSB:  GP Algorithms control: Subsampling  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x48  </TD> 
   <TD> Address: 0x20013090 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x49                   /*!< Persistent: Hw operating capabilities; Session: Read sensors mask 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x49  </TD> 
   <TD COLSPAN=2> Hw operating capabilities (Configurable)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x49  </TD> 
   <TD> Address: 0x08081092 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref HwConfigChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x49  </TD> 
   <TD COLSPAN=2> Read sensors mask: bitmask for enable/disable sensors  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x49  </TD> 
   <TD> Address: 0x20013092 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref HwConfigChanged  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x4A                   /*!< Persistent: Fw operating capabilities; Session: Process algorithms mask 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x4A  </TD> 
   <TD COLSPAN=2> Fw operating capabilities (Configurable)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x4A  </TD> 
   <TD> Address: 0x08081094 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref FwConfigChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x4A  </TD> 
   <TD COLSPAN=2> Process algorithms mask: bitmask for enable/disable algorithms  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x4A  </TD> 
   <TD> Address: 0x20013094 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref FwConfigChanged  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x4B                   /*!< Persistent: BlueNRG connection interval; Session: BlueNRG connection interval 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x4B  </TD> 
   <TD COLSPAN=2> BlueNRG connection interval  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x4B  </TD> 
   <TD> Address: 0x08081096 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref BleConIntvChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x4B  </TD> 
   <TD COLSPAN=2> BlueNRG connection interval  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x4B  </TD> 
   <TD> Address: 0x20013096 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref BleConIntvChanged  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x4C                   /*!< Persistent: LED blinking interval disconnected; Session: BlueNRG HW version 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x4C  </TD> 
   <TD COLSPAN=2> LED smart blinking interval when BlueNRG disconnected  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x4C  </TD> 
   <TD> Address: 0x08081098 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref ReadOnly  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x4C  </TD> 
   <TD COLSPAN=2> BlueNRG HW version  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x4C  </TD> 
   <TD> Address: 0x20013098 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x4D                   /*!< Persistent: LED blinking interval disconnected on-phase; Session: BlueNRG FW version 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x4D  </TD> 
   <TD COLSPAN=2> LED smart blinking interval when BlueNRG disconnected: on-phase  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x4D  </TD> 
   <TD> Address: 0x0808109A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref ReadOnly  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x4D  </TD> 
   <TD COLSPAN=2> BlueNRG FW version  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x4D  </TD> 
   <TD> Address: 0x2001309A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x4E                   /*!< Persistent: LED blinking interval connected; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x4E  </TD> 
   <TD COLSPAN=2> LED smart blinking interval when BlueNRG connected  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x4E  </TD> 
   <TD> Address: 0x0808109C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x4E  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x4E  </TD> 
   <TD> Address: 0x2001309C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x4F                   /*!< Persistent: LED blinking interval connected on-phase; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x4F  </TD> 
   <TD COLSPAN=2> LED smart blinking interval when BlueNRG connected: on-phase  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x4F  </TD> 
   <TD> Address: 0x0808109E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x4F  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x4F  </TD> 
   <TD> Address: 0x2001309E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x50                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x50  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x50  </TD> 
   <TD> Address: 0x080810A0 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x50  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x50  </TD> 
   <TD> Address: 0x200130A0 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x51                   /*!< Persistent: RFU; Session: Led mode control 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x51  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x51  </TD> 
   <TD> Address: 0x080810A2 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x51  </TD> 
   <TD COLSPAN=2> Led mode control (blinking)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x51  </TD> 
   <TD> Address: 0x200130A2 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x52                   /*!< Persistent: USB power cable disconnection; Session: Low speed oscillator frequency (0x52-0x53) 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x52  </TD> 
   <TD COLSPAN=2> Notification of USB power cable disconnection  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x52  </TD> 
   <TD> Address: 0x080810A4 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x52  </TD> 
   <TD COLSPAN=2> Low speed oscillator frequency (0x52-0x53)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x52  </TD> 
   <TD> Address: 0x200130A4 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x53                   /*!< Persistent: Battery charger control register; Session: Low speed oscillator frequency (0x52-0x53) 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x53  </TD> 
   <TD COLSPAN=2> Battery charger control register: 0 enable, 1 disable  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x53  </TD> 
   <TD> Address: 0x080810A6 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x53  </TD> 
   <TD COLSPAN=2> Low speed oscillator frequency (0x52-0x53)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x53  </TD> 
   <TD> Address: 0x200130A6 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x54                   /*!< Persistent: RFU; Session: Number of seconds after system boot (reg54->reg55) 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x54  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x54  </TD> 
   <TD> Address: 0x080810A8 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x54  </TD> 
   <TD COLSPAN=2> Number of milliseconds after system boot (reg54->reg55)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x54  </TD> 
   <TD> Address: 0x200130A8 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x55                   /*!< Persistent: Autosleep timer in seconds; Session: Number of seconds after system boot (reg54->reg55) 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x55  </TD> 
   <TD COLSPAN=2> Autosleep timer in seconds (0 to disable)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x55  </TD> 
   <TD> Address: 0x080810AA <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x55  </TD> 
   <TD COLSPAN=2> Number of milliseconds after system boot (reg54->reg55)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x55  </TD> 
   <TD> Address: 0x200130AA <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x56                   /*!< Persistent: BlueNRG random public address control; Session: Number of power read (reg56->reg57) 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x56  </TD> 
   <TD COLSPAN=2> BlueNRG random public address control (BLE_USE_RANDOM_ADDRESS_ALWAYS, BLE_USE_RANDOM_ADDRESS_IF_DEFAULT)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x56  </TD> 
   <TD> Address: 0x080810AC <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x56  </TD> 
   <TD COLSPAN=2> number of power read (reg56->reg57)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x56  </TD> 
   <TD> Address: 0x200130AC <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x57                   /*!< Persistent: Usb wakeup control register; Session: Number of power read (reg56->reg57) 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x57  </TD> 
   <TD COLSPAN=2> Usb wakeup control register: 0 enable, 1 disable   </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x57  </TD> 
   <TD> Address: 0x080810AE <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x57  </TD> 
   <TD COLSPAN=2> number of power read (reg56->reg57)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x57  </TD> 
   <TD> Address: 0x200130AE <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x58                   /*!< Persistent: Gas gauge power mode control register; Session: MCU wakeup source (reg58->reg59) 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x58  </TD> 
   <TD COLSPAN=2> Gas gauge power mode control register: 0 enable during low power mode, 1 shutdown during low power mode  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x58  </TD> 
   <TD> Address: 0x080810B0 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x58  </TD> 
   <TD COLSPAN=2> MCU wakeup source (reg58->reg59)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x58  </TD> 
   <TD> Address: 0x200130B0 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x59                   /*!< Persistent: Button press power mode control register; Session: MCU wakeup source (reg58->reg59) 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x59  </TD> 
   <TD COLSPAN=2> Button press power mode control register: can be one of WESU_SYS_POWER_* power mode  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x59  </TD> 
   <TD> Address: 0x080810B2 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x59  </TD> 
   <TD COLSPAN=2> MCU wakeup source (reg58->reg59)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x59  </TD> 
   <TD> Address: 0x200130B2 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x5A                   /*!< Persistent: Power mode to be entered on timeout when power mode is WESU_SYS_POWER_CONN_ONLY; Session: MCU operations before low power mode (reg5A->reg5B) 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x5A  </TD> 
   <TD COLSPAN=2> Power mode to be entered on timeout when power mode is WESU_SYS_POWER_CONN_ONLY  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x5A  </TD> 
   <TD> Address: 0x080810B4 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x5A  </TD> 
   <TD COLSPAN=2> MCU operations before low power mode (reg5A->reg5B)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x5A  </TD> 
   <TD> Address: 0x200130B4 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x5B                   /*!< Persistent: Power mode to be entered on timeout when AUTOSLEEP_TIME_REG is set; Session: MCU operations before low power mode (reg5A->reg5B) 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x5B  </TD> 
   <TD COLSPAN=2> Power mode to be entered on timeout when AUTOSLEEP_TIME_REG is set  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x5B  </TD> 
   <TD> Address: 0x080810B6 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x5B  </TD> 
   <TD COLSPAN=2> MCU operations before low power mode (reg5A->reg5B)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x5B  </TD> 
   <TD> Address: 0x200130B6 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x5C                   /*!< Persistent: RFU; Session: MCU low power mode 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x5C  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x5C  </TD> 
   <TD> Address: 0x080810B8 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x5C  </TD> 
   <TD COLSPAN=2> LSB MCU low power mode, MSB RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x5C  </TD> 
   <TD> Address: 0x200130B8 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x5D                   /*!< Persistent: RFU; Session: Application Timer control mask 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x5D  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x5D  </TD> 
   <TD> Address: 0x080810BA <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x5D  </TD> 
   <TD COLSPAN=2> Application Timer control mask: mcu timer or Gyroscope data-ready   </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x5D  </TD> 
   <TD> Address: 0x200130BA <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x5E                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x5E  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x5E  </TD> 
   <TD> Address: 0x080810BC <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x5E  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x5E  </TD> 
   <TD> Address: 0x200130BC <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x5F                   /*!< Persistent: RFU; Session: BlueNRG error monitoring on AHRS characteristic 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x5F  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x5F  </TD> 
   <TD> Address: 0x080810BE <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x5F  </TD> 
   <TD COLSPAN=2> BlueNRG error monitoring on AHRS characteristic  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x5F  </TD> 
   <TD> Address: 0x200130BE <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x60                   /*!< Persistent: Magnetometer calibration control register; Session: BlueNRG error monitoring on Motion characteristic 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x60  </TD> 
   <TD COLSPAN=2> Magnetometer calibration control register  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x60  </TD> 
   <TD> Address: 0x080810C0 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref FXMagCalibChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x60  </TD> 
   <TD COLSPAN=2> BlueNRG error monitoring on Motion characteristic  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x60  </TD> 
   <TD> Address: 0x200130C0 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref FXMagCalibChanged  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x61                   /*!< Persistent: RFU; Session: BlueNRG error monitoring on pressure characteristic 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x61  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x61  </TD> 
   <TD> Address: 0x080810C2 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x61  </TD> 
   <TD COLSPAN=2> BlueNRG error monitoring on pressure characteristic  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x61  </TD> 
   <TD> Address: 0x200130C2 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x62                   /*!< Persistent: Magnetometer calibration data; Session: BlueNRG error monitoring on power characteristic 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x62  </TD> 
   <TD COLSPAN=2> Magnetometer calibration data  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x62  </TD> 
   <TD> Address: 0x080810C4 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x62  </TD> 
   <TD COLSPAN=2> BlueNRG error monitoring on power characteristic  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x62  </TD> 
   <TD> Address: 0x200130C4 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x63                   /*!< Persistent: Magnetometer calibration data; Session: Counter to manage magnetometer calibration 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x63  </TD> 
   <TD COLSPAN=2> Magnetometer calibration data  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x63  </TD> 
   <TD> Address: 0x080810C6 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x63  </TD> 
   <TD COLSPAN=2> Counter to manage magnetometer calibration  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x63  </TD> 
   <TD> Address: 0x200130C6 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x64                   /*!< Persistent: Magnetometer calibration data; Session: Pressure output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x64  </TD> 
   <TD COLSPAN=2> Magnetometer calibration data  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x64  </TD> 
   <TD> Address: 0x080810C8 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x64  </TD> 
   <TD COLSPAN=2> Pressure output            (mbar) (reg64->reg65)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x64  </TD> 
   <TD> Address: 0x200130C8 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x65                   /*!< Persistent: Magnetometer calibration data; Session: Pressure output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x65  </TD> 
   <TD COLSPAN=2> Magnetometer calibration data  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x65  </TD> 
   <TD> Address: 0x080810CA <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x65  </TD> 
   <TD COLSPAN=2> Pressure output            (mbar) (reg64->reg65)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x65  </TD> 
   <TD> Address: 0x200130CA <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x66                   /*!< Persistent: Magnetometer calibration data; Session: Accelerometer output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x66  </TD> 
   <TD COLSPAN=2> Magnetometer calibration data  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x66  </TD> 
   <TD> Address: 0x080810CC <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x66  </TD> 
   <TD COLSPAN=2> Accelerometer output       (XXXXXX) (reg66->reg6B)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x66  </TD> 
   <TD> Address: 0x200130CC <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x67                   /*!< Persistent: Magnetometer calibration data; Session: Accelerometer output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x67  </TD> 
   <TD COLSPAN=2> Magnetometer calibration data  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x67  </TD> 
   <TD> Address: 0x080810CE <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x67  </TD> 
   <TD COLSPAN=2> Accelerometer output       (XXXXXX) (reg66->reg6B)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x67  </TD> 
   <TD> Address: 0x200130CE <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x68                   /*!< Persistent: Magnetometer calibration data; Session: Accelerometer output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x68  </TD> 
   <TD COLSPAN=2> Magnetometer calibration data  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x68  </TD> 
   <TD> Address: 0x080810D0 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x68  </TD> 
   <TD COLSPAN=2> Accelerometer output       (XXXXXX) (reg66->reg6B)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x68  </TD> 
   <TD> Address: 0x200130D0 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x69                   /*!< Persistent: Magnetometer calibration data; Session: Accelerometer output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x69  </TD> 
   <TD COLSPAN=2> Magnetometer calibration data  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x69  </TD> 
   <TD> Address: 0x080810D2 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x69  </TD> 
   <TD COLSPAN=2> Accelerometer output       (XXXXXX) (reg66->reg6B)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x69  </TD> 
   <TD> Address: 0x200130D2 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x6A                   /*!< Persistent: Magnetometer calibration data; Session: Accelerometer output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x6A  </TD> 
   <TD COLSPAN=2> Magnetometer calibration data  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x6A  </TD> 
   <TD> Address: 0x080810D4 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x6A  </TD> 
   <TD COLSPAN=2> Accelerometer output       (XXXXXX) (reg66->reg6B)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x6A  </TD> 
   <TD> Address: 0x200130D4 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x6B                   /*!< Persistent: Magnetometer calibration data; Session: Accelerometer output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x6B  </TD> 
   <TD COLSPAN=2> Magnetometer calibration data  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x6B  </TD> 
   <TD> Address: 0x080810D6 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x6B  </TD> 
   <TD COLSPAN=2> Accelerometer output       (XXXXXX) (reg66->reg6B)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x6B  </TD> 
   <TD> Address: 0x200130D6 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x6C                   /*!< Persistent: Magnetometer calibration data; Session: Gyroscope output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x6C  </TD> 
   <TD COLSPAN=2> Magnetometer calibration data  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x6C  </TD> 
   <TD> Address: 0x080810D8 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x6C  </TD> 
   <TD COLSPAN=2> Gyroscope output           (XXXXXX) (reg6C->reg71)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x6C  </TD> 
   <TD> Address: 0x200130D8 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x6D                   /*!< Persistent: Magnetometer calibration data; Session: Gyroscope output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x6D  </TD> 
   <TD COLSPAN=2> Magnetometer calibration data  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x6D  </TD> 
   <TD> Address: 0x080810DA <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x6D  </TD> 
   <TD COLSPAN=2> Gyroscope output           (XXXXXX) (reg6C->reg71)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x6D  </TD> 
   <TD> Address: 0x200130DA <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x6E                   /*!< Persistent: RFU; Session: Gyroscope output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x6E  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x6E  </TD> 
   <TD> Address: 0x080810DC <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x6E  </TD> 
   <TD COLSPAN=2> Gyroscope output           (XXXXXX) (reg6C->reg71)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x6E  </TD> 
   <TD> Address: 0x200130DC <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x6F                   /*!< Persistent: RFU; Session: Gyroscope output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x6F  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x6F  </TD> 
   <TD> Address: 0x080810DE <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x6F  </TD> 
   <TD COLSPAN=2> Gyroscope output           (XXXXXX) (reg6C->reg71)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x6F  </TD> 
   <TD> Address: 0x200130DE <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x70                   /*!< Persistent: BlueNRG Adv interval 1; Session: Gyroscope output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x70  </TD> 
   <TD COLSPAN=2> BlueNRG Adverising interval in run mode  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x70  </TD> 
   <TD> Address: 0x080810E0 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x70  </TD> 
   <TD COLSPAN=2> Gyroscope output           (XXXXXX) (reg6C->reg71)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x70  </TD> 
   <TD> Address: 0x200130E0 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x71                   /*!< Persistent: BlueNRG Adv interval 2; Session: Gyroscope output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x71  </TD> 
   <TD COLSPAN=2> BlueNRG Adverising interval in sleep mode  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x71  </TD> 
   <TD> Address: 0x080810E2 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x71  </TD> 
   <TD COLSPAN=2> Gyroscope output           (XXXXXX) (reg6C->reg71)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x71  </TD> 
   <TD> Address: 0x200130E2 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x72                   /*!< Persistent: RFU; Session: Magnetometer output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x72  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x72  </TD> 
   <TD> Address: 0x080810E4 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x72  </TD> 
   <TD COLSPAN=2> Magnetometer output        (XXXXXX) (reg72->reg77)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x72  </TD> 
   <TD> Address: 0x200130E4 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x73                   /*!< Persistent: RFU; Session: Magnetometer output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x73  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x73  </TD> 
   <TD> Address: 0x080810E6 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x73  </TD> 
   <TD COLSPAN=2> Magnetometer output        (XXXXXX) (reg72->reg77)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x73  </TD> 
   <TD> Address: 0x200130E6 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x74                   /*!< Persistent: Accelerometer FullScale; Session: Magnetometer output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x74  </TD> 
   <TD COLSPAN=2> Accelerometer FullScale  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x74  </TD> 
   <TD> Address: 0x080810E8 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref AccFsChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x74  </TD> 
   <TD COLSPAN=2> Magnetometer output        (XXXXXX) (reg72->reg77)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x74  </TD> 
   <TD> Address: 0x200130E8 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x75                   /*!< Persistent: Accelerometer OutputDataRate; Session: Magnetometer output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x75  </TD> 
   <TD COLSPAN=2> Accelerometer OutputDataRate  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x75  </TD> 
   <TD> Address: 0x080810EA <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref AccOdrChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x75  </TD> 
   <TD COLSPAN=2> Magnetometer output        (XXXXXX) (reg72->reg77)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x75  </TD> 
   <TD> Address: 0x200130EA <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x76                   /*!< Persistent: Gyroscope FullScale; Session: Magnetometer output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x76  </TD> 
   <TD COLSPAN=2> Gyroscope FullScale  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x76  </TD> 
   <TD> Address: 0x080810EC <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref GyroFsChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x76  </TD> 
   <TD COLSPAN=2> Magnetometer output        (XXXXXX) (reg72->reg77)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x76  </TD> 
   <TD> Address: 0x200130EC <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x77                   /*!< Persistent: Gyroscope OutputDataRate; Session: Magnetometer output 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x77  </TD> 
   <TD COLSPAN=2> Gyroscope OutputDataRate  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x77  </TD> 
   <TD> Address: 0x080810EE <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref GyroOdrChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x77  </TD> 
   <TD COLSPAN=2> Magnetometer output        (XXXXXX) (reg72->reg77)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x77  </TD> 
   <TD> Address: 0x200130EE <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x78                   /*!< Persistent: Magnetometer FullScale; Session: Temperature 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x78  </TD> 
   <TD COLSPAN=2> Magnetometer FullScale  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x78  </TD> 
   <TD> Address: 0x080810F0 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref MagFsChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x78  </TD> 
   <TD COLSPAN=2> Temperature                (XXXXXX) (reg78->reg79)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x78  </TD> 
   <TD> Address: 0x200130F0 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x79                   /*!< Persistent: Magnetometer OutputDataRate; Session: Temperature 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x79  </TD> 
   <TD COLSPAN=2> Magnetometer OutputDataRate  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x79  </TD> 
   <TD> Address: 0x080810F2 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref MagOdrChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x79  </TD> 
   <TD COLSPAN=2> Temperature                (XXXXXX) (reg78->reg79)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x79  </TD> 
   <TD> Address: 0x200130F2 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x7A                   /*!< Persistent: AccEvent config register; Session: Accelerometer Event Config 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x7A  </TD> 
   <TD COLSPAN=2> AccEvent configuration register  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x7A  </TD> 
   <TD> Address: 0x080810F4 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref AccEventConfChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x7A  </TD> 
   <TD COLSPAN=2> Accelerometer Event Config  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x7A  </TD> 
   <TD> Address: 0x200130F4 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x7B                   /*!< Persistent: Pressure OutputDataRate; Session: LSB Accelerometer Event status, MSB GP Algorithms status 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x7B  </TD> 
   <TD COLSPAN=2> Pressure OutputDataRate  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x7B  </TD> 
   <TD> Address: 0x080810F6 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref PressOdrChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x7B  </TD> 
   <TD COLSPAN=2> LSB Accelerometer Event status, MSB GP Algorithms status  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x7B  </TD> 
   <TD> Address: 0x200130F6 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x7C                   /*!< Persistent: RFU; Session: Step counter 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x7C  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x7C  </TD> 
   <TD> Address: 0x080810F8 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x7C  </TD> 
   <TD COLSPAN=2> Step counter, output of pedometer algorithm (reg7A->reg7B)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x7C  </TD> 
   <TD> Address: 0x200130F8 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x7D                   /*!< Persistent: Application Timer control mask; Session: Step counter 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x7D  </TD> 
   <TD COLSPAN=2> Application Timer control mask: mcu timer or Gyroscope data-ready   </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x7D  </TD> 
   <TD> Address: 0x080810FA <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x7D  </TD> 
   <TD COLSPAN=2> Step counter, output of pedometer algorithm (reg7A->reg7B)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x7D  </TD> 
   <TD> Address: 0x200130FA <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x7E                   /*!< Persistent: Battery low threshold; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x7E  </TD> 
   <TD COLSPAN=2> Battery low threshold  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x7E  </TD> 
   <TD> Address: 0x080810FC <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x7E  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x7E  </TD> 
   <TD> Address: 0x200130FC <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x7F                   /*!< Persistent: Battery very-low threshold; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x7F  </TD> 
   <TD COLSPAN=2> Battery very-low threshold  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x7F  </TD> 
   <TD> Address: 0x080810FE <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x7F  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x7F  </TD> 
   <TD> Address: 0x200130FE <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x80                   /*!< Persistent: GP Algorithm threshold 1; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x80  </TD> 
   <TD COLSPAN=2> GP Algorithm threshold 1  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x80  </TD> 
   <TD> Address: 0x08081100 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x80  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x80  </TD> 
   <TD> Address: 0x20013100 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x81                   /*!< Persistent: GP Algorithm threshold 2; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x81  </TD> 
   <TD COLSPAN=2> GP Algorithm threshold 2  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x81  </TD> 
   <TD> Address: 0x08081102 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x81  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x81  </TD> 
   <TD> Address: 0x20013102 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x82                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x82  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x82  </TD> 
   <TD> Address: 0x08081104 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x82  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x82  </TD> 
   <TD> Address: 0x20013104 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x83                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x83  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x83  </TD> 
   <TD> Address: 0x08081106 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x83  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x83  </TD> 
   <TD> Address: 0x20013106 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x84                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x84  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x84  </TD> 
   <TD> Address: 0x08081108 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x84  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x84  </TD> 
   <TD> Address: 0x20013108 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x85                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x85  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x85  </TD> 
   <TD> Address: 0x0808110A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x85  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x85  </TD> 
   <TD> Address: 0x2001310A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x86                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x86  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x86  </TD> 
   <TD> Address: 0x0808110C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x86  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x86  </TD> 
   <TD> Address: 0x2001310C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x87                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x87  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x87  </TD> 
   <TD> Address: 0x0808110E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x87  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x87  </TD> 
   <TD> Address: 0x2001310E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x88                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x88  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x88  </TD> 
   <TD> Address: 0x08081110 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x88  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x88  </TD> 
   <TD> Address: 0x20013110 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x89                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x89  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x89  </TD> 
   <TD> Address: 0x08081112 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x89  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x89  </TD> 
   <TD> Address: 0x20013112 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x8A                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x8A  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x8A  </TD> 
   <TD> Address: 0x08081114 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x8A  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x8A  </TD> 
   <TD> Address: 0x20013114 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x8B                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x8B  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x8B  </TD> 
   <TD> Address: 0x08081116 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x8B  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x8B  </TD> 
   <TD> Address: 0x20013116 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x8C                   /*!< Persistent: RFU; Session: LSB FX Calibration status, MSB FX license status 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x8C  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x8C  </TD> 
   <TD> Address: 0x08081118 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x8C  </TD> 
   <TD COLSPAN=2> LSB FX Calibration status, MSB FX license status  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x8C  </TD> 
   <TD> Address: 0x20013118 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x8D                   /*!< Persistent: RFU; Session: Motion Activity Recognition 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x8D  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x8D  </TD> 
   <TD> Address: 0x0808111A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x8D  </TD> 
   <TD COLSPAN=2> LSB Motion Activity Recognition Value, MSB Motion Activity Recognition Status  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x8D  </TD> 
   <TD> Address: 0x2001311A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x8E                   /*!< Persistent: RFU; Session: Motion Carry Position  
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x8E  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x8E  </TD> 
   <TD> Address: 0x0808111C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x8E  </TD> 
   <TD COLSPAN=2> LSB Motion Carry Position Value, MSB Motion Carry Position Status  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x8E  </TD> 
   <TD> Address: 0x2001311C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x8F                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x8F  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x8F  </TD> 
   <TD> Address: 0x0808111E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x8F  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x8F  </TD> 
   <TD> Address: 0x2001311E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x90                   /*!< Persistent: RTC Data; Session: RTC Data 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x90  </TD> 
   <TD COLSPAN=2> RTC Data: LSB Hours, MSB Minutes  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x90  </TD> 
   <TD> Address: 0x08081120 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref RtcConfChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x90  </TD> 
   <TD COLSPAN=2> RTC Data: LSB Hours, MSB Minutes  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x90  </TD> 
   <TD> Address: 0x20013120 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref RtcConfChanged  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x91                   /*!< Persistent: RTC Data; Session: RTC Data 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x91  </TD> 
   <TD COLSPAN=2> RTC Data: LSB Seconds, MSB Day  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x91  </TD> 
   <TD> Address: 0x08081122 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref RtcConfChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x91  </TD> 
   <TD COLSPAN=2> RTC Data: LSB Seconds, MSB Day  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x91  </TD> 
   <TD> Address: 0x20013122 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref RtcConfChanged  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x92                   /*!< Persistent: RTC Data; Session: RTC Data 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x92  </TD> 
   <TD COLSPAN=2> RTC Data: LSB Month, MSB Year  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x92  </TD> 
   <TD> Address: 0x08081124 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref RtcConfChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x92  </TD> 
   <TD COLSPAN=2> RTC Data: LSB Month, MSB Year  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x92  </TD> 
   <TD> Address: 0x20013124 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref RtcConfChanged  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x93                   /*!< Persistent: RTC Data/Conf; Session: RTC Data/Conf 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x93  </TD> 
   <TD COLSPAN=2> RTC Data/Conf: LSB Weekday (see \ref RTC_WeekDay_Definitions) MSB Configuration (see \ref RTC_AppConfig)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x93  </TD> 
   <TD> Address: 0x08081126 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref RtcConfChanged  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x93  </TD> 
   <TD COLSPAN=2> RTC Data/Conf: LSB Weekday (see \ref RTC_WeekDay_Definitions) MSB Configuration (see \ref RTC_AppConfig)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x93  </TD> 
   <TD> Address: 0x20013126 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref RtcConfChanged  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x94                   /*!< Persistent: RTC Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x94  </TD> 
   <TD COLSPAN=2> RTC Backup Data: LSB Hours, MSB Minutes  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x94  </TD> 
   <TD> Address: 0x08081128 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x94  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x94  </TD> 
   <TD> Address: 0x20013128 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x95                   /*!< Persistent: RTC Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x95  </TD> 
   <TD COLSPAN=2> RTC Backup Data: LSB Seconds, MSB Day  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x95  </TD> 
   <TD> Address: 0x0808112A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x95  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x95  </TD> 
   <TD> Address: 0x2001312A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x96                   /*!< Persistent: RTC Backup Data; Session: Reboot sensors  
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x96  </TD> 
   <TD COLSPAN=2> RTC Backup Data: LSB Month, MSB Year  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x96  </TD> 
   <TD> Address: 0x0808112C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x96  </TD> 
   <TD COLSPAN=2> Bitmask to reboot sensors at system startup  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x96  </TD> 
   <TD> Address: 0x2001312C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x97                   /*!< Persistent: RTC Backup Data/Conf; Session: BlueNRG flag for discoverable mode 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x97  </TD> 
   <TD COLSPAN=2> RTC Backup Data/Conf: LSB Weekday (see \ref RTC_WeekDay_Definitions) MSB Configuration (see \ref RTC_AppConfig)  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x97  </TD> 
   <TD> Address: 0x0808112E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x97  </TD> 
   <TD COLSPAN=2> LSB BlueNRG flag for discoverable mode, BlueNRG error flag; MSB  Bluenrg error flag, RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x97  </TD> 
   <TD> Address: 0x2001312E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x98                   /*!< Persistent: RFU; Session: Processing count 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x98  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x98  </TD> 
   <TD> Address: 0x08081130 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x98  </TD> 
   <TD COLSPAN=2> Processing count: number of calls to "User_Process()" function  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x98  </TD> 
   <TD> Address: 0x20013130 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x99                   /*!< Persistent: RFU; Session: Processing count 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x99  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x99  </TD> 
   <TD> Address: 0x08081132 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x99  </TD> 
   <TD COLSPAN=2> Processing count: number of calls to "User_Process()" function  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x99  </TD> 
   <TD> Address: 0x20013132 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x9A                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x9A  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x9A  </TD> 
   <TD> Address: 0x08081134 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x9A  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x9A  </TD> 
   <TD> Address: 0x20013134 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x9B                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x9B  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x9B  </TD> 
   <TD> Address: 0x08081136 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x9B  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x9B  </TD> 
   <TD> Address: 0x20013136 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x9C                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x9C  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x9C  </TD> 
   <TD> Address: 0x08081138 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x9C  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x9C  </TD> 
   <TD> Address: 0x20013138 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x9D                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x9D  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x9D  </TD> 
   <TD> Address: 0x0808113A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x9D  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x9D  </TD> 
   <TD> Address: 0x2001313A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x9E                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x9E  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x9E  </TD> 
   <TD> Address: 0x0808113C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x9E  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x9E  </TD> 
   <TD> Address: 0x2001313C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0x9F                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0x9F  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0x9F  </TD> 
   <TD> Address: 0x0808113E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0x9F  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0x9F  </TD> 
   <TD> Address: 0x2001313E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xA0                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xA0  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xA0  </TD> 
   <TD> Address: 0x08081140 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xA0  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xA0  </TD> 
   <TD> Address: 0x20013140 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xA1                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xA1  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xA1  </TD> 
   <TD> Address: 0x08081142 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xA1  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xA1  </TD> 
   <TD> Address: 0x20013142 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xA2                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xA2  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xA2  </TD> 
   <TD> Address: 0x08081144 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xA2  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xA2  </TD> 
   <TD> Address: 0x20013144 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xA3                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xA3  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xA3  </TD> 
   <TD> Address: 0x08081146 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xA3  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xA3  </TD> 
   <TD> Address: 0x20013146 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xA4                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xA4  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xA4  </TD> 
   <TD> Address: 0x08081148 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xA4  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xA4  </TD> 
   <TD> Address: 0x20013148 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xA5                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xA5  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xA5  </TD> 
   <TD> Address: 0x0808114A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xA5  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xA5  </TD> 
   <TD> Address: 0x2001314A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xA6                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xA6  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xA6  </TD> 
   <TD> Address: 0x0808114C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xA6  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xA6  </TD> 
   <TD> Address: 0x2001314C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xA7                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xA7  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xA7  </TD> 
   <TD> Address: 0x0808114E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xA7  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xA7  </TD> 
   <TD> Address: 0x2001314E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xA8                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xA8  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xA8  </TD> 
   <TD> Address: 0x08081150 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xA8  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xA8  </TD> 
   <TD> Address: 0x20013150 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xA9                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xA9  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xA9  </TD> 
   <TD> Address: 0x08081152 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xA9  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xA9  </TD> 
   <TD> Address: 0x20013152 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xAA                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xAA  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xAA  </TD> 
   <TD> Address: 0x08081154 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xAA  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xAA  </TD> 
   <TD> Address: 0x20013154 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xAB                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xAB  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xAB  </TD> 
   <TD> Address: 0x08081156 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xAB  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xAB  </TD> 
   <TD> Address: 0x20013156 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xAC                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xAC  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xAC  </TD> 
   <TD> Address: 0x08081158 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xAC  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xAC  </TD> 
   <TD> Address: 0x20013158 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xAD                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xAD  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xAD  </TD> 
   <TD> Address: 0x0808115A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xAD  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xAD  </TD> 
   <TD> Address: 0x2001315A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xAE                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xAE  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xAE  </TD> 
   <TD> Address: 0x0808115C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xAE  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xAE  </TD> 
   <TD> Address: 0x2001315C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xAF                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xAF  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xAF  </TD> 
   <TD> Address: 0x0808115E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xAF  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xAF  </TD> 
   <TD> Address: 0x2001315E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xB0                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xB0  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xB0  </TD> 
   <TD> Address: 0x08081160 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xB0  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xB0  </TD> 
   <TD> Address: 0x20013160 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xB1                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xB1  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xB1  </TD> 
   <TD> Address: 0x08081162 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xB1  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xB1  </TD> 
   <TD> Address: 0x20013162 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xB2                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xB2  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xB2  </TD> 
   <TD> Address: 0x08081164 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xB2  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xB2  </TD> 
   <TD> Address: 0x20013164 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xB3                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xB3  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xB3  </TD> 
   <TD> Address: 0x08081166 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xB3  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xB3  </TD> 
   <TD> Address: 0x20013166 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xB4                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xB4  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xB4  </TD> 
   <TD> Address: 0x08081168 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xB4  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xB4  </TD> 
   <TD> Address: 0x20013168 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xB5                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xB5  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xB5  </TD> 
   <TD> Address: 0x0808116A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xB5  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xB5  </TD> 
   <TD> Address: 0x2001316A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xB6                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xB6  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xB6  </TD> 
   <TD> Address: 0x0808116C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xB6  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xB6  </TD> 
   <TD> Address: 0x2001316C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xB7                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xB7  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xB7  </TD> 
   <TD> Address: 0x0808116E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xB7  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xB7  </TD> 
   <TD> Address: 0x2001316E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xB8                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xB8  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xB8  </TD> 
   <TD> Address: 0x08081170 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xB8  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xB8  </TD> 
   <TD> Address: 0x20013170 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xB9                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xB9  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xB9  </TD> 
   <TD> Address: 0x08081172 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xB9  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xB9  </TD> 
   <TD> Address: 0x20013172 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xBA                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xBA  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xBA  </TD> 
   <TD> Address: 0x08081174 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xBA  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xBA  </TD> 
   <TD> Address: 0x20013174 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xBB                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xBB  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xBB  </TD> 
   <TD> Address: 0x08081176 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xBB  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xBB  </TD> 
   <TD> Address: 0x20013176 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xBC                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xBC  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xBC  </TD> 
   <TD> Address: 0x08081178 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xBC  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xBC  </TD> 
   <TD> Address: 0x20013178 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xBD                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xBD  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xBD  </TD> 
   <TD> Address: 0x0808117A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xBD  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xBD  </TD> 
   <TD> Address: 0x2001317A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xBE                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xBE  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xBE  </TD> 
   <TD> Address: 0x0808117C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xBE  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xBE  </TD> 
   <TD> Address: 0x2001317C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xBF                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xBF  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xBF  </TD> 
   <TD> Address: 0x0808117E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xBF  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xBF  </TD> 
   <TD> Address: 0x2001317E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xC0                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xC0  </TD> 
   <TD COLSPAN=2> Backup Data, sytem ticks  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xC0  </TD> 
   <TD> Address: 0x08081180 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xC0  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xC0  </TD> 
   <TD> Address: 0x20013180 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xC1                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xC1  </TD> 
   <TD COLSPAN=2> Backup Data, sytem ticks  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xC1  </TD> 
   <TD> Address: 0x08081182 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xC1  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xC1  </TD> 
   <TD> Address: 0x20013182 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xC2                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xC2  </TD> 
   <TD COLSPAN=2> Backup Data, battery voltage  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xC2  </TD> 
   <TD> Address: 0x08081184 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xC2  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xC2  </TD> 
   <TD> Address: 0x20013184 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xC3                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xC3  </TD> 
   <TD COLSPAN=2> Backup Data, battery voltage  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xC3  </TD> 
   <TD> Address: 0x08081186 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xC3  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xC3  </TD> 
   <TD> Address: 0x20013186 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xC4                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xC4  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xC4  </TD> 
   <TD> Address: 0x08081188 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xC4  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xC4  </TD> 
   <TD> Address: 0x20013188 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xC5                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xC5  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xC5  </TD> 
   <TD> Address: 0x0808118A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xC5  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xC5  </TD> 
   <TD> Address: 0x2001318A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xC6                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xC6  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xC6  </TD> 
   <TD> Address: 0x0808118C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xC6  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xC6  </TD> 
   <TD> Address: 0x2001318C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xC7                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xC7  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xC7  </TD> 
   <TD> Address: 0x0808118E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xC7  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xC7  </TD> 
   <TD> Address: 0x2001318E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xC8                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xC8  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xC8  </TD> 
   <TD> Address: 0x08081190 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xC8  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xC8  </TD> 
   <TD> Address: 0x20013190 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xC9                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xC9  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xC9  </TD> 
   <TD> Address: 0x08081192 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xC9  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xC9  </TD> 
   <TD> Address: 0x20013192 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xCA                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xCA  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xCA  </TD> 
   <TD> Address: 0x08081194 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xCA  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xCA  </TD> 
   <TD> Address: 0x20013194 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xCB                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xCB  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xCB  </TD> 
   <TD> Address: 0x08081196 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xCB  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xCB  </TD> 
   <TD> Address: 0x20013196 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xCC                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xCC  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xCC  </TD> 
   <TD> Address: 0x08081198 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xCC  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xCC  </TD> 
   <TD> Address: 0x20013198 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xCD                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xCD  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xCD  </TD> 
   <TD> Address: 0x0808119A <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xCD  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xCD  </TD> 
   <TD> Address: 0x2001319A <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xCE                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xCE  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xCE  </TD> 
   <TD> Address: 0x0808119C <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xCE  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xCE  </TD> 
   <TD> Address: 0x2001319C <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xCF                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xCF  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xCF  </TD> 
   <TD> Address: 0x0808119E <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xCF  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xCF  </TD> 
   <TD> Address: 0x2001319E <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xD0                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xD0  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xD0  </TD> 
   <TD> Address: 0x080811A0 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xD0  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xD0  </TD> 
   <TD> Address: 0x200131A0 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xD1                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xD1  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xD1  </TD> 
   <TD> Address: 0x080811A2 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xD1  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xD1  </TD> 
   <TD> Address: 0x200131A2 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xD2                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xD2  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xD2  </TD> 
   <TD> Address: 0x080811A4 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xD2  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xD2  </TD> 
   <TD> Address: 0x200131A4 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xD3                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xD3  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xD3  </TD> 
   <TD> Address: 0x080811A6 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xD3  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xD3  </TD> 
   <TD> Address: 0x200131A6 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xD4                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xD4  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xD4  </TD> 
   <TD> Address: 0x080811A8 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xD4  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xD4  </TD> 
   <TD> Address: 0x200131A8 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xD5                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xD5  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xD5  </TD> 
   <TD> Address: 0x080811AA <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xD5  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xD5  </TD> 
   <TD> Address: 0x200131AA <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xD6                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xD6  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xD6  </TD> 
   <TD> Address: 0x080811AC <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xD6  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xD6  </TD> 
   <TD> Address: 0x200131AC <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xD7                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xD7  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xD7  </TD> 
   <TD> Address: 0x080811AE <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xD7  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xD7  </TD> 
   <TD> Address: 0x200131AE <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xD8                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xD8  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xD8  </TD> 
   <TD> Address: 0x080811B0 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xD8  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xD8  </TD> 
   <TD> Address: 0x200131B0 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xD9                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xD9  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xD9  </TD> 
   <TD> Address: 0x080811B2 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xD9  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xD9  </TD> 
   <TD> Address: 0x200131B2 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xDA                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xDA  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xDA  </TD> 
   <TD> Address: 0x080811B4 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xDA  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xDA  </TD> 
   <TD> Address: 0x200131B4 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xDB                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xDB  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xDB  </TD> 
   <TD> Address: 0x080811B6 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xDB  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xDB  </TD> 
   <TD> Address: 0x200131B6 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xDC                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xDC  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xDC  </TD> 
   <TD> Address: 0x080811B8 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xDC  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xDC  </TD> 
   <TD> Address: 0x200131B8 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xDD                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xDD  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xDD  </TD> 
   <TD> Address: 0x080811BA <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xDD  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xDD  </TD> 
   <TD> Address: 0x200131BA <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xDE                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xDE  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xDE  </TD> 
   <TD> Address: 0x080811BC <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xDE  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xDE  </TD> 
   <TD> Address: 0x200131BC <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xDF                   /*!< Persistent: Backup Data; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xDF  </TD> 
   <TD COLSPAN=2> Backup Data, Step counter  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xDF  </TD> 
   <TD> Address: 0x080811BE <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xDF  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xDF  </TD> 
   <TD> Address: 0x200131BE <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xE0                   /*!< Persistent: Step Counter: Monday LSB; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xE0  </TD> 
   <TD COLSPAN=2> Step Counter: Monday LSB  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xE0  </TD> 
   <TD> Address: 0x080811C0 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xE0  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xE0  </TD> 
   <TD> Address: 0x200131C0 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xE1                   /*!< Persistent: Step Counter: Monday MSB; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xE1  </TD> 
   <TD COLSPAN=2> Step Counter: Monday MSB  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xE1  </TD> 
   <TD> Address: 0x080811C2 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xE1  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xE1  </TD> 
   <TD> Address: 0x200131C2 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xE2                   /*!< Persistent: Step Counter: Tuesday LSB; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xE2  </TD> 
   <TD COLSPAN=2> Step Counter: Tuesday LSB  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xE2  </TD> 
   <TD> Address: 0x080811C4 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xE2  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xE2  </TD> 
   <TD> Address: 0x200131C4 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xE3                   /*!< Persistent: Step Counter: Tuesday MSB; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xE3  </TD> 
   <TD COLSPAN=2> Step Counter: Tuesday MSB  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xE3  </TD> 
   <TD> Address: 0x080811C6 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xE3  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xE3  </TD> 
   <TD> Address: 0x200131C6 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xE4                   /*!< Persistent: Step Counter: Wednesday LSB; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xE4  </TD> 
   <TD COLSPAN=2> Step Counter: Wednesday LSB  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xE4  </TD> 
   <TD> Address: 0x080811C8 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xE4  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xE4  </TD> 
   <TD> Address: 0x200131C8 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xE5                   /*!< Persistent: Step Counter: Wednesday MSB; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xE5  </TD> 
   <TD COLSPAN=2> Step Counter: Wednesday MSB  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xE5  </TD> 
   <TD> Address: 0x080811CA <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xE5  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xE5  </TD> 
   <TD> Address: 0x200131CA <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xE6                   /*!< Persistent: Step Counter: Thursday LSB; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xE6  </TD> 
   <TD COLSPAN=2> Step Counter: Thursday LSB  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xE6  </TD> 
   <TD> Address: 0x080811CC <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xE6  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xE6  </TD> 
   <TD> Address: 0x200131CC <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xE7                   /*!< Persistent: Step Counter: Thursday MSB; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xE7  </TD> 
   <TD COLSPAN=2> Step Counter: Thursday MSB  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xE7  </TD> 
   <TD> Address: 0x080811CE <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xE7  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xE7  </TD> 
   <TD> Address: 0x200131CE <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xE8                   /*!< Persistent: Step Counter: Friday LSB; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xE8  </TD> 
   <TD COLSPAN=2> Step Counter: Friday LSB  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xE8  </TD> 
   <TD> Address: 0x080811D0 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xE8  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xE8  </TD> 
   <TD> Address: 0x200131D0 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xE9                   /*!< Persistent: Step Counter: Friday MSB; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xE9  </TD> 
   <TD COLSPAN=2> Step Counter: Friday MSB  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xE9  </TD> 
   <TD> Address: 0x080811D2 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xE9  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xE9  </TD> 
   <TD> Address: 0x200131D2 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xEA                   /*!< Persistent: Step Counter: Saturday LSB; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xEA  </TD> 
   <TD COLSPAN=2> Step Counter: Saturday LSB  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xEA  </TD> 
   <TD> Address: 0x080811D4 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xEA  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xEA  </TD> 
   <TD> Address: 0x200131D4 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xEB                   /*!< Persistent: Step Counter: Saturday MSB; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xEB  </TD> 
   <TD COLSPAN=2> Step Counter: Saturday MSB  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xEB  </TD> 
   <TD> Address: 0x080811D6 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xEB  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xEB  </TD> 
   <TD> Address: 0x200131D6 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xEC                   /*!< Persistent: Step Counter: Sunday LSB; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xEC  </TD> 
   <TD COLSPAN=2> Step Counter: Sunday LSB  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xEC  </TD> 
   <TD> Address: 0x080811D8 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xEC  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xEC  </TD> 
   <TD> Address: 0x200131D8 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xED                   /*!< Persistent: Step Counter: Sunday MSB; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xED  </TD> 
   <TD COLSPAN=2> Step Counter: Sunday MSB  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xED  </TD> 
   <TD> Address: 0x080811DA <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xED  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xED  </TD> 
   <TD> Address: 0x200131DA <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xEE                   /*!< Persistent: Last saved Day; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xEE  </TD> 
   <TD COLSPAN=2> Last saved Day  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xEE  </TD> 
   <TD> Address: 0x080811DC <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xEE  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xEE  </TD> 
   <TD> Address: 0x200131DC <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xEF                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xEF  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xEF  </TD> 
   <TD> Address: 0x080811DE <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xEF  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xEF  </TD> 
   <TD> Address: 0x200131DE <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xF0                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xF0  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xF0  </TD> 
   <TD> Address: 0x080811E0 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xF0  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xF0  </TD> 
   <TD> Address: 0x200131E0 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref DummyRegChange  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xF1                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xF1  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xF1  </TD> 
   <TD> Address: 0x080811E2 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xF1  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xF1  </TD> 
   <TD> Address: 0x200131E2 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xF2                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xF2  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xF2  </TD> 
   <TD> Address: 0x080811E4 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xF2  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xF2  </TD> 
   <TD> Address: 0x200131E4 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: @ref PwrModeAltRegChange  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xF3                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xF3  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xF3  </TD> 
   <TD> Address: 0x080811E6 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xF3  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xF3  </TD> 
   <TD> Address: 0x200131E6 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xF4                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xF4  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xF4  </TD> 
   <TD> Address: 0x080811E8 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xF4  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xF4  </TD> 
   <TD> Address: 0x200131E8 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xF5                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xF5  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xF5  </TD> 
   <TD> Address: 0x080811EA <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xF5  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xF5  </TD> 
   <TD> Address: 0x200131EA <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xF6                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xF6  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xF6  </TD> 
   <TD> Address: 0x080811EC <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xF6  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xF6  </TD> 
   <TD> Address: 0x200131EC <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xF7                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xF7  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xF7  </TD> 
   <TD> Address: 0x080811EE <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xF7  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xF7  </TD> 
   <TD> Address: 0x200131EE <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xF8                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xF8  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xF8  </TD> 
   <TD> Address: 0x080811F0 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xF8  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xF8  </TD> 
   <TD> Address: 0x200131F0 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xF9                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xF9  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xF9  </TD> 
   <TD> Address: 0x080811F2 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xF9  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xF9  </TD> 
   <TD> Address: 0x200131F2 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xFA                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xFA  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xFA  </TD> 
   <TD> Address: 0x080811F4 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xFA  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xFA  </TD> 
   <TD> Address: 0x200131F4 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xFB                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xFB  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xFB  </TD> 
   <TD> Address: 0x080811F6 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xFB  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xFB  </TD> 
   <TD> Address: 0x200131F6 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xFC                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xFC  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xFC  </TD> 
   <TD> Address: 0x080811F8 <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xFC  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xFC  </TD> 
   <TD> Address: 0x200131F8 <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xFD                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xFD  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xFD  </TD> 
   <TD> Address: 0x080811FA <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xFD  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xFD  </TD> 
   <TD> Address: 0x200131FA <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xFE                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xFE  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xFE  </TD> 
   <TD> Address: 0x080811FC <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xFE  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xFE  </TD> 
   <TD> Address: 0x200131FC <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */

#define PERS_SESS_DESC_REG_0xFF                   /*!< Persistent: RFU; Session: RFU 
 <TABLE> 
  <TR BGCOLOR=orange> 
   <TD> Persistent register 0xFF  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref PERSISTENT_DEFAULT_REG_0xFF  </TD> 
   <TD> Address: 0x080811FE <BR> see \ref eeprommemorylayout "EEPROM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
  <TR BGCOLOR=yellow> 
   <TD> Session register 0xFF  </TD> 
   <TD COLSPAN=2> RFU  </TD> 
  </TR> 
  <TR> 
   <TD> Default value: @ref SESSION_DEFAULT_REG_0xFF  </TD> 
   <TD> Address: 0x200131FE <BR> see \ref rammemorylayout "RAM mapping"  </TD> 
   <TD> Data valid function: None <BR> Write action function: None  </TD> 
  </TR> 
 </TABLE> */


/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

  
    
#ifdef __cplusplus
}
#endif

#endif /* __WESU_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/                                                    
