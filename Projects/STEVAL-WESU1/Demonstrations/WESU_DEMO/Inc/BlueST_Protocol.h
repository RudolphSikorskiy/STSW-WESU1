/**
  ******************************************************************************
  * @file    WESU_DEMO/Inc/BlueST_Protocol.h
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Header for BlueST_Protocol.c module
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
#ifndef _BLUEST_PROTOCOL_H_
#define _BLUEST_PROTOCOL_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "hal_types.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "string.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "hci_const.h"
#include "gp_timer.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"
#include "hci.h"
#include "hal.h"
#include "sm.h"

#include <stdlib.h>
#include "main.h"
   
   
/** @addtogroup WeSU_Demo       WeSU Demo      
  * @{
  */

/** @addtogroup WeSU_User               WeSU User
  * @{
  */

/** @defgroup BlueST_PROTOCOL         BlueST PROTOCOL
  * @{
  */

/** @defgroup BlueST_PROTOCOL_Private_Types BlueST PROTOCOL Private Types
 * @{
 */

/**
  * @brief Register management frame structure: send commands via BlueST protocol Config characteristic to read/write session/permanen registers
  */
typedef struct sRegFrameStruct
{
  uint8_t nControlByte;                                 //!< Control byte for frame management
  uint8_t nRegAddress;                                  //!< Address of the register to be managed by the frame
  uint8_t nErrorCode;                                   //!< Last error code
  uint8_t nPayloadLenght;                               //!< Lenght of the payload (number of registers)
  uint8_t pPayload[0];                                  //!< Pointer to the payload
} RegFrameStruct_t;

/**
  * @}
  */

/** @defgroup BlueST_PROTOCOL_Exported_Functions    BlueST PROTOCOL Exported Functions
  * @{
  */

tBleStatus Add_Feature_Service(void);
tBleStatus AccGyroMag_Update(AxesRaw_TypeDef *Acc,AxesRaw_TypeDef *Gyro,AxesRaw_TypeDef *Mag);
tBleStatus Temp_Update(int16_t temp);
tBleStatus Press_Update(int32_t press);
tBleStatus Pwr_Update(int16_t chg, int16_t voltage, int16_t current, uint8_t nPwrStat);
#if USE_QUATERNION_FLOAT
tBleStatus Quat_Float_Update (uint32_t *data);
#endif /* USE_QUATERNION_FLOAT */ 
tBleStatus Quat_Update(SensorAxes_t *data);

tBleStatus Algorithm3_Update(uint32_t nSteps, uint16_t nFreq);
tBleStatus Algorithm4_Update(uint8_t nFreeFall);
tBleStatus AccEvent_Update(uint8_t nAccEvStatus,uint16_t nStepCounter);
tBleStatus GP_ALGORITHM_Update(uint8_t nStatus);

tBleStatus Add_Console_Service(void);
tBleStatus Stderr_Update(uint8_t *data,uint8_t length);
tBleStatus Term_Update(uint8_t *data,uint8_t length);

tBleStatus Algorithm1_Update(uint16_t xMAR_Value);
tBleStatus Algorithm2_Update(uint16_t xMCP_Value);
tBleStatus Add_Config_Service(void);

void UpdateConnectionParameters(void);
void BluenrgResetParameters(void);

void BSP_REGS_Manage(uint8_t optype, uint8_t nReg, uint8_t *pPayload, uint32_t nLen);
void BSP_REGS_ManageWAction(uint8_t optype, uint8_t nReg, uint8_t *pPayload, uint32_t nLen);

void BSP_LICENSE_Read(uint8_t nLic, uint8_t *pPayload);
void BSP_LICENSE_Write(uint8_t nLic, uint8_t *pPayload);

void BSP_PERSREGS_WriteBackup(uint8_t nReg, uint8_t *pPayload, uint32_t nLen);
void BSP_PERSREGS_ReadBackup(uint8_t nReg, uint8_t *pPayload, uint32_t nLen);

void HCI_Event_CB(void *pckt);

/**
  * @}
  */


/** @addtogroup BlueST_PROTOCOL_Exported_Defines    BlueST PROTOCOL Exported Defines
  * @{
  */

#define REGS_MANAGE_READ                0x00    //!< Bit Mask to control a READ Action
#define REGS_MANAGE_WRITE               0x01    //!< Bit Mask to control a WRITE Action
#define REGS_MANAGE_PERSISTENT          0x10    //!< Bit Mask to control an Action on a Persistent Register
#define REGS_MANAGE_SESSION             0x00    //!< Bit Mask to control an Action on a Session Register

#define BSP_PERSREGS_READ(R,P,L)        BSP_REGS_Manage(REGS_MANAGE_PERSISTENT|REGS_MANAGE_READ,R,P,L)  //!< Read Macro for Persistent Regs
#define BSP_PERSREGS_WRITE(R,P,L)       BSP_REGS_Manage(REGS_MANAGE_PERSISTENT|REGS_MANAGE_WRITE,R,P,L) //!< Write Macro for Persistent Regs
#define BSP_SESSIONREGS_READ(R,P,L)     BSP_REGS_Manage(REGS_MANAGE_SESSION|REGS_MANAGE_READ,R,P,L)     //!< Read Macro for Session Regs
#define BSP_SESSIONREGS_WRITE(R,P,L)    BSP_REGS_Manage(REGS_MANAGE_SESSION|REGS_MANAGE_WRITE,R,P,L)    //!< Write Macro for Session Regs

#define BSP_PERSREGS_READ_WACTION(R,P,L)        BSP_REGS_ManageWAction(REGS_MANAGE_PERSISTENT|REGS_MANAGE_READ,R,P,L)   //!< Read Macro for Pers Regs, triggering associated actions
#define BSP_PERSREGS_WRITE_WACTION(R,P,L)       BSP_REGS_ManageWAction(REGS_MANAGE_PERSISTENT|REGS_MANAGE_WRITE,R,P,L)  //!< Write Macro for Pers Regs, triggering associated actions
#define BSP_SESSIONREGS_READ_WACTION(R,P,L)     BSP_REGS_ManageWAction(REGS_MANAGE_SESSION|REGS_MANAGE_READ,R,P,L)      //!< Read Macro for Session Regs, triggering associated actions
#define BSP_SESSIONREGS_WRITE_WACTION(R,P,L)    BSP_REGS_ManageWAction(REGS_MANAGE_SESSION|REGS_MANAGE_WRITE,R,P,L)     //!< Write Macro for Session Regs, triggering associated actions

#define BSP_PERSREGS_WRITE_BACKUP(R,P,L)        BSP_PERSREGS_WriteBackup(R,P,L)                                         //!< Write Macro for Backup Persistent Regs
#define BSP_PERSREGS_READ_BACKUP(R,P,L)         BSP_PERSREGS_ReadBackup(R,P,L)                                          //!< Read Macro for Backup Persistent Regs

#define CONSOLE_MAX_CHAR_LEN 20                                                 //!< Define the Max dimension of the Bluetooth characteristics for each packet used for Console Service

#define CONSOLE_END_STRING "\0"                                                 //!< Define the symbol used for defining each termination string used in Console service

/* Define How Many quaterions you want to trasmit (from 1 to 3) */
#define SEND_N_QUATERNIONS 3

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  #define ACC_BLUENRG_CONGESTION                                                //!< For enabling the capability to handle BlueNRG Congestion

#ifdef ACC_BLUENRG_CONGESTION
  #define ACC_BLUENRG_CONGESTION_SKIP     3                                     //!< For defining how many Events skip when there is a congestion
#endif /* ACC_BLUENRG_CONGESTION */

#define SET_BNRG_ERROR_FLAG()   SESSION_REG(bBlueNRGErrorFlag)++
#define BNRG_MAX_ERRORS         50
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

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

#endif /* _BLUEST_PROTOCOL_H_ */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
