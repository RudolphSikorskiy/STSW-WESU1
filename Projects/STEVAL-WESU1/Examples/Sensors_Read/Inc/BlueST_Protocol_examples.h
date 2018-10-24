/**
  ******************************************************************************
  * @file    Sensors_Read/Inc/BlueST_Protocol_examples.h
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
#include "main_examples.h"
   
   
/** @addtogroup WeSU_Examples        WeSU Examples       
  * @{
  */

/** @addtogroup WeSU_User_Examples               WeSU User Examples
  * @{
  */

/** @defgroup BlueST_PROTOCOL_Examples         BlueST PROTOCOL Examples
  * @{
  */   


tBleStatus Add_Feature_Service(void);
tBleStatus AccGyroMag_Update(AxesRaw_TypeDef *Acc,AxesRaw_TypeDef *Gyro,AxesRaw_TypeDef *Mag);
tBleStatus Temp_Update(int16_t temp);
tBleStatus Press_Update(int32_t press);
tBleStatus Pwr_Update(int16_t chg, int16_t voltage, int16_t current, uint8_t nPwrStat);

tBleStatus Algorithm1_Update(uint16_t xMCP_Value);

tBleStatus Add_Console_Service(void);
tBleStatus Stderr_Update(uint8_t *data,uint8_t length);
tBleStatus Term_Update(uint8_t *data,uint8_t length);

void HCI_Event_CB(void *pckt);


#define CONSOLE_MAX_CHAR_LEN 20                                                 //!< Define the Max dimension of the Bluetooth characteristics for each packet used for Console Service 

#define CONSOLE_END_STRING "\0"                                                 //!< Define the symbol used for defining each termination string used in Console service

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  #define ACC_BLUENRG_CONGESTION                                                //!< For enabling the capability to handle BlueNRG Congestion                                              

#ifdef ACC_BLUENRG_CONGESTION
  #define ACC_BLUENRG_CONGESTION_SKIP     3                                     //!< For defining how many Events skip when there is a congestion
#endif /* ACC_BLUENRG_CONGESTION */

#define SET_BNRG_ERROR_FLAG()   SESSION_REG(bBlueNRGErrorFlag)++
#define BNRG_MAX_ERRORS         20
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


#ifdef __cplusplus
}
#endif

#endif /* _BLUEST_PROTOCOL_H_ */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
