/**
  ******************************************************************************
  * @file    wesu_charger.h
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Header for wesu_charger.c module
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
#ifndef __WESU_BATTERY_CHARGER_H
#define __WESU_BATTERY_CHARGER_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

/** @addtogroup BSP     BSP
  * @{
  */
  
/** @addtogroup STEVAL-WESU1            STEVAL-WESU1
  * @{
  */

/** @addtogroup PLATFORM        PLATFORM
  * @{
  */

/** @addtogroup WeSU_BATTERY_CHARGER    WeSU_BATTERY_CHARGER
  * @{
  */ 


/** @defgroup WeSU_BATTERY_CHARGER_Exported_Types       WeSU_BATTERY_CHARGER_Exported_Types
  * @{
  */

/**
  * @}
  */ 

/** @defgroup WeSU_BATTERY_CHARGER_Exported_Constants   WeSU_BATTERY_CHARGER_Exported_Constants
  * @{
  */

/**
 * @brief  TYPEDEF to define the charger condition  
 */   
typedef enum charger_conditions 
{
  CHARGER_DISCHARGING=0x01,
  CHARGER_CHARGE_DONE=0x02,
  CHARGER_CHARGE_IN_PROGRESS=0x03,
  CHARGER_IN_STAND_BY=0x04,
  CHARGER_INVALID_STATE=0xFF
} charger_conditions_t;

/**
 * @brief  TYPEDEF to control the usb cable  
 */
typedef enum usb_cable_condition
{
  USB_CABLE_NOT_CONNECTED = 0,
  USB_CABLE_CONNECTED
}usb_cable_condition_t;

/**
  * @}
  */

/** @defgroup WeSU_BATTERY_CHARGER_Exported_Functions   WeSU_BATTERY_CHARGER_Exported_Functions
  * @{
  */   
extern void BSP_CHARGER_Init(void);
extern void BSP_PWMGT_Init(void);
extern charger_conditions_t BSP_CHARGER_GetState(void);
extern void BSP_PWMGT_BoardShutdown();

/* Link function for CHARGER peripheral */
extern GPIO_PinState BSP_CHARGER_IO_GetGPIOState();
extern void BSP_BatteryChargerTask();

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
  
/**
  * @}
  */ 
  
#ifdef __cplusplus
}
#endif

#endif /* __WESU_BATTERY_CHARGER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
