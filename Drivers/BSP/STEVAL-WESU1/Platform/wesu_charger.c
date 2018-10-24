/**
  ******************************************************************************
  * @file    wesu_charger.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   This file includes the driver for battery charger STNS01
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
    
#include <stdint.h>
#include "wesu_gg.h"

/** @addtogroup BSP     BSP
  * @{
  */

/** @addtogroup STEVAL-WESU1    STEVAL-WESU1
  * @{
  */

/** @addtogroup PLATFORM        PLATFORM
  * @{
  */

/** @addtogroup WeSU_BATTERY_CHARGER    WeSU_BATTERY_CHARGER
  * @brief BSP functions implementation for WESU Battery Charger
  * @{
  */ 


/** @defgroup WeSU_BATTERY_CHARGER_Private_Defines   WeSU_BATTERY_CHARGER_Private_Defines
  * @{
  */

#define RESTART_CHARGER_INTERVAL                10*60*1000      /*!< Interval inside the charger task */
#define RESTART_CHARGER_OFF_INTERVAL            1000            /*!< Interval OFF in the charger task */
#define RESTART_CHARGER_INTERVAL_STARTUP        20*1000         /*!< Startup Interval in the charger task */         

/**
  * @}
  */

/** @defgroup WeSU_BATTERY_CHARGER_Exported_Functions   WeSU_BATTERY_CHARGER_Exported_Functions
  * @{
  */

/**
  * @brief  Enables the STNS01 Battery Charger CEN Pin.
  * @retval None
  */
void BSP_CHARGER_Enable()
{
  HAL_GPIO_WritePin(CHARGER_GPIO_CEN_PORT, CHARGER_GPIO_CEN_PIN, GPIO_PIN_SET);
}

/**
  * @brief  Disables the STNS01 Battery Charger CEN Pin.
  * @retval None
  */
void BSP_CHARGER_Disable()
{
  HAL_GPIO_WritePin(CHARGER_GPIO_CEN_PORT, CHARGER_GPIO_CEN_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  Initializes the STNS01 Battery Charger.
  * @retval None
  */
void BSP_CHARGER_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* CEN & CHG GPIO Ports Clock Enable */
    CHARGER_GPIO_CEN_CLK_ENABLE();CHARGER_GPIO_CHG_CLK_ENABLE();POWER_USB_GPIO_CLK_ENABLE();

    /* CEN High to Enable STNS01 Charger */
    HAL_GPIO_WritePin(CHARGER_GPIO_CEN_PORT, CHARGER_GPIO_CEN_PIN, GPIO_PIN_SET);
    
    /* No need to configure CHG GPIO pin, it is configured by BSP_LED_InitGpio in order for the mcu to drive the red led */
    /* Configure CHG GPIO pins : PC2 */
    GPIO_InitStruct.Pin = CHARGER_GPIO_CHG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(CHARGER_GPIO_CHG_PORT, &GPIO_InitStruct);
    
    /* Configure Usb power pin : PC5 */
    GPIO_InitStruct.Pin = POWER_USB_GPIO_PIN;
    HAL_GPIO_Init(POWER_USB_GPIO_PORT, &GPIO_InitStruct);

    /* Configure CEN & SD GPIO pin : PC1 & PC4 */
    GPIO_InitStruct.Pin = CHARGER_GPIO_CEN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(CHARGER_GPIO_CEN_PORT, &GPIO_InitStruct);
    
}

/**
  * @brief  Initializes the STNS01 SD Pin.
  * @retval None
  */
void BSP_PWMGT_Init()
{
  GPIO_InitTypeDef   GPIO_InitStructure;
  
  PWMGT_SD_CLK_ENABLE();
  
  /*!< Configure Shutdown pin for STNS01 */
  GPIO_InitStructure.Pin = PWMGT_SD_GPIO_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(PWMGT_SD_GPIO_PORT, &GPIO_InitStructure);

  /* SD Low to Enable STNS01 Power Path */
  HAL_GPIO_WritePin(PWMGT_SD_GPIO_PORT, PWMGT_SD_GPIO_PIN, GPIO_PIN_RESET);  
  
}

/**
  * @brief  Force the STNS01 ShutDown putting the SD Pin to High Level.
  * @retval None
  */
void BSP_PWMGT_BoardShutdown()
{
  HAL_GPIO_WritePin(PWMGT_SD_GPIO_PORT, PWMGT_SD_GPIO_PIN, GPIO_PIN_SET);
}

/**
  * @brief  Provide the Battery Charger Task.
  * @retval None
  */
void BSP_BatteryChargerTask()
{
  /* force STNS01 restart after 10 minutes */
  static uint32_t nTicksRestartCharger = RESTART_CHARGER_INTERVAL_STARTUP;
  
  if(HAL_GetTick()>=nTicksRestartCharger)
  {
    if( BSP_PwrUsbMonitor_GetState() != USB_CABLE_NOT_CONNECTED )
    {
      if( HAL_GetTick() > nTicksRestartCharger + RESTART_CHARGER_OFF_INTERVAL )
      {
        BSP_CHARGER_Enable();
        nTicksRestartCharger = HAL_GetTick() + RESTART_CHARGER_INTERVAL;
      }
      else
      {
        BSP_CHARGER_Disable();
      }
    }
  }
}

/**
  * @brief  Get current state reading the CHG PIN.
  * @retval CHARGER_CHARGE_IN_PROGRESS: charge is in progress,
  *         CHARGER_CHARGE_DONE: charge done,
  */
charger_conditions_t BSP_CHARGER_GetState(void) 
{
  charger_conditions_t temp_nPowerMode = CHARGER_INVALID_STATE;
  
  if(BSP_PwrUsbMonitor_GetState() == USB_CABLE_NOT_CONNECTED)
  {
    temp_nPowerMode= CHARGER_DISCHARGING;
  }
  else
  {
    GPIO_PinState state_chg = BSP_CHARGER_IO_GetGPIOState();
    
    if((state_chg == GPIO_PIN_RESET))
      temp_nPowerMode = CHARGER_CHARGE_IN_PROGRESS;
    
    else if((state_chg == GPIO_PIN_SET))
      temp_nPowerMode = CHARGER_CHARGE_DONE;
    
    else  
      temp_nPowerMode = CHARGER_INVALID_STATE;
  }
  return temp_nPowerMode;  
}


/**
 * @brief Get GPIO Status
 * @retval The Charger GPIO pin value
 */
GPIO_PinState BSP_CHARGER_IO_GetGPIOState()
{
  return HAL_GPIO_ReadPin(CHARGER_GPIO_CHG_PORT, CHARGER_GPIO_CHG_PIN);
}


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

/************************ (C) COPYRIGHT STMicroelectronics  *****END OF FILE****/
