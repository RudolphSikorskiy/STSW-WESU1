/**
  ******************************************************************************
  * @file    WESU_DEMO/Src/low_power.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   WeSU Low Power Management Application API
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

#include "low_power.h"
#include "BlueST_Protocol.h"
#include "wesu_config.h"
#include "stm32_bluenrg_ble.h"


/** @addtogroup WeSU_Demo       WeSU Demo      
  * @{
  */

/** @addtogroup WeSU_User               WeSU User
  * @{
  */

/** @addtogroup Low_Power               
  * @{
  * @brief WeSU Low Power API
  */

/** @defgroup Low_Power_Private_Variables     Low Power Private Variables                
  * @{
  */

static uint8_t nLedStatusBeforeLP = 0;          //!< Led Status Before Low Power 

/**
  * @}
  */

/** @defgroup Low_Power_Exported_Variables     Low Power Exported Variables                
  * @{
  */

uint8_t xCurSmFunc = WESU_SYS_POWER_UNKNOWN;                                    //!< current power state machine index

/**
  * @}
  */

/** @defgroup Low_Power_Private_Functions       Low Power Private Functions              
  * @{
  */

void ConfigAllGpioAsANInput(void);

/**
  * @}
  */

/** @defgroup Low_Power_Imported_Functions     Low Power Imported Functions                
  * @{
  */

extern void SystemClock_Config_RTC_HSE32MHz(void);
extern void SystemClock_Config_MSI_2MHz(void);

/**
  * @}
  */


/** @addtogroup Low_Power_Private_Functions                
  * @{
  */
void (* SystemClock_Config_APP )   (void) = SystemClock_Config_RTC_HSE32MHz;    //!< pointer to current clock configuration function



/**
 * @brief Configure All the STM32 GPIO such as Analog Input in order to reduce the consumption
 * @retval none
 */
void ConfigAllGpioAsANInput()
{
  /* Configure all GPIO as analog to reduce current consumption on non used IOs */
    GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable GPIOs clock */
  __GPIOA_CLK_ENABLE();__GPIOB_CLK_ENABLE();__GPIOC_CLK_ENABLE();__GPIOD_CLK_ENABLE();__GPIOE_CLK_ENABLE();__GPIOH_CLK_ENABLE();__GPIOF_CLK_ENABLE();__GPIOG_CLK_ENABLE();
    
  /* Configure the GPIO_LED pin */
  
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  if(*USB_WAKEUP_DISABLE_REGADDR==0)
  {
    GPIO_InitStruct.Pin &= ~( USB_PWR_GPIO_PIN );
  }
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  GPIO_InitStruct.Pin &= ~( GPIO_PIN_13 | GPIO_PIN_14 );
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  __GPIOA_CLK_DISABLE();__GPIOB_CLK_DISABLE();__GPIOC_CLK_DISABLE();__GPIOD_CLK_DISABLE();__GPIOE_CLK_DISABLE();__GPIOH_CLK_DISABLE();__GPIOF_CLK_DISABLE();__GPIOG_CLK_DISABLE();
}

/**
  * @}
  */


/** @addtogroup Low_Power_Exported_Functions               
  * @{
  */

/**
 * @brief Restore the configuration to work in Run MODE
 * @retval none
 */
void RestoreRunMode()
{
  HAL_Init();
  SystemClock_Config_APP();
  xCurSmFunc = WESU_SYS_POWER_UNKNOWN;
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  
  SENSORSSpiHandle.State = HAL_SPI_STATE_RESET;
  BSP_LED_InitGpio(LED);
  BSP_LED_InitGpio(RED_LED);
  if(nLedStatusBeforeLP)
  {
    APP_BSP_LED_OnPrivate(LED);
  }
  extern SPI_HandleTypeDef BNRGSpiHandle;
  BNRGSpiHandle.State = HAL_SPI_STATE_RESET;
  BNRG_SPI_Init();
  HCI_Process();
  SET_HIGH_BYTE(SESSION_REG(nRsvdDummyRegStc3115), APP_STC3115_KO);
  
  /* Initialize power management pins */
  BSP_PWMGT_Init();
  /* Initialize the Charger */
  BSP_CHARGER_Init();
  
#if USE_STC3115
  /* Initialize Gas Gauge */
  BSP_GG_IO_DeInit();
  BSP_GG_IO_Init();
#endif //USE_STC3115
  
  SESSION_REG(bLP) = 0;
}


/**
 * @brief Prepare different subpart before to enter ini Low Power MODE
 * @retval none
 */
void PrepareLowPowerMode()
{
  __disable_irq();      //To run with Keil
  
  if(SESSION_REG(nLPMask)&LPCFG_LED_OFF)
  {
    nLedStatusBeforeLP = BSP_LED_GetState(LED);
    APP_BSP_LED_Off(LED);
  }
  
  if(SESSION_REG(nLPMask)&LPCFG_LED_OFF_FORCED)
  {
    nLedStatusBeforeLP = BSP_LED_GetState(LED);
    APP_BSP_LED_OffPrivate(LED);
  }
  
  if(SESSION_REG(nLPMask)&LPCFG_GPIO_AN_IN)
  {
    BSP_PB_DeInit(BUTTON_USER);
    ConfigAllGpioAsANInput();
  }
  
  __enable_irq();       //To run with Keil
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

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
