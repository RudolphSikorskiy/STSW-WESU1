/**
  ******************************************************************************
  * @file    wesu_gg.h
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Header for wesu_gg.c module
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
#ifndef __WESU_BATTERY_GG_H
#define __WESU_BATTERY_GG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "platform_config.h"

   

/** @addtogroup BSP             BSP
  * @{
  */

/** @addtogroup STEVAL-WESU1            STEVAL-WESU1
  * @{
  */

/** @addtogroup PLATFORM        PLATFORM
  * @{
  */

/** @addtogroup WeSU_BATTERY_GAS_GAUGE          WeSU_BATTERY_GAS_GAUGE
  * @{
  */
  
/** @defgroup WeSU_BATTERY_GAS_GAUGE_Exported_Types WeSU_BATTERY_GAS_GAUGE_Exported_Types
  * @{
  */
   
/**
 * @brief  GG_StatusTypeDef
 */   
typedef enum 
{
  GG_OK = 0,            /*!< GAS GAUGE OK STATUS        */
  GG_ERROR = 1,         /*!< GAS GAUGE ERROR STATUS     */
  GG_TIMEOUT = 2        /*!< GAS GAUGE TIMEOUT STATUS   */
}GG_StatusTypeDef;

/**
  * @}
  */
  
/** @defgroup WeSU_BATTERY_GAS_GAUGE_Exported_Constants WeSU_BATTERY_GAS_GAUGE_Exported_Constants
  * @{
  */

#define APP_STC3115_KO  1       //!< STC3115 OK STATUS
#define APP_STC3115_OK  0       //!< STC3115 KO STATUS
    
 /* I2C clock speed configuration (in Hz) */
#ifndef WESU_PWR_I2C_SPEED
    #define WESU_PWR_I2C_SPEED    100000        //!< I2C SPEED FOR PWR Subsystem
#endif /* I2C_ONBOARD_SENSORS_SPEED */

/**
  * @}
  */

/** @defgroup WeSU_BATTERY_GAS_GAUGE_Exported_Defines WeSU_BATTERY_GAS_GAUGE_Exported_Defines
  * @{
  */
/* I2C peripheral configuration defines (control interface of the audio codec) */

  #define STC3115_PWR_I2C                       I2C1                    //!< uC I2C Instance for the PWR SubSystem
  #define STC3115_I2C_SCL_SDA_GPIO_PORT         GPIOB                   //!< STC3115_PWR_I2C SCL & SDA PORT
  #define STC3115_I2C_SCL_PIN                   GPIO_PIN_8              //!< STC3115_PWR_I2C SCL PIN
  #define STC3115_I2C_SDA_PIN                   GPIO_PIN_9              //!< STC3115_PWR_I2C SDA PIN
  #define STC3115_I2C_SCL_SDA_AF                GPIO_AF4_I2C1           //!< STC3115_PWR_I2C SCL & SDA Alternate Functions
  #define STC3115_I2C_CLK_ENABLE                __I2C1_CLK_ENABLE       //!< STC3115_PWR_I2C CLK ENABLE
  #define STC3115_I2C_SCL_SDA_GPIO_CLK_ENABLE() __GPIOB_CLK_ENABLE()    //!< STC3115_PWR_I2C SCL & SDA ENABLE
  #define STC3115_I2C_SCL_GPIO_CLK_DISABLE      __GPIOB_CLK_DISABLE()   //!< STC3115_PWR_I2C SCL DISABLE
  #define STC3115_I2C_SDA_GPIO_CLK_DISABLE      __GPIOB_CLK_DISABLE()   //!< STC3115_PWR_I2C SDA DISABLE
  #define STC3115_I2C_FORCE_RESET()             __I2C1_FORCE_RESET()    //!< STC3115_PWR_I2C FORCE RESET
  #define STC3115_I2C_RELEASE_RESET()           __I2C1_RELEASE_RESET()  //!< STC3115_PWR_I2C RELEASE RESET

  #define STC3115_I2C_DR                        ((uint32_t)0x40005410)  //!< STC3115_PWR_I2C DATA REGISTER
  #define STC3115_DMA_CLK                       RCC_AHBPeriph_DMA1      //!< STC3115 DMA CLK
  #define STC3115_DMA_TX_CHANNEL                DMA1_Channel6           //!< STC3115 DMA TX CHANNEL
  #define STC3115_DMA_RX_CHANNEL                DMA1_Channel7           //!< STC3115 DMA RX CHANNEL
  #define STC3115_DMA_TX_TCFLAG                 DMA1_FLAG_TC6           //!< STC3115 DMA TX FLAG         
  #define STC3115_DMA_RX_TCFLAG                 DMA1_FLAG_TC7           //!< STC3115 DMA RX FLAG   
   
/* I2C interrupt requests */
  #define STC3115_EV_IRQn               I2C1_EV_IRQn            //!< STC3115 I2C_EV IRQ Handler
  #define STC3115_ER_IRQn               I2C1_ER_IRQn            //!< STC3115 I2C_ER IRQ Handler
   
  #define WESU_PWR_I2C_TIMEOUT_MAX      0x1000                  //!< Max Timeout when I2C fails                   

                  
  /* Definition for GG interrupt Pins (STC3115_ALM) */
  #define GG_INT_GPIO_PORT                 GPIOB                //!< GAS GAUGE INT PORT
  #define GG_INT_PIN                      GPIO_PIN_15           //!< GAS GAUGE INT PIN
  #define GG_INT_EXTI_IRQn                EXTI15_10_IRQn        //!< GAS GAUGE INT IRQ HANDLER             
  #define GG_INT_GPIO_CLK_ENABLE()         __GPIOB_CLK_ENABLE() //!< GAS GAUGE ENABLING
  #define GG_INT_GPIO_CLK_DISABLE()        __GPIOB_CLK_DISABLE()//!< GAS GAUGE DISABLING
     
  #define GG_RSTIO_GPIO_PORT              GPIOB                 //!< GAS GAUGE RSTIO PORT
  #define GG_RSTIO_PIN                    GPIO_PIN_13           //!< GAS GAUGE RSTIO PIN   
  #define GG_RSTIO_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()  //!< GAS GAUGE RSTIO ENABLING
  #define GG_RSTIO_GPIO_CLK_DISABLE()     __GPIOB_CLK_DISABLE() //!< GAS GAUGE RSTIO DISABLING

  #define CHARGER_GPIO_CEN_PORT           GPIOC                 //!< GAS GAUGE CEN PORT
  #define CHARGER_GPIO_CEN_PIN            GPIO_PIN_1            //!< GAS GAUGE CEN PIN
  #define CHARGER_GPIO_CEN_CLK_ENABLE()   __GPIOC_CLK_ENABLE()  //!< GAS GAUGE CEN ENABLING
  #define CHARGER_GPIO_CEN_CLK_DISABLE()  __GPIOC_CLK_DISABLE() //!< GAS GAUGE CEN DISABLING
     
  #define CHARGER_GPIO_CHG_PORT           GPIOC                 //!< GAS GAUGE CHG PORT
  #define CHARGER_GPIO_CHG_PIN            GPIO_PIN_2            //!< GAS GAUGE CHG PIN
  #define CHARGER_GPIO_CHG_CLK_ENABLE()   __GPIOC_CLK_ENABLE()  //!< GAS GAUGE CHG ENABLING 
  #define CHARGER_GPIO_CHG_CLK_DISABLE()  __GPIOC_CLK_DISABLE() //!< GAS GAUGE CHG DISABLING

  #define POWER_USB_GPIO_PORT             GPIOC                 //!< POWER USB GPIO PORT
  #define POWER_USB_GPIO_PIN              GPIO_PIN_5            //!< POWER USB GPIO PIN
  #define POWER_USB_GPIO_CLK_ENABLE()     __GPIOC_CLK_ENABLE()  //!< POWER USB GPIO ENABLING
  #define POWER_USB_GPIO_CLK_DISABLE()    __GPIOC_CLK_DISABLE() //!< POWER USB GPIO DISABLING   

  #define PWMGT_SD_GPIO_PORT              GPIOC                 //!< POWER MNGMT SD PORT
  #define PWMGT_SD_GPIO_PIN               GPIO_PIN_4            //!< POWER MNGMT SD PIN
  #define PWMGT_SD_CLK_ENABLE()           __GPIOC_CLK_ENABLE()  //!< POWER MNGMT SD ENABLING
  #define PWMGT_SD_CLK_DISABLE()          __GPIOC_CLK_DISABLE() //!< POWER MNGMT SD DISABLING

/**
  * @}
  */
    
/** @defgroup WeSU_BATTERY_GAS_GAUGE_Exported_Functions_Prototypes WeSU_BATTERY_GAS_GAUGE_Exported_Functions_Prototypes
  * @{
  */
    
void BSP_GG_IO_Init(void);
void BSP_GG_IO_DeInit(void);
void BSP_GG_IO_ITConfig(void);
int BSP_GG_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
int BSP_GG_IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead);
uint32_t BSP_GG_IO_GetITState(void);

    
/* Gas Gauge Configuration Functions */ 
void BSP_GG_Init(void);
GG_StatusTypeDef BSP_GG_Task(void);
void BSP_GG_Reset(void);
uint8_t BSP_GG_Stop(void);
int BSP_GG_GetOCV(void);
int BSP_GG_GetSOC(void);
int BSP_GG_GetChargeValue(void);
int BSP_GG_GetPresence(void);
int BSP_GG_GetAlarmStatus(void);
int BSP_GG_GetCurrent(void);
int BSP_GG_GetTemperature(void);
int BSP_GG_GetVoltage(void);
int BSP_GG_GetITState(void);
uint8_t BSP_GG_AlarmSet(void);
uint8_t BSP_GG_AlarmStop(void);
uint8_t BSP_GG_AlarmGet(void);
uint8_t BSP_GG_AlarmClear(void);
uint8_t BSP_GG_SetVoltageThreshold(int Th);
uint8_t BSP_GG_SetSOCThreshold(int Th);
uint8_t BSP_GG_PowerSavingMode(void);

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
  
#endif /* __WESU_BATTERY_GG_H */
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
