/**
  ******************************************************************************
  * @file    wesu_gg.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   This file provides a set of functions needed to manage 
  *          the Gas Gauge sensor.
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

/* Includes ------------------------------------------------------------------*/
#include "wesu_gg.h"
#include "stc3115_Driver.h"
#include "stc3115.h"

//#include "wesu_charger.h"
//#include "wesu_rtc.h"

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
 * @brief BSP functions implementation for STEVAL-WESU1 Gas Gauge
 * @{
 */


/** @defgroup WeSU_BATTERY_GAS_GAUGE_Private_Defines      WeSU_BATTERY_GAS_GAUGE_Private_Defines
 * @{
 */
#ifndef NULL
  #define NULL      (void *) 0  //!< NULL DEFINE
#endif
/**
 * @}
 */


/** @defgroup WeSU_BATTERY_GAS_GAUGE_Private_Macros     WeSU_BATTERY_GAS_GAUGE_Private_Macros
 * @{
 */
#define GgDrv (&stc3115_drv)    //!< Gas Gauge Handler

/**
 * @}
 */


/** @defgroup WeSU_BATTERY_GAS_GAUGE_Private_Variables  WeSU_BATTERY_GAS_GAUGE_Private_Variables
 * @{
 */

uint32_t PWR_I2C_Timeout = WESU_PWR_I2C_TIMEOUT_MAX;    //!< Max Timeout when I2C fails    
static I2C_HandleTypeDef    PWR_I2C_Handle;             //!< I2C Handler
static uint16_t nReservedDummyRegStc3115 = 0x0000;      //!< STC3115 Regs
	
/**
 * @}
 */

/** @defgroup WeSU_BATTERY_GAS_GAUGE_Private_FunctionPrototypes WeSU_BATTERY_GAS_GAUGE_Private_FunctionPrototypes
 * @{
 */
static void PWR_I2C_MspInit(void);
static void PWR_I2C_Error(void);
static void PWR_I2C_Init(void); 
static void PWR_I2C_DeInit(void); 


/**
 * @}
 */

/** @defgroup WeSU_BATTERY_GAS_GAUGE_Exported_Functions         WeSU_BATTERY_GAS_GAUGE_Exported_Functions
 * @{
 */

/**
 * @brief  Set GG Initialization.
 * @retval GG_OK if no problem during initialization
 */
void BSP_GG_Init(void)
{
  GgDrv->Init();
}


/**
 * @brief  Task to read periodically data of STC3115
 *  
 * @retval GG_ERROR or GG_OK
 */
GG_StatusTypeDef BSP_GG_Task()
{
  GG_StatusTypeDef ret = GG_ERROR; 
  
  if(GgDrv->Task != NULL)
  {
    int taskret = -1;
    taskret = GgDrv->Task();
  
    if(taskret < 0)
    {
      ret = GG_ERROR;
      SET_HIGH_BYTE(nReservedDummyRegStc3115, APP_STC3115_KO);
      SET_LOW_BYTE(nReservedDummyRegStc3115, LOW_BYTE(nReservedDummyRegStc3115)+1);
    }
    else
    { 
      ret = GG_OK;
      SET_HIGH_BYTE(nReservedDummyRegStc3115, APP_STC3115_OK);
    }
   }
 return ret;
}

/**
 * @brief  Reboot memory content of GG
 *  
 * @retval None
 */
void BSP_GG_Reset(void)
{  
    if(GgDrv->Reset != NULL)
    {
        GgDrv->Reset();
    }
}


/**
 * @brief  Stop the gas gauge system
 *  
 * @retval GG_ERROR or GG_OK
 */
uint8_t BSP_GG_Stop(void)
{  
   uint8_t ret = GG_ERROR;
   
    if(GgDrv->Stop != NULL)
    {
      ret = GgDrv->Stop();
    }
  return ret;
}


/**
 * @brief  Force the gas gauge system to PowerSavingMode
 *  
 * @retval GG_ERROR or GG_OK
 */
uint8_t BSP_GG_PowerSavingMode(void)
{
   uint8_t ret = GG_ERROR;
   
    if(GgDrv->PowerSavingMode != NULL)
    {
      ret = GgDrv->PowerSavingMode();
    }
    
    return ret;
}


/**
 * @brief  Get the OCV
 *  
 * @retval value
 */
int BSP_GG_GetOCV(void)
{ 
   uint32_t value = 0;
 
    if(GgDrv->GetOCV != NULL)
    {
        value = GgDrv->GetOCV();
    }

   return value;
}


/**
 * @brief  Get the SOC
 *  
 * @retval value
 */
int BSP_GG_GetSOC(void)
{ 
   uint32_t value = 0;
 
    if(GgDrv->GetSOC != NULL)
    {
        value = GgDrv->GetSOC();
    }

   return value;
}


/**
 * @brief  Get the charge value
 *  
 * @retval value
 */
int BSP_GG_GetChargeValue(void)
{ 
   uint32_t value = 0;
 
    if(GgDrv->GetChargeValue != NULL)
    {
        value = GgDrv->GetChargeValue();
    }

   return value;
}


/**
 * @brief  Get the presence
 *  
 * @retval value
 */
int BSP_GG_GetPresence(void)
{ 
   uint8_t value = 0;
 
    if(GgDrv->GetPresence != NULL)
    {
        value = GgDrv->GetPresence();
    }

   return value;
}


/**
 * @brief  Get the alarm status
 *  
 * @retval value
 */
int BSP_GG_GetAlarmStatus(void)
{ 
   uint32_t value = 0;
 
    if(GgDrv->GetAlarmStatus != NULL)
    {
        value = GgDrv->GetAlarmStatus();
    }

   return value;
}


/**
 * @brief  Get the IT state
 *  
 * @retval value
 */
int BSP_GG_GetITState(void)
{ 
   uint8_t value = 0;
 
    if(GgDrv->GetITState != NULL)
    {
        value = GgDrv->GetITState();
    }

   return value;
}


/**
 * @brief  Get the current
 *  
 * @retval Current value
 */
int BSP_GG_GetCurrent(void)
{  
   int32_t value = 0;

    if(GgDrv->GetCurrent != NULL)
    {
        value = GgDrv->GetCurrent();
    }
	
   return value;
}


/**
 * @brief  Get the voltage
 *  
 * @retval Value
 */
int BSP_GG_GetVoltage(void)
{ 
   uint32_t value = 0;
 
    if(GgDrv->GetVoltage != NULL)
    {
        value = GgDrv->GetVoltage();
    }

   return value;
}


/**
 * @brief  Get the temperature
 *  
 * @retval Value
 */
int BSP_GG_GetTemperature(void)
{ 
   int32_t value = 0;
 
    if(GgDrv->GetTemperature != NULL)
    {
        value = GgDrv->GetTemperature();
    }

   return value;
}


/**
 * @brief  Set the INT
 * @retval ret: (uint8_t)GG_StatusTypeDef
 */
uint8_t BSP_GG_AlarmSet()
{
   uint8_t ret = GG_ERROR;
   
    if(GgDrv->SetIT != NULL)
    {
        ret = GgDrv->SetIT();
    }
  return ret;
}


/**
 * @brief  Disable INT
 * @retval ret: GG_OK or GG_ERROR
 */
uint8_t BSP_GG_AlarmStop()
{
   uint8_t ret = GG_ERROR;
   
    if(GgDrv->StopIT != NULL)
    {
      ret = GgDrv->StopIT();
    }

   return ret;
}


/**
 * @brief  Get the INT status
 * @retval ALM status 00 : no alarm 
 *                    01 : SOC alarm
 *                    10 : Voltage alarm
 *                    11 : SOC and voltage alarm
 */
uint8_t BSP_GG_AlarmGet(void)
{
   uint8_t value = 0;
   
    if(GgDrv->GetIT != NULL)
    {
      value = GgDrv->GetIT();
    }

   return value;
}


/**
 * @brief  Clear INT
 * @retval ret: GG_OK or GG_ERROR
 */
uint8_t BSP_GG_AlarmClear(void)
{
   uint8_t ret = GG_ERROR;
   
    if(GgDrv->ClearIT != NULL)
    {
      ret = GgDrv->ClearIT();
    }

   return ret;
}

/**
 * @brief Set Alarm Voltage Threshold
 * @param Th: Voltage Threshold
 * @retval ret: GG_OK or GG_ERROR
 */
uint8_t BSP_GG_SetVoltageThreshold(int Th)
{
   uint8_t ret = GG_ERROR;

    if(GgDrv->AlarmSetVoltageThreshold!= NULL)
    {
      GgDrv->AlarmSetVoltageThreshold(Th);
    }

   return ret;
}

/**
 * @brief Set Alarm SOC Threshold
 * @param Th: SOC Threshold
 * @retval ret: GG_OK or GG_ERROR
 */
uint8_t BSP_GG_SetSOCThreshold(int Th)
{
   uint8_t ret = GG_ERROR;

    if(GgDrv->AlarmSetSOCThreshold!= NULL)
    {
      GgDrv->AlarmSetSOCThreshold(Th);
    }

   return ret;
}


/**
 * @brief  Configures Gas Gauge I2C interface and interrupt lines
 * @retval None
 */
void BSP_GG_IO_Init(void)
{
  PWR_I2C_Init();
  BSP_GG_IO_ITConfig();
}

/**
 * @brief  DeInitialize the Gas Gauge I2C interface and interrupt lines
 * @retval None
 */
void BSP_GG_IO_DeInit(void)
{  
  PWR_I2C_DeInit();
}

/**
 * @brief  Configures Gas Gauge (ALM) interrupt
 * @retval None
 */
void BSP_GG_IO_ITConfig()
  {
      GPIO_InitTypeDef GPIO_InitStructure;

      /* Enable INT GPIO clock */
      GG_INT_GPIO_CLK_ENABLE();

      /* Configure GPIO PINs to detect Interrupts */
      GPIO_InitStructure.Pin = GG_INT_PIN;
      GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
      GPIO_InitStructure.Pull  = GPIO_NOPULL;
      HAL_GPIO_Init(GG_INT_GPIO_PORT, &GPIO_InitStructure);

      /* Enable and set EXTI Interrupt priority */
      HAL_NVIC_SetPriority(GG_INT_EXTI_IRQn, 0x0F, 0x00);
      HAL_NVIC_EnableIRQ(GG_INT_EXTI_IRQn);
  }


/**
 * @brief Gives Gas Gauge IT line status
 * @retval The Button GPIO pin value
 */
uint32_t BSP_GG_IO_GetITState(void)
{
  return HAL_GPIO_ReadPin(GG_INT_GPIO_PORT, GG_INT_PIN);
}



/**
 * @brief  Writes a buffer to the Gas Gauge sensor.
 * @param  pBuffer: pointer to data to be written.
 * @param  RegisterAddr: specifies the Gas Gauge register to be written.
 * @param  NumByteToWrite: number of bytes to be written.
 * @retval 0 if ok, -1 if an I2C error has occured
 */
int BSP_GG_IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    if(HIGH_BYTE(nReservedDummyRegStc3115) == APP_STC3115_KO)
    {
      PWR_I2C_Error();
    }
    
    /* Note: STC3115 does not differentiat among single/multiple byte acccess */
    status = HAL_I2C_Mem_Write(&PWR_I2C_Handle, STC3115_SLAVE_ADDRESS, (uint16_t)RegisterAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToWrite, PWR_I2C_Timeout);
    if(status != HAL_OK)
    {
      PWR_I2C_Error();
      return -1;
    }
  return 0;
}

/**
 * @brief  Reads a buffer from the GasGauge sensor.
 * @param  pBuffer: pointer to data to be read.
 * @param  RegisterAddr: specifies the Gas Gauge internal address register to read from.
 * @param  NumByteToRead: number of bytes to be read.
 * @retval 0 if ok, -1 if an I2C error has occured
 */
int BSP_GG_IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
{
  HAL_StatusTypeDef status = HAL_OK;

  if(HIGH_BYTE(nReservedDummyRegStc3115) == APP_STC3115_KO)
  {
    PWR_I2C_Error();
  }
  
  /* Note: STC3115 does not differentiat among single/multiple byte acccess */
  status = HAL_I2C_Mem_Read(&PWR_I2C_Handle, STC3115_SLAVE_ADDRESS, (uint16_t)RegisterAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToRead, PWR_I2C_Timeout);
  if(status != HAL_OK)
  {
    PWR_I2C_Error();
    return -1;
  }
  
  return 0;
}

/******************************* I2C Routines**********************************/

/**
 * @brief  Configures I2C interface.
 * @retval None
 */
static void PWR_I2C_Init(void)
{
  if(HAL_I2C_GetState(&PWR_I2C_Handle) == HAL_I2C_STATE_RESET)
  {
    /* PWR_I2C peripheral configuration */
    PWR_I2C_Handle.Init.ClockSpeed = WESU_PWR_I2C_SPEED;
    PWR_I2C_Handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    PWR_I2C_Handle.Init.OwnAddress1 = 0x33;
    PWR_I2C_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    PWR_I2C_Handle.Instance = STC3115_PWR_I2C;

    /* Init the I2C */
    PWR_I2C_MspInit();
    HAL_I2C_Init(&PWR_I2C_Handle);
  }
}

/**
 * @brief  Write only the I2C State just to restart I2C when it is useful
 * @retval None
 */
void PWR_I2C_DeInit(void)
{
  PWR_I2C_Handle.State = HAL_I2C_STATE_RESET;

}

/**
 * @brief  API Function to allow a right I2C Reset using STC3115
 * @retval None
 */
void ProcDbgResetStc3115I2C(char *pcInStr)
{
    uint32_t temp;
    static uint32_t nStc3115Reset = 0;
    
    nStc3115Reset++;
    
    temp = GPIOB->MODER;
    
    CLEAR_BIT(temp, 3 << (8 * 2));
    SET_BIT(temp, 1 << (8 * 2));
    GPIOB->MODER = temp;
    
    /* Safe delay */
    uint32_t n= 100;
    n = n * HAL_RCC_GetHCLKFreq()/1000;
    n/=10;
    while(n--);
    
    CLEAR_BIT(temp, 3 << (8 * 2));
    SET_BIT(temp, 2 << (8 * 2));
    GPIOB->MODER = temp;
    
    PRINTF("STC3115 Reset command %d times issued\r\n", nStc3115Reset);
    
    SET_HIGH_BYTE(nReservedDummyRegStc3115, APP_STC3115_KO);
//    ReleaseTerminal(pcInStr);
}

/**
 * @brief  Manages error callback by re-initializing I2C.
 * @retval None
 */
static void PWR_I2C_Error(void)
{
	/* De-initialize the I2C comunication bus */
	HAL_I2C_DeInit(&PWR_I2C_Handle);
        
        ProcDbgResetStc3115I2C(NULL);
        
	/* Re-Initiaize the I2C comunication bus */
	PWR_I2C_Init();
}

/**
 * @brief I2C MSP Initialization
 * @retval None
 */
static void PWR_I2C_MspInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* Enable I2C GPIO clocks */
    STC3115_I2C_SCL_SDA_GPIO_CLK_ENABLE();

    /* PWR_I2C SCL and SDA pins configuration -------------------------------------*/
    GPIO_InitStruct.Pin = STC3115_I2C_SCL_PIN | STC3115_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Alternate  = STC3115_I2C_SCL_SDA_AF;
    HAL_GPIO_Init(STC3115_I2C_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);

    /* Enable the PWR_I2C peripheral clock */
    STC3115_I2C_CLK_ENABLE();

    /* Force the I2C peripheral clock reset */
    STC3115_I2C_FORCE_RESET();

    /* Release the I2C peripheral clock reset */
    STC3115_I2C_RELEASE_RESET();

    /* Enable and set PWR_I2C Interrupt to the highest priority */
    HAL_NVIC_SetPriority(STC3115_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(STC3115_EV_IRQn);

    /* Enable and set PWR_I2C Interrupt to the highest priority */
    HAL_NVIC_SetPriority(STC3115_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(STC3115_ER_IRQn);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/     
