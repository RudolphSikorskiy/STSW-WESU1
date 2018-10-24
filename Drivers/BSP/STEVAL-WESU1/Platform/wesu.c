/**
  ******************************************************************************
  * @file    wesu.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   This file provides set of firmware functions to manage:
  *          - LEDs and push-button available on STEVAL-WESU1
  *            from STMicroelectronics
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
#include "platform_config.h"
#include "wesu_sensors.h"


/** @addtogroup BSP             BSP
  * @{
  */ 

/** @addtogroup STEVAL-WESU1            STEVAL-WESU1
  * @{
  */   
    
/** @addtogroup PLATFORM        PLATFORM
  * @{
  */

/** @addtogroup WeSU_LOW_LEVEL          WeSU_LOW_LEVEL 
  * @brief This file provides set of firmware functions to manage Leds and push-button
  *        available on STEVAL-WESU1 board from STMicroelectronics.
  * @{
  */ 

/** @defgroup WeSU_LOW_LEVEL_Private_Defines    WeSU_LOW_LEVEL_Private_Defines
  * @{
  */ 

//#if !USE_REGS
#ifndef USE_REGS
    #define nLedControlMask                 LED_CONFIG_APP_CONTROLLED_MASK      //!< Led control mask
#endif //USE_REGS
    
#define USE_LED_DMA             SESSION_REG(nRsvd0x7D)          //!< USE_LED_DMA    
#define BUFFER_LENGHT           150                             //!< MAX LED Buffer Size
#define WESU_FLASH_TYPEPROGRAM  FLASH_TYPEPROGRAMDATA_BYTE      //!< WeSU Programming Type

#define SAVE_LED_TURN_ON_TICK(L,T)  nLAST_LED_TURN_ON[L] = T;   //!< Save Led turn ON 
#define GET_LED_TURN_ON_TICK(L)     nLAST_LED_TURN_ON[L]        //!< Get Led Turn ON  

/**
  * @}
  */ 

/** @defgroup WeSU_LOW_LEVEL_Exported_Variables  WeSU_LOW_LEVEL_Exported_Variables
  * @{
  */ 
    
#ifdef USE_REGS
#if defined (__IAR_SYSTEMS_ICC__)
    __no_init GlobalVarsStruct_t SessionRegsReservedArea @SESSIONREG_STRUCT_START_ADDRESS;
#elif defined (__CC_ARM)
	__attribute__((section ("RamReg")))
      GlobalVarsStruct_t SessionRegsReservedArea;
#elif defined (__GNUC__)
      GlobalVarsStruct_t SessionRegsReservedArea __attribute__ ((section (".noinit")));
#else
      #error "Toolchain not supported"
#endif 
      
  pGlobalVarsStruct_t GlobalSessionStruct = (pGlobalVarsStruct_t)(&SessionRegsReservedArea);
#endif //USE_REGS
  
/**
  * @}
  */ 

/** @defgroup WeSU_LOW_LEVEL_Private_Variables  WeSU_LOW_LEVEL_Private_Variables
  * @{
  */ 

GPIO_TypeDef*           GPIO_PORT[LEDn]                 = {LED_GPIO_PORT, RED_LED_GPIO_PORT};                //!< MACRO to assign LED PORT 

const uint16_t          GPIO_PIN[LEDn]                  = {LED_GPIO_PIN, RED_LED_GPIO_PIN};                 //!< MACRO to assign LED PIN

const uint16_t          GPIO_MODE[LEDn]                 = {LED_GPIO_MODE, RED_LED_GPIO_MODE};                 //!< MACRO to assign LED PIN mode

const GPIO_PinState     GPIO_ON_PIN_STATUS[LEDn]       = {LED_GPIO_ON_PIN_STATUS, RED_LED_GPIO_ON_PIN_STATUS};                 //!< MACRO to assign LED PIN status in ON state

const GPIO_PinState     GPIO_OFF_PIN_STATUS[LEDn]       = {LED_GPIO_OFF_PIN_STATUS, RED_LED_GPIO_OFF_PIN_STATUS};                 //!< MACRO to assign LED PIN status in OFF state


GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {BUTTON_GPIO_PORT};        //!< MACRO to assign BUTTON PORT
const uint16_t BUTTON_PIN[BUTTONn] = {BUTTON_GPIO_PIN};         //!< MACRO to assign BUTTON PIN
const uint8_t BUTTON_IRQn[BUTTONn] = {BUTTON_EXTI_IRQn};        //!< MACRO to assign BUTTON IRQn

uint32_t          nLAST_LED_TURN_ON[LEDn]                 = {0xFFFFFFFF, 0xFFFFFFFF};                 //!< MACRO to save last LED turn on tick

#define TIMx_CLK_ENABLE()       __HAL_RCC_TIM10_CLK_ENABLE()
#define TIMx_CLK_DISABLE()      __HAL_RCC_TIM10_CLK_DISABLE()

#define TIMx_IRQn                         TIM10_IRQn

IWDG_HandleTypeDef hiwdg;
TIM_HandleTypeDef  TimInputCaptureHandle;

#ifndef USE_REGS
  uint32_t nLsiFreq;                    //!<  Low speed oscillator frequency
  uint16_t WeSU_IWDG_RELOAD_VALUE = WeSU_IWDG_DEFAULT_RELOAD_VALUE;      //!<  IWDG Reload value
#endif

#define uwLsiFreq SESSION_REG(nLsiFreq)
__IO uint32_t uwCaptureNumber = 0;
__IO uint32_t uwPeriodValue = 0;
__IO uint32_t uwMeasurementDone = 0;
uint8_t bWdgConfigured = FALSE;

uint16_t tmpCC1[2] = {0, 0};

/**
  * @}
  */ 

extern TIM_HandleTypeDef htim3;
extern void Error_Handler(void);

/** @defgroup WeSU_LOW_LEVEL_Private_Functions  WeSU_LOW_LEVEL_Private_Functions
  * @{
  */ 

uint32_t nLedMaxValue = LED_CONFIG_BRIGHT_MAX;  //!< Set the LED brightness

uint32_t bufferT3[BUFFER_LENGHT] = {0};         //!< Init the bufferT3 with Zero array

uint32_t nLastLedTurnOn = 0xFFFFFFFF;

/**
  * @brief TIM3 Initialization for WeSU Platform
  * @retval None
  */
void BSP_LED_TIM_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = LED_GPIO_TIMER;
  htim3.Init.Prescaler = 239;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = LED_CONFIG_BRIGHT_MAX - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
}

/** 
  * @brief Enable DMA controller clock
  */
void BSP_LED_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}


/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void BSP_LED_InitDma(Led_TypeDef Led)
{
  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);
  
  nLedMaxValue = SESSION_REG(nLedControlMask) & 0xFF00;
  
  BSP_LED_DMA_Init();
  BSP_LED_TIM_Init();
}

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void BSP_LED_InitGpio(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);
  
  /* Configure the GPIO_LED pin */
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_OFF_PIN_STATUS[Led]);
  
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE[Led];
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
}

/**
  * @brief  Configures the Max Value for the LED brightness.
* @param  nMaxVal: Specifies the Led brightness to be set: 0x0000 -> 0x0C00 
  * @retval None
  */
void SetMaxValue(uint32_t nMaxVal)
{
#ifdef USE_REGS
  SESSION_REG(nLedControlMask) = nMaxVal & 0xFF00;
#endif //USE_REGS
    nLedMaxValue = SESSION_REG(nLedControlMask) & 0xFF00;
}

/**
  * @brief  Turns selected LED On using DMA.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void BSP_LED_OnDma(Led_TypeDef Led)
{
    LedSmoothRampUpDma(5);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void BSP_LED_OnGpio(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_ON_PIN_STATUS[Led]); 
  SAVE_LED_TURN_ON_TICK(Led, HAL_GetTick());
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void BSP_LED_OffDma(Led_TypeDef Led)
{
    bufferT3[0]=0;
    if(htim3.State == HAL_TIM_STATE_READY)
    {
      HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, bufferT3, 1);
    }
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void BSP_LED_OffGpio(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_OFF_PIN_STATUS[Led]); 
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void BSP_LED_OffTimerGpio(Led_TypeDef Led, uint32_t nTimer)
{
  if(HAL_GetTick() > GET_LED_TURN_ON_TICK(Led) + nTimer)
  {
    SAVE_LED_TURN_ON_TICK(Led, 0xFFFFFFFF);
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_OFF_PIN_STATUS[Led]); 
  }
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED  
  * @retval None
  */
void BSP_LED_ToggleDma(Led_TypeDef Led)
{
  if(htim3.State == HAL_TIM_STATE_READY)
  {
    bufferT3[0]= (bufferT3[0]==0)?nLedMaxValue:0;
    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, bufferT3, 1);
  }
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED  
  * @retval None
  */
void BSP_LED_ToggleGpio(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}

/**
  * @brief  Read status of the selected LED.
  * @param  Led: Specifies the Led to be read. 
  *   This parameter can be one of following parameters:
  *     @arg LED  
  * @retval Led state: 0 - off, 1 - on
  */
uint8_t BSP_LED_GetState(Led_TypeDef Led)
{
  return (GPIO_OFF_PIN_STATUS[Led]==HAL_GPIO_ReadPin(GPIO_PORT[Led], GPIO_PIN[Led]));
}



/**
  * @brief API to Set the falling edge Pulse Led duration
  * @param nRep: 
  * @param nPwm: 
  * @retval None
  */
void Led_Pwm(uint8_t nPwm, uint32_t nRep)
{
  if(!LED_CONFIG_USER_ENABLED())
  {
	  int i,j;
    for( i=0;i<nRep;i++)
    {
      for( j=0;j<100;j++)
      {
        if(j<nPwm)
          BSP_LED_OnGpio(LED);
        else
          BSP_LED_OffGpio(LED);
      }
    }
  }
}


/**
  * @brief API to Set the rising edge Pulse Led Period
  * @param nSpeed: Ramp Up Timing Speed
  * @retval None
  */
void LedSmoothRampUpDma(uint8_t nSpeed)
{
  if(nSpeed==0)nSpeed=1;
  if(nSpeed>=BUFFER_LENGHT)nSpeed=BUFFER_LENGHT;
  if(!LED_CONFIG_USER_ENABLED())
  {
	  int i;
    for( i = 0;i<nSpeed;i++)
      bufferT3[i] = ( ((i * nLedMaxValue)/nSpeed) );
    
    bufferT3[nSpeed-1] = nLedMaxValue;
    if(htim3.State == HAL_TIM_STATE_READY)
    {
      HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, bufferT3, nSpeed);
    }
  }
}

/**
  * @brief API to Set the rising edge Pulse Led Period
  * @param nSpeed: Ramp Up Timing Speed
  * @retval None
  */
void LedSmoothRampUpGpio(uint8_t nSpeed)
{
  if(!LED_CONFIG_USER_ENABLED())
  {
	  int i;
    for( i=0; i<100; i++)
    {
      Led_Pwm(i,nSpeed);
    }
    BSP_LED_OnGpio(LED);
  }
}

/**
  * @brief API to Set the falling edge Pulse Led Period
  * @param nSpeed: Ramp Down Timing Speed
  * @retval None
  */
void LedSmoothRampDownDma(uint8_t nSpeed)
{
  if(nSpeed==0)nSpeed=1;
  if(nSpeed>=BUFFER_LENGHT)nSpeed=BUFFER_LENGHT;
  if(!LED_CONFIG_USER_ENABLED())
  {
	  int i;
    for(i = 0;i<nSpeed;i++)
      bufferT3[i] = (nLedMaxValue - ((i * nLedMaxValue)/nSpeed));
    
    bufferT3[nSpeed-1] = 0;
    if(htim3.State == HAL_TIM_STATE_READY)
    {
      HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, bufferT3, nSpeed);
    }
  }
}

/**
  * @brief API to Set the falling edge Pulse Led Period
  * @param nSpeed: Ramp Down Timing Speed
  * @retval None
  */
void LedSmoothRampDownGpio(uint8_t nSpeed)
{
  if(!LED_CONFIG_USER_ENABLED())
  {
	  int i;
    for(i=0; i<100; i++)
    {
      Led_Pwm(100-i,nSpeed);
    }
  }
}


/**
  * @brief API to Set the blinking duration
  * @param nSpeed: Blinking Timing Speed
  * @retval None
  */
void LedSmoothBlinkDma(uint8_t nSpeed)
{
  LedSmoothRampUpDma(nSpeed);
  while(htim3.State != HAL_TIM_STATE_READY);
  LedSmoothRampDownDma(nSpeed);
  while(htim3.State != HAL_TIM_STATE_READY);
}

/**
  * @brief API to Set the blinking duration
  * @param nSpeed: Blinking Timing Speed
  * @retval None
  */
void LedSmoothBlinkGpio(uint8_t nSpeed)
{
  LedSmoothRampUpGpio(nSpeed);
  LedSmoothRampDownGpio(nSpeed);
}

/**
  * @brief  Input Capture callback in non blocking mode 
  * @param  htim : TIM IC handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Get the Input Capture value */
  tmpCC1[uwCaptureNumber++] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

  if (uwCaptureNumber >= 2)
  {
    /* Compute the period length */
    uwPeriodValue = (uint16_t)(0xFFFF - tmpCC1[0] + tmpCC1[1] + 1);
    uwMeasurementDone = 1;
    uwCaptureNumber = 0;
  }
}

/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  /* TIMx Peripheral clock enable */
  TIMx_CLK_ENABLE();

  /* Configure the NVIC for TIMx */
  HAL_NVIC_SetPriority(TIMx_IRQn, 0, 0);

  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIMx_IRQn);
}

/**
  * @brief  Configures TIM10 to measure the LSI oscillator frequency.
  * @param  None
  * @retval LSI Frequency
  */
uint32_t GetLSIFrequency(void)
{
  uint32_t pclk2 = 0, latency = 0;
  TIM_IC_InitTypeDef timinputconfig = {0};
  RCC_OscInitTypeDef oscinit = {0};
  RCC_ClkInitTypeDef  clkinit =  {0};
  
  /* Enable LSI Oscillator */
  oscinit.OscillatorType = RCC_OSCILLATORTYPE_LSI;
  oscinit.LSIState = RCC_LSI_ON;
  oscinit.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&oscinit)!= HAL_OK)
  {
    Error_Handler(); 
  }

  /* Configure the TIM peripheral */
  /* Set TIMx instance */
  TimInputCaptureHandle.Instance = TIM10;

  /* TIMx configuration: Input Capture mode ---------------------
  The LSI clock is connected to TIM10 CH1.
  The Rising edge is used as active edge.
  The TIM10 CCR1 is used to compute the frequency value.
  ------------------------------------------------------------ */
  TimInputCaptureHandle.Init.Prescaler         = 0;
  TimInputCaptureHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimInputCaptureHandle.Init.Period            = 0xFFFF;
  TimInputCaptureHandle.Init.ClockDivision     = 0;
  if (HAL_TIM_IC_Init(&TimInputCaptureHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  /* Connect internally the  TIM10 CH1 Input Capture to the LSI clock output */
  HAL_TIMEx_RemapConfig(&TimInputCaptureHandle, TIM_TIM10_LSI);

  /* Configure the Input Capture of channel 1 */
  timinputconfig.ICPolarity  = TIM_ICPOLARITY_RISING;
  timinputconfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  timinputconfig.ICPrescaler = TIM_ICPSC_DIV8;
  timinputconfig.ICFilter    = 0;

  if (HAL_TIM_IC_ConfigChannel(&TimInputCaptureHandle, &timinputconfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Reset the flags */
  TimInputCaptureHandle.Instance->SR = 0;

  /* Start the TIM Input Capture measurement in interrupt mode */
  if (HAL_TIM_IC_Start_IT(&TimInputCaptureHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  /* Wait until the TIM10 get 2 LSI edges (refer to TIM10_IRQHandler() in
  stm32l1xx_it.c file) */
  while (uwMeasurementDone == 0)
  {
  }
  uwCaptureNumber = 0;

  /* Deinitialize the TIM10 peripheral registers to their default reset values */
  HAL_TIM_IC_DeInit(&TimInputCaptureHandle);

    if (HAL_TIM_IC_Stop_IT(&TimInputCaptureHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
//    Error_Handler();
  }

  /* Compute the LSI frequency, depending on TIM10 input clock frequency (PCLK2)*/
  /* Get PCLK2 frequency */
  pclk2 = HAL_RCC_GetPCLK2Freq();
  HAL_RCC_GetClockConfig(&clkinit, &latency);

  /* Get PCLK2 prescaler */
  if ((clkinit.APB2CLKDivider) == RCC_HCLK_DIV1)
  {
    /* PCLK2 prescaler equal to 1 => TIMCLK = PCLK2 */
    return ((pclk2 / uwPeriodValue) * 8);
  }
  else
  {
    /* PCLK2 prescaler different from 1 => TIMCLK = 2 * PCLK2 */
    return (((2 * pclk2) / uwPeriodValue) * 8) ;
  }
}

/**
  * @brief Low level function to enable Watchdog
  * @retval None
  */
void BSP_WD_Init()
{
#if USE_IWDG
  uwLsiFreq = GetLSIFrequency();
  
    /* Set counter reload value to obtain 250ms IWDG TimeOut.
     IWDG counter clock Frequency = LsiFreq / 32
     Counter Reload Value = (WD_RELOAD_IN_MS / 1000) / IWDG counter clock period
                          = (WD_RELOAD_IN_MS / 1000) / (256/LsiFreq)
                          = (LsiFreq * WD_RELOAD_IN_MS) / ((256 * 1000) )
                          */
  hiwdg.Instance = IWDG;

  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload    = uwLsiFreq * SESSION_REG(WeSU_IWDG_RELOAD_VALUE);
  hiwdg.Init.Reload    = uwLsiFreq * SESSION_REG(WeSU_IWDG_RELOAD_VALUE) / 256000;

  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
  {
  }
  
  bWdgConfigured = TRUE;
#endif //USE_IWDG
}

void BSP_WD_Refresh()
{
#if USE_IWDG
  if(!bWdgConfigured)
  {
    BSP_WD_Init();
  }
  if(bWdgConfigured)
  {
    if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
    {
    }
  }
#endif //USE_IWDG
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_USER
  * @param  ButtonMode: Specifies Button mode.
  *   This parameter can be one of following parameters:   
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO 
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability  
  * @retval None
  */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);
  
  if(ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
  }
  
  if(ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; 
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
    
    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *                 This parameter should be: BUTTON_USER
  * @retval None
  */
void BSP_PB_DeInit(Button_TypeDef Button)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);
  
    /* Configure Button pin as input */
    GPIO_InitStruct.Pin = BUTTON_PIN[Button];
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
  
    /* Configure Button pin as input with External interrupt */
    HAL_NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
}

/**
  * @brief  Returns the selected Button state.
  * @retval The Button GPIO pin value.
  */
uint32_t BSP_PwrUsbMonitor_GetState()
{
  return (HAL_GPIO_ReadPin(USB_PWR_GPIO_PORT, USB_PWR_GPIO_PIN) != GPIO_PIN_RESET);
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  UsbPwrMode: Specifies Usb Pwr Mode.
  *   This parameter can be one of following parameters:   
  *             USB_PWR_MODE_GPIO = 0,  GPIO MODE         
  *             USB_PWR_MODE_EXTI = 1   EXTI MODE  
  * @retval None
  */
void BSP_PwrUsbMonitor_Init(UsbPwrMode_TypeDef UsbPwrMode)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Enable the BUTTON Clock */
  USB_PWR_GPIO_CLK_ENABLE();
  
  if(UsbPwrMode == USB_PWR_MODE_GPIO)
  {
    /* Configure Button pin as input */
    GPIO_InitStruct.Pin = USB_PWR_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(USB_PWR_GPIO_PORT, &GPIO_InitStruct);
  }
  
  if(UsbPwrMode == USB_PWR_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    GPIO_InitStruct.Pin = USB_PWR_GPIO_PIN;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; 
    HAL_GPIO_Init(USB_PWR_GPIO_PORT, &GPIO_InitStruct);
    
    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(USB_PWR_EXTI_IRQn), 0x0F, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)(USB_PWR_EXTI_IRQn));
  }
}

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @retval None
  */
void BSP_PwrUsbMonitor_DeInit()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Enable the USB_PWR pin Clock */
  USB_PWR_GPIO_CLK_ENABLE();
  
    /* Configure Button pin as input */
    GPIO_InitStruct.Pin = USB_PWR_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(USB_PWR_GPIO_PORT, &GPIO_InitStruct);
  
    /* Configure USB_PWR pin as input with External interrupt */
    HAL_NVIC_DisableIRQ((IRQn_Type)(USB_PWR_EXTI_IRQn));
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_USER  
  * @retval The Button GPIO pin value.
  */
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return GPIO_PIN_RESET == HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/**
  * @brief EEPROM Buffer Write API
  * @param  pWriteAddress: Address to Write
  * @param  pBuf: Pointer to the input buffer.
  * @param  nBufLen: Input Buffer Size.
  * @retval None
  */
void BSP_EEPROM_WriteBuffer(uint32_t pWriteAddress, uint32_t pBuf, uint32_t nBufLen)
{
  HAL_FLASHEx_DATAEEPROM_Unlock();
  
  uint32_t tickstart = HAL_GetTick();   
  uint32_t Timeout = FLASH_TIMEOUT_VALUE;
  
  while(FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE) != HAL_OK) 
  { 
    if(Timeout != HAL_MAX_DELAY)
    {
      if((Timeout == 0)||((HAL_GetTick() - tickstart ) > Timeout))
      {
        return /*HAL_TIMEOUT*/;
      }
    } 
  }
  
  int i;
  for(i=0;i<nBufLen;)
  {
    if(HAL_FLASHEx_DATAEEPROM_Program(WESU_FLASH_TYPEPROGRAM, pWriteAddress+i, *(uint8_t*)(pBuf+i)) == HAL_OK)
    {
      if(WESU_FLASH_TYPEPROGRAM == FLASH_TYPEPROGRAMDATA_BYTE)
      {
        i=i+1;
      }
    }
    else
    {
      i=i+1;
    }
  }
  
  HAL_FLASHEx_DATAEEPROM_Lock();
}


/**
  * @brief EEPROM Buffer Read API
  * @param  pReadAddress: Address to Read
  * @param  pBuf: Pointer to the output buffer.
  * @param  nBufLen: Input Buffer Size.
  * @retval None
  */
void BSP_EEPROM_ReadBuffer(uint32_t pReadAddress, uint32_t pBuf, uint32_t nBufLen)
{
	int i;
  for(i=0;i<nBufLen;)
  {
    if(WESU_FLASH_TYPEPROGRAM == FLASH_TYPEPROGRAMDATA_BYTE)
    {
      *(uint8_t*)(pBuf+i) = *(uint8_t*)(pReadAddress+i);
      i+=1;
    }
  }
}

/**
  * @brief DBG Disabling for WeSU Platform
  * @retval None
  */
void BSP_DbgDisable()
{
  HAL_DBGMCU_DisableDBGStopMode();
  HAL_DBGMCU_DisableDBGStandbyMode();
  HAL_DBGMCU_DisableDBGSleepMode();
}

/**
  * @brief DBG Enabling for WeSU Platform
  * @retval None
  */
void BSP_DbgEnable()
{
  HAL_DBGMCU_EnableDBGStopMode();
  HAL_DBGMCU_EnableDBGStandbyMode();
  HAL_DBGMCU_EnableDBGSleepMode();
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
