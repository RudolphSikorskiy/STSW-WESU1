/**
  ******************************************************************************
  * @file    WESU_DEMO/Src/stm32l1xx_hal_msp.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   HAL MSP module.
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
#include "stm32l1xx_hal.h"
#include "main.h"

/** @addtogroup WeSU_Demo       WeSU Demo
  * @{
  */

/** @addtogroup WeSU_User               WeSU User
  * @{
  */

/** @defgroup STM32L1xx_HAL_MSP         STM32L1xx HAL MSP
  * @{
  * @brief WeSU MSP Init/DeInit Functionalities for peripherals
  */
    
/** @defgroup HAL_MSP_Exported_Variables        HAL MSP Exported Variables
  * @{
  */    

TIM_HandleTypeDef htim3;                                //!< LED_TIMER HANDLER
DMA_HandleTypeDef hdma_tim3_ch3;                        //!< LED_DMA_TIMER HANDLER

/**
 * @}
 */

/** @addtogroup HAL_MSP_Exported_Functions      HAL MSP Exported Functions
  * @{
  */

/**
  * @brief  Initializes the Global MSP.
  * @retval None
  */
void HAL_MspInit(void)
{
  /* NOTE : This function is generated automatically by MicroXplorer and eventually  
            modified by the user
   */ 
}

/**
  * @brief  DeInitializes the Global MSP.
  * @retval None
  */
void HAL_MspDeInit(void)
{
  /* NOTE : This function is generated automatically by MicroXplorer and eventually  
            modified by the user
   */
}

/**
  * @brief  Initializes the RTC MSP.
  * @param  hrtc: pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.  
  * @retval None
  */
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
    /* Peripheral clock enable */
    __HAL_RCC_RTC_ENABLE();
   /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
  }
}

/**
  * @brief  DeInitializes the RTC MSP.
  * @param  hrtc: pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC. 
  * @retval None
  */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{

  if(hrtc->Instance==RTC)
  {
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);

    HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);
  }
}


/**
  * @brief TIMER_BASE MSP Initialization (Used to drive the Led)
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param htim_base: TIM_BASE handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_base->Instance==LED_GPIO_TIMER)
  {
    /* Peripheral clock enable */
    LED_GPIO_TIMER_ENABLE;
  
    GPIO_InitStruct.Pin = LED_GPIO_PIN;
    GPIO_InitStruct.Mode = LED_GPIO_MODE;
    GPIO_InitStruct.Pull = LED_GPIO_PULL;
    GPIO_InitStruct.Speed = LED_GPIO_SPEED;
    GPIO_InitStruct.Alternate = LED_GPIO_TIMER_AF;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_tim3_ch3.Instance = DMA1_Channel2;
    hdma_tim3_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim3_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim3_ch3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim3_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim3_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim3_ch3.Init.Mode = DMA_NORMAL;//DMA_CIRCULAR;
    hdma_tim3_ch3.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_tim3_ch3);

    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC3],hdma_tim3_ch3);
  }
   if(htim_base->Instance==TIM_process)
  {
    /* Peripheral clock enable */
    TIM_process_CLK_ENABLE();
    /* System interrupt init*/
    HAL_NVIC_SetPriority(TIM_process_IRQn, 0x0F, 0x0F); //low priority
    HAL_NVIC_EnableIRQ(TIM_process_IRQn);
  }
}

/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();
  
  /* Enable USARTx clock */
  USARTx_CLK_ENABLE(); 
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_MEDIUM;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;
  
  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
    
  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Alternate = USARTx_RX_AF;
    
  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
  
  HAL_NVIC_SetPriority(USARTx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

/**
  * @brief UART MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  /*##-1- Reset peripherals ##################################################*/
  USARTx_FORCE_RESET();
  USARTx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);
}

/**
 * @brief  This function is used for low level initialization of the SPI 
 *         communication with the BlueNRG & Sensors .
 * @param  hspi: SPI handle.
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hspi->Instance==BNRG_SPI_INSTANCE)
  {
    /* Enable peripherals and GPIOs Port Clock */  
    BNRG_RESET_CLK_ENABLE();
    BNRG_SPI_SCLK_CLK_ENABLE();
    BNRG_SPI_MISO_CLK_ENABLE();
    BNRG_SPI_MOSI_CLK_ENABLE();
    BNRG_SPI_CS_CLK_ENABLE();
    BNRG_IRQ_CLK_ENABLE();

    /* Enable SPI clock */
    BNRG_SPI_CLK_ENABLE();

    /* Reset */
    GPIO_InitStruct.Pin = BNRG_RESET_PIN;
    GPIO_InitStruct.Mode = BNRG_RESET_MODE;
    GPIO_InitStruct.Pull = BNRG_RESET_PULL;
    GPIO_InitStruct.Speed = BNRG_RESET_SPEED;
    GPIO_InitStruct.Alternate = BNRG_RESET_ALTERNATE;
    HAL_GPIO_Init(BNRG_RESET_PORT, &GPIO_InitStruct);	

    /* SCLK */
    GPIO_InitStruct.Pin = BNRG_SPI_SCLK_PIN;
    GPIO_InitStruct.Mode = BNRG_SPI_SCLK_MODE;
    GPIO_InitStruct.Pull = BNRG_SPI_SCLK_PULL;
    GPIO_InitStruct.Speed = BNRG_SPI_SCLK_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_SCLK_ALTERNATE;
    HAL_GPIO_Init(BNRG_SPI_SCLK_PORT, &GPIO_InitStruct); 

    /* MISO */
    GPIO_InitStruct.Pin = BNRG_SPI_MISO_PIN;
    GPIO_InitStruct.Mode = BNRG_SPI_MISO_MODE;
    GPIO_InitStruct.Pull = BNRG_SPI_MISO_PULL;
    GPIO_InitStruct.Speed = BNRG_SPI_MISO_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_MISO_ALTERNATE;
    HAL_GPIO_Init(BNRG_SPI_MISO_PORT, &GPIO_InitStruct);

    /* MOSI */
    GPIO_InitStruct.Pin = BNRG_SPI_MOSI_PIN;
    GPIO_InitStruct.Mode = BNRG_SPI_MOSI_MODE;
    GPIO_InitStruct.Pull = BNRG_SPI_MOSI_PULL;
    GPIO_InitStruct.Speed = BNRG_SPI_MOSI_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_MOSI_ALTERNATE;
    HAL_GPIO_Init(BNRG_SPI_MOSI_PORT, &GPIO_InitStruct);

    /* NSS/CSN/CS */
    GPIO_InitStruct.Pin = BNRG_SPI_CS_PIN;
    GPIO_InitStruct.Mode = BNRG_SPI_CS_MODE;
    GPIO_InitStruct.Pull = BNRG_SPI_CS_PULL;
    GPIO_InitStruct.Speed = BNRG_SPI_CS_SPEED;
    GPIO_InitStruct.Alternate = BNRG_SPI_CS_ALTERNATE;
    HAL_GPIO_Init(BNRG_SPI_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);

    /* IRQ -- INPUT */
    GPIO_InitStruct.Pin = BNRG_IRQ_PIN;
    GPIO_InitStruct.Mode = BNRG_IRQ_MODE;
    GPIO_InitStruct.Pull = BNRG_IRQ_PULL;
    GPIO_InitStruct.Speed = BNRG_IRQ_SPEED;
    GPIO_InitStruct.Alternate = BNRG_IRQ_ALTERNATE;
    HAL_GPIO_Init(BNRG_IRQ_PORT, &GPIO_InitStruct);

    /* Configure the NVIC for SPI */  
    HAL_NVIC_SetPriority(BNRG_EXTI_IRQn, BNRG_EXTI_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(BNRG_EXTI_IRQn);
  }
  
    if(hspi->Instance==SENSORS_SPI_INSTANCE)
  {
    /* Enable peripherals and GPIOs Port Clock */  
    SENSORS_SPI_SCK_CLK_ENABLE();
    SENSORS_SPI_MISO_CLK_ENABLE();
    SENSORS_SPI_MOSI_CLK_ENABLE();
    PRESSURE_SPI_CS_CLK_ENABLE();
    IMU_6AXES_SPI_CS_CLK_ENABLE();
    MAGNETO_SPI_CS_CLK_ENABLE() ;


    /* Enable SPI clock */
    SENSORS_SPI_CLK_ENABLE();

    /* SCK */
    GPIO_InitStruct.Pin = SENSORS_SPI_SCK_PIN;
    GPIO_InitStruct.Mode = SENSORS_SPI_SCK_MODE;
    GPIO_InitStruct.Pull = SENSORS_SPI_SCK_PULL;
    GPIO_InitStruct.Speed = SENSORS_SPI_SCK_SPEED;
    GPIO_InitStruct.Alternate = SENSORS_SPI_SCK_ALTERNATE;
    HAL_GPIO_Init(SENSORS_SPI_SCK_PORT, &GPIO_InitStruct); 

    /* MISO */
    GPIO_InitStruct.Pin = SENSORS_SPI_MISO_PIN;
    GPIO_InitStruct.Mode = SENSORS_SPI_MISO_MODE;
    GPIO_InitStruct.Pull = SENSORS_SPI_MISO_PULL;
    GPIO_InitStruct.Speed = SENSORS_SPI_MISO_SPEED;
    GPIO_InitStruct.Alternate = SENSORS_SPI_MISO_ALTERNATE;
    HAL_GPIO_Init(SENSORS_SPI_MISO_PORT, &GPIO_InitStruct);

    /* MOSI */
    GPIO_InitStruct.Pin = SENSORS_SPI_MOSI_PIN;
    GPIO_InitStruct.Mode = SENSORS_SPI_MOSI_MODE;
    GPIO_InitStruct.Pull = SENSORS_SPI_MOSI_PULL;
    GPIO_InitStruct.Speed = SENSORS_SPI_MOSI_SPEED;
    GPIO_InitStruct.Alternate = SENSORS_SPI_MOSI_ALTERNATE;
    HAL_GPIO_Init(SENSORS_SPI_MOSI_PORT, &GPIO_InitStruct);

  /* SPI PRESSURE_CS GPIO pin configuration  */
    GPIO_InitStruct.Pin           = PRESSURE_SPI_CS_PIN;
    GPIO_InitStruct.Mode          = PRESSURE_SPI_CS_MODE;
    GPIO_InitStruct.Pull          = PRESSURE_SPI_CS_PULL;
    GPIO_InitStruct.Speed         = PRESSURE_SPI_CS_SPEED;
    GPIO_InitStruct.Alternate     = PRESSURE_SPI_CS_AF;
    HAL_GPIO_Init(PRESSURE_SPI_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(PRESSURE_SPI_CS_PORT, PRESSURE_SPI_CS_PIN, GPIO_PIN_SET);

  /* SPI IMU_6AXES_CS GPIO pin configuration  */
    GPIO_InitStruct.Pin           = IMU_6AXES_SPI_CS_PIN;
    GPIO_InitStruct.Mode          = IMU_6AXES_SPI_CS_MODE;
    GPIO_InitStruct.Pull          = IMU_6AXES_SPI_CS_PULL;
    GPIO_InitStruct.Speed         = IMU_6AXES_SPI_CS_SPEED;
    GPIO_InitStruct.Alternate     = IMU_6AXES_SPI_CS_AF;
    HAL_GPIO_Init(IMU_6AXES_SPI_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(IMU_6AXES_SPI_CS_PORT, IMU_6AXES_SPI_CS_PIN, GPIO_PIN_SET);
  
  /* SPI LSM9DS1_M_CS GPIO pin configuration  */
    GPIO_InitStruct.Pin           = MAGNETO_SPI_CS_PIN;
    GPIO_InitStruct.Mode          = MAGNETO_SPI_CS_MODE;
    GPIO_InitStruct.Pull          = MAGNETO_SPI_CS_PULL;
    GPIO_InitStruct.Speed         = MAGNETO_SPI_CS_SPEED;
    GPIO_InitStruct.Alternate     = MAGNETO_SPI_CS_AF;
    HAL_GPIO_Init(MAGNETO_SPI_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(MAGNETO_SPI_CS_PORT, MAGNETO_SPI_CS_PIN, GPIO_PIN_SET);
 }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==BNRG_SPI_INSTANCE)
  {
    /* Peripheral clock disable */
    BNRG_SPI_CLK_DISABLE();
    
    /**SPI3 GPIO DeInit Configuration */
    HAL_GPIO_DeInit(BNRG_SPI_SCLK_PORT, BNRG_SPI_SCLK_PIN|BNRG_SPI_MISO_PIN|BNRG_SPI_MOSI_PIN);
  }  
  
  if(hspi->Instance==SENSORS_SPI_INSTANCE)
  {
    /* Peripheral clock disable */
    SENSORS_SPI_CLK_DISABLE();
  
    /**SPI3 GPIO DeInit Configuration */
    HAL_GPIO_DeInit(SENSORS_SPI_SCK_PORT, SENSORS_SPI_SCK_PIN|SENSORS_SPI_MISO_PIN|SENSORS_SPI_MOSI_PIN);
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
