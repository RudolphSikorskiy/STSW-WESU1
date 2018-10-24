/**
  ******************************************************************************
  * @file    WESU_DEMO/Src/stm32l1xx_it.c 
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_it.h"
   
/** @addtogroup WeSU_Demo       WeSU Demo
  * @{
  */

/** @addtogroup WeSU_User               WeSU User
  * @{
  */

/** @addtogroup STM32L1xx_IT_Handlers
  * @{
  * @brief Cortex-M3 Processor Exceptions Handlers
  */

/**
  * @brief   This function handles NMI exception.
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  LED_GPIO_PORT->ODR |= LED_GPIO_PIN;
  
  while (1)
  {
    if(0==(BUTTON_GPIO_PORT->IDR & BUTTON_GPIO_PIN))
    {
      HAL_NVIC_SystemReset();
    }
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}


/**
  * @brief  EXTI0_IRQHandler This function handles External line 0
  *         interrupt request 
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
}

/**
  * @brief  EXTI2_IRQHandler This function handles External line
  *         interrupt request for USER_BUTTON.
  * @retval None
  */
 void EXTI2_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(BUTTON_GPIO_PIN);
}

/**
  * @brief  EXTI9_5_IRQHandler This function handles External line
  *         interrupt request for BlueNRG.
  * @retval None
  */

void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(USB_PWR_GPIO_PIN);     /* PC_5 */
  HAL_GPIO_EXTI_IRQHandler(IMU_6AXES_INT1_PIN);   /* PC_6 */
  HAL_GPIO_EXTI_IRQHandler(IMU_6AXES_INT2_PIN);   /* PC_7 */         
  HAL_GPIO_EXTI_IRQHandler(PRESSURE_INT_PIN);     /* PC_8 */
  HAL_GPIO_EXTI_IRQHandler(MAGNETO_DRDY_PIN);     /* PC_9 */
}


/**
* @brief  This function handles External line 15-10 interrupt request.
* @retval None
*/
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(MAGNETO_INT_PIN);  /* PC_10 */
    HAL_GPIO_EXTI_IRQHandler(BNRG_IRQ_PIN);     /* PC_13 */
    HAL_GPIO_EXTI_IRQHandler(GG_INT_PIN);       /* PB_15 */
}

/**
  * @brief  This function handles TIM10 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM10_IRQHandler(void)
{
  extern TIM_HandleTypeDef    TimInputCaptureHandle;
  HAL_TIM_IRQHandler(&TimInputCaptureHandle);
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&UartHandle);
}

/**
* @brief This function handles DMA1 channel2 global interrupt.
*/
void DMA1_Channel2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_tim3_ch3);
}

/**
* @brief This function handles TIM_process global interrupt.
* @param  None
* @retval None
*/
void TIM_process_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&processTimHandle);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
