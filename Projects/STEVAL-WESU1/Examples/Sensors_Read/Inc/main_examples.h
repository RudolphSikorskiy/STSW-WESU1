/**
  ******************************************************************************
  * @file    Sensors_Read/Inc/main_examples.h 
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Header for main.c module
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
#ifndef __MAIN_H
#define __MAIN_H

#include "platform_config.h"
#include "stm32_bluenrg_ble.h"


/** @addtogroup WeSU_Examples        WeSU Examples 
  * @{
  */

/** @addtogroup WeSU_User_Examples               WeSU User Examples
  * @{
  */


/** @addtogroup WeSU_MAIN_Examples               WeSU MAIN Examples
  * @{
  */   


extern const char nFwVer[];                     //!< this string is needed to detect firmware version by reading back the device internal flash


#define USARTx                           USART2                                 //!< WeSU usart peripheral (see datasheet, expansion connector)
#define USARTx_CLK_ENABLE()              __USART2_CLK_ENABLE()                  //!< usart clock enable
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()                   //!< usart gpio clock enable
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()                   //!< usart gpio clock enable

#define USARTx_FORCE_RESET()             __USART2_FORCE_RESET()                 //!< usart force reset
#define USARTx_RELEASE_RESET()           __USART2_RELEASE_RESET()               //!< usart release reset

#define USARTx_TX_PIN                    GPIO_PIN_2                             //!< usart TX pin
#define USARTx_TX_GPIO_PORT              GPIOA                                  //!< usart TX port
#define USARTx_TX_AF                     GPIO_AF7_USART2                        //!< usart TX alternate function
#define USARTx_RX_PIN                    GPIO_PIN_3                             //!< usart TX pin
#define USARTx_RX_GPIO_PORT              GPIOA                                  //!< usart TX port
#define USARTx_RX_AF                     GPIO_AF7_USART2                        //!< usart TX alternate function
#define USARTx_IRQn                      USART2_IRQn                            //!< usart IRQn
#define COM_BAUDRATE                    115200                                  //!< usart baudrate


extern void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
