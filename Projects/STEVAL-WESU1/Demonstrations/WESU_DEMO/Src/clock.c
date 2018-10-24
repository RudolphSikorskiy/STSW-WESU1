/**
  ******************************************************************************
  * @file    WESU_DEMO/Src/clock.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Clock library, a simple time reference to the BLE Stack.
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
#include "clock.h"
#include "platform_config.h"

/** @addtogroup WeSU_Demo       WeSU Demo      
  * @{
  */

/** @addtogroup WeSU_User               WeSU User
  * @{
  */

/** @addtogroup Clock          Clock
  * @{
  * @brief Clock management
  */    
     
const uint32_t CLOCK_SECOND = 1000;



/** @addtogroup Clock_Private_Functions    Clock Private Functions
  * @{
  */ 

/**
 * @brief  Clock_Init
 * @retval None
 */
void Clock_Init(void)
{
  // FIXME: as long as Cube HAL is initialized this is OK
  // Cube HAL default is one clock each 1 ms
}

/**
 * @brief  Clock_Time
 * @retval tClockTime
 */
tClockTime Clock_Time(void)
{
  return HAL_GetTick();
}

/**
 * @brief  Clock_Wait Wait for a multiple of 1 ms.
 * @param  i milliseconds to wait
 * @retval None
 */
void Clock_Wait(uint32_t i)
{
  HAL_Delay(i);
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

