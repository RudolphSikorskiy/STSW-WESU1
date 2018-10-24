/**
  ******************************************************************************
  * @file    Sensors_Read/Src/algorithms_examples.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Manage algorithms.
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

#include "main_examples.h"
#include "console_examples.h"
#include <stdio.h>
#include "algorithms_examples.h"
#include "BlueST_Protocol_examples.h"
#include "wesu_config_examples.h"

/** @addtogroup WeSU_Examples        WeSU Examples       
  * @{
  */

/** @addtogroup WeSU_User_Examples               WeSU User Examples
  * @{
  */
   
/** @addtogroup Algorithms_Examples             Algorithms Examples
  * @{
  */   
 
   

#if USE_CUSTOM_ALGORITHM1
uint8_t algorithm1_value = 0;                                                   //!< Algorithm1 value
/**
  * @brief  Dummy algorithm 1 init function
  * @retval NONE
  */
void Algorithm1_init()
{
  algorithm1_value = 1;
}

/**
  * @brief  Dummy algorithm 1  run function
  * @retval NONE
  */
void Algorithm1_run()
{
    algorithm1_value++;
    if(algorithm1_value>6)
    {
      algorithm1_value=0;
    }
}
#endif //USE_CUSTOM_ALGORITHM1

/**
  * @brief  Initialize algorithms
  * @param bFromScratch Set to 1 if "scratch" initialization, 0 if runtime initialization
  * @retval None
  */
void Algorithms_Init(uint8_t bFromScratch)
{
#if USE_CUSTOM_ALGORITHM1
  /* Initialize Algorithm1 */
  if(bFromScratch)
  {
    Algorithm1_init();
  }
#endif /* USE_CUSTOM_ALGORITHM1 */
}

/**
  * @brief  Process algorithms
  * @retval None
  */
void Algorithms_Process()
{
    /* Put here the algorithm implementation, check frequency */
#if USE_CUSTOM_ALGORITHM1
  /* Execute the algorithm */
  EXEC_WITH_INTERVAL(Algorithm1_run(), 1000/ALGORITHM1_FREQUENCY);
#endif //USE_CUSTOM_ALGORITHM1
  
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


/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
