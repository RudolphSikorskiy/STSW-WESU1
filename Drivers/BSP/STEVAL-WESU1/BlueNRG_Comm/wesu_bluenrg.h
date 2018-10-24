/**
  ******************************************************************************
  * @file    wesu_bluenrg.h
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   This file contains definitions for SPI communication on
  *          WeSU Kit from STMicroelectronics for BLE BlueNRG
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
#ifndef __WESU_BLUENRG_H
#define __WESU_BLUENRG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
  #include "platform_config.h"

      
/** @addtogroup BSP             BSP
 *  @{
 */

/** @defgroup STEVAL-WESU1      STEVAL-WESU1
 *  @{
 */

/** @defgroup BNRG              BNRG
 *  @{
 */
   
/** @defgroup WeSU_BLUENRG      WESU_BLUENRG
 *  @{
 */   

/** @defgroup WeSU_BLUENRG_Exported_Defines     WeSU_BLUENRG_Exported_Defines
  * @brief SPI communication MACRO between WESU STM32L151 and BlueNRG Embedded Device.
  * @{
  */
  
   
#define BNRG_SPI_INSTANCE           SPI1                        //!< SPI instance dedicated to BNRG
#define BNRG_SPI_CLK_ENABLE()       __SPI1_CLK_ENABLE()         //!< BNRG_SPI ENABLING
#define BNRG_SPI_CLK_DISABLE()      __SPI1_CLK_DISABLE()        //!< BNRG_SPI DISABLING

#define BNRG_SPI_MODE               SPI_MODE_MASTER             //!< BNRG_SPI MODE
#define BNRG_SPI_DIRECTION          SPI_DIRECTION_2LINES        //!< BNRG_SPI DIRECTION
#define BNRG_SPI_DATASIZE           SPI_DATASIZE_8BIT           //!< BNRG_SPI DATASIZE
#define BNRG_SPI_CLKPOLARITY        SPI_POLARITY_LOW            //!< BNRG_SPI POLARITY  
#define BNRG_SPI_CLKPHASE           SPI_PHASE_1EDGE             //!< BNRG_SPI CLK PHASE
#define BNRG_SPI_NSS                SPI_NSS_SOFT                //!< BNRG_SPI NSS SELECTION
#define BNRG_SPI_FIRSTBIT           SPI_FIRSTBIT_MSB            //!< BNRG_SPI FIRSTBIT          
#define BNRG_SPI_TIMODE             SPI_TIMODE_DISABLED         //!< BNRG_SPI TIMODE
#define BNRG_SPI_CRCPOLYNOMIAL      7                           //!< BNRG_SPI CRC POLY
#define BNRG_SPI_BAUDRATEPRESCALER  SPI_BAUDRATEPRESCALER_4     //!< BNRG_SPI BAUDRATE
#define BNRG_SPI_CRCCALCULATION     SPI_CRCCALCULATION_DISABLED //!< BNRG_SPI CRC CALCULUS
   
 
#if ING_PROTO
  #define BNRG_SPI_CS_PORT            GPIOA                     //!< BNRG_SPI CS PORT
  #define BNRG_SPI_CS_PIN             GPIO_PIN_4                //!< BNRG_SPI CS PIN PA.4
  #define BNRG_SPI_CS_MODE            GPIO_MODE_OUTPUT_PP       //!< BNRG_SPI CS MODE
  #define BNRG_SPI_CS_PULL            GPIO_PULLUP               //!< BNRG_SPI CS PULL SELECTION
  #define BNRG_SPI_CS_SPEED           GPIO_SPEED_HIGH           //!< BNRG_SPI CS SPEED
  #define BNRG_SPI_CS_ALTERNATE       0                         //!< BNRG_SPI CS AF
  #define BNRG_SPI_CS_CLK_ENABLE()    __GPIOA_CLK_ENABLE()      //!< BNRG_SPI CS ENABLING   
  #define BNRG_SPI_CS_CLK_DISABLE()   __GPIOA_CLK_DISABLE()     //!< BNRG_SPI CS DISABLING   

#else //ING_PROTO
  #define BNRG_SPI_CS_PORT            GPIOC                     //!< BNRG_SPI CS PORT
  #define BNRG_SPI_CS_PIN             GPIO_PIN_3                //!< BNRG_SPI CS PIN PC.3
  #define BNRG_SPI_CS_MODE            GPIO_MODE_OUTPUT_PP       //!< BNRG_SPI CS MODE
  #define BNRG_SPI_CS_PULL            GPIO_PULLUP               //!< BNRG_SPI CS PULL SELECTION
  #define BNRG_SPI_CS_SPEED           GPIO_SPEED_HIGH           //!< BNRG_SPI CS SPEED
  #define BNRG_SPI_CS_ALTERNATE       0                         //!< BNRG_SPI CS AF
  #define BNRG_SPI_CS_CLK_ENABLE()    __GPIOC_CLK_ENABLE()      //!< BNRG_SPI CS ENABLING   
  #define BNRG_SPI_CS_CLK_DISABLE()   __GPIOC_CLK_DISABLE()     //!< BNRG_SPI CS DISABLING   
#endif //ING_PROTO
   
#define BNRG_SPI_SCLK_PORT          GPIOA                       //!< BNRG_SPI CLK PORT
#define BNRG_SPI_SCLK_PIN           GPIO_PIN_5                  //!< BNRG_SPI CLK PIN PA5
#define BNRG_SPI_SCLK_MODE          GPIO_MODE_AF_PP             //!< BNRG_SPI CLK MODE
#define BNRG_SPI_SCLK_PULL          GPIO_PULLDOWN               //!< BNRG_SPI CLK PULL SELECTION
#define BNRG_SPI_SCLK_SPEED         GPIO_SPEED_HIGH             //!< BNRG_SPI CLK SPEED
#define BNRG_SPI_SCLK_ALTERNATE     GPIO_AF5_SPI1               //!< BNRG_SPI CLK AF
#define BNRG_SPI_SCLK_CLK_ENABLE()  __GPIOA_CLK_ENABLE()        //!< BNRG_SPI CLK ENABLING               
#define BNRG_SPI_SCLK_CLK_DISABLE() __GPIOA_CLK_DISABLE()       //!< BNRG_SPI CLK DISABLING

#define BNRG_SPI_MISO_PORT          GPIOA                       //!< BNRG_SPI MISO PORT
#define BNRG_SPI_MISO_PIN           GPIO_PIN_6                  //!< BNRG_SPI MISO PIN PA.6
#define BNRG_SPI_MISO_MODE          GPIO_MODE_AF_PP             //!< BNRG_SPI MISO MODE SELECTION
#define BNRG_SPI_MISO_PULL          GPIO_NOPULL                 //!< BNRG_SPI MISO PULL SELECTION
#define BNRG_SPI_MISO_SPEED         GPIO_SPEED_HIGH             //!< BNRG_SPI MISO SPEED
#define BNRG_SPI_MISO_ALTERNATE     GPIO_AF5_SPI1               //!< BNRG_SPI MISO AF
#define BNRG_SPI_MISO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()        //!< BNRG_SPI MISO ENABLING
#define BNRG_SPI_MISO_CLK_DISABLE() __GPIOA_CLK_DISABLE()       //!< BNRG_SPI MISO DISABLING

#define BNRG_SPI_MOSI_PORT          GPIOA                       //!< BNRG_SPI MOSI PORT
#define BNRG_SPI_MOSI_PIN           GPIO_PIN_7                  //!< BNRG_SPI MOSI PIN PA.7
#define BNRG_SPI_MOSI_MODE          GPIO_MODE_AF_PP             //!< BNRG_SPI MOSI MODE SELECTION
#define BNRG_SPI_MOSI_PULL          GPIO_NOPULL                 //!< BNRG_SPI MOSI PULL SELECTION
#define BNRG_SPI_MOSI_SPEED         GPIO_SPEED_HIGH             //!< BNRG_SPI MOSI SPEED
#define BNRG_SPI_MOSI_ALTERNATE     GPIO_AF5_SPI1               //!< BNRG_SPI MOSI AF
#define BNRG_SPI_MOSI_CLK_ENABLE()  __GPIOA_CLK_ENABLE()        //!< BNRG_SPI MOSI ENABLING
#define BNRG_SPI_MOSI_CLK_DISABLE() __GPIOA_CLK_DISABLE()       //!< BNRG_SPI MOSI DISABLING

#define BNRG_RESET_PORT             GPIOA                       //!< BNRG_SPI RST PORT
#define BNRG_RESET_PIN              GPIO_PIN_9                  //!< BNRG_SPI RST PIN PA.9
#define BNRG_RESET_MODE             GPIO_MODE_OUTPUT_PP         //!< BNRG_SPI RST MOSE SELECTION
#define BNRG_RESET_PULL             GPIO_PULLUP                 //!< BNRG_SPI RST PULL SELECTION      
#define BNRG_RESET_SPEED            GPIO_SPEED_LOW              //!< BNRG_SPI RST SPEED
#define BNRG_RESET_ALTERNATE        0                           //!< BNRG_SPI RST AF                            
#define BNRG_RESET_CLK_ENABLE()     __GPIOA_CLK_ENABLE()        //!< BNRG_SPI RST ENABLING
#define BNRG_RESET_CLK_DISABLE()    __GPIOA_CLK_DISABLE()       //!< BNRG_SPI RST DISABLING
   
#define BNRG_IRQ_PORT           GPIOC                           //!< BNRG_SPI INT PORT
#define BNRG_IRQ_PIN            GPIO_PIN_13                     //!< BNRG_SPI INT PIN PC.13
#define BNRG_IRQ_MODE           GPIO_MODE_IT_RISING             //!< BNRG_SPI INT MODE SELECTION                     
#define BNRG_IRQ_PULL           GPIO_NOPULL                     //!< BNRG_SPI INT PULL SELECTION
#define BNRG_IRQ_SPEED          GPIO_SPEED_HIGH                 //!< BNRG_SPI INT SPEED
#define BNRG_IRQ_ALTERNATE      0                               //!< BNRG_SPI INT AF
#define BNRG_IRQ_CLK_ENABLE()   __GPIOC_CLK_ENABLE()            //!< BNRG_SPI INT ENABLING
#define BNRG_IRQ_CLK_DISABLE()  __GPIOC_CLK_DISABLE()           //!< BNRG_SPI INT DISABLING

#define BNRG_EXTI_PORT          BNRG_IRQ_PORT                   //!< BNRG_SPI EXTI PORT
#define BNRG_EXTI_PIN           BNRG_IRQ_PIN                    //!< BNRG_SPI EXTI PIN
#define BNRG_EXTI_IRQn          EXTI15_10_IRQn                  //!< BNRG_SPI EXTI IRQ                  
#define BNRG_EXTI_PRIORITY      8                               //!< BNRG_SPI EXTI PRIORITY
#define BNRG_EXTI_IRQHandler    EXTI15_10_IRQHandler            //!< BNRG_SPI EXTI IRQ Handler
   
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

#endif /* __WESU_BLUENRG_H */

    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
