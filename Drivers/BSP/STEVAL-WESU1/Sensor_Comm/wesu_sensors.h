/**
  ******************************************************************************
  * @file    wesu_sensors.h
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Header for wesu_sensors.c module
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
#ifndef __WESU_SENSORS_H
#define __WESU_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "platform_config.h"
#include "wesu_accelero.h"
#include "wesu_gyro.h"
#include "wesu_magneto.h"
#include "wesu_pressure.h"
#include "wesu_temperature.h"
#include "wesu_gg.h"
#include "wesu_charger.h"
#include "wesu_rtc.h"
  
/** @addtogroup BSP             BSP
  * @{
  */

/** @addtogroup STEVAL-WESU1    STEVAL-WESU1
 * @{
 */  
  
/** @addtogroup SENSORS         SENSORS
  * @{
  */
      
/** @addtogroup Common_Functions         Common_Functions
 * @{
 */

/** @defgroup Sensors_Exported_Types Sensors_Exported_Types
  * @{
  */
  
/**
  * @brief  WESU Sensor TypeDef
  */
typedef enum
 {
  PRESSURE,
  IMU_6AXES,
  MAGNETO,
 }SensorTypedef;  

/**
  * @brief  WESU Sensor SPI handler declaration
  */
extern SPI_HandleTypeDef        SENSORSSpiHandle;

/**
  * @}
  */

/** @defgroup Sensors_Exported_Defines Sensors_Exported_Defines
  * @{
  */

  /**
    * @brief Maximum Timeout values for flags waiting loops. These timeouts are not based
     on accurate values, they just guarantee that the application will not remain
     stuck if the SPI communication is corrupted.
     You may modify these timeout values depending on CPU frequency and application
     conditions (interrupts routines ...). 
  */   
    #define SENSORS_SPI_TIMEOUT_MAX         15    //!< Timeout Max in ms 

/**
  * @}
  */

/** @addtogroup Sensors_SPI_BUS         Sensors SPI BUS 
  * @{
  */
    #define SENSORS_SPI_INSTANCE            SPI3                        //!< uC SPI Instance Used for the Sensors 
    #define SENSORS_SPI_CLK_ENABLE()        __HAL_RCC_SPI3_CLK_ENABLE() //!< Sensors SPI CLK Enabling 
    #define SENSORS_SPI_CLK_DISABLE()       __HAL_RCC_SPI3_CLK_DISABLE()//!< Sensors SPI CLK Disabling

  // WESU SENSORS SPI Configuration
    #define SENSORS_SPI_MODE                SPI_MODE_MASTER             //!< Sensors SPI MODE           
    #define SENSORS_SPI_DIRECTION           SPI_DIRECTION_2LINES        //!< Sensors SPI DIRECTION      
    #define SENSORS_SPI_DATASIZE            SPI_DATASIZE_8BIT           //!< Sensors SPI DATASIZE       
    #define SENSORS_SPI_CLKPOLARITY         SPI_POLARITY_LOW            //!< Sensors SPI CLK POLARITY   
    #define SENSORS_SPI_CLKPHASE            SPI_PHASE_1EDGE             //!< Sensors SPI CLK EDGE       
    #define SENSORS_SPI_NSS                 SPI_NSS_SOFT                //!< Sensors SPI NSS Selection  
    #define SENSORS_SPI_FIRSTBIT            SPI_FIRSTBIT_MSB            //!< Sensors SPI FirstBit       
    #define SENSORS_SPI_TIMODE              SPI_TIMODE_DISABLED         //!< Sensors SPI TIMODE         
    #define SENSORS_SPI_CRCPOLYNOMIAL       7                           //!< Sensors SPI CRC Control                                                
    #define SENSORS_SPI_BAUDRATEPRESCALER   SPI_BAUDRATEPRESCALER_64    //!< Sensors SPI BAUDRATEPSC    
    #define SENSORS_SPI_CRCCALCULATION      SPI_CRCCALCULATION_DISABLED //!< Sensors SPI CRCCALCULATION                                                       
                                                          
  // SENSORS SCK: PB.3
    #define SENSORS_SPI_SCK_PORT            GPIOB                       //!< Sensors SPI CLK PORT        
    #define SENSORS_SPI_SCK_PIN             GPIO_PIN_3                  //!< Sensors SPI CLK PIN        
    #define SENSORS_SPI_SCK_MODE            GPIO_MODE_AF_PP             //!< Sensors SPI CLK PIN MODE   
    #define SENSORS_SPI_SCK_PULL            GPIO_PULLDOWN               //!< Sensors SPI CLK PIN PULL   
    #define SENSORS_SPI_SCK_SPEED           GPIO_SPEED_HIGH             //!< Sensors SPI CLK PIN SPEED  
    #define SENSORS_SPI_SCK_ALTERNATE       GPIO_AF6_SPI3               //!< Sensors SPI CLK PIN AF     
    #define SENSORS_SPI_SCK_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()//!< Sensors SPI CLK ENABLING   
    #define SENSORS_SPI_SCK_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()//!< Sensors SPI CLK DISABLING 

  // SENSORS MISO (Master Input Slave Output): PB.4
    #define SENSORS_SPI_MISO_PORT          GPIOB                        //!< Sensors SPI MISO PORT       
    #define SENSORS_SPI_MISO_PIN           GPIO_PIN_4                   //!< Sensors SPI MISO PIN        
    #define SENSORS_SPI_MISO_MODE          GPIO_MODE_AF_PP              //!< Sensors SPI MISO PIN MODE  
    #define SENSORS_SPI_MISO_PULL          GPIO_NOPULL                  //!< Sensors SPI MISO PIN PULL  
    #define SENSORS_SPI_MISO_SPEED         GPIO_SPEED_HIGH              //!< Sensors SPI MISO PIN SPEED             
    #define SENSORS_SPI_MISO_ALTERNATE     GPIO_AF6_SPI3                //!< Sensors SPI MISO PIN AF                  
    #define SENSORS_SPI_MISO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE() //!< Sensors SPI MISO ENABLING  
    #define SENSORS_SPI_MISO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()//!< Sensors SPI MISO DISABLING 

  // SENSORS MOSI (Master Output Slave Input): PB.5
    #define SENSORS_SPI_MOSI_PORT          GPIOB                        //!< Sensors SPI MOSI PORT      
    #define SENSORS_SPI_MOSI_PIN           GPIO_PIN_5                   //!< Sensors SPI MOSI PIN       
    #define SENSORS_SPI_MOSI_MODE          GPIO_MODE_AF_PP              //!< Sensors SPI MOSI PIN MODE  
    #define SENSORS_SPI_MOSI_PULL          GPIO_NOPULL                  //!< Sensors SPI MOSI PIN PULL  
    #define SENSORS_SPI_MOSI_SPEED         GPIO_SPEED_HIGH              //!< Sensors SPI MOSI PIN SPEED 
    #define SENSORS_SPI_MOSI_ALTERNATE     GPIO_AF6_SPI3                //!< Sensors SPI MOSI PIN AF    
    #define SENSORS_SPI_MOSI_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE() //!< Sensors SPI MOSI ENABLING  
    #define SENSORS_SPI_MOSI_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()//!< Sensors SPI MOSI DISABLING 
/**
  * @}
  */
    
/** @defgroup IMU-6AXES_Exported_Defines IMU-6AXES Exported Defines
  * @{
  */

  // IMU_6AXES_CS: PB.6
    #define IMU_6AXES_SPI_CS_PORT               GPIOB                   //!< IMU_6AXES_CS SPI PORT      
    #define IMU_6AXES_SPI_CS_PIN                GPIO_PIN_6              //!< IMU_6AXES_CS SPI PIN       
    #define IMU_6AXES_SPI_CS_MODE               GPIO_MODE_OUTPUT_PP     //!< IMU_6AXES_CS SPI PIN MODE  
    #define IMU_6AXES_SPI_CS_PULL               GPIO_PULLUP             //!< IMU_6AXES_CS SPI PIN PULL  
    #define IMU_6AXES_SPI_CS_SPEED              GPIO_SPEED_HIGH         //!< IMU_6AXES_CS SPI PIN SPEED 
    #define IMU_6AXES_SPI_CS_AF                 GPIO_AF6_SPI3           //!< IMU_6AXES_CS SPI PIN AF    
    #define IMU_6AXES_SPI_CS_CLK_ENABLE()       __GPIOB_CLK_ENABLE()    //!< IMU_6AXES_CS SPI ENABLING  
    #define IMU_6AXES_SPI_CS_CLK_DISABLE()      __GPIOB_CLK_DISABLE()   //!< IMU_6AXES_CS SPI DISABLING 

  // IMU_6AXES_CS: PC.6
    #define IMU_6AXES_INT1_GPIO_PORT            GPIOC                   //!< IMU_6AXES_INT1 PORT        
    #define IMU_6AXES_INT1_PIN                  GPIO_PIN_6              //!< IMU_6AXES_INT1 PIN         
    #define IMU_6AXES_INT1_EXTI_IRQn            EXTI9_5_IRQn            //!< IMU_6AXES_INT1 IRQ_Handler 
    #define IMU_6AXES_INT1_EXTI_PRIORITY        6
    #define IMU_6AXES_INT1_GPIO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()    //!< IMU_6AXES_INT1 PIN ENABLE  
    #define IMU_6AXES_INT1_GPIO_CLK_DISABLE()   __GPIOC_CLK_DISABLE()   //!< IMU_6AXES_INT1 PIN DISABLE 

  // IMU_6AXES_CS: PC.7
    #define IMU_6AXES_INT2_GPIO_PORT            GPIOC                   //!< IMU_6AXES_INT2 PORT        
    #define IMU_6AXES_INT2_PIN                  GPIO_PIN_7              //!< IMU_6AXES_INT2 PIN         
    #define IMU_6AXES_INT2_EXTI_IRQn            EXTI9_5_IRQn            //!< IMU_6AXES_INT2 IRQ_Handler 
    #define IMU_6AXES_INT2_EXTI_PRIORITY        7
    #define IMU_6AXES_INT2_GPIO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()    //!< IMU_6AXES_INT2 PIN ENABLE  
    #define IMU_6AXES_INT2_GPIO_CLK_DISABLE()   __GPIOC_CLK_DISABLE()   //!< IMU_6AXES_INT2 PIN DISABLE 
/**
  * @}
  */

/** @defgroup Magneto_Exported_Defines Magneto Exported Defines
  * @{
  */
  // MAGNETO_CS: PB.7
    #define MAGNETO_SPI_CS_PORT               GPIOB                     //!< MAGNETO_CS SPI PORT        
    #define MAGNETO_SPI_CS_PIN                GPIO_PIN_7                //!< MAGNETO_CS SPI PIN         
    #define MAGNETO_SPI_CS_MODE               GPIO_MODE_OUTPUT_PP       //!< MAGNETO_CS SPI PIN MODE    
    #define MAGNETO_SPI_CS_PULL               GPIO_PULLUP               //!< MAGNETO_CS SPI PIN PULL    
    #define MAGNETO_SPI_CS_SPEED              GPIO_SPEED_HIGH           //!< MAGNETO_CS SPI PIN SPEED   
    #define MAGNETO_SPI_CS_AF                 GPIO_AF6_SPI3             //!< MAGNETO_CS SPI PIN AF      
    #define MAGNETO_SPI_CS_CLK_ENABLE()       __GPIOB_CLK_ENABLE()      //!< MAGNETO_CS SPI PIN ENABLE  
    #define MAGNETO_SPI_CS_CLK_DISABLE()      __GPIOB_CLK_DISABLE()     //!< MAGNETO_CS SPI PIN DISABLE  

  // MAGNETO_DRDY: PC.9
    #define MAGNETO_DRDY_GPIO_PORT           GPIOC                      //!< MAGNETO_DRDY PORT          
    #define MAGNETO_DRDY_PIN                 GPIO_PIN_9                 //!< MAGNETO_DRDY PIN           
    #define MAGNETO_DRDY_EXTI_IRQn           EXTI9_5_IRQn               //!< MAGNETO_DRDY IRQ_Handler   
    #define MAGNETO_DRDY_GPIO_CLK_ENABLE()   __GPIOC_CLK_ENABLE()       //!< MAGNETO_DRDY PIN ENABLE    
    #define MAGNETO_DRDY_GPIO_CLK_DISABLE()  __GPIOC_CLK_DISABLE()      //!< MAGNETO_DRDY PIN DISABLE   

  // MAGNETO_INT: PC.10
    #define MAGNETO_INT_GPIO_PORT           GPIOC                       //!< MAGNETO_INT PORT           
    #define MAGNETO_INT_PIN                 GPIO_PIN_10                 //!< MAGNETO_INT PIN                             
    #define MAGNETO_INT_EXTI_IRQn           EXTI15_10_IRQn              //!< MAGNETO_INT IRQ_Handler    
    #define MAGNETO_INT_GPIO_CLK_ENABLE()   __GPIOC_CLK_ENABLE()        //!< MAGNETO_INT PIN ENABLE     
    #define MAGNETO_INT_GPIO_CLK_DISABLE()  __GPIOC_CLK_DISABLE()       //!< MAGNETO_INT PIN DISABLE    
/**
  * @}
  */


/** @defgroup Pressure_Exported_Defines Pressure Exported Defines
  * @{
  */

// PRESSURE_CS: PB.11
    #define PRESSURE_SPI_CS_PORT          GPIOB                         //!< PRESSURE_CS SPI PORT       
    #define PRESSURE_SPI_CS_PIN           GPIO_PIN_11                   //!< PRESSURE_CS SPI PIN        
    #define PRESSURE_SPI_CS_MODE          GPIO_MODE_OUTPUT_PP           //!< PRESSURE_CS SPI PIN MODE   
    #define PRESSURE_SPI_CS_PULL          GPIO_PULLUP                   //!< PRESSURE_CS SPI PIN PULL   
    #define PRESSURE_SPI_CS_SPEED         GPIO_SPEED_HIGH               //!< PRESSURE_CS SPI PIN SPEED  
    #define PRESSURE_SPI_CS_AF            GPIO_AF6_SPI3                 //!< PRESSURE_CS SPI PIN AF     
    #define PRESSURE_SPI_CS_CLK_ENABLE()  __GPIOB_CLK_ENABLE()          //!< PRESSURE_CS SPI PIN ENABLE 
    #define PRESSURE_SPI_CS_CLK_DISABLE() __GPIOB_CLK_DISABLE()         //!< PRESSURE_CS SPI PIN DISABLE

// PRESSURE_INT IRQ: PC.8
    #define PRESSURE_INT_PORT                   GPIOC                   //!< PRESSURE_INT PORT          
    #define PRESSURE_INT_PIN                    GPIO_PIN_8              //!< PRESSURE_INT PIN           
    #define PRESSURE_INT_MODE                   GPIO_MODE_IT_RISING     //!< PRESSURE_INT PIN MODE      
    #define PRESSURE_INT_PULL                   GPIO_NOPULL             //!< PRESSURE_INT PIN PULL      
    #define PRESSURE_INT_SPEED                  GPIO_SPEED_HIGH         //!< PRESSURE_INT PIN SPEED     
    #define PRESSURE_INT_AF                     0                       //!< PRESSURE_INT PIN AF        
    #define PRESSURE_INT_EXTI_IRQn              EXTI9_5_IRQn            //!< PRESSURE_INT IRQ_Handler   
    #define PRESSURE_INT_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()    //!< PRESSURE_INT PIN ENABLE     
    #define PRESSURE_INT_GPIO_CLK_DISABLE()     __GPIOC_CLK_DISABLE()   //!< PRESSURE_INT PIN DISABLE   
/**
  * @}
  */ 

/** @defgroup SPI_MACRO_Exported_Defines SPI MACRO Exported Defines
  * @{
  */
    #define NONE_SPI_CS_PIN     GPIO_PIN_0      //!< Error Selection for CS_PIN, pin PC.0 is not used   
    #define NONE_SPI_CS_PORT    GPIOC           //!< Error Selection for CS_PORT                         

    #define CS_PIN(P)    ((P==PRESSURE)?PRESSURE_SPI_CS_PIN: \
                          (P==MAGNETO)?MAGNETO_SPI_CS_PIN: \
                          (P==IMU_6AXES)?IMU_6AXES_SPI_CS_PIN: \
                              NONE_SPI_CS_PIN)                          //!< CS_PIN Macro Selection    

    #define CS_PORT(P)    ((P==PRESSURE)?PRESSURE_SPI_CS_PORT: \
                          (P==MAGNETO)?MAGNETO_SPI_CS_PORT: \
                          (P==IMU_6AXES)?IMU_6AXES_SPI_CS_PORT: \
                              NONE_SPI_CS_PORT)                         //!< CS_PORT Macro Selection        
/**
  * @}
  */

/** @addtogroup WESU_IO_Exported_FunctionPrototypes WESU_IO Exported FunctionPrototypes
 * @{
 */
DrvStatusTypeDef Sensor_IO_Init( void );
void LSM6DS3_Sensor_IO_ITConfig( void );
void LIS3MDL_IO_ITConfig( void );
void LPS25HB_IO_ITConfig( void ); 

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

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __WESU_SENSORS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
