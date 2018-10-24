/**
  ******************************************************************************
  * @file    wesu_sensors.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   This file provides specific functions to manage the 
  *          STEVAL-WESU1 sensors
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
#include "wesu_sensors.h"

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

/** @defgroup Sensors_Private_Defines Sensors_Private_Defines
 * @{
 */

#ifndef NULL
  #define NULL      (void *) 0  //!< NULL MACRO
#endif

#ifndef PRINTF_SENSORS
  #define PRINTF_SENSORS(...)   //!< PRINTF_SENSORS MACRO
  #define GET_SENSOR_STRING(S)  ((S==PRESSURE)?"P":((S==IMU_6AXES)?"6x":((S==MAGNETO)?"M":"UNK")))      //!< GET_SENSOR_STRING MACRO
#endif //PRINTF_SENSORS

/**
 * @}
 */

/** @defgroup Sensors_Private_Variables         Sensors_Private_Variables
 * @{
 */
   
SPI_HandleTypeDef       SENSORSSpiHandle;        //<! SPI Handle for Sensors

/**
 * @}
 */

/** @defgroup Sensors_Private_Functions_Prototypes Sensors_Private_Functions_Prototypes
 * @{
 */

DrvStatusTypeDef LSM6DS3_IO_Init( void );              
DrvStatusTypeDef LSM6DS3_IO_Write (uint8_t* pBuffer, 
                                          uint8_t DeviceAddr,
                                          uint8_t RegisterAddr, 
                                          uint16_t NumByteToWrite ); 

DrvStatusTypeDef LSM6DS3_IO_Read (uint8_t* pBuffer,
                                         uint8_t DeviceAddr,
                                         uint8_t RegisterAddr,
                                         uint16_t NumByteToRead );


DrvStatusTypeDef LIS3MDL_IO_Init(void); 
DrvStatusTypeDef LIS3MDL_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,
                                       uint16_t NumByteToWrite);
DrvStatusTypeDef LIS3MDL_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr, 
                                      uint16_t NumByteToRead);

DrvStatusTypeDef LPS25HB_IO_Init(void);

DrvStatusTypeDef LPS25HB_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, 
                                  uint8_t RegisterAddr, uint16_t NumByteToWrite);

DrvStatusTypeDef LPS25HB_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, 
                                 uint8_t RegisterAddr, uint16_t NumByteToRead);

/* Link function for 6_AXES peripheral */

static DrvStatusTypeDef IMU_6AXES_IO_Init(void);
static DrvStatusTypeDef IMU_6AXES_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr,
                                           uint8_t RegisterAddr,uint16_t NumByteToWrite);
static DrvStatusTypeDef IMU_6AXES_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, 
                                          uint8_t RegisterAddr,uint16_t NumByteToRead);
static DrvStatusTypeDef MAGNETO_IO_Init(void);
static DrvStatusTypeDef MAGNETO_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, 
                                         uint8_t RegisterAddr,uint16_t NumByteToWrite);
static DrvStatusTypeDef MAGNETO_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, 
                                        uint8_t RegisterAddr, uint16_t NumByteToRead);
static DrvStatusTypeDef PRESSURE_IO_Init(void);
static DrvStatusTypeDef PRESSURE_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, 
                                          uint8_t RegisterAddr, uint16_t NumByteToWrite);
static DrvStatusTypeDef PRESSURE_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, 
                                         uint8_t RegisterAddr, uint16_t NumByteToRead);

static HAL_StatusTypeDef SensorsSpiInit(void);
static HAL_StatusTypeDef Sensors_SPI_Read(SPI_HandleTypeDef *hspi, SensorTypedef Sensor, 
                                          uint8_t *ReadBuffer, uint8_t Address, uint8_t buff_size);

static HAL_StatusTypeDef Sensors_SPI_Write(SPI_HandleTypeDef *hspi, SensorTypedef Sensor, 
                                           uint8_t *WriteBuffer, uint8_t Address, uint8_t buff_size);

static void SPIx_Error (void);

/**
 * @}
 */

/** @defgroup Sensors_Exported_Functions Sensors_Exported_Functions
 * @{
 */


/**
 * @brief  Configures LSM6DS3 IO Init interface
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
DrvStatusTypeDef LSM6DS3_IO_Init( void )
{
  return IMU_6AXES_IO_Init();
}


/**
 * @brief  Configures LSM6DS3 interrupt lines for WESU boards
 * @retval None
 */
void LSM6DS3_Sensor_IO_ITConfig( void )
{
  GPIO_InitTypeDef GPIO_InitStructureInt1;
  GPIO_InitTypeDef GPIO_InitStructureInt2;
  /* Enable INT1 GPIO clock */
  IMU_6AXES_INT1_GPIO_CLK_ENABLE();
  
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt1.Pin = IMU_6AXES_INT1_PIN;
  GPIO_InitStructureInt1.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt1.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStructureInt1.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_6AXES_INT1_GPIO_PORT, &GPIO_InitStructureInt1);
  
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(IMU_6AXES_INT1_EXTI_IRQn, IMU_6AXES_INT1_EXTI_PRIORITY, 0x00);
  HAL_NVIC_EnableIRQ(IMU_6AXES_INT1_EXTI_IRQn);
  
  /* Enable INT2 GPIO clock */
  IMU_6AXES_INT2_GPIO_CLK_ENABLE();
  
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt2.Pin = IMU_6AXES_INT2_PIN;
  GPIO_InitStructureInt2.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt2.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStructureInt2.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_6AXES_INT2_GPIO_PORT, &GPIO_InitStructureInt2);
  
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(IMU_6AXES_INT2_EXTI_IRQn, IMU_6AXES_INT2_EXTI_PRIORITY, 0x00);
  HAL_NVIC_EnableIRQ(IMU_6AXES_INT2_EXTI_IRQn);
}

/**
 * @brief  Writes a buffer to the LSM6DS3 sensor
 * @param  pBuffer the pointer to data to be written
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the IMU 6 axes internal address register to be written
 * @param  NumByteToWrite the number of bytes to be written
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
DrvStatusTypeDef LSM6DS3_IO_Write (uint8_t* pBuffer,
                                   uint8_t DeviceAddr,
                                   uint8_t RegisterAddr,
                                   uint16_t NumByteToWrite )
{
  return IMU_6AXES_IO_Write( pBuffer, DeviceAddr, RegisterAddr, NumByteToWrite );
}


/**
 * @brief  Reads a buffer from the LSM6DS3 sensor
 * @param  pBuffer the pointer to data to be read
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the IMU 6 axes internal address register to be read
 * @param  NumByteToRead the number of bytes to be read
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
DrvStatusTypeDef LSM6DS3_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, 
                                 uint8_t RegisterAddr, uint16_t NumByteToRead )
{
  return IMU_6AXES_IO_Read( pBuffer, DeviceAddr, RegisterAddr, NumByteToRead );
}


/********************************* LINK MAGNETO *****************************/
/**
 * @brief  Configures LIS3MDL I2C interface
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
DrvStatusTypeDef LIS3MDL_IO_Init(void)
{
  return MAGNETO_IO_Init();
}

/**
 * @brief  Configures LIS3MDL interrupt lines for WESU boards
 * @retval None
 */
void LIS3MDL_IO_ITConfig( void )
{
  GPIO_InitTypeDef GPIO_InitStructureInt;
  
  /* Enable INT GPIO clock */
  MAGNETO_INT_GPIO_CLK_ENABLE();
  
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt.Pin = MAGNETO_INT_PIN;
  GPIO_InitStructureInt.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStructureInt.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(MAGNETO_INT_GPIO_PORT, &GPIO_InitStructureInt);
  
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(MAGNETO_INT_EXTI_IRQn, 0x00, 0x00);
  HAL_NVIC_EnableIRQ(MAGNETO_INT_EXTI_IRQn);

  /* Enable INT GPIO clock */
  MAGNETO_DRDY_GPIO_CLK_ENABLE();

  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt.Pin = MAGNETO_DRDY_PIN;
  GPIO_InitStructureInt.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStructureInt.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(MAGNETO_INT_GPIO_PORT, &GPIO_InitStructureInt);
  
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(MAGNETO_DRDY_EXTI_IRQn, 0x00, 0x00);
  HAL_NVIC_EnableIRQ(MAGNETO_DRDY_EXTI_IRQn);
}

/**
 * @brief  Writes a buffer to the LIS3MDL sensor
 * @param  pBuffer the pointer to data to be written
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the magneto internal address register to be written
 * @param  NumByteToWrite the number of bytes to be written
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
DrvStatusTypeDef LIS3MDL_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, 
                                  uint8_t RegisterAddr, uint16_t NumByteToWrite)
{
  return MAGNETO_IO_Write(pBuffer, DeviceAddr, RegisterAddr, NumByteToWrite);
}

/**
 * @brief  Reads a buffer from the LIS3MDL sensor
 * @param  pBuffer the pointer to data to be read
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the magneto internal address register to be read
 * @param  NumByteToRead the number of bytes to be read
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
DrvStatusTypeDef LIS3MDL_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, 
                                 uint8_t RegisterAddr, uint16_t NumByteToRead)
{
  return MAGNETO_IO_Read(pBuffer, DeviceAddr, RegisterAddr, NumByteToRead);
}



/********************************* LINK PRESSURE *****************************/

/**
 * @brief  Configures LPS25H Interface
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
DrvStatusTypeDef LPS25HB_IO_Init(void)
{
  return PRESSURE_IO_Init();
}

/**
 * @brief  Configures LPS25H interrupt lines for WESU boards
 * @retval None
 */
void LPS25HB_IO_ITConfig( void )
{
  GPIO_InitTypeDef GPIO_InitStructureInt;
  
  /* Enable INT1 GPIO clock */
  PRESSURE_INT_GPIO_CLK_ENABLE();
  
  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt.Pin = PRESSURE_INT_PIN;
  GPIO_InitStructureInt.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructureInt.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStructureInt.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(PRESSURE_INT_PORT, &GPIO_InitStructureInt);
  
  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(PRESSURE_INT_EXTI_IRQn, 0x00, 0x00);
  HAL_NVIC_EnableIRQ(PRESSURE_INT_EXTI_IRQn);
}

/**
 * @brief  Writes a buffer to the LPS25H sensor
 * @param  pBuffer the pointer to data to be written
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the pressure internal address register to be written
 * @param  NumByteToWrite the number of bytes to be written
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
DrvStatusTypeDef LPS25HB_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,
                                       uint16_t NumByteToWrite)
{
  return PRESSURE_IO_Write(pBuffer, DeviceAddr, RegisterAddr, NumByteToWrite);
}

/**
 * @brief  Reads a buffer from the LPS25H sensor
 * @param  pBuffer the pointer to data to be read
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the pressure internal address register to be read
 * @param  NumByteToRead the number of bytes to be read
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
DrvStatusTypeDef LPS25HB_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, 
                                 uint8_t RegisterAddr, uint16_t NumByteToRead)
{
  return PRESSURE_IO_Read(pBuffer, DeviceAddr, RegisterAddr, NumByteToRead);
}

/** @defgroup Sensors_Private_Functions Sensors_Private_Functions
 * @{
 */

/**
 * @brief  Configures Imu 6 axes I2C interface
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
static DrvStatusTypeDef IMU_6AXES_IO_Init( void )
{

#if USE_SPI_SENSORS
  if(SensorsSpiInit() != HAL_OK)
  {
    return COMPONENT_ERROR;
  }
#endif
  
  return COMPONENT_OK;
}

/**
 * @brief  Writes a buffer to the IMU 6 axes sensor
 * @param  pBuffer the pointer to data to be written
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the IMU 6 axes internal address register to be written
 * @param  NumByteToWrite the number of bytes to be written
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
static DrvStatusTypeDef IMU_6AXES_IO_Write(uint8_t* pBuffer,
                                           uint8_t DeviceAddr,
                                           uint8_t RegisterAddr,
                                           uint16_t NumByteToWrite)
{
  DrvStatusTypeDef ret_val = COMPONENT_OK;

  #if USE_SPI_SENSORS
    uint8_t a; a=RegisterAddr; a&=0x7F;
    if((Sensors_SPI_Write(&SENSORSSpiHandle,IMU_6AXES,pBuffer,a,NumByteToWrite)) != HAL_OK)
      {
        ret_val = COMPONENT_ERROR;
      }
  #endif  
  return ret_val;
}

/**
 * @brief  Reads a buffer from the IMU 6 axes sensor
 * @param  pBuffer the pointer to data to be read
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the IMU 6 axes internal address register to be read
 * @param  NumByteToRead the number of bytes to be read
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
static DrvStatusTypeDef IMU_6AXES_IO_Read( uint8_t* pBuffer, 
                                                 uint8_t DeviceAddr, 
                                                 uint8_t RegisterAddr, 
                                                 uint16_t NumByteToRead )
{
  DrvStatusTypeDef ret_val = COMPONENT_OK;

  #if USE_SPI_SENSORS
    uint8_t a; a=RegisterAddr; a|=0x80;
    if((Sensors_SPI_Read(&SENSORSSpiHandle,IMU_6AXES,pBuffer,a,NumByteToRead)) != HAL_OK)
      {
        ret_val = COMPONENT_ERROR;
      }
  #endif
  return ret_val;
}


/**
 * @brief  Configures magneto I2C interface
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
static DrvStatusTypeDef MAGNETO_IO_Init(void)
{
  DrvStatusTypeDef ret_val = COMPONENT_OK;

  #if USE_SPI_SENSORS
    if(SensorsSpiInit() != HAL_OK)
      {
        ret_val = COMPONENT_ERROR;
      }
  #endif  
  return ret_val;
}

/**
 * @brief  Writes a buffer to the magneto sensor
 * @param  pBuffer the pointer to data to be written
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the magneto internal address register to be written
 * @param  NumByteToWrite the number of bytes to be written
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
static DrvStatusTypeDef MAGNETO_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,
    uint16_t NumByteToWrite)
{
  DrvStatusTypeDef ret_val = COMPONENT_OK;
   
  #if USE_SPI_SENSORS            /* call Sensor_SPI_Write data bus function */ 
    int a; a=RegisterAddr; a|=0x40;
  
    if((Sensors_SPI_Write(&SENSORSSpiHandle,MAGNETO,pBuffer,a,NumByteToWrite)) != HAL_OK)
      {
        ret_val = COMPONENT_ERROR;
      }
  #endif  
  return ret_val;
}

/**
 * @brief  Reads a buffer from the magneto sensor
 * @param  pBuffer the pointer to data to be read
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the magneto internal address register to be read
 * @param  NumByteToRead the number of bytes to be read
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
static DrvStatusTypeDef MAGNETO_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,
    uint16_t NumByteToRead)
{
  DrvStatusTypeDef ret_val = COMPONENT_OK;
   
  #if USE_SPI_SENSORS            /* call Sensor_SPI_Read data bus function */ 
    int a; a=RegisterAddr; a|=0xC0; 

    if((Sensors_SPI_Read(&SENSORSSpiHandle,MAGNETO,pBuffer,a,NumByteToRead)) != HAL_OK)
      {
        ret_val = COMPONENT_ERROR;
      }
  #endif    
  return ret_val;
}

/**
 * @brief  Configures pressure Interface in I2C or SPI
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
static DrvStatusTypeDef PRESSURE_IO_Init(void)
{
  DrvStatusTypeDef ret_val = COMPONENT_OK;

  #if USE_SPI_SENSORS            /* call Sensor_SPI_Init data bus function */ 
    if(SensorsSpiInit() != HAL_OK)
    {
      ret_val = COMPONENT_ERROR;
    }
  #endif  

  return ret_val;
}


/**
 * @brief  Writes a buffer to the pressure sensor
 * @param  pBuffer the pointer to data to be written
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the pressure internal address register to be written
 * @param  NumByteToWrite the number of bytes to be written
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
static DrvStatusTypeDef PRESSURE_IO_Write(uint8_t* pBuffer,
                                                uint8_t DeviceAddr,
                                                uint8_t RegisterAddr,
                                                uint16_t NumByteToWrite)
{
  DrvStatusTypeDef ret_val = COMPONENT_OK;

#if USE_SPI_SENSORS            /* call Sensor_SPI_Write data bus function */ 
    int b; b=RegisterAddr; b|=0x40; if(NumByteToWrite<=1) b&=0x3F;
  
    if((Sensors_SPI_Write(&SENSORSSpiHandle,PRESSURE,pBuffer,b,NumByteToWrite)) != HAL_OK)
    {
      ret_val = COMPONENT_ERROR;
    }
#endif
   return ret_val;
}

/**
 * @brief  Reads a buffer from the pressure sensor
 * @param  pBuffer the pointer to data to be read
 * @param  DeviceAddr the slave address to be programmed
 * @param  RegisterAddr the pressure internal address register to be read
 * @param  NumByteToRead number of bytes to be read
 * @retval COMPONENT_OK in case of success, an error code otherwise
 */
static DrvStatusTypeDef PRESSURE_IO_Read(uint8_t* pBuffer,
                                               uint8_t DeviceAddr,
                                               uint8_t RegisterAddr,
                                               uint16_t NumByteToRead)
{
  DrvStatusTypeDef ret_val = COMPONENT_OK;

  #if USE_SPI_SENSORS            /* call Sensor_SPI_Read data bus function */ 
    int b; b=RegisterAddr; b|=0xC0; if(NumByteToRead<=1) b&=0xBF; 

    if((Sensors_SPI_Read(&SENSORSSpiHandle,PRESSURE,pBuffer,b,NumByteToRead)) != HAL_OK)
      {
        ret_val = COMPONENT_ERROR;
      }
  #endif    
  return ret_val;
}

/**
 * @brief  Reads from Sensors SPI buffer and store data into local buffer.
 * @param  hspi         : SPI handle
 * @param  Sensor       : Device to Read
 * @param  ReadBuffer   : Buffer where data from SPI are stored
 * @param  Address      : First Address Byte reading
 * @param  buff_size    : Buffer size
 * @retval int32_t      : Number of read bytes
 */
static HAL_StatusTypeDef Sensors_SPI_Read(SPI_HandleTypeDef *hspi,
                                          SensorTypedef Sensor, 
                                          uint8_t *ReadBuffer,
                                          uint8_t Address,
                                          uint8_t buff_size)
{
//  uint8_t WriteBuffer[sizeof(buff_size)];
//  uint8_t TempBuffer = 0x00;
//  memset(WriteBuffer,0, buff_size);
  
  HAL_StatusTypeDef status = HAL_OK;
  
  /* CS reset */
  HAL_GPIO_WritePin(CS_PORT(Sensor), CS_PIN(Sensor), GPIO_PIN_RESET);
  
  /* Send the Address and Receive FF */
  status = HAL_SPI_Transmit(hspi, &Address, 1, SENSORS_SPI_TIMEOUT_MAX);

    /* Check the address writing */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }

  /* Send WriteBuffer all Zero in order to Receive the bytes to read */
  status = HAL_SPI_Receive(hspi, ReadBuffer, buff_size, SENSORS_SPI_TIMEOUT_MAX);  

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
  
  /* Release CS line */
  HAL_GPIO_WritePin(CS_PORT(Sensor), CS_PIN(Sensor), GPIO_PIN_SET);
  
  PRINTF_SENSORS("SR:%s,A=0x%.2X,N=%d: ",GET_SENSOR_STRING(Sensor),Address&0x7F,buff_size);int i;for(i =0;i<buff_size;i++){PRINTF_SENSORS("%.2X ",ReadBuffer[i]);}PRINTF_SENSORS("\r\n");
  
  return status;
}

/**
* @brief  Write data from local buffer to SPI
* @param  hspi          handle of the STM32Cube HAL SPI interface
* @param  Sensor        Device to Write
* @param  WriteBuffer:  data buffer to be written
* @param  Address:      Address to be written
* @param  buff_size:    size of data buffer to be written
*/
static HAL_StatusTypeDef Sensors_SPI_Write(SPI_HandleTypeDef *hspi,
                                            SensorTypedef Sensor, 
                                            uint8_t *WriteBuffer,
                                            uint8_t Address,
                                            uint8_t buff_size)
{  
  HAL_StatusTypeDef status = HAL_OK;
  
  PRINTF_SENSORS("SW:%s,A=0x%.2X,N=%d: ",GET_SENSOR_STRING(Sensor),Address,buff_size);int i;for( i =0;i<buff_size;i++){PRINTF_SENSORS("%.2X ",WriteBuffer[i]);}PRINTF_SENSORS("\r\n");
  
  /* CS reset */
  HAL_GPIO_WritePin(CS_PORT(Sensor), CS_PIN(Sensor), GPIO_PIN_RESET);
  
  /* Send the Address and Receive FF */
  status = HAL_SPI_Transmit(hspi, &Address, 1, SENSORS_SPI_TIMEOUT_MAX);
  
  /* Check the address writing */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }

  /* Send the WriteBuffer in order to Transmit the bytes */
  status = HAL_SPI_Transmit(hspi, WriteBuffer, buff_size, SENSORS_SPI_TIMEOUT_MAX);

    /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
  
  /* Release CS line */
  HAL_GPIO_WritePin(CS_PORT(Sensor), CS_PIN(Sensor), GPIO_PIN_SET);

  return status;
}

/**
 * @brief  Configures Sensors comunication interface
 * @retval COMPONENT_OK in case of success, COMPONENT_ERROR otherwise
 */
DrvStatusTypeDef Sensor_IO_Init( void )
{

  #if USE_SPI_SENSORS
    if(SensorsSpiInit() != HAL_OK)
    {
      return COMPONENT_ERROR;
    }
  #endif
  
  return COMPONENT_OK;
}

/**
 * @brief  Reads a from the sensor to buffer
 * @param  handle instance handle
 * @param  ReadAddr specifies the internal sensor address register to be read from
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToRead number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
int Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  uint8_t who_am_i = ctx->who_am_i;
  SensorTypedef Sensor;
  switch(who_am_i)
  {
  case LPS25HB_WHO_AM_I_VAL:
    Sensor = PRESSURE;
    ReadAddr |=0x80;
    if(nBytesToRead>1)ReadAddr |= 0x40;
    break;
  case LSM6DS3_ACC_GYRO_WHO_AM_I:
    Sensor = IMU_6AXES;
    ReadAddr |= 0x80;
    break;
  case LIS3MDL_MAG_WHO_AM_I:
    Sensor = MAGNETO;
    ReadAddr |= 0xC0;
    break;
  default:
    return 1;
  }
  
  /* call Sensors_SPI_Read Read data bus function */
  if ( Sensors_SPI_Read( &SENSORSSpiHandle, Sensor, pBuffer, ReadAddr, nBytesToRead ) )
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

/**
 * @brief  Writes a buffer to the sensor
 * @param  handle instance handle
 * @param  WriteAddr specifies the internal sensor address register to be written to
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToWrite number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
int Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  uint8_t who_am_i = ctx->who_am_i;
  SensorTypedef Sensor;
  switch(who_am_i)
  {
  case LPS25HB_WHO_AM_I_VAL:
    Sensor = PRESSURE;
    WriteAddr &= 0x7F;
    break;
  case LSM6DS3_ACC_GYRO_WHO_AM_I:
    Sensor = IMU_6AXES;
    WriteAddr &= 0x7F;
    break;
  case LIS3MDL_MAG_WHO_AM_I:
    Sensor = MAGNETO;
    WriteAddr |= 0x40;
    break;
  default:
    return 1;
  }

  /* call Sensors_SPI_Write Read data bus function */
  if ( Sensors_SPI_Write( &SENSORSSpiHandle, Sensor, pBuffer, WriteAddr, nBytesToWrite ) )
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

/**
 * @brief  Initializes the SPI communication with the BlueNRG
 *         Expansion Board.
 * @retval None
 */
HAL_StatusTypeDef SensorsSpiInit(void)
{
  HAL_StatusTypeDef ret_val = HAL_ERROR;
  
  if(HAL_SPI_GetState(&SENSORSSpiHandle) == HAL_SPI_STATE_RESET)
    {
    /* SPI peripheral configuration */
    SENSORSSpiHandle.Instance = SENSORS_SPI_INSTANCE;
    SENSORSSpiHandle.Init.Mode = SENSORS_SPI_MODE;
    SENSORSSpiHandle.Init.Direction = SENSORS_SPI_DIRECTION;
    SENSORSSpiHandle.Init.DataSize = SENSORS_SPI_DATASIZE;
    SENSORSSpiHandle.Init.CLKPolarity = SENSORS_SPI_CLKPOLARITY;
    SENSORSSpiHandle.Init.CLKPhase = SENSORS_SPI_CLKPHASE;
    SENSORSSpiHandle.Init.NSS = SENSORS_SPI_NSS;
    SENSORSSpiHandle.Init.FirstBit = SENSORS_SPI_FIRSTBIT;
    SENSORSSpiHandle.Init.TIMode = SENSORS_SPI_TIMODE;
    SENSORSSpiHandle.Init.CRCPolynomial = SENSORS_SPI_CRCPOLYNOMIAL;
    SENSORSSpiHandle.Init.BaudRatePrescaler = SENSORS_SPI_BAUDRATEPRESCALER;
    SENSORSSpiHandle.Init.CRCCalculation = SENSORS_SPI_CRCCALCULATION;
    ret_val = HAL_SPI_Init(&SENSORSSpiHandle);
    }
  if(HAL_SPI_GetState(&SENSORSSpiHandle) == HAL_SPI_STATE_READY)
    {
      ret_val = HAL_OK ;
    }
  return ret_val ;
}


       
/**
  * @brief  SPI error treatment function.
  * @retval None
  */
static void SPIx_Error (void)
{
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&SENSORSSpiHandle);
  
  /* Re-Initiaize the SPI communication BUS */
  SensorsSpiInit();
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

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
