/**
  ******************************************************************************
  * @file    wesu_magneto.h
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Header for wesu_magneto.c module
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
#ifndef __WESU_MAGNETO_H
#define __WESU_MAGNETO_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "LIS3MDL_MAG_driver_HL.h"
#include "platform_config.h"

/** @addtogroup BSP	        BSP
 * @{	
 */

/** @addtogroup STEVAL-WESU1    STEVAL-WESU1
 * @{
 */

/** @addtogroup SENSORS       SENSORS
 * @{
 */

/** @addtogroup WESU_MAGNETO	Magnetometer
 * @{
 */

/** @addtogroup WESU_MAGNETO_Public_Types Public types
  * @{
  */
  
/**
  * @brief  Struct that specify the Magnetic sensor on Board
  */
typedef enum
{
  MAGNETO_SENSORS_AUTO = -1,     /* Always first element and equal to -1 */
  LIS3MDL_0                      /* Default on board. */
} MAGNETO_ID_t;

/**
 * @}
 */

/** @addtogroup WESU_MAGNETO_Public_Defines Public defines
  * @{
  */

#define MAGNETO_SENSORS_MAX_NUM 1       /*!< Number of Magnetic sensor on Board */

/**
 * @}
 */


/** @addtogroup WESU_MAGNETO_Public_Function_Prototypes Public function prototypes
 * @{
 */

/* Sensor Configuration Functions */
DrvStatusTypeDef BSP_MAGNETO_Init( MAGNETO_ID_t id, void **handle );
DrvStatusTypeDef BSP_MAGNETO_DeInit( void **handle );
DrvStatusTypeDef BSP_MAGNETO_Sensor_Enable( void *handle );
DrvStatusTypeDef BSP_MAGNETO_Sensor_Disable( void *handle );
DrvStatusTypeDef BSP_MAGNETO_IsInitialized( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_MAGNETO_IsEnabled( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_MAGNETO_IsCombo( void *handle, uint8_t *status );
DrvStatusTypeDef BSP_MAGNETO_Get_Instance( void *handle, uint8_t *instance );
DrvStatusTypeDef BSP_MAGNETO_Get_WhoAmI( void *handle, uint8_t *who_am_i );
DrvStatusTypeDef BSP_MAGNETO_Check_WhoAmI( void *handle );
DrvStatusTypeDef BSP_MAGNETO_Get_Axes( void *handle, SensorAxes_t *magnetic_field );
DrvStatusTypeDef BSP_MAGNETO_Get_AxesRaw( void *handle, SensorAxesRaw_t *value );
DrvStatusTypeDef BSP_MAGNETO_Get_Sensitivity( void *handle, float *sensitivity );
DrvStatusTypeDef BSP_MAGNETO_Get_ODR( void *handle, float *odr );
DrvStatusTypeDef BSP_MAGNETO_Set_ODR( void *handle, SensorOdr_t odr );
DrvStatusTypeDef BSP_MAGNETO_Set_ODR_Value( void *handle, float odr );
DrvStatusTypeDef BSP_MAGNETO_Get_FS( void *handle, float *fullScale );
DrvStatusTypeDef BSP_MAGNETO_Set_FS( void *handle, SensorFs_t fullScale );
DrvStatusTypeDef BSP_MAGNETO_Set_FS_Value( void *handle, float fullScale );

DrvStatusTypeDef BSP_MAGNETO_Set_LP_Mode( void *handle, LIS3MDL_MAG_LP_t newValue );
DrvStatusTypeDef BSP_MAGNETO_Sensor_Reboot(void *handle, LIS3MDL_MAG_REBOOT_t newValue);
DrvStatusTypeDef BSP_MAGNETO_Sensor_SoftRST(void *handle, LIS3MDL_MAG_SOFT_RST_t newValue);



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

#endif /* __WESU_MAGNETO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
