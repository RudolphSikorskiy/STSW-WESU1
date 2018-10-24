/**
  ******************************************************************************
  * @file    wesu_magneto.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   This file provides a set of functions needed to manage 
  *          the LIS3MDL sensor.
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
#include "wesu_magneto.h"



/** @addtogroup BSP	        BSP
 * @{	
 */

/** @addtogroup STEVAL-WESU1	STEVAL-WESU1
 * @{
 */

/** @addtogroup SENSORS       SENSORS
 * @{
 */

/** @addtogroup WESU_MAGNETO	Magnetometer
 * @{
 */

/** @addtogroup WESU_MAGNETO_Private_Variables Private variables
 * @{
 */

static DrvContextTypeDef MAGNETO_SensorHandle[ MAGNETO_SENSORS_MAX_NUM ];       //!< Magnetometer Handle.
static MAGNETO_Data_t MAGNETO_Data[ MAGNETO_SENSORS_MAX_NUM ];                  //!< Magnetometer - all.
static LIS3MDL_Data_t LIS3MDL_0_Data;                                           //!< Magnetometer - sensor 0.

/**
 * @}
 */

/** @addtogroup WESU_MAGNETO_Private_FunctionPrototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef BSP_LIS3MDL_MAGNETO_Init( void **handle );

/**
 * @}
 */


/** @addtogroup WESU_MAGNETO_Exported_Functions Exported functions
 * @{
 */

/**
 * @brief Initialize a magnetometer sensor
 * @param id the magnetometer sensor identifier
 * @param handle the device handle
 * @retval MAGNETO_OK in case of success, MAGNETO_ERROR otherwise
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Init( MAGNETO_ID_t id, void **handle )
{
 switch(id)
  {
    case MAGNETO_SENSORS_AUTO:
    default:
    {
      if( BSP_LIS3MDL_MAGNETO_Init(handle)  == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
    case LIS3MDL_0:
    {
      if( BSP_LIS3MDL_MAGNETO_Init(handle)  == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
  }
 return COMPONENT_OK;
}


/**
 * @brief Initialize the LIS3MDL magnetometer sensor
 * @param handle the device handle
 * @retval MAGNETO_OK in case of success, MAGNETO_ERROR otherwise
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_LIS3MDL_MAGNETO_Init( void **handle )
{
  if(MAGNETO_SensorHandle[ LIS3MDL_0 ].isInitialized == 1)
  {
    /* We have reached the max num of instance for this component */
    return COMPONENT_OK;
  }
  
  MAGNETO_Drv_t *driver = NULL;
  if ( Sensor_IO_Init() == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  /* Setup sensor handle. */
  MAGNETO_SensorHandle[ LIS3MDL_0 ].who_am_i      = LIS3MDL_MAG_WHO_AM_I;
  MAGNETO_SensorHandle[ LIS3MDL_0 ].address       = LIS3MDL_MAG_I2C_ADDRESS_HIGH;
  MAGNETO_SensorHandle[ LIS3MDL_0 ].instance      = LIS3MDL_0;
  MAGNETO_SensorHandle[ LIS3MDL_0 ].isInitialized = 0;
  MAGNETO_SensorHandle[ LIS3MDL_0 ].isEnabled     = 0;
  MAGNETO_SensorHandle[ LIS3MDL_0 ].isCombo       = 0;
  MAGNETO_SensorHandle[ LIS3MDL_0 ].pData         = ( void * )&MAGNETO_Data[ LIS3MDL_0 ];
  MAGNETO_SensorHandle[ LIS3MDL_0 ].pVTable       = ( void * )&LIS3MDLDrv;
  MAGNETO_SensorHandle[ LIS3MDL_0 ].pExtVTable    = 0;
  
  MAGNETO_Data[ LIS3MDL_0 ].pComponentData = ( void * )&LIS3MDL_0_Data;
  MAGNETO_Data[ LIS3MDL_0 ].pExtData       = 0;
  
  *handle = (void *)&MAGNETO_SensorHandle[ LIS3MDL_0 ];
  
  driver = ( MAGNETO_Drv_t * )((DrvContextTypeDef *)(*handle))->pVTable;
  
  if ( driver->Init == NULL )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }
  
  if ( driver->Init( (DrvContextTypeDef *)(*handle) ) == COMPONENT_ERROR )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}


/**
 * @brief Deinitialize a magnetometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_DeInit( void **handle )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( driver->DeInit == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( driver->DeInit( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  memset(ctx, 0, sizeof(DrvContextTypeDef));
  
  *handle = NULL;
  
  return COMPONENT_OK;
}

/**
 * @brief Enable magnetometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Sensor_Enable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( driver->Sensor_Enable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( driver->Sensor_Enable( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Disable magnetometer sensor
 * @param handle the device handle
 * @retval MAGNETO_OK in case of success, MAGNETO_ERROR otherwise
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Sensor_Disable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( driver->Sensor_Disable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( driver->Sensor_Disable( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}


/**
 * @brief Reboot Magnetometer sensor with regs default values
 * @param handle the device handle
 * @param newValue  LIS3MDL_MAG_REBOOT_NO or 
 *                  LIS3MDL_MAG_REBOOT_YES
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Sensor_Reboot(void *handle, LIS3MDL_MAG_REBOOT_t newValue)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( driver->Sensor_Disable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( LIS3MDL_MAG_W_Reboot(handle,newValue) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

/**
 * @brief Reboot Magnetometer sensor with regs default values
 * @param handle the device handle
 * @param newValue    LIS3MDL_MAG_SOFT_RST_NO  or
 *                    LIS3MDL_MAG_SOFT_RST_YES
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Sensor_SoftRST(void *handle, LIS3MDL_MAG_SOFT_RST_t newValue)
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( driver->Sensor_Disable == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( LIS3MDL_MAG_W_SoftRST(handle,newValue) == MEMS_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

/**
 * @brief Check if the magnetometer sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_IsInitialized( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  *status = ctx->isInitialized;
  
  return COMPONENT_OK;
}


/**
 * @brief Check if the magnetometer sensor is enabled
 * @param handle the device handle
 * @param status the pointer to the enable status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_IsEnabled( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  *status = ctx->isEnabled;
  
  return COMPONENT_OK;
}


/**
 * @brief Check if the magnetometer sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_IsCombo( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  *status = ctx->isCombo;
  
  return COMPONENT_OK;
}


/**
 * @brief Get the magnetometer sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_Instance( void *handle, uint8_t *instance )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  if ( instance == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  *instance = ctx->instance;
  
  return COMPONENT_OK;
}

/**
 * @brief Get the WHO_AM_I ID of the magnetometer sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_WhoAmI( void *handle, uint8_t *who_am_i )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( who_am_i == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_WhoAmI == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_WhoAmI( ctx, who_am_i ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Check the WHO_AM_I ID of the magnetometer sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Check_WhoAmI( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( driver->Check_WhoAmI == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Check_WhoAmI( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}

/**
 * @brief Get the magnetometer sensor axes
 * @param handle the device handle
 * @param magnetic_field pointer where the values of the axes are written [mgauss]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_Axes( void *handle, SensorAxes_t *magnetic_field )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( magnetic_field == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Axes == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Axes( ctx, magnetic_field ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Get the magnetometer sensor raw axes
 * @param handle the device handle
 * @param value pointer where the raw values of the axes are written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_AxesRaw( void *handle, SensorAxesRaw_t *value )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( value == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( driver->Get_AxesRaw == NULL )
  {
    return COMPONENT_ERROR;
  }
  
  if ( driver->Get_AxesRaw( ctx, value ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}




/**
 * @brief Get the magnetometer sensor sensitivity
 * @param handle the device handle
 * @param sensitivity pointer where the sensitivity value is written [LSB/gauss]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_Sensitivity( void *handle, float *sensitivity )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( sensitivity == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Sensitivity == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Sensitivity( ctx, sensitivity ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Get the magnetometer sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_ODR( void *handle, float *odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( odr == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_ODR == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_ODR( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Set the magnetometer sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Set_ODR( void *handle, SensorOdr_t odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( driver->Set_ODR == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_ODR( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Set the magnetometer sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Set_ODR_Value( void *handle, float odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( driver->Set_ODR_Value == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_ODR_Value( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Get the magnetometer sensor full scale
 * @param handle the device handle
 * @param fullScale pointer where the full scale is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Get_FS( void *handle, float *fullScale )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( fullScale == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_FS == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_FS( ctx, fullScale ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Set the magnetometer sensor full scale
 * @param handle the device handle
 * @param fullScale the functional full scale to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Set_FS( void *handle, SensorFs_t fullScale )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( driver->Set_FS == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_FS( ctx, fullScale ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief Set the magnetometer sensor full scale
 * @param handle the device handle
 * @param fullScale the full scale value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Set_FS_Value( void *handle, float fullScale )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  MAGNETO_Drv_t *driver = NULL;
  
  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }
  
  driver = ( MAGNETO_Drv_t * )ctx->pVTable;
  
  if ( driver->Set_FS_Value == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_FS_Value( ctx, fullScale ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}


/**
 * @brief Force the magnetometer sensor in Low Power Mode full scale
 * @param handle the device handle
 * @param newValue LIS3MDL_MAG_LP_ENABLE or 
 *                 LIS3MDL_MAG_LP_DISABLE
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_MAGNETO_Set_LP_Mode( void *handle, LIS3MDL_MAG_LP_t newValue )
{
  if (newValue == LIS3MDL_MAG_LP_ENABLE)
    { 
      if (LIS3MDL_MAG_W_FastLowPowerXYZ(handle, newValue) == MEMS_ERROR)
      {
        return COMPONENT_ERROR;
      }      

      if (LIS3MDL_MAG_W_OperatingModeXY(handle,LIS3MDL_MAG_OM_LOW_POWER) == MEMS_ERROR)
      {
        return COMPONENT_ERROR;
      }
      
      if (LIS3MDL_MAG_W_OperatingModeZ(handle,LIS3MDL_MAG_OMZ_LOW_POWER) == MEMS_ERROR)
      {
        return COMPONENT_ERROR;
      }
      
      if (BSP_MAGNETO_Set_FS_Value(handle,MAG_FullScale_LOWPOWER) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      } 
      
      if (BSP_MAGNETO_Set_ODR_Value(handle,MAG_OutputDataRate_LOWPOWER) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }
    }
  else  // LP DISABLING
    { 
      if (LIS3MDL_MAG_W_FastLowPowerXYZ(handle, newValue) == MEMS_ERROR)
      {
        return COMPONENT_ERROR;
      }      
      
      if (LIS3MDL_MAG_W_OperatingModeXY(handle,LIS3MDL_MAG_OM_MEDIUM) == MEMS_ERROR)
      {
        return COMPONENT_ERROR;
      }
      
      if (LIS3MDL_MAG_W_OperatingModeZ(handle,LIS3MDL_MAG_OMZ_MEDIUM) == MEMS_ERROR)
      {
        return COMPONENT_ERROR;
      }
      
      if (BSP_MAGNETO_Set_ODR_Value(handle,MAG_OutputDataRate_DEFAULT) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }
      
      if (BSP_MAGNETO_Set_FS_Value(handle,MAG_OutputDataRate_DEFAULT) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }  
    } 
  return COMPONENT_OK;
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
