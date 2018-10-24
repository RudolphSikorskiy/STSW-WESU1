/**
  ******************************************************************************
  * @file    WESU_DEMO/Src/algorithms.c
  * @author  System Lab
  * @version V1.1.0
  * @date    15-Sept-2016
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

#include "main.h"
#include "console.h"
#include <stdio.h>
#include "algorithms.h"
#include "BlueST_Protocol.h"
#include "wesu_config.h"
#include "../../../../../Middlewares/ST/STM32_OSX_MotionCP_Library/osx_license.h"
#include "../../../../../Middlewares/ST/STM32_OSX_MotionAR_Library/osx_license.h"
#include "../../../../../Middlewares/ST/STM32_OSX_MotionFX_Library/osx_license.h" 



/** @addtogroup WeSU_Demo       WeSU Demo      
  * @{
  */

/** @addtogroup WeSU_User               WeSU User
  * @{
  */
   
/** @addtogroup Algorithms
  * @{
  */   
 
   
/** @addtogroup ALGORITHMS_Private_Defines    ALGORITHMS Private Defines
  * @{
  */   
    
#define ACCEL_VALIDATION(a)         ((a & 0x3F) == 0x3F)        //!< Bitmask to validate when magnetometer calibration can be done

#define FROM_MDPS_TO_DPS    0.001f              
#define FROM_MGAUSS_TO_UT50 (0.1f/50.0f)        
#define SAMPLETODISCARD 15                      
#define MOTIONFX_ENGINE_DELTATIME 0.01f         
#define GBIAS_ACC_TH_SC_6X (2.0f*0.000765f)     
#define GBIAS_GYRO_TH_SC_6X (2.0f*0.002f)       
#define GBIAS_MAG_TH_SC_6X (2.0f*0.001500f)     
#define GBIAS_ACC_TH_SC_9X (2.0f*0.000765f)     
#define GBIAS_GYRO_TH_SC_9X (2.0f*0.002f)       
#define GBIAS_MAG_TH_SC_9X (2.0f*0.001500f)     

/**
  * @}
  */


/** @defgroup ALGORITHMS_Private_Variables    ALGORITHMS Private Variables
  * @{
  */   
   
float MagOff[3]={0.0f,0.0f,0.0f};                               //!< Array of magnetometer offset
float MagGain[3]={1.0f,1.0f,1.0f};                              //!< Array of magnetometer gain

volatile  osxMFX_output iDataOUT_FX;                            
volatile  osxMFX_input iDataIN;                                 
volatile int sampleToDiscard = SAMPLETODISCARD;                 
float MotionFX_engine_deltatime = MOTIONFX_ENGINE_DELTATIME;    
int discardedCount = 0;                                         
extern osxMFX_calibFactor magOffset;                             
uint8_t calibIndex = 0;         // run calibration @ 25Hz       
unsigned char isCal = 0;                                        
unsigned char isStartFxCalib = 0;                               
osxMFX_knobs iKnobs;                                            
osxMFX_knobs* ipKnobs;                                          
uint8_t reset_day = 0;
uint32_t calib_counter;
uint32_t calib_counter_saved;
/**
  * @}
  */

  
/** @defgroup ALGORITHMS_Imported_Variables    ALGORITHMS Imported Variables
  * @{
  */ 

  extern void *ACCELERO_handle;
  extern DrvStatusTypeDef sensorStatusAccelero;
 
/**
  * @}
  */

/** @defgroup ALGORITHMS_Private_Functions    ALGORITHMS Private Functions
  * @{
  */ 
static unsigned char ReCallCalibrationFromMemory(void);
 /**
  * @}
  */

/** @addtogroup ALGORITHMS_Exported_Functions
  * @{
  */ 

#if USE_CUSTOM_ALGORITHMFX

/**
  * @brief  Motion FX Magnetometer Calibration
  * @retval None
  */

void FxMagCalibration ()
{
  if(isCal != 0x01)
  {
    /* Run Compass Calibration @ 25Hz */
    calibIndex++;
    if (calibIndex == 4)
    {
      calib_counter++;
      SensorAxes_t ACC_Loc, MAG_Loc;
      calibIndex = 0;
      ACC_Loc.AXIS_X = (int32_t)ACC_READ.AXIS_X;
      ACC_Loc.AXIS_Y = (int32_t)ACC_READ.AXIS_Y;
      ACC_Loc.AXIS_Z = (int32_t)ACC_READ.AXIS_Z;
      MAG_Loc.AXIS_X = (int32_t)MAG_READ.AXIS_X;
      MAG_Loc.AXIS_Y = (int32_t)MAG_READ.AXIS_Y;
      MAG_Loc.AXIS_Z = (int32_t)MAG_READ.AXIS_Z;
      osx_MotionFX_compass_saveAcc(ACC_Loc.AXIS_X, ACC_Loc.AXIS_Y, ACC_Loc.AXIS_Z); /* Accelerometer data ENU systems coordinate  */
      osx_MotionFX_compass_saveMag(MAG_Loc.AXIS_X, MAG_Loc.AXIS_Y, MAG_Loc.AXIS_Z); /* Magnetometer data ENU systems coordinate */
      
      osx_MotionFX_compass_run();
      
      //          isStartFxCalib=0;
    }
    
    /* Check if is calibrated */
    isCal = osx_MotionFX_compass_isCalibrated();
    if(isCal == 0x01)
    {
      calib_counter_saved= calib_counter;
      calib_counter=0;
      SESSION_REG(xFXCalib_Status) = 1;
      /* Get new magnetometer offset */
      osx_MotionFX_getCalibrationData(&magOffset);
      
      /* Save the calibration in Memory */
      BSP_PERSREGS_WRITE(FX_MAG_CALIB_OFFSET_0_REG, (uint8_t*)&magOffset, 11/*3 * N_REGS_FLOAT*/);
      
      DBG_PRINTF_ALGORITHMS(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nSave calibration data");
    }
  }
  if (isStartFxCalib==1)
  {  
    isStartFxCalib=0;
    isCal=0;
    SESSION_REG(xFXCalib_Status) = 0;
    /* Reset the Calibration */
    osx_MotionFX_compass_forceReCalibration();
    uint8_t reset[22];for (int i=0; i<22; i++)
    {        
      reset[i]=0;
    }
    DBG_PRINTF_ALGORITHMS(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nReset calibration data");
    BSP_PERSREGS_WRITE(FX_MAG_CALIB_OFFSET_0_REG, (uint8_t*)&reset, 11/*3 * N_REGS_FLOAT*/);
    /* Reset Calibation offset */
    magOffset.magOffX = magOffset.magOffY= magOffset.magOffZ=0;
  }
}

/**
  * @brief  Process Motion FX algorithm
  * @retval None
  */
void FX_Process()
{
  if(PROCESS_MOTION_FX_ENABLED())
  {
    iDataIN.gyro[0] = GYRO_READ.AXIS_X  * FROM_MDPS_TO_DPS;
    iDataIN.gyro[1] = GYRO_READ.AXIS_Y  * FROM_MDPS_TO_DPS;
    iDataIN.gyro[2] = GYRO_READ.AXIS_Z  * FROM_MDPS_TO_DPS;
    
    iDataIN.acc[0] = ACC_READ.AXIS_X * FROM_MG_TO_G;
    iDataIN.acc[1] = ACC_READ.AXIS_Y * FROM_MG_TO_G;
    iDataIN.acc[2] = ACC_READ.AXIS_Z * FROM_MG_TO_G;
    
    
    iDataIN.mag[0] = (MAG_READ.AXIS_X - magOffset.magOffX) * FROM_MGAUSS_TO_UT50;
    iDataIN.mag[1] = (MAG_READ.AXIS_Y - magOffset.magOffY) * FROM_MGAUSS_TO_UT50;
    iDataIN.mag[2] = (MAG_READ.AXIS_Z - magOffset.magOffZ) * FROM_MGAUSS_TO_UT50;
    
    
    if(discardedCount == sampleToDiscard)
    {
      osx_MotionFX_update((osxMFX_output*)&iDataOUT_FX, (osxMFX_input*)&iDataIN, MotionFX_engine_deltatime, NULL);
      
    }
    else
    {
      discardedCount++;
    } 
  #if (USE_TIMER == 0)

    if(USE_FX_MAG_CALIBRATION)
      FxMagCalibration ();

  #endif //USE_TIMER
  }
}

/**
  * @brief  Configure Motion FX algorithm at startup
  * @retval None
  */
void FX_StartupConfig()
{
  osx_lm_result_t nRet = OSX_FAILED;
  if(PROCESS_MOTION_FX_ENABLED())
  {
    if(OSX_FAILED == PreCheckLicense((uint8_t*)osx_mfx_license))
    {
      //license not valid, load license from EEPROM
      BSP_LICENSE_Read(ALGORITHMFX_LICENSE_NUMBER, (uint8_t*)osx_mfx_license);
    }
    else
    {
      BSP_LICENSE_Write(ALGORITHMFX_LICENSE_NUMBER, (uint8_t*)osx_mfx_license);
    }
    
    DBG_PRINTF_ALGORITHMS(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nPRE_Init FX");
    if(GET_MFX_INIT_STATE() < LICENSE_INIT_MAX_TRIES)
    {
      SET_MFX_INIT_STATE();
      nRet = (osx_lm_result_t)osx_MotionFX_initialize();
      RESET_MFX_INIT_STATE();
    }
    if( nRet != OSX_SUCCESS )
    {
      DBG_PRINTF_ALGORITHMS(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nERROR FX");
      SESSION_REG(xMFX_Status) = OSX_FAILED;
    }
    else
    {
      char VersionMotionFX[36];
      osx_MotionFX_getLibVersion(VersionMotionFX);
      DBG_PRINTF_ALGORITHMS(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\n\rEnabled %s",VersionMotionFX);
      SESSION_REG(xMFX_Status) = OSX_SUCCESS;
    }
    
    if( nRet != OSX_SUCCESS )
    {
      DISABLE_PERMANENT_MFX();
    }
    else
    {
      
      magOffset.magOffX = 0;
      magOffset.magOffY = 0;
      magOffset.magOffZ = 0;
      
      ipKnobs = &iKnobs;
      
      
      osx_MotionFX_compass_init();
      osx_MotionFX_getKnobs(ipKnobs);
      ipKnobs->gbias_acc_th_sc_6X = GBIAS_ACC_TH_SC_6X;
      ipKnobs->gbias_gyro_th_sc_6X = GBIAS_GYRO_TH_SC_6X;
      ipKnobs->gbias_mag_th_sc_6X = GBIAS_MAG_TH_SC_6X;
      
      ipKnobs->gbias_acc_th_sc_9X = GBIAS_ACC_TH_SC_9X;
      ipKnobs->gbias_gyro_th_sc_9X = GBIAS_GYRO_TH_SC_9X;
      ipKnobs->gbias_mag_th_sc_9X = GBIAS_MAG_TH_SC_9X;
      
      ipKnobs->acc_orientation[0] = 'n';
      ipKnobs->acc_orientation[1] = 'w';
      ipKnobs->acc_orientation[2] = 'u';
      
      ipKnobs->gyro_orientation[0] = 'n';
      ipKnobs->gyro_orientation[1] = 'w';
      ipKnobs->gyro_orientation[2] = 'u';
      
      ipKnobs->mag_orientation[0] = 'n';
      ipKnobs->mag_orientation[1] = 'w';
      ipKnobs->mag_orientation[2] = 'u';
      
      ipKnobs->output_type = OSXMFX_ENGINE_OUTPUT_ENU;
      
      
      ipKnobs->LMode = 1;
      
      ipKnobs->modx = 4;
      
      osx_MotionFX_setKnobs(ipKnobs);
      
      osx_MotionFX_enable_6X(OSXMFX_ENGINE_DISABLE);
      osx_MotionFX_enable_9X(OSXMFX_ENGINE_DISABLE);
      
      
      /* Number of Sample to Discard */
      sampleToDiscard = SAMPLETODISCARD;
      discardedCount = 0;
      osx_MotionFX_enable_9X(OSXMFX_ENGINE_ENABLE);   
      /* Control if the calibration is already available in memory */
      ReCallCalibrationFromMemory();
    }
  }
}


/**
 * @brief  Get MotionFX engine data OUT
 * @param  None
 * @retval osxMFX_output pointer to data output
 */
osxMFX_output* MotionFX_manager_getDataOUT(void)
{
  return (osxMFX_output*)&iDataOUT_FX;
}

/**
  * @brief  Process Motion FX algorithm routine with high priority
  * @retval None
  */  

void FX_Process_High() 
{
  if(PROCESS_MOTION_FX_ENABLED())
  { 
    iDataIN.gyro[0] = GYRO_READ.AXIS_X  * FROM_MDPS_TO_DPS;
    iDataIN.gyro[1] = GYRO_READ.AXIS_Y  * FROM_MDPS_TO_DPS;
    iDataIN.gyro[2] = GYRO_READ.AXIS_Z  * FROM_MDPS_TO_DPS;
    
    iDataIN.acc[0] = ACC_READ.AXIS_X * FROM_MG_TO_G;
    iDataIN.acc[1] = ACC_READ.AXIS_Y * FROM_MG_TO_G;
    iDataIN.acc[2] = ACC_READ.AXIS_Z * FROM_MG_TO_G;
    
    iDataIN.mag[0] = (MAG_READ.AXIS_X - magOffset.magOffX) * FROM_MGAUSS_TO_UT50;
    iDataIN.mag[1] = (MAG_READ.AXIS_Y - magOffset.magOffY) * FROM_MGAUSS_TO_UT50;
    iDataIN.mag[2] = (MAG_READ.AXIS_Z - magOffset.magOffZ) * FROM_MGAUSS_TO_UT50;
    if(discardedCount == sampleToDiscard)
    {
      
      osx_MotionFX_propagate((osxMFX_output*)&iDataOUT_FX, (osxMFX_input*)&iDataIN, MotionFX_engine_deltatime); 
    }
    
  }
}


/**
 * @brief  Check if there are a valid Calibration Values in Memory and read them
 * @param None
 * @retval unsigned char Success/Not Success
 */
static unsigned char ReCallCalibrationFromMemory(void)
{
  /* ReLoad the Calibration Values from FLASH */
  unsigned char Success=1;

  BSP_PERSREGS_READ(FX_MAG_CALIB_OFFSET_0_REG, (uint8_t*)&magOffset, 11);
  DBG_PRINTF_ALGORITHMS(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\n\rFX ReCallCalibrationFromMemory ongoing");

  
    if( (USE_FX_MAG_CALIBRATION) && (magOffset.magOffX != 0)&& (magOffset.magOffY != 0)&& (magOffset.magOffZ != 0))
  {
    /* Set the Calibration Structure */
    osx_MotionFX_setCalibrationData(&magOffset);

    /* Control the calibration status */
    isCal = osx_MotionFX_compass_isCalibrated();

    if (isCal)
    {
       SESSION_REG(xFXCalib_Status) = 1;
       DBG_PRINTF_ALGORITHMS(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\n\rCalibration data loaded, board calibrated");
    }
    else
    {
      DBG_PRINTF_ALGORITHMS(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\n\rCalibration data loaded, board not calibrated");
    }

                 
  } else {

    isCal=0;
    DBG_PRINTF_ALGORITHMS(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\n\rNo calibration data");
  }

  return Success;
}
#endif //USE_CUSTOM_ALGORITHMFX

/**
  * @brief  Check if license is valid (!=0)
  * @param pLicense pointer to the license
  * @retval OSX_SUCCESS if license found, OSX_FAILED otherwise
  */
osx_lm_result_t PreCheckLicense(uint8_t* pLicense)
{
  uint8_t nTempLicense[48] = {0};
  if(0 == memcmp(pLicense,nTempLicense,LICENSE_SIZE))
  {
    return OSX_FAILED;
  }
  else
  {
    return OSX_SUCCESS;
  }
}

#if USE_CUSTOM_ALGORITHM1

/**
  * @brief  Algorithm1 initialization
  * @retval NONE
  */
void Algorithm1_init()
{
  osx_lm_result_t nRet = OSX_FAILED;
  
  if(PROCESS_MOTION_AR_ENABLED())
  {
    if(OSX_FAILED == PreCheckLicense((uint8_t*)osx_mar_license))
    {
      //license not valid, load license from EEPROM
      BSP_LICENSE_Read(ALGORITHM1_LICENSE_NUMBER, (uint8_t*)osx_mar_license);
    }
    else
    {
      BSP_LICENSE_Write(ALGORITHM1_LICENSE_NUMBER, (uint8_t*)osx_mar_license);
    }
    
    DBG_PRINTF_ALGORITHMS(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nPRE_Init AR");
    if(GET_MAR_INIT_STATE() < LICENSE_INIT_MAX_TRIES)
    {
      SET_MAR_INIT_STATE();
      nRet = (osx_lm_result_t)osx_MotionAR_Initialize();
      RESET_MAR_INIT_STATE();
    }
    if( nRet != OSX_SUCCESS )
    {
      DBG_PRINTF_ALGORITHMS(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nERROR AR");
      SESSION_REG(xMAR_Status) = OSX_FAILED;
    }
    else
    {
      char VersionMotionAR[36];
      osx_MotionAR_GetLibVersion(VersionMotionAR);
      DBG_PRINTF_ALGORITHMS(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\n\rEnabled %s",VersionMotionAR);
      SESSION_REG(xMAR_Status) = OSX_SUCCESS;
    }
    
    if( nRet != OSX_SUCCESS )
    {
      DISABLE_PERMANENT_MAR();
    }
    
    /* DS3 */
    char acc_orientation[3];
    strcpy(&acc_orientation[0], "nwu");
    osx_MotionAR_SetOrientation_Acc(acc_orientation);
  }
}

/**
  * @brief  Algorithm1 run function
  * @retval NONE
  */
void Algorithm1_run()
{
  if(PROCESS_MOTION_AR_ENABLED())
  {
    if(READ_SENSORS_ACC_ENABLED())
    {
      /* Acceleration can be retrieved using ACC_READ define
      */
      osx_MAR_input_t iDataIN;
      
      iDataIN.AccX = ACC_READ.AXIS_X * FROM_MG_TO_G;
      iDataIN.AccY = ACC_READ.AXIS_Y * FROM_MG_TO_G;
      iDataIN.AccZ = ACC_READ.AXIS_Z * FROM_MG_TO_G;
      
      SESSION_REG(xMAR_Value) = osx_MotionAR_Update(&iDataIN);
    }
    else
    {
      SESSION_REG(xMAR_Value) = 0;
    }
  }
}
#endif //USE_CUSTOM_ALGORITHM1

#if USE_CUSTOM_ALGORITHM2

/**
  * @brief  Algorithm2 initialization
  * @retval NONE
  */
void Algorithm2_init()
{
  uint8_t nRet = OSX_FAILED;
  
  if(PROCESS_MOTION_CP_ENABLED())
  {
    if(OSX_FAILED == PreCheckLicense((uint8_t*)osx_mcp_license))
    {
      //license not valid, load license from EEPROM
      BSP_LICENSE_Read(ALGORITHM2_LICENSE_NUMBER, (uint8_t*)osx_mcp_license);
    }
    else
    {
      BSP_LICENSE_Write(ALGORITHM2_LICENSE_NUMBER, (uint8_t*)osx_mcp_license);
    }
    
    DBG_PRINTF_ALGORITHMS(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nPRE_Init CP");
    if(GET_MCP_INIT_STATE() < LICENSE_INIT_MAX_TRIES)
    {
      SET_MCP_INIT_STATE();
      nRet = osx_MotionCP_Initialize();
      RESET_MCP_INIT_STATE();
    }
    if( nRet != OSX_SUCCESS)
    {
      DBG_PRINTF_ALGORITHMS(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nERROR CP");
      SESSION_REG(xMCP_Status) = OSX_FAILED;
    }
    else
    {
      char VersionMotionCP[36];
      osx_MotionCP_GetLibVersion(VersionMotionCP);
      DBG_PRINTF_ALGORITHMS(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\n\rEnabled %s",VersionMotionCP);
      SESSION_REG(xMCP_Status) = OSX_SUCCESS;
    }
    
    if( nRet != OSX_SUCCESS )
    {
      DISABLE_PERMANENT_MCP();
    }
    
    /* DS3 */
    char acc_orientation[3];
    strcpy(&acc_orientation[0], "nwu");
    osx_MotionCP_SetOrientation_Acc(acc_orientation);
  }
}

/**
  * @brief  Algorithm2 run function
  * @retval NONE
  */
void Algorithm2_run()
{
  if(PROCESS_MOTION_CP_ENABLED())
  {
    if(READ_SENSORS_ACC_ENABLED())
    {
      /* Acceleration can be retrieved using ACC_READ define
      */
      osx_MCP_input_t iDataIN;
      
      iDataIN.AccX = ACC_READ.AXIS_X * FROM_MG_TO_G;
      iDataIN.AccY = ACC_READ.AXIS_Y * FROM_MG_TO_G;
      iDataIN.AccZ = ACC_READ.AXIS_Z * FROM_MG_TO_G;
      
      SESSION_REG(xMCP_Value) = osx_MotionCP_Update(&iDataIN);
    }
    else
    {
      SESSION_REG(xMCP_Value) = 0;
    }
  }
}
#endif //USE_CUSTOM_ALGORITHM2

#if USE_CUSTOM_ALGORITHM3

/**
  * @brief  Initialize algorithm 3
  * @retval None
  */
void Algorithm3_init()
{
   extern void *ACCELERO_handle;
   extern DrvStatusTypeDef sensorStatusAccelero;
  
  if(PROCESS_PEDOMETER_ENABLED())
  {
    sensorStatusAccelero = BSP_ACCELERO_Enable_Pedometer_Ext( ACCELERO_handle );
  }
  else
  {
    sensorStatusAccelero = BSP_ACCELERO_Disable_Pedometer_Ext( ACCELERO_handle );
  }
}
#endif // USE_CUSTOM_ALGORITHM3

#if USE_CUSTOM_ALGORITHM4
/**
  * @brief  Initialize algorithm 4
  * @retval None
  */
void Algorithm4_init()
{

}
#endif // USE_CUSTOM_ALGORITHM4
  
#if USE_CUSTOM_ACCEVENT

void AccEvent_Deinit()
{
   extern void *ACCELERO_handle;
   extern DrvStatusTypeDef sensorStatusAccelero;
  
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_PEDOMETER))
  {
  }
  else
  {
    sensorStatusAccelero = BSP_ACCELERO_Disable_Pedometer_Ext( ACCELERO_handle );
  }
  
  
  //for AccEventCharHandle
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_6DORIENTATION))
  {
    /* Enable 6D orientation detection */
  }
  else
  {
    if(BSP_ACCELERO_Disable_6D_Orientation_Ext(ACCELERO_handle)==COMPONENT_ERROR){}
    if(LSM6DS3_ACC_GYRO_W_6DEvOnInt1( ACCELERO_handle, LSM6DS3_ACC_GYRO_INT1_6D_DISABLED ) == MEMS_ERROR ) {}
  }
  
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_SINGLE_TAP))
  {
    /* Enable Single Tap detection */
  }
  else
  {
    if(BSP_ACCELERO_Disable_Single_Tap_Detection_Ext(ACCELERO_handle)==COMPONENT_ERROR) {}
      if(LSM6DS3_ACC_GYRO_W_SingleTapOnInt1( ACCELERO_handle, LSM6DS3_ACC_GYRO_INT1_SINGLE_TAP_DISABLED ) == MEMS_ERROR ) {}
  }
    
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_DOUBLE_TAP))
  {
    /* Enable Double Tap detection */
  }
  else
  {
    if(BSP_ACCELERO_Disable_Double_Tap_Detection_Ext(ACCELERO_handle)==COMPONENT_ERROR) {}
      if(LSM6DS3_ACC_GYRO_W_TapEvOnInt1( ACCELERO_handle, LSM6DS3_ACC_GYRO_INT1_TAP_DISABLED ) == MEMS_ERROR ) {}
  }
  
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_FREE_FALL))
  {
    /* Enable Free Fall detection */
  }
  else
  {
    if(BSP_ACCELERO_Disable_Free_Fall_Detection_Ext(ACCELERO_handle)==COMPONENT_ERROR) {}
    if(LSM6DS3_ACC_GYRO_W_FFEvOnInt1( ACCELERO_handle, LSM6DS3_ACC_GYRO_INT1_FF_DISABLED ) == MEMS_ERROR ) {}
  }
  
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_WAKE_UP))
  {
    /* Enable Wake up detection */
  }
  else
  {
    if(BSP_ACCELERO_Disable_Wake_Up_Detection_Ext(ACCELERO_handle)==COMPONENT_ERROR) {}
    if(LSM6DS3_ACC_GYRO_W_WUEvOnInt1( ACCELERO_handle, LSM6DS3_ACC_GYRO_INT1_WU_DISABLED ) == MEMS_ERROR ) {}
  }
  
    if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_TILT))
  {
    /* Enable Tilt detection */
  }
  else
  {
    if(BSP_ACCELERO_Disable_Tilt_Detection_Ext(ACCELERO_handle)==COMPONENT_ERROR) {}
    if(LSM6DS3_ACC_GYRO_W_TiltEvOnInt1( ACCELERO_handle, LSM6DS3_ACC_GYRO_INT1_TILT_DISABLED ) == MEMS_ERROR ) {}
  }
}

void AccEvent_init()
{
  AccEvent_Deinit();
   extern void *ACCELERO_handle;
   extern DrvStatusTypeDef sensorStatusAccelero;
  
   SESSION_REG(nAccEventConf) &= ~ACCEVENT_FEATURE_DOUBLE_TAP;
   SESSION_REG(nAccEventConf) &= ~ACCEVENT_FEATURE_WAKE_UP;
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_PEDOMETER))
  {
    sensorStatusAccelero = BSP_ACCELERO_Enable_Pedometer_Ext( ACCELERO_handle );
  }
  else
  {
  }
  
  //for AccEventCharHandle
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_6DORIENTATION))
  {
    /* Enable 6D orientation detection */
    if(BSP_ACCELERO_Enable_6D_Orientation_Ext(ACCELERO_handle)==COMPONENT_ERROR) {}
  }
  else
  {
  }
  
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_SINGLE_TAP))
  {
    /* Enable Single Tap detection */
    if(BSP_ACCELERO_Enable_Single_Tap_Detection_Ext(ACCELERO_handle)==COMPONENT_ERROR) {}
  }
  else
  {
  }
    
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_DOUBLE_TAP))
  {
    /* Enable Double Tap detection */
    if(BSP_ACCELERO_Enable_Double_Tap_Detection_Ext(ACCELERO_handle)==COMPONENT_ERROR) {}
    if(BSP_ACCELERO_Set_Tap_Threshold_Ext(ACCELERO_handle,LSM6DS3_TAP_THRESHOLD_MID)==COMPONENT_ERROR) {}
  }
  else
  {
  }
  
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_FREE_FALL))
  {
    /* Enable Free Fall detection */
    if(BSP_ACCELERO_Enable_Free_Fall_Detection_Ext(ACCELERO_handle)==COMPONENT_ERROR) {}
    if(BSP_ACCELERO_Set_Free_Fall_Threshold_Ext(ACCELERO_handle,LSM6DS3_ACC_GYRO_FF_THS_5)==COMPONENT_ERROR) {}
  }
  else
  {
  }
  
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_WAKE_UP))
  {
    /* Enable Wake up detection */
    if(BSP_ACCELERO_Enable_Wake_Up_Detection_Ext(ACCELERO_handle)==COMPONENT_ERROR) {}
  }
  else
  {
  }
  
    if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_TILT))
  {
    /* Enable Tilt detection */
    if(BSP_ACCELERO_Enable_Tilt_Detection_Ext(ACCELERO_handle)==COMPONENT_ERROR) {}
  }
  else
  {
  }
}
#endif // USE_CUSTOM_ACCEVENT

#if USE_CUSTOM_GP_ALGORITHM
/**
  * @brief  Initialize algorithm 4
  * @retval None
  */
void GP_ALGORITHM_init()
{
}
#endif // USE_CUSTOM_GP_ALGORITHM

/**
  * @brief  Initialize algorithms
  * @param bFromScratch Set to 1 if "scratch" initialization, 0 if runtime initialization
  * @retval None
  */
void Algorithms_Init(uint8_t bFromScratch)
{
#if USE_CUSTOM_ALGORITHMFX
  if(bFromScratch)
  {
    FX_StartupConfig();
}
#endif /* USE_CUSTOM_ALGORITHMFX */

#if USE_CUSTOM_ALGORITHM1
  /* Initialize Algorithm1 */
  if(bFromScratch)
  {
    Algorithm1_init();
  }
#endif /* USE_CUSTOM_ALGORITHM1 */

#if USE_CUSTOM_ALGORITHM2
  /* Initialize Algorithm2 */
  if(bFromScratch)
  {
    Algorithm2_init();
  }
#endif /* USE_CUSTOM_ALGORITHM2 */

#if USE_CUSTOM_ALGORITHM3
  /* Initialize Algorithm3 */
  Algorithm3_init();
#endif /* USE_CUSTOM_ALGORITHM3 */

#if USE_CUSTOM_ALGORITHM4
  /* Initialize Algorithm4 */
  Algorithm4_init();
#endif /* USE_CUSTOM_ALGORITHM4 */

#if USE_CUSTOM_ACCEVENT
  /* Initialize AccEvent HW Features */
  AccEvent_init();
#endif /* USE_CUSTOM_ACCEVENT */

#if USE_CUSTOM_GP_ALGORITHM
  /* Initialize GP_ALGORITHM */
  GP_ALGORITHM_init();
#endif /* USE_CUSTOM_GP_ALGORITHM */
}


/**
  * @brief  Save the Data for Pedometer
  * @retval None
  */
void SavePedometerData()
{
  //Check current day in PEDO_SAVE_DATA_E_REG
  uint8_t nTodaysReg = PEDO_SAVE_DATA_0_REG + (2 * (SESSION_REG(tm_wday)-1));
  BSP_PERSREGS_WRITE(nTodaysReg, (void*)&SESSION_REG(nPedoStepCounter),2);
  if(*PEDO_SAVE_DATA_E_REGADDR != SESSION_REG(tm_day))
  {
    //Write current day in PEDO_SAVE_DATA_E_REG
    DBG_PRINTF_ALGORITHMS(SESSION_REG(WeSU_APP_DEFAULT_SEVERITY), "\r\nReset pedometer data: %d steps TODAY\r\n", SESSION_REG(nPedoStepCounter));
    uint16_t r = SESSION_REG(tm_day);
    BSP_PERSREGS_WRITE(PEDO_SAVE_DATA_E_REG, (void*)&r,1);
    //Reset current day counter
    reset_day=1;
    SESSION_REG(nPedoStepCounter) = 0;
  }
  else
  {
    DBG_PRINTF_ALGORITHMS(SESSION_REG(WeSU_APP_DEFAULT_SEVERITY), "\r\nPedometer data: %d steps\r\n", SESSION_REG(nPedoStepCounter));
  }
}

#if USE_CUSTOM_ALGORITHM3
/**
  * @brief  Run Algorithm3
  * @retval None
  */
void Algorithm3_run()
{
  extern void *ACCELERO_handle;
  uint16_t nStepCount = 0;
  if(PROCESS_PEDOMETER_ENABLED())
  {
    BSP_ACCELERO_Get_Step_Count_Ext( ACCELERO_handle, &nStepCount);
    SESSION_REG(nPedoStepCounter) = nStepCount;
  }
 }
#endif //USE_CUSTOM_ALGORITHM3

#if USE_CUSTOM_ALGORITHM4

/**
  * @brief  Run Algorithm4
  * @retval None
  */
void Algorithm4_run()
{
}
#endif //USE_CUSTOM_ALGORITHM4


#if USE_CUSTOM_ACCEVENT

/**
  * @brief  This function eturns the HW's 6D Orientation result
  * @param  None
  * @retval AccEventType 6D Orientation Found
  */
AccEventType GetHWOrientation6D(void)
{  
  AccEventType OrientationResult = ACC_NOT_USED;
  
  if(ACCELERO_handle) {
    uint8_t xl = 0;
    uint8_t xh = 0;
    uint8_t yl = 0;
    uint8_t yh = 0;
    uint8_t zl = 0;
    uint8_t zh = 0;
    
    if ( BSP_ACCELERO_Get_6D_Orientation_XL_Ext( ACCELERO_handle, &xl ) == COMPONENT_ERROR ){
      PRINTF("Error getting 6D orientation XL axis from LSM6DS3\r\n");
    }

    if ( BSP_ACCELERO_Get_6D_Orientation_XH_Ext( ACCELERO_handle, &xh ) == COMPONENT_ERROR ){
      PRINTF("Error getting 6D orientation XH axis from LSM6DS3\r\n");
    }

    if ( BSP_ACCELERO_Get_6D_Orientation_YL_Ext( ACCELERO_handle, &yl ) == COMPONENT_ERROR ){
      PRINTF("Error getting 6D orientation YL axis from LSM6DS3\r\n");
    }

    if ( BSP_ACCELERO_Get_6D_Orientation_YH_Ext( ACCELERO_handle, &yh ) == COMPONENT_ERROR ){
      PRINTF("Error getting 6D orientation YH axis from LSM6DS3\r\n");
    }

    if ( BSP_ACCELERO_Get_6D_Orientation_ZL_Ext( ACCELERO_handle, &zl ) == COMPONENT_ERROR ){
      PRINTF("Error getting 6D orientation ZL axis from LSM6DS3\r\n");
    }

    if ( BSP_ACCELERO_Get_6D_Orientation_ZH_Ext( ACCELERO_handle, &zh ) == COMPONENT_ERROR ){
      PRINTF("Error getting 6D orientation ZH axis from LSM6DS3\r\n");
    }
    
    if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0 ) {
      OrientationResult = ACC_6D_OR_RIGTH;
    } else if ( xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
      OrientationResult = ACC_6D_OR_TOP;
    } else if ( xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0 ) {
      OrientationResult = ACC_6D_OR_BOTTOM;
    } else if ( xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
      OrientationResult = ACC_6D_OR_LEFT;
    } else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1 ) {
      OrientationResult = ACC_6D_OR_UP;
    } else if ( xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0 ){
      OrientationResult = ACC_6D_OR_DOWN;
    } else {
      PRINTF("None of the 6D orientation axes is set in LSM6DS3\r\n");
    }
  }
  return OrientationResult;
}


/**
  * @brief  Run AccEvent_run
  * @retval None
  */
void AccEvent_run()
{
  uint8_t stat = 0;
  
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_FREE_FALL))
  {
    /* Check if the interrupt is due to Free Fall */
    BSP_ACCELERO_Get_Free_Fall_Detection_Status_Ext(ACCELERO_handle,&stat);
    if(stat)
    {
      SET_BIT(SESSION_REG(nAccEventStatus), ACC_FREE_FALL);
    }
    else
    {
    }
  }
  
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_DOUBLE_TAP))
  {
    /* Check if the interrupt is due to Double Tap */
    BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext(ACCELERO_handle,&stat);
    if(stat)
    {
      SET_BIT(SESSION_REG(nAccEventStatus), ACC_DOUBLE_TAP);
    }
    else
    {
    }
  }
  
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_SINGLE_TAP))
  {
    /* Check if the interrupt is due to Single Tap */
    BSP_ACCELERO_Get_Single_Tap_Detection_Status_Ext(ACCELERO_handle,&stat);
    if(stat)
    {
      SET_BIT(SESSION_REG(nAccEventStatus), ACC_SINGLE_TAP);
    }
    else
    {
    }
  }
  
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_WAKE_UP))
  {
    /* Check if the interrupt is due to Wake Up */
    BSP_ACCELERO_Get_Wake_Up_Detection_Status_Ext(ACCELERO_handle,&stat);
    if(stat)
    {
      SET_BIT(SESSION_REG(nAccEventStatus), ACC_WAKE_UP);
    }
    else
    {
    }
  }
  
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_TILT))
  {
    /* Check if the interrupt is due to Tilt */
    BSP_ACCELERO_Get_Tilt_Detection_Status_Ext(ACCELERO_handle,&stat);
    if(stat)
    {
      SET_BIT(SESSION_REG(nAccEventStatus), ACC_TILT);
    }
    else
    {
    }
  }
  
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_6DORIENTATION))
  {
    /* Check if the interrupt is due to 6D Orientation */
    BSP_ACCELERO_Get_6D_Orientation_Status_Ext(ACCELERO_handle,&stat);
    
    static AccEventType OldOrientation = ACC_NOT_USED;
      AccEventType Orientation = GetHWOrientation6D();
    if(OldOrientation!=Orientation)
    {
      CLEAR_BIT(SESSION_REG(nAccEventStatus), 0x07);
      SET_BIT(SESSION_REG(nAccEventStatus), Orientation);
      OldOrientation=Orientation;
    }
  }
  
}
#endif //USE_CUSTOM_ACCEVENT


#if USE_CUSTOM_GP_ALGORITHM
/**
  * @brief  Run GP_ALGORITHM
  * @retval None
  */
void GP_ALGORITHM_run()
{
  //////////////////////////////        GP_ALGORITHM    /////////////////////////////////
}
#endif //USE_CUSTOM_GP_ALGORITHM

/**
  * @brief  Process algorithms
  * @retval None
  */
void Algorithms_Process()
{
#if USE_CUSTOM_ALGORITHMFX
  FX_Process();
#endif //USE_CUSTOM_ALGORITHMFX
 /* Put here the algorithm implementation, check frequency */
#if USE_CUSTOM_ALGORITHM1
  /* Execute the algorithm */
  EXEC_WITH_INTERVAL(Algorithm1_run(), 1000/ALGORITHM1_FREQUENCY);
#endif //USE_CUSTOM_ALGORITHM1
  
#if USE_CUSTOM_ALGORITHM2
  /* Execute the algorithm */
  EXEC_WITH_INTERVAL(Algorithm2_run(), 1000/ALGORITHM2_FREQUENCY);
#endif //USE_CUSTOM_ALGORITHM2

#if USE_CUSTOM_ALGORITHM3
  /* Execute the algorithm */
#endif //USE_CUSTOM_ALGORITHM3

#if USE_CUSTOM_ALGORITHM4
  /* Execute the algorithm */
  EXEC_WITH_INTERVAL(Algorithm4_run(), 1000/ALGORITHM4_FREQUENCY);
#endif //USE_CUSTOM_ALGORITHM4
  
#if USE_CUSTOM_GP_ALGORITHM
  /* Execute the algorithm */
  EXEC_WITH_INTERVAL(GP_ALGORITHM_run(), 1000/GP_ALGORITHM_FREQUENCY);
#endif //USE_CUSTOM_GP_ALGORITHM


#if USE_CUSTOM_ACCEVENT
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_PEDOMETER)) 
  {
    EXEC_WITH_INTERVAL(SavePedometerData(), 10000);// ten seconds
  }
#endif //USE_CUSTOM_ACCEVENT
}


void Algorithms_Hw_Process()
{  
#if USE_CUSTOM_ALGORITHM3
  /* Execute the algorithm */
#endif //USE_CUSTOM_ALGORITHM3
  
#if USE_CUSTOM_ACCEVENT
  if(ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_PEDOMETER)) 
  {
    uint16_t nStepCount = 0;
    BSP_ACCELERO_Get_Step_Count_Ext( ACCELERO_handle, &nStepCount);
    SESSION_REG(nPedoStepCounter) = nStepCount;
    if(reset_day){
      BSP_ACCELERO_Enable_Step_Counter_Reset_Ext( ACCELERO_handle ); 
      BSP_ACCELERO_Disable_Step_Counter_Reset_Ext( ACCELERO_handle );
      reset_day = 0;
    }
  }
#endif //USE_CUSTOM_ACCEVENT
}

void Algorithms_Hw_Process_INT1()
{  
#if USE_CUSTOM_ACCEVENT
  /* Execute the algorithm */
  AccEvent_run();
#endif //USE_CUSTOM_ACCEVENT
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
