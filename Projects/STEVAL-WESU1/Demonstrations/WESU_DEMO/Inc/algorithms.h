/**
  ******************************************************************************
  * @file    WESU_DEMO/Inc/algorithms.h
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Header for algorithms.c module
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
#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_

#ifdef __cplusplus
 extern "C" {
#endif 
   
#include "osx_motion_ar.h"
#include "osx_motion_cp.h"
#include "osx_motion_fx.h" 


/** @addtogroup WeSU_Demo       WeSU Demo      
  * @{
  */

/** @addtogroup WeSU_User               WeSU User
  * @{
  */
   
/** @defgroup Algorithms      Algorithms
  * @{
  * @brief Algorithms Functionalities
  */   
 
/** @defgroup ALGORITHMS_Exported_Macros    ALGORITHMS Exported Macros
  * @{
  */   

#define USE_FX_MAG_CALIBRATION            (*MAG_CALIB_CONTROL_REGADDR & FX_CALIBRATION)                //!< Macro to get FX_CALIBRATION enable setting

/**
  * @}
  */
  
/** @defgroup ALGORITHMS_Exported_Types    ALGORITHMS Exported Types
  * @{
  */   

typedef enum
{
  ACC_NOT_USED     = 0x00,
  ACC_6D_OR_TOP    = 0x01,
  ACC_6D_OR_LEFT   = 0x02,
  ACC_6D_OR_BOTTOM = 0x03,
  ACC_6D_OR_RIGTH  = 0x04,
  ACC_6D_OR_UP     = 0x05,
  ACC_6D_OR_DOWN   = 0x06,
  ACC_TILT         = 0x08,
  ACC_FREE_FALL    = 0x10,
  ACC_SINGLE_TAP   = 0x20,
  ACC_DOUBLE_TAP   = 0x40,
  ACC_WAKE_UP      = 0x80
} AccEventType;                 //!< HW Accelerometer Event Types
   

typedef enum
{
  OSX_FAILED  = 0x00,           //!< License authentication failed
  OSX_SUCCESS = 0x01            //!< License authentication ok
} osx_lm_result_t;              //!< License authentication Results
  
/**
  * @}
  */

/** @defgroup ALGORITHMS_Exported_Functions    ALGORITHMS Exported Functions
  * @{
  */   
  

void FX_Process(void);
void FX_Process_High(void);
void FX_StartupConfig(void);
void FxMagCalibration (void);
osx_lm_result_t PreCheckLicense(uint8_t* pLicense);
void Algorithms_Init(uint8_t bFromScratch);
void Algorithms_Process(void);
void Algorithms_Hw_Process();
void Algorithms_Hw_Process_INT1();
void SavePedometerData();
osxMFX_output* MotionFX_manager_getDataOUT(void); 

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

#endif /* _ALGORITHM_H_ */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/  
