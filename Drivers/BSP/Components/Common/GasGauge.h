/**
  ******************************************************************************
  * @file    GasGauge.h
  * @author  AST
  * @version V1.0.0
  * @date    6-August-2014
  * @brief   This header file contains the functions prototypes for the gas gauge driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __GasGauge_H
#define __GasGauge_H

#ifdef __cplusplus
#include "Utilities.h"	
extern "C"				
{					
#endif					


/* Includes ------------------------------------------------------------------*/
#include <stdint.h> 



/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  GG driver structure definition  
  */ 
typedef struct
{  
	void (*Init)(void);
	int (*Task)(void);
	void (*Reset)(void);
	int (*Stop)(void);

	int (*GetSOC)(void);
	int (*GetOCV)(void);
	int (*GetCurrent)(void);
	int (*GetTemperature)(void);
	int (*GetVoltage)(void);
	int (*GetChargeValue)(void);
	int (*GetPresence)(void);
	int (*GetAlarmStatus)(void);
	int (*GetITState)(void);

	int (*AlarmSetVoltageThreshold)(int);
	int (*AlarmSetSOCThreshold)(int);

	int (*GetIT)(void);
	int (*SetIT)(void);
	int (*StopIT)(void);
	int (*ClearIT)(void); 
        int (*PowerSavingMode)(void);
}GG_DrvTypeDef;

#endif /* __GasGauge_H */

#ifdef __cplusplus	
}				
#endif			
 
 


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

