/**
  ******************************************************************************
  * @file    stc3115.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   STC3115 Driver Component Function Vector
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
#include "stc3115_Driver.h"
#include "stc3115.h"
#include "GasGauge.h"

/** @addtogroup BSP             BSP
  * @{
  */ 

/** @addtogroup COMPONENTS      COMPONENTS
  * @{
  */ 

/** @addtogroup STC3115                 STC3115
  * @{
  */

/** @defgroup STC3115_Private_FunctionPrototypes        STC3115_Private_FunctionPrototypes
  * @{
  */
static void STC3115_Init(void);
static int STC3115_Task(void);
static void STC3115_Reset(void);
static int STC3115_Stop(void);

static int STC3115_GetSOC(void);
static int STC3115_GetOCV(void);
static int STC3115_GetCurrent(void);
static int STC3115_GetTemperature(void);
static int STC3115_GetVoltage(void);
static int STC3115_GetChargeValue(void);
static int STC3115_GetPresence(void);
static int STC3115_GetAlarmStatus(void);
static int STC3115_GetITState(void);

static int STC3115_AlarmSetVoTh(int);
static int STC3115_AlarmSetSOCTh(int);

static int STC3115_GetIT(void);
static int STC3115_SetIT(void);
static int STC3115_StopIT(void);
static int STC3115_ClearIT(void);

static int STC3115_PowerSavingMode(void);

/**
 * @}
 */ 

/** @defgroup STC3115_Private_Variables         STC3115_Private_Variables
  * @{
  */ 
static STC3115_ConfigData_TypeDef STC3115_ConfigData;   //!< stc3115 configuration structure
static STC3115_BatteryData_TypeDef STC3115_BatteryData; //!< stc3115 battery output structure


/**
 * @brief STC3115 Functions Handlers. 
 */

GG_DrvTypeDef   stc3115_drv = 
{
  .Init = STC3115_Init,
  .Task = STC3115_Task,
  .Reset = STC3115_Reset,
  .Stop = STC3115_Stop,
  
  .GetSOC = STC3115_GetSOC,
  .GetOCV = STC3115_GetOCV,
  .GetCurrent = STC3115_GetCurrent,
  .GetTemperature = STC3115_GetTemperature,
  .GetVoltage = STC3115_GetVoltage,
  .GetChargeValue = STC3115_GetChargeValue,
  .GetPresence = STC3115_GetPresence,
  .GetAlarmStatus = STC3115_GetAlarmStatus,
  .GetITState = STC3115_GetITState,
  
  .AlarmSetVoltageThreshold = STC3115_AlarmSetVoTh,
  .AlarmSetSOCThreshold = STC3115_AlarmSetSOCTh,
  
  .GetIT = STC3115_GetIT,
  .SetIT = STC3115_SetIT,
  .StopIT = STC3115_StopIT,
  .ClearIT = STC3115_ClearIT,
  .PowerSavingMode = STC3115_PowerSavingMode,
};

/**
* @}
*/ 


/** @defgroup STC3115_Imported_Functions         STC3115_Imported_Functions
  * @{
  */

extern uint32_t BSP_GG_IO_GetITState(void);

/**
* @}
*/ 

/** @defgroup STC3115_Private_Functions         STC3115_Private_Functions
  * @{
  */

/**
 * @brief Initializes STC3115 Gas Gauge. Hides low level device and battery details 
 * 
 */
static void STC3115_Init(void) 
{
  uint8_t nTry = 3;
  while(GasGauge_Initialization(&STC3115_ConfigData, &STC3115_BatteryData) < 0)
  {
    if(nTry--)break;
  }
}

/**
 * @brief Task function for STC3115 Gas Gauge. Hides low level device and battery details
 * @retval Same as GasGauge_Task
 */
static int STC3115_Task(void) {
	return GasGauge_Task(&STC3115_ConfigData, &STC3115_BatteryData);
}

/**
 * @brief Resets Gas Gauge
 * 
 */
static void STC3115_Reset(void) {
	GasGauge_Reset();
}

/**
 * @brief Stops Gas Gauge
 * @retval Same as GasGauge_Stop
 */
static int STC3115_Stop(void){
	return GasGauge_Stop();
}

/**
 * @brief Put Gas Gauge in PowerSavingMode
 * @retval Same as GasGauge_PowerSavingMode
 */
static int STC3115_PowerSavingMode(void){
	return GasGauge_PowerSavingMode();
}

/**
 * @brief Returns battery State of Charge
 * @retval Battery State of Charge value
 */
static int STC3115_GetSOC(void) {
//	return (STC3115_BatteryData.SOC+5)/10;
	return (STC3115_BatteryData.SOC);
}

/**
 * @brief Returns battery Open circuit Voltage
 * @retval Open circuit Voltage value
 */
static int STC3115_GetOCV(void) {
	return STC3115_BatteryData.OCV;
}

/**
 * @brief Returns absorbed current
 * @retval Current value
 */
static int STC3115_GetCurrent(void) {
	return STC3115_BatteryData.Current;
}

/**
 * @brief Returns temperature value
 * @retval Temperature value
 */
static int STC3115_GetTemperature(void) {
	return STC3115_BatteryData.Temperature;
}

/**
 * @brief Returns battery voltage
 * @retval Battery voltage value
 */
static int STC3115_GetVoltage(void) {
	return STC3115_BatteryData.Voltage;
}

/**
 * @brief Returns remaining battery charge value
 * @retval battery charge value
 */
static int STC3115_GetChargeValue(void) {
	return STC3115_BatteryData.ChargeValue;
}

/**
 * @brief Tells whether battery is present or not
 * @retval 0 if battery is not present, 1 if present
 */
static int STC3115_GetPresence(void) {
	return STC3115_BatteryData.Presence;
}

/**
 * @brief Returns Alarm bits status
 * @retval STC3115 Alarm bits status
 */
static int STC3115_GetAlarmStatus(void) {
	return (STC3115_BatteryData.status >> 13) & 0x3;
}

/**
 * @brief Returns Interrupt state
 * @retval Same as BSP_GG_IO_GetITState (GPIO_PIN_SET or GPIO_PIN_RESET)
 */
static int STC3115_GetITState(void) {
	return BSP_GG_IO_GetITState();
}

/**
 * @brief Sets Alarm Voltage Threshold
 * @param VoltThresh Volage Threshold to be set
 * @retval Same as STC3115_AlarmSetVoltageThreshold
 */
static int STC3115_AlarmSetVoTh(int VoltThresh) {
	int max_value = (0xFF * 17.6);
	if(VoltThresh > max_value) VoltThresh = max_value;
	return STC3115_AlarmSetVoltageThreshold(&STC3115_ConfigData, VoltThresh);
}

/**
 * @brief Sets Alarm State of Charge threshold
 * @param SOCThresh alarm threshold
 * @retval Same as STC3115_AlarmSetSOCThreshold
 */
static int STC3115_AlarmSetSOCTh(int SOCThresh) {
	int max_value = (int)(0xFF * 0.5);
	if(SOCThresh > max_value) SOCThresh = max_value;
	return STC3115_AlarmSetSOCThreshold(&STC3115_ConfigData, SOCThresh);
}

/**
 * @brief Returns Alarm interrupt status bits
 * @retval Same as STC3115_AlarmGet
 */
static int STC3115_GetIT(void) {
	return STC3115_AlarmGet();
}

/**
 * @brief Enables alarm interrupts
 * @retval Same as STC3115_AlarmSet
 */
static int STC3115_SetIT(void) {
	return STC3115_AlarmSet();
}

/**
 * @brief Disables alarm interrupts
 * @retval Same as STC3115_AlarmStop
 */
static int STC3115_StopIT(void) {
	return STC3115_AlarmStop();
}

/**
 * @brief Clears alarm interrupts
 * @retval Same as STC3115_AlarmClear
 */
static int STC3115_ClearIT(void) {
	return STC3115_AlarmClear();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

