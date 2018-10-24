/**
  ******************************************************************************
  * @file    wesu_rtc.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   This file includes the driver for the low-power real-time 
  *          clock (RTC)    
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
#include <stdint.h>
#include <string.h>
#include "wesu_gg.h"
#include "wesu_rtc.h"

/** @addtogroup BSP             BSP
  * @{
  */

/** @addtogroup STEVAL-WESU1            STEVAL-WESU1
  * @{
  */


/** @addtogroup PLATFORM        PLATFORM
  * @{
  */


/** @addtogroup WeSU_RTC        WeSU_RTC
  * @brief BSP functions implementation for WESU Real Time Clock
  * @{
  */ 
    
/** @defgroup WeSU_RTC_Imported_Variables       WeSU_RTC_Imported_Variables
  * @{
  */ 
    
extern RTC_HandleTypeDef RtcHandle;

/**
  * @}
  */ 

/** @defgroup WeSU_RTC_Private_Variables        WeSU_RTC_Private_Variables
  * @{
  */ 
/**
 * @brief Week and months names
 * 
 */
static const char* weekDayNames[] = {"Monday", 
                                     "Tuesday", 
                                     "Wednesday", 
                                     "Thursday", 
                                     "Friday", 
                                     "Saturday", 
                                     "Sunday"};         /*!< Day Name Array */

static const char* monthsNames[] = {"January",
                                    "February", 
                                    "March", 
                                    "April", 
                                    "May", 
                                    "June", 
                                    "July", 
                                    "August", 
                                    "September", 
                                    "October", 
                                    "November", 
                                    "December"};        /*!< Months Name Array */

/**
  * @}
  */ 

/** @defgroup WeSU_RTC_Private_FunctionPrototypes       WeSU_RTC_Private_FunctionPrototypes
  * @{
  */ 

/**
 * @brief  bcd2bin
 * @retval val
 */
inline unsigned int bcd2bin(uint8_t val)
{
  return ((val) & 0x0f) + ((val) >> 4)  * 10;
}


/**
 * @brief  bin2bcd
 * @retval val
 */
inline uint8_t bin2bcd (unsigned int val)
{
  return (((val / 10) << 4) | (val % 10));
}

/* alarm helper function */
void prepare_alarm_buffer(uint8_t*, rtc_alarm_t*);   /*!< alarm helper function */

/**
  * @}
  */ 


/** @defgroup WeSU_RTC_Exported_Functions       WeSU_RTC_Exported_Functions
  * @{
  */
  

/** @brief Returns RTC cofiguration (human-readable)
  * @param xrtcConf RTC cofiguration value (see \ref RTC_AppConfig)
  * @retval RTC cofiguration string
  */
const char* BSP_RTC_GetRtcConfigAsString(uint8_t xrtcConf)
{
  switch(xrtcConf)
  {
    //already configured with default time
  case RTC_AppConfigDefault:
    return "RTC_AppConfigDefault";
  case RTC_AppConfigDefaultRunning:
    return "RTC_AppConfigDefaultRunning";
  case RTC_AppConfigRestored:
    return "RTC_AppConfigRestored";
  case RTC_AppConfigRestoredRunning:
    return "RTC_AppConfigRestoredRunning";
    //already configured with correct time
  case RTC_AppConfigUser:
    return "RTC_AppConfigUser";
  case RTC_AppConfigUserRunning:
    return "RTC_AppConfigUserRunning";
  case RTC_AppConfigForced:
    return "RTC_AppConfigForced";
    //not configured
  default:
    return "RTC_AppConfigInvalid";
  }
}

/** @brief Returns week day name
  * @param week day number [1..7]
  * @retval weekDayNames string if week day number is in [1..7] range. NULL otherwise
  */
const char* BSP_RTC_GetWeekDayName(int week)
{
  if(week < 1 || week > 7)
    return " ";
	
  return weekDayNames[week-1];
}

/** @brief Returns month day name
  * @param month day number [1..12]
  * @retval monthsNames string if month day number is in [1..12] range. NULL otherwise
  */
const char* BSP_RTC_GetMonthName(int month)
{
  if(month < 1 || month > 12)
    return " ";
  
  return monthsNames[month-1];
}

/**
  * @brief Returns Rtc configuration 
  * @param p pointer to the configuration function
  * @retval 0: no configuration need; 1: rtc not configured; 2: rtc backup regs not correct
  */
uint8_t WesuRtcConfig(pConfigRtcDefault p)
{
  uint8_t bRtcConfig = 0;
  if(HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR0) == HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR1))
  {
    uint16_t bkpVal = HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR0);
    switch(bkpVal)
    {
      //already configured with default time
    case RTC_AppConfigDefault:
      bRtcConfig = 1;
      break;
    case RTC_AppConfigDefaultRunning:
      bRtcConfig = 1;
      break;
    case RTC_AppConfigRestored:
      bRtcConfig = 1;
      break;
    case RTC_AppConfigRestoredRunning:
      bRtcConfig = 1;
      break;
      //already configured with correct time
    case RTC_AppConfigUser:
      bRtcConfig = 2;
      break;
    case RTC_AppConfigUserRunning:
      bRtcConfig = 2;
      break;
    case RTC_AppConfigForced:
      bRtcConfig = 2;
      break;
      //not configured
    default:
      bRtcConfig = 0;
      break;
    }
  }
  else
  {
    //not configured, backup regs not equal
    bRtcConfig = 0;
  }
  
  if(bRtcConfig == 0)
  {
    if(p)
    {
      p();
      bRtcConfig = 1;
    }
  }
  return bRtcConfig;
}

/**
  * @brief Set RTC time
  * @param t pointer to a rtc_time_t structure in 24h bin format
  * @retval 0: OK; 1: error
  */
uint8_t WeSU_SetDateTime(rtc_time_t *t)
{
  uint8_t nRet = 0;
  if(HAL_RTC_GetState(&RtcHandle) == HAL_RTC_STATE_READY)
  {
    RTC_DateTypeDef sdatestructure;
    RTC_TimeTypeDef stimestructure;
    
    sdatestructure.Year = t->tm_year;
    sdatestructure.Month = t->tm_mon;
    sdatestructure.Date = t->tm_day;
    sdatestructure.WeekDay = t->tm_wday;
    
    if(HAL_RTC_SetDate(&RtcHandle, &sdatestructure, RTC_FORMAT_BIN) != HAL_OK)
    {
      /* Initialization Error */
      //    Error_Handler();
      nRet |=1;
    }
    
    stimestructure.Hours = HOURS24_to_HOURS12(t->tm_hour);
    stimestructure.Minutes = t->tm_min;
    stimestructure.Seconds = t->tm_sec;
    stimestructure.TimeFormat = HOURS24_to_TIME_FORMAT(t->tm_hour);
    stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
    
    if(HAL_RTC_SetTime(&RtcHandle, &stimestructure, RTC_FORMAT_BIN) != HAL_OK)
    {
      /* Initialization Error */
      //    Error_Handler();
      nRet |=1;
    }
    HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR0, t->tm_rtcConf);
    HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR1, t->tm_rtcConf);
  }
  return nRet;
}

/**
  * @brief Get RTC time
  * @param t pointer to a returning rtc_time_t structure in 24h bin format
  * @retval 0: OK; 1: error
  */
uint8_t WeSU_GetDateTime(rtc_time_t *t)
{
  uint8_t nRet = 1;
  t->tm_rtcConf = 0xFF;
  if(HAL_RTC_GetState(&RtcHandle) == HAL_RTC_STATE_READY)
  {
    RTC_TimeTypeDef sTime;
    HAL_RTC_GetTime(&RtcHandle, &sTime, RTC_FORMAT_BIN);
    
    RTC_DateTypeDef sDate;
    HAL_RTC_GetDate(&RtcHandle, &sDate, RTC_FORMAT_BIN);
    
    t->tm_sec = sTime.Seconds;
    t->tm_min = sTime.Minutes;
    t->tm_hour = HOURS12_to_HOURS24(sTime.Hours, sTime.TimeFormat);
    
    t->tm_day = sDate.Date;
    t->tm_mon = sDate.Month;
    t->tm_year = sDate.Year;
    
    t->tm_wday = sDate.WeekDay;
    t->tm_rtcConf = HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR0);
    nRet = 0;
  }
  return nRet;
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

/************************ (C) COPYRIGHT STMicroelectronics  *****END OF FILE****/
