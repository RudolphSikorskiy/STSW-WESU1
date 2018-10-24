/**
  ******************************************************************************
  * @file    wesu_rtc.h
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Header for wesu_rtc.c module
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
#ifndef __WESU_RTC_H
#define __WESU_RTC_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

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

/** @defgroup WeSU_RTC_Exported_Types   WeSU_RTC_Exported_Types
  * @{
  */
   
/**
  * @brief  Structure defining Time of Day
  */   
typedef enum 
{
  RTC_AppConfigInvalid          = 0x00,
  RTC_AppConfigDefault          = 0xD0,
  RTC_AppConfigDefaultRunning   = 0xD1,
  RTC_AppConfigRestored         = 0xA0,
  RTC_AppConfigRestoredRunning  = 0xA1,
  RTC_AppConfigUser             = 0xB0,
  RTC_AppConfigUserRunning      = 0xB1,
  RTC_AppConfigForced           = 0xB3,
} RTC_AppConfig;

#define IS_RTC_APP_CONFIG(APP_CONF)     (((APP_CONF) == RTC_AppConfigInvalid) || \
                                         ((APP_CONF) == RTC_AppConfigDefault) || \
                                         ((APP_CONF) == RTC_AppConfigDefaultRunning) || \
                                         ((APP_CONF) == RTC_AppConfigRestored) || \
                                         ((APP_CONF) == RTC_AppConfigRestoredRunning) || \
                                         ((APP_CONF) == RTC_AppConfigUser) || \
                                         ((APP_CONF) == RTC_AppConfigUserRunning) || \
                                         ((APP_CONF) == RTC_AppConfigForced))   //!< RTC_AppConfig assert macro

/**
  * @brief  Structure defining Time of Day
  */   
typedef struct 
{
    uint8_t tm_hour;    /*!< Time in Hours.     */
    uint8_t tm_min;     /*!< Time in Min.       */
    uint8_t tm_sec;     /*!< Time in Sec.       */
    uint8_t tm_day;     /*!< Time in Day.       */
    uint8_t tm_mon;     /*!< Time in Month.     */
    uint8_t tm_year;    /*!< Time in Year.      */
    uint8_t tm_wday;    /*!< Week Day           */
    uint8_t tm_rtcConf; /*!< One of the following values: \ref RTC_AppConfig    */
} rtc_time_t ;
   
/**
  * @brief Structure defining an alarm
  */ 
typedef enum 
{
    ONCE_PER_SEC,   /*!< One Alarm for Sec  */
    ONCE_PER_MIN,   /*!< One Alarm for Min  */
    ONCE_PER_HOUR,  /*!< One Alarm for Hours*/
    ONCE_PER_DAY,   /*!< One Alarm for DAY  */
    ONCE_PER_MONTH, /*!< One Alarm for MONTH*/
    ONCE_PER_YEAR   /*!< One Alarm for YEAR */
} rtc_repeat_t;

/**
  * @brief Structure defining an alarm
  */ 
typedef struct 
{
    int alm_sec;                        /*!< One Alarm for Sec  */  
    int alm_min;                        /*!< One Alarm for Min  */
    int alm_hour;                       /*!< One Alarm for Hours*/
    int alm_day;                        /*!< One Alarm for DAY  */
    int alm_mon;                        /*!< One Alarm for MONTH*/
    rtc_repeat_t alm_repeat_mode;       /*!< Alarm Repeat Mode */
} rtc_alarm_t ;

/**
  * @brief pointer to the configuration function
  */ 
typedef void (*pConfigRtcDefault)();

/**
  * @}
  */ 

/** @defgroup WeSU_RTC_Exported_Macros       WeSU_RTC_Exported_Macros
  * @{
  */   
#define __IO    volatile        //!< IO MACRO

#define HOURS12_to_HOURS24(h,f)         (f==RTC_HOURFORMAT12_AM)?((h==12)?(0):(h)):((h==12)?(12):(12+h))        //!< From 12h to 24h HOUR FORMAT

#define HOURS24_to_HOURS12(h)           ((h==0)?12:(h==12)?12:(h<12)?h:h-12)                                    //!< From 24h to 12h HOUR FORMAT
#define HOURS24_to_TIME_FORMAT(h)       ((h==0)?RTC_HOURFORMAT12_PM:(h==12)?RTC_HOURFORMAT12_AM:(h<12)?RTC_HOURFORMAT12_AM:RTC_HOURFORMAT12_PM) //!< Hours in AM/PM Format
/**
  * @}
  */ 

/** @defgroup WeSU_RTC_Exported_Functions       WeSU_RTC_Exported_Functions
  * @{
  */   
void BSP_RTC_Init(void);                        //!< RTC Initialization API
uint8_t WeSU_GetDateTime(rtc_time_t *t);        //!< RTC GET Date and Time Function
uint8_t WeSU_SetDateTime(rtc_time_t *t);        //!< RTC SET Date and Time Function
uint8_t WesuRtcConfig(pConfigRtcDefault p);     //!< RTC Configuration
int BSP_RTC_GetTime(rtc_time_t*);               //!< RTC GET Time Function
int BSP_RTC_SetTime(rtc_time_t*);               //!< RTC SET Time Function
int BSP_RTC_IsTimeOfDayValid(void);             //!< RTC SET Time Function
void BSP_RTC_RestartOscillator(void);           //!< RTC Restart Oscillator
const char* BSP_RTC_GetWeekDayName(int);           //!< RTC GET Week Name
const char* BSP_RTC_GetMonthName(int);          //!< RTC GET Month Name
const char* BSP_RTC_GetRtcConfigAsString(uint8_t xrtcConf);     //!< Get RTC Configuration as string
int BSP_RTC_SetAlarm(rtc_alarm_t*);             //!< RTC SET Alarm
int BSP_RTC_ClearAlarm(void);                   //!< RTC Clear Alarm
int BSP_RTC_ClearIrq(void);                     //!< RTC Clear IRQ

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

#endif /* __WESU_RTC_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
