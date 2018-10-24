/**
  ******************************************************************************
  * @file    platform_config.h
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   This file contains definitions for the STEVAL-WESU1
  *          board specific functions.
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
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

#include <stdint.h>

/** @addtogroup BSP             BSP
  * @{
  */

/** @defgroup STEVAL-WESU1      STEVAL-WESU1
 *  @{
 */

/** @addtogroup PLATFORM        PLATFORM
  * @{
  */

 
#define N_PAYLOAD_LENGHT(S)     ((S)->nPayloadLenght)   //!< FRAME PAYLOAD SIZE */  

#define UPPER_CASE(c) (c>='A' && c<='Z')                        //!< UPPER_CASE UTILITY 
#define LOWER_CASE(c) (c>='a' && c<='z')                        //!< LOWER_CASE UTILITY 
#define IS_CIPHER(c) (c>='0' && c<='9')                         //!< IS_CIPHER UTILITY  
#define countof(A)                      (sizeof(A)/sizeof(*A))  //!< countof UTILITY    
#define ABS(A)                          (((A)>0)?(A):(-A))      //!< ABS UTILITY        

#define in_range(c, lo, up)   ((u8_t)c >= lo && (u8_t)c <= up)  //!< in_range UTILITY   
#define isprint(c)            in_range(c, 0x20, 0x7f)           //!< isprint UTILITY    
#define isdigit(c)            in_range(c, '0', '9')             //!< isdigit UTILITY    
#define digit(c)              (c - '0')                         //!< digit UTILITY      
#define xdigit(c)             isdigit(c)?digit(c):in_range(c, 'a', 'f')?(10+c-'a'):(10+c-'A')   //!< xdigit UTILITY     
#define isxdigit(c)           (isdigit(c) || in_range(c, 'a', 'f') || in_range(c, 'A', 'F'))    //!< isxdigit UTILITY   
#define islower(c)            in_range(c, 'a', 'z')             //!< islower UTILITY                                
#define isupper(c)            in_range(c, 'A', 'Z')             //!< isupper UTILITY    
#define ischar(c)             (islower(c) || isupper(c))        //!< ischar UTILITY     
#define isspace(c)            (c == ' ' || c == '\f' || c == '\n' || c == '\r' || c == '\t' || c == '\v')       //!< isspace UTILITY    
#define isEOL(c)              (c == '\r' || c == '\n')          //!< isEOL UTILITY      
#define isNULL(c)             (c == '\0')                       //!< isNULL UTILITY     
#define isescape(c)           (c == 0x1B)                       //!< isescape UTILITY   


//#if DEBUG
//#include <stdio.h>
//#define PRINTF(...) printf(__VA_ARGS__)
//#else
//#define PRINTF(...)
//#endif

/* Print the data travelling over the SPI in the .csv format for the GUI */
//#define PRINT_CSV_FORMAT 
#ifdef PRINT_CSV_FORMAT
#include <stdio.h>
  #define PRINT_CSV(...) printf(__VA_ARGS__)    //!< PRINT_CSV Macro with PRINT_CSV_FORMAT 
#else
  #define PRINT_CSV(...)                        //!< PRINT_CSV Macro
#endif
  
/**
 * @brief It defines the types of string to be printed for a particular task. 
 *      Through this variable it is possible to enable/disable all debug strings
 *      of a particular task or disable all the debug messages below a certain severity level.
 */
typedef enum WeSU_Debug_Severity
{
  WeSU_DEBUG_SEVERITY_NONE=0,     //!< No severity. It must be used to disable all the print-out 

  WeSU_DEBUG_SEVERITY_CRITICAL=1, //!< Severity critical. It must be used to disable all the print-out, with severity lower then CRITICAL.

  WeSU_DEBUG_SEVERITY_WARNING=2,  //!< Severity warning. It must be used to disable all the print-out, with severity lower then WARNING.

  WeSU_DEBUG_SEVERITY_INFO=3,     //!< Info severity. It must be used to disable all the print-out, with severity lower then INFO.

  WeSU_DEBUG_SEVERITY_VERBOSE=4   //!< Verbose severity. It must be used to enable all the print-out.

} WeSU_Debug_Severity_t;

extern uint8_t bDebuggerConnected;
extern int ButtonPressed;
extern int LowPowerFlag;
extern volatile uint8_t bConnParamUpdate;
extern uint8_t bWdgConfigured;

#define USE_PRINTF      1                       //!< PRINTF DEFINE

#define USE_RTC         1                       //!< RTC DEFINE

#define USE_IWDG        1                       //!< WATCHDOG DEFINE

#define USE_USB_MONITOR_CALLBACK        0       //!< set to 1 to activate UsbMonitorCallback

#define USE_LOW_BATTERY_CALLBACK        0       //!< set to 1 to activate LowBatteryCallback

#define STD_OUT_ARRAY_LENGHT  100               //!< STD_OUT Lenght

#ifdef USE_REGS

#define WeSU_DEBUG_MAIN_MASK            0x0001  /* Include WeSU_DEBUG_MAIN_MASK in WeSU_DEBUG_DEFAULT_VALUE to enable DBG_PRINTF_MAIN */
#define WeSU_DEBUG_WESU_CONFIG_MASK     0x0002  /* Include WeSU_DEBUG_WESU_CONFIG_MASK in WeSU_DEBUG_DEFAULT_VALUE to enable DBG_PRINTF_WESU_CONFIG */
#define WeSU_DEBUG_TERMINAL_MASK        0x0004  /* Include WeSU_DEBUG_TERMINAL_MASK in WeSU_DEBUG_DEFAULT_VALUE to enable DBG_PRINTF_TERMINAL */
#define WeSU_DEBUG_ALGORITHMS_MASK      0x0008  /* Include WeSU_DEBUG_ALGORITHMS_MASK in WeSU_DEBUG_DEFAULT_VALUE to enable DBG_PRINTF_ALGORITHMS */
#define WeSU_DEBUG_BLUENRG_MASK         0x0010  /* Include WeSU_DEBUG_BLUENRG_MASK in WeSU_DEBUG_DEFAULT_VALUE to enable DBG_PRINTF_BLUENRG */
#define WeSU_DEBUG_PWR_MGNT_MASK        0x0020  /* Include WeSU_DEBUG_PWR_MGNT_MASK in WeSU_DEBUG_DEFAULT_VALUE to enable DBG_PRINTF_PWR_MGNT */

#define WeSU_DEBUG_MAIN_ENABLED         (WeSU_DEBUG_MAIN_MASK           & SESSION_REG(WeSU_DEBUG_MASK))        //!< debug enable bit for main
#define WeSU_DEBUG_WESU_CONFIG_ENABLED  (WeSU_DEBUG_WESU_CONFIG_MASK    & SESSION_REG(WeSU_DEBUG_MASK))        //!< debug enable bit for wesu_config
#define WeSU_DEBUG_TERMINAL_ENABLED     (WeSU_DEBUG_TERMINAL_MASK       & SESSION_REG(WeSU_DEBUG_MASK))        //!< debug enable bit for terminal
#define WeSU_DEBUG_ALGORITHMS_ENABLED   (WeSU_DEBUG_ALGORITHMS_MASK     & SESSION_REG(WeSU_DEBUG_MASK))        //!< debug enable bit for algorithms
#define WeSU_DEBUG_BLUENRG_ENABLED      (WeSU_DEBUG_BLUENRG_MASK        & SESSION_REG(WeSU_DEBUG_MASK))        //!< debug enable bit for bluenrg
#define WeSU_DEBUG_PWR_MGNT_ENABLED     (WeSU_DEBUG_PWR_MGNT_MASK       & SESSION_REG(WeSU_DEBUG_MASK))        //!< debug enable bit for power management

uint8_t PRINTF(const char* sFormatString, ...);		//!< PRINTF ...


uint8_t PRINTF_INFO(const char* sFormatString, ...);	//!< PRINTF_INFO ...


uint8_t DBG_PRINTF(uint16_t L, const char* sFormatString, ...);	//!< DBG_PRINTF ...


uint8_t DBG_PRINTF_TERMINAL(uint16_t L, const char* sFormatString, ...);	//!< DBG_PRINTF_TERMINAL ...


#else //USE_REGS

extern   uint16_t WeSU_DEBUG_SEVERITY_ble;                                      //!<  Configured severity level for BLE
extern uint16_t WeSU_DEBUG_SEVERITY_usart;                                      //!<  Configured severity level for USART
#define WeSU_APP_DEFAULT_SEVERITY       WeSU_DEBUG_SEVERITY_CRITICAL            //!<  Configured severity level for USART

#define WeSU_DEBUG_MAIN_ENABLED     (1)        //!< debug enable bit for main
#define WeSU_DEBUG_WESU_CONFIG_ENABLED     (1)        //!< debug enable bit for wesu_config
#define WeSU_DEBUG_TERMINAL_ENABLED     (1)        //!< debug enable bit for terminal
#define WeSU_DEBUG_ALGORITHMS_ENABLED   (1)        //!< debug enable bit for algorithms
#define WeSU_DEBUG_BLUENRG_ENABLED      (1)        //!< debug enable bit for bluenrg
#define WeSU_DEBUG_PWR_MGNT_ENABLED     (1)        //!< debug enable bit for power management

#define PRINTF(...)     do{if(USE_PRINTF)\
                            if(WeSU_DEBUG_SEVERITY_VERBOSE<=ENABLED_DEBUG_LEVEL_USART || WeSU_DEBUG_SEVERITY_VERBOSE<=ENABLED_DEBUG_LEVEL_BLE || bDebuggerConnected)\
                            {\
                              if(WeSU_DEBUG_SEVERITY_VERBOSE<=ENABLED_DEBUG_LEVEL_USART){int bOldBpVal=ButtonPressed;BSP_PB_DeInit(BUTTON_USER);consoleInit();WESU_PRINTF_USART(__VA_ARGS__);consoleDeInit();BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);ButtonPressed = bOldBpVal;}\
                              if(WeSU_DEBUG_SEVERITY_VERBOSE<=ENABLED_DEBUG_LEVEL_BLE)WESU_PRINTF_BLE(__VA_ARGS__);\
                              if(bDebuggerConnected)printf(__VA_ARGS__);\
                            }\
                          }while(0)             //!< PRINTF ...

#define PRINTF_INFO(...)  do{if(USE_PRINTF)\
                              if(WeSU_DEBUG_SEVERITY_INFO<=ENABLED_DEBUG_LEVEL_BLE)WESU_PRINTF_BLE(__VA_ARGS__);\
                            }while(0)           //!< PRINTF_INFO ...

#define DBG_PRINTF(L,...) do{if(USE_PRINTF)\
                              if(L<=ENABLED_DEBUG_LEVEL_USART || L<=ENABLED_DEBUG_LEVEL_BLE)\
                              {\
                                if(L<=ENABLED_DEBUG_LEVEL_USART){int bOldBpVal=ButtonPressed;BSP_PB_DeInit(BUTTON_USER);consoleInit();WESU_PRINTF_USART(__VA_ARGS__);consoleDeInit();BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);ButtonPressed = bOldBpVal;}\
                                if(L<=ENABLED_DEBUG_LEVEL_BLE){WESU_PRINTF_BLE(__VA_ARGS__);}\
                              }\
                            }while(0)           //!< DBG_PRINTF ...

#define DBG_PRINTF_TERMINAL(L,...)      do{if(USE_PRINTF)\
                                          if(1)\
                                          {\
                                            BSP_PB_DeInit(BUTTON_USER);consoleInit();WESU_PRINTF_USART(__VA_ARGS__);consoleDeInit();BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);\
                                            WESU_PRINTF_BLE(__VA_ARGS__);\
                                          }\
                                        }while(0)       //!< DBG_PRINTF_TERMINAL ...

#endif //USE_REGS

uint8_t WESU_PRINTF_USART(const char* sFormatString, ...);
uint8_t WESU_PRINTF_BLE(const char* sFormatString, ...);

#define USE_TERMINAL                    1       //!< USE_TERMINAL ...

#define DBG_PRINTF_MAIN(L,...)          if(WeSU_DEBUG_MAIN_ENABLED){DBG_PRINTF(L,__VA_ARGS__);}     //!< DBG_PRINTF_MAIN ...
#define DBG_PRINTF_WESU_CONFIG(L,...)   if(WeSU_DEBUG_WESU_CONFIG_ENABLED){DBG_PRINTF(L,__VA_ARGS__);}     //!< DBG_PRINTF_WESU_CONFIG ...
#define DBG_PRINTF_PWR_MGNT(L,...)      if(WeSU_DEBUG_PWR_MGNT_ENABLED){DBG_PRINTF(L,__VA_ARGS__);}     //!< DBG_PRINTF_PWR_MGNT ...

#define DBG_PRINTF_ALGORITHMS(L,...)    if(WeSU_DEBUG_ALGORITHMS_ENABLED){DBG_PRINTF(L,__VA_ARGS__);}     //!< DBG_PRINTF_ALGORITHMS ...

#define DBG_PRINTF_BLUENRG(L,...)       if(WeSU_DEBUG_BLUENRG_ENABLED){DBG_PRINTF(L,__VA_ARGS__);}     //!< DBG_PRINTF_BLUENRG ...

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES

#ifndef DOXYGEN_SHOULD_SKIP_THIS

typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef unsigned int u32_t;
typedef int i32_t;
typedef short int i16_t;
typedef signed char i8_t;

#endif /* DOXYGEN_SHOULD_SKIP_THIS */

#endif /*__ARCHDEP__TYPES*/

/**
 * @struct NewAxesRaw_t
 * @brief  Axes raw structure definition
 */
typedef struct {
  u16_t TIME;           //!< Time frame
  i16_t AAXIS_X;        //!< Accelerometer X axes
  i16_t AAXIS_Y;        //!< Accelerometer Y axes
  i16_t AAXIS_Z;        //!< Accelerometer Z axes
  i16_t GAXIS_X;        //!< Gyroscope X axes
  i16_t GAXIS_Y;        //!< Gyroscope Y axes
  i16_t GAXIS_Z;        //!< Gyroscope Z axes
  i16_t MAXIS_X;        //!< Magnetometer X axes      
  i16_t MAXIS_Y;        //!< Magnetometer Y axes
  i16_t MAXIS_Z;        //!< Magnetometer Z axes
} NewAxesRaw_t;

/**
 * @struct AxesRaw_TypeDef
 * @brief  Axes raw structure definition
 */
typedef struct
{
  int16_t AXIS_X;       //!< Generic X int16 axes sensor
  int16_t AXIS_Y;       //!< Generic Y int16 axes sensor
  int16_t AXIS_Z;       //!< Generic Z int16 axes sensor
} AxesRaw_TypeDef;

/**
 * @struct AxesRawFloat_TypeDef
 * @brief  Axes raw Float structure definition
 */
typedef struct
{
  float AXIS_X;         //!< Generic X float axes sensor
  float AXIS_Y;         //!< Generic X float axes sensor
  float AXIS_Z;         //!< Generic X float axes sensor
} AxesRawFloat_TypeDef;


#define AUX_PERIPH_RAM_N_REG    20              //!< Regs Number in Ram for AUX Control  

  #ifdef USE_WESU
    #include "wesu.h"
    #include "wesu_sensors.h"
#ifdef WESU_DEMO
    #include "console.h"
#else //WESU_DEMO
    #include "console_examples.h"
#endif //WESU_DEMO
#else
    #error "Platform not defined"
  #endif


#ifndef FALSE
  #define FALSE         0                       //!< FALSE MACRO 
  #define TRUE !FALSE                           //!< TRUE MACRO
#endif

/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */

#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
