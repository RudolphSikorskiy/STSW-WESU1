/**
  ******************************************************************************
  * @file    Sensors_Read/Src/console_examples.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Console management function using BlueNRG interface or 
  *          Serial interface
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
 
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "console_examples.h"
#include "BlueST_Protocol_examples.h"
#include "wesu_config_examples.h"

/** @addtogroup WeSU_Examples        WeSU Examples       
  * @{
  */

/** @addtogroup WeSU_User_Examples               WeSU User Examples
  * @{
  */

/** @addtogroup Console_Examples          Console Examples
  * @{
  * @brief Console functionalities
  */

#define HUART_REC_BYTES                                 (UartHandle.RxXferSize - UartHandle.RxXferCount)        //!< Facility for calculation of number of received bytes on serial port
#define UART_ECHO                                       1                                                       //!< Enable/disable echo for serial port
#define UART_MINIMAL_ECHO                               1                                                       //!< Enable/disable minimal echo
#define COMMAND_OFFSET                                  0                                                       //!< Offset for console command frame

#define SERIAL_STRING_LENGHT                            TERMINAL_STRING_LENGHT-1                                //!< Buffer lenght for serial port
                                                        
/** 
  * @brief Command handler function typedef
  */
typedef void (*pCmdHandlerFunc_t)(char *pcInStr);

/**
  * @brief Command handler descriptor
  */
typedef struct s_cmd_struct {
  char cCmdString[TERMINAL_STRING_LENGHT];                              //!< Command string
  char cCmdHelp[TERMINAL_MAX_HELP_LEN];                                 //!< Command help string
  pCmdHandlerFunc_t pCmdHandlerFunc;                                    //!< Command function pointer
  uint8_t bShowInHelp;                                                  //!< To show the function with '?' command
} xCmdStruct_t;


void ProcDbgRebootOta(char *pcInStr);
void ProcDbgLedBlinkme(char *pcInStr);
void ProcDbgViewtime(char *pcInStr);
void ProcDbgSettime(char *pcInStr);
void ProcDbgGetFwVersion(char *pcInStr);
void ProcDbgGetMcuId(char *pcInStr);
void ProcDbgGetExample(char *pcInStr);
void ProcDbgHelp(char *pcInStr);


/**
  * @brief Console command descriptors array
  */
xCmdStruct_t xCmdStructVect[] = {
  { "?fwversion",       "View fw info",                         ProcDbgGetFwVersion,            1 },
  { "?mcuid",           "View STM32L1 MCU ID",                  ProcDbgGetMcuId,                1 },
  { "?example",         "View active examples",                 ProcDbgGetExample,              1 },
  { "!blinkme",         "Led ping",                             ProcDbgLedBlinkme,              1 },
  { "?time",            "Get system time",                      ProcDbgViewtime,                1 },
  { "!time",            "Set system time",                      ProcDbgSettime,                 1 },
  { "!ota",             "Set OTA mode",                         ProcDbgRebootOta,               1 },
  { "?",                "View help",                            ProcDbgHelp,                    1 },
};

UART_HandleTypeDef UartHandle;                                                          //!< Serial port handle

char cTerminalString[TERMINAL_STRING_LENGHT+1] = {0};                                   //!< Buffer to be processed by terminal
  
char cSerialString[SERIAL_STRING_LENGHT+1] = {0};                                       //!< Buffer for serial port

char sDestString[STD_OUT_ARRAY_LENGHT+1];                                               //!< Buffer to store the custom printf output
uint8_t bWesuPrintfBleMutex = 1;                                                        //!< Printf on Bletooth low energy mutex
uint8_t bCmdStringsVectLocked = 0;                                                      //!< Bluetooth low energy console mutex
//
uint32_t nTerminalIndex = 0;                                                            //!< Index to terminal buffer
char cChannel = '\0';                                                                   //!< Console channel: 'B' for Bluetooth low energy, 'U' for serial/usart


/** @brief Printf handler for usart
 * @param sFormatString printf format string
 * @retval None
 */
uint8_t WESU_PRINTF_USART(const char* sFormatString, ...)
{
  uint8_t xRes = 0;
  
  va_list ap;
  va_start (ap, sFormatString);
  vsnprintf(sDestString, STD_OUT_ARRAY_LENGHT, sFormatString, ap);
  va_end (ap);
  
  for(int i = 0; i < strlen(sDestString); i++)
  {
    uartSendChar(sDestString[i]);
  }
  
  return xRes;
}

/** @brief Printf handler for Bluetooth low energy
 * @param sFormatString printf format string
 * @retval None
 */
uint8_t WESU_PRINTF_BLE(const char* sFormatString, ...)
{
  uint8_t xRes = 0;
  if(bWesuPrintfBleMutex)
  {
  bWesuPrintfBleMutex = 0;
  va_list ap;
  va_start (ap, sFormatString);
  vsnprintf(sDestString, STD_OUT_ARRAY_LENGHT, sFormatString, ap);
  va_end (ap);
  
  if(Term_Update((uint8_t*)sDestString, strlen(sDestString))){ }
  
  bWesuPrintfBleMutex = 1;
  }
  return xRes;
}


/** @brief Get serial input
 * @retval 1: data present, 0: no data
 */
uint8_t GetSerialInput()
{
  uint8_t nRet = 0;
  static uint32_t previousRxXferSizeHuart = 0;
  
  if(0 != previousRxXferSizeHuart)
  {
    HAL_Delay(100);
    cChannel = 'U';
    memcpy(&cTerminalString[COMMAND_OFFSET], cSerialString, HUART_REC_BYTES);
    
    if(UART_ECHO)
    {
      if(UartHandle.Init.Mode == UART_MODE_TX_RX)
      {
        HAL_UART_Transmit(&UartHandle, (void*)cSerialString, HUART_REC_BYTES, HAL_MAX_DELAY);
      }
      else
      {
        consoleInit();
        HAL_UART_Transmit(&UartHandle, (void*)cSerialString, HUART_REC_BYTES, HAL_MAX_DELAY);
        consoleDeInit();
      }
    }
    UartHandle.RxXferCount = SERIAL_STRING_LENGHT;
    UartHandle.pRxBuffPtr = (void*)cSerialString;
    nRet = 1;
  }
  previousRxXferSizeHuart = HUART_REC_BYTES;
  return nRet;
}

/** @brief Show help
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgHelp(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nTerminal HELP:\r\n");
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\n");
  for(int i=0;i<countof(xCmdStructVect);i++)
  {
    if(xCmdStructVect[i].bShowInHelp == 1)
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, xCmdStructVect[i].cCmdString);
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "  - ");
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, xCmdStructVect[i].cCmdHelp);
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\n");
    }
  }
}

/**
 * @brief Get system time
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgViewtime(char *pcInStr)
{
  rtc_time_t t;
  WeSU_GetDateTime(&t);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nRTC config: %s", BSP_RTC_GetRtcConfigAsString(t.tm_rtcConf));
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nDate: %s %.2d-%s-%d",BSP_RTC_GetWeekDayName(t.tm_wday),t.tm_day,BSP_RTC_GetMonthName(t.tm_mon),t.tm_year);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nTime: %.2d:%.2d:%.2d\r\n",t.tm_hour,t.tm_min,t.tm_sec);
}

/**
 * @brief Set system time
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgSettime(char *pcInStr)
{
  char * pc = 0;
  uint8_t h = 0;
  uint8_t m = 0;
  uint8_t s = 0;
  uint8_t n = 0;
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Set RTC time\r\n");
  pc=strstr(pcInStr, "H");
  if(pc)
  {
    if(isdigit(*(pc+1)) && isdigit(*(pc+2)))
    {
      n = digit(*(pc+1));//MSB
      h = n;
      h = (h*10);
      n = digit(*(pc+2));
      h += n;
      pc=strstr(pcInStr, "M");
      if(pc)
      {
        if(isdigit(*(pc+1)) && isdigit(*(pc+2)))
        {
          n = digit(*(pc+1));//MSB
          m = n;
          m = (m*10);
          n = digit(*(pc+2));
          m += n;
          pc=strstr(pcInStr, "S");
          if(pc)
          {
            if(isdigit(*(pc+1)) && isdigit(*(pc+2)))
            {
              n = digit(*(pc+1));//MSB
              s = n;
              s = (s*10);
              n = digit(*(pc+2));
              s += n;
              
              /* Ready to set time */
              rtc_time_t t;
              WeSU_GetDateTime(&t);
              t.tm_hour = h;
              t.tm_min = m;
              t.tm_sec = s;
              t.tm_rtcConf = RTC_AppConfigForced;
              WeSU_SetDateTime(&t);
              DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "RTC set to:\r\n");
              WeSU_GetDateTime(&t);
              DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nDate: %s %.2d-%s-%d",BSP_RTC_GetWeekDayName(t.tm_wday),t.tm_day,BSP_RTC_GetMonthName(t.tm_mon),t.tm_year);
              DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nTime: %.2d:%.2d:%.2d\r\n",t.tm_hour,t.tm_min,t.tm_sec);
            }
            else
            {
              pc = 0;
            }
          }
        }
        else
        {
          pc = 0;
        }
      }
    }
    else
    {
      pc = 0;
    }
  }
  if(pc == 0)
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "USAGE: !timeHhhMmmSss\r\n");
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "where hh mm ss are hour, minutes and seconds\r\n");
  }
}

/** @brief Get active example list
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgGetExample(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---    VIEW ACTIVE EXAMPLES    ---\r\n"));
#if ACCELEROMETER_EXAMPLE
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---    ACCELEROMETER_EXAMPLE   ---\r\n"));
#endif //ACCELEROMETER_EXAMPLE
#if GYROSCOPE_EXAMPLE    
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---      GYROSCOPE_EXAMPLE     ---\r\n"));
#endif //GYROSCOPE_EXAMPLE    
#if MAGNETOMETER_EXAMPLE 
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---    MAGNETOMETER_EXAMPLE    ---\r\n"));
#endif //MAGNETOMETER_EXAMPLE 
#if GAS_GAUGE            
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---         GAS_GAUGE          ---\r\n"));
#endif //GAS_GAUGE            
#if PRESS_EXAMPLE        
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---       PRESS_EXAMPLE        ---\r\n"));
#endif //PRESS_EXAMPLE        
#if TEMPERATURE_EXAMPLE
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---     TEMPERATURE_EXAMPLE    ---\r\n"));
#endif //TEMPERATURE_EXAMPLE
#if USE_CUSTOM_ALGORITHM1
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---    USE_CUSTOM_ALGORITHM1   ---\r\n"));
#endif //USE_CUSTOM_ALGORITHM1
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---  END VIEW ACTIVE EXAMPLES  ---\r\n"));
}

/** @brief Get MCU factory ID
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgGetMcuId(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---    VIEW MCU ID    ---\r\n"));
  uint8_t sn[12];
  memcpy(sn, (void*)0x1FF80050, 12);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "--- %.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X ---\r\n", 
                      sn[0], sn[1], sn[2],  sn[3], 
                      sn[4], sn[5], sn[6],  sn[7], 
                      sn[8], sn[9], sn[10], sn[11]);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---  END VIEW MCU ID  ---\r\n"));
}

/** @brief Get WeSU firmware version
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgGetFwVersion(char *pcInStr)
{
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Current Example Fw version: %X.%X.%.2X\r\n", (0xF0&HIGH_BYTE(FIRMWARE_VERSION))>>4, 0x0F&HIGH_BYTE(FIRMWARE_VERSION), LOW_BYTE(FIRMWARE_VERSION));
  }
}

/** @brief Perform 4 Blinks on the led to locate the board
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgLedBlinkme(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("--- BLINK ME COMMAND ---\r\n"));
  APP_BSP_LedSmoothBlink(10);APP_BSP_LedSmoothBlink(10);
  APP_BSP_LedSmoothBlink(10);APP_BSP_LedSmoothBlink(10);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("--- END BLINK ME COMMAND ---\r\n"));
}

/** @brief Set BlueNRG connection interval
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgRebootOta(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---    Restart in OTA mode   ---\r\n"));
  HAL_Delay(1000);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("--- 3\r\n"));
  HAL_Delay(1000);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("--- 2\r\n"));
  HAL_Delay(1000);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("--- 1\r\n"));
  HAL_Delay(1000);
  // Check if OTA_FW_ADDRESS is a RAM address
  if (((*(__IO uint32_t*)OTA_FW_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
  {
    //OK, prepare address for restart and reset
    Switch_To_OTA_Service_Manager_Application(OTA_FW_ADDRESS);
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("--- Restart!\r\n"));
    HAL_Delay(WESU_DELAY_BEFORE_RESET);
    WESU_SYSTEM_RESET();
    return;
  }
  
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("Address not valid\r\n"));
  HAL_Delay(1000);
}

/**
 * @brief  Initialises STEVAL-WESU1 UART port
 * @retval 0
 */
int consoleInit(void)
{
  UartHandle.Instance        = USART2;

  UartHandle.Init.BaudRate   = COM_BAUDRATE;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;

  HAL_UART_DeInit(&UartHandle);
  HAL_UART_Init(&UartHandle);
  HAL_UART_Receive_IT(&UartHandle, (void*)cSerialString, SERIAL_STRING_LENGHT);
  
  return 0;
}


/**
 * @brief  DeInitialises STEVAL-WESU1 UART port
 * @retval 0
 */
int consoleDeInit(void)
{
  UartHandle.Instance        = USART2;

  UartHandle.Init.BaudRate   = COM_BAUDRATE;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_RX;

  HAL_UART_DeInit(&UartHandle);
  HAL_UART_Init(&UartHandle);
  HAL_UART_Receive_IT(&UartHandle, (void*)cSerialString, SERIAL_STRING_LENGHT);
  
  return 0;
}


/** @brief Sends a character to serial port
 * @param ch Character to send
 * @retval Character sent
 */
int uartSendChar(int ch)
{
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/** @brief Process terminal command
 * @param pCommand pointer to command buffer
 * @param nFrameLenght Frame lenght
 * @retval None
 */
void ProcessTermCommand(char *pCommand, int nFrameLenght)
{
#if !UART_MINIMAL_ECHO
  PRINTF("ECHO - ");
  PRINTF(pCommand);
  PRINTF("\r\n");
#endif
  
      /* search cmd in cmds list */
      pCmdHandlerFunc_t pHandle = NULL;
      
      for(int i=0;i<countof(xCmdStructVect);i++)
      {
        if(0==strncmp(xCmdStructVect[i].cCmdString,pCommand+COMMAND_OFFSET,strlen(xCmdStructVect[i].cCmdString)-COMMAND_OFFSET))
        {
          pHandle = xCmdStructVect[i].pCmdHandlerFunc;
          break;
        }
      }
      
      if(pHandle)
      {
#if !UART_MINIMAL_ECHO
        DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Handling command...\r\n");
#endif        
        /* Call the command handler */
        pHandle(pCommand+COMMAND_OFFSET);
        
#if !UART_MINIMAL_ECHO
        DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Command complete\r\n");
#endif
      }
      else
      {
        DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nCommand\r\n%s\r\n",pCommand+COMMAND_OFFSET);
        DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "not found, '?' for HELP\r\n");
      }
      memset(pCommand, 0x00, nFrameLenght);
}


/** @brief Manage terminal input
 * @retval None
 */
void ManageTerminal()
{
  if(0!=strstr(cTerminalString, "\n"))
  {
    ProcessTermCommand(cTerminalString, strlen(cTerminalString));
  }
  if(!bCmdStringsVectLocked)
  {
  if(GetSerialInput())
  {
    ProcessTermCommand(cTerminalString, strlen(cTerminalString));
  }
  }
  
  nTerminalIndex = 0;
  cChannel = '\0';
  bCmdStringsVectLocked = 0;
}


/** @brief Welcome message on terminal
 * @retval None
 */
void TerminalShowWelcome()
{
#if COMMAND_OFFSET==1
  ProcessTermCommand("B?", 1);
#else//COMMAND_OFFSET
  ProcessTermCommand("?", 1);
#endif//COMMAND_OFFSET
}

/** @brief Process terminal string from Bluetooth low energy
 * @param att_data
 * @param data_length
 * @retval None
 */
void ProcessTerminalStringBle(uint8_t * att_data, uint8_t data_length)
{
  if(!bCmdStringsVectLocked || cChannel == 'B')
  {
    bCmdStringsVectLocked = 1;
    if(data_length > TERMINAL_STRING_LENGHT - nTerminalIndex)
    {
      data_length = TERMINAL_STRING_LENGHT - nTerminalIndex;
    }
    
    cChannel = 'B';
    uint32_t nLen = strlen(cTerminalString);
        memcpy(&cTerminalString[nLen], att_data, data_length);
        nTerminalIndex += data_length;
        DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, &cTerminalString[nLen]);
  }
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
