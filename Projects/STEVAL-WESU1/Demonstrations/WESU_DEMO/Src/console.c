/**
  ******************************************************************************
  * @file    WESU_DEMO/Src/console.c
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
#include "console.h"
#include "BlueST_Protocol.h"
#include "wesu_config.h"

/** @addtogroup WeSU_Demo       WeSU Demo      
  * @{
  */

/** @addtogroup WeSU_User               WeSU User
  * @{
  */

/** @addtogroup Console          Console
  * @{
  * @brief Console functionalities
  */

/** @addtogroup Console_Private_Defines        Console Private Defines
  * @{
  */
#define HUART_REC_BYTES                                 (UartHandle.RxXferSize - UartHandle.RxXferCount)        //!< Facility for calculation of number of received bytes on serial port
#define UART_ECHO                                       1                                                       //!< Enable/disable echo for serial port
#define UART_MINIMAL_ECHO                               1                                                       //!< Enable/disable minimal echo
#define COMMAND_OFFSET                                  0                                                       //!< Offset for console command frame

#define SERIAL_STRING_LENGHT                            TERMINAL_STRING_LENGHT-1                                //!< Buffer lenght for serial port
#define P_SIZE                                          6                                                       //!< License management facility
#define DBG_PRINTF_LICENSE_P(L, LICENSE, P)             DBG_PRINTF_TERMINAL(L, "%.2X%.2X" "%.2X%.2X" "%.2X%.2X",\
                                                        LICENSE[(P_SIZE*P)+0],LICENSE[(P_SIZE*P)+1],\
                                                        LICENSE[(P_SIZE*P)+2],LICENSE[(P_SIZE*P)+3],\
                                                        LICENSE[(P_SIZE*P)+4],LICENSE[(P_SIZE*P)+5])            //!< License management facility

#define DBG_PRINTF_LICENSE(L, pLicense)                 DBG_PRINTF_LICENSE_P(L, pLicense, 0);\
                                                        DBG_PRINTF_LICENSE_P(L, pLicense, 1);\
                                                        DBG_PRINTF_TERMINAL(L, "\r\n");\
                                                        DBG_PRINTF_LICENSE_P(L, pLicense, 2);\
                                                        DBG_PRINTF_LICENSE_P(L, pLicense, 3);\
                                                        DBG_PRINTF_TERMINAL(L, "\r\n");\
                                                        DBG_PRINTF_LICENSE_P(L, pLicense, 4);\
                                                        DBG_PRINTF_LICENSE_P(L, pLicense, 5);\
                                                        DBG_PRINTF_TERMINAL(L, "\r\n");\
                                                        DBG_PRINTF_LICENSE_P(L, pLicense, 6);\
                                                        DBG_PRINTF_LICENSE_P(L, pLicense, 7);\
                                                        DBG_PRINTF_TERMINAL(L, "\r\n");                         //!< License management facility

#define DBG_PRINTF_LICENSE_CMD(L, pLicense, N)          DBG_PRINTF_TERMINAL(L, "!lic%d_", N);\
                                                        DBG_PRINTF_LICENSE_P(L, pLicense, 0);\
                                                        DBG_PRINTF_LICENSE_P(L, pLicense, 1);\
                                                        DBG_PRINTF_LICENSE_P(L, pLicense, 2);\
                                                        DBG_PRINTF_LICENSE_P(L, pLicense, 3);\
                                                        DBG_PRINTF_LICENSE_P(L, pLicense, 4);\
                                                        DBG_PRINTF_LICENSE_P(L, pLicense, 5);\
                                                        DBG_PRINTF_LICENSE_P(L, pLicense, 6);\
                                                        DBG_PRINTF_LICENSE_P(L, pLicense, 7);\
                                                        DBG_PRINTF_TERMINAL(L, "\r\n");                         //!< License management facility
/**
  * @}
  */
                                                        
/** @defgroup Console_Private_Types      Console Private Types
  * @{
  */

/** 
  * @brief Command handler function typedef
  */
typedef void (*pCmdHandlerFunc_t)(char *pcInStr);

/**
  * @brief Command handler descriptor
  */
typedef struct s_cmd_struct {
  char cCmdString[20];                          //!< Command string
  char cCmdHelp[50];                            //!< Command help string
  pCmdHandlerFunc_t pCmdHandlerFunc;            //!< Command function pointer
  uint8_t bShowInHelp;                          //!< To show the function with '?' command
} xCmdStruct_t;


/**
  * @}
  */

/** @defgroup Console_Private_Functions       Console Private Functions
  * @{
  */

void ProcDbgViewtime(char *pcInStr);
void ProcDbgSettime(char *pcInStr);
void ProcDbgViewdate(char *pcInStr);
void ProcDbgSetdate(char *pcInStr);
void ProcDbgGetSysInfo(char *pcInStr);
void ProcDbgGetBackupData(char *pcInStr);
void ProcDbgGetPowerInfo(char *pcInStr);
void ProcDbgResetPower(char *pcInStr);
void ProcDbgGetFwVersion(char *pcInStr);
void ProcDbgGetMcuId(char *pcInStr);
void ProcDbgEmptyCmd(char *pcInStr);
void ProcDbgTestIwdg(char *pcInStr);
void ProcDbgLedHigh(char *pcInStr);
void ProcDbgLedBlinkme(char *pcInStr);
void ProcDbgLedLow(char *pcInStr);
void ProcDbgGetSessionRegVal(char *pcInStr);
void ProcDbgSetSessionRegVal(char *pcInStr);
void ProcDbgGetPersRegVal(char *pcInStr);
void ProcDbgSetPersRegVal(char *pcInStr);
void ProcDbgGetAlgoStatus(char *pcInStr);
void ProcDbgEraseLicenses(char *pcInStr);
void ProcDbgGetLic1(char *pcInStr);
void ProcDbgSetLic1(char *pcInStr);
void ProcDbgGetLic2(char *pcInStr);
void ProcDbgSetLic2(char *pcInStr);
void ProcDbgGetLic3(char *pcInStr);
void ProcDbgSetLic3(char *pcInStr);
void ProcDbgHelpHidden(char *pcInStr);
void ProcDbgEchoNameHidden(char *pcInStr);
void ProcDbgDoNotReplyHidden(char *pcInStr);
void ProcDbgSTC3115VoltHidden(char *pcInStr);
void ProcDbgSTC3115CurrentHidden(char *pcInStr);
void ProcDbgHelp(char *pcInStr);
void ProcDbgSetBleConIntvMode(char *pcInStr);
void ProcDbgSetUsbBleBridge(char *pcInStr);
void ProcDbgGetBluenrgInfo(char *pcInStr);

#if USE_CUSTOM_ALGORITHM3
void ProcDbgViewPedometer(char *pcInStr);
void ProcDbgViewPedometerOn(char *pcInStr);
void ProcDbgViewPedometerOff(char *pcInStr);
void ProcDbgViewPedometerRst(char *pcInStr);
#endif //USE_CUSTOM_ALGORITHM3

void ProcDbgEnterStop(char *pcInStr);
void ProcDbgEnterStopWBle(char *pcInStr);
void ProcDbgEnterConnOnly(char *pcInStr);
void ProcDbgReboot(char *pcInStr);
void ProcDbgRebootDefSettings(char *pcInStr);
void ProcDbgDumpAccGyroHidden(char *pcInStr);
void ProcDbgDumpMagHidden(char *pcInStr);
void ProcDbgDumpPressTempHidden(char *pcInStr);
void ProcDbgRebootAccGyroHidden(char *pcInStr);
void ProcDbgRebootMagHidden(char *pcInStr);
void ProcDbgRebootPressTempHidden(char *pcInStr);

void ProcDbgShutdown(char *pcInStr);
void ProcDbgMinPower(char *pcInStr);
void ProcDbgNormPower(char *pcInStr);

/**
  * @}
  */


/** @addtogroup Console_Private_Variables        Console Private Variables
  * @{
  */

/**
  * @brief Console command descriptors array
  */
xCmdStruct_t xCmdStructVect[] = {
  { "?fwversion",       "View fw info",                         ProcDbgGetFwVersion,            1 },
  { "?mcuid",           "View STM32L1 MCU ID",                  ProcDbgGetMcuId,                1 },
  { "?sysinfo",         "Get system information",               ProcDbgGetSysInfo,              1 },
  { "!empty",           "Empty test command",                   ProcDbgEmptyCmd,                1 },
  { "?regsession",      "View session reg value",               ProcDbgGetSessionRegVal,        1 },
  { "!regsession",      "Set session reg value",                ProcDbgSetSessionRegVal,        1 },
  { "?regpers",         "View pers reg value",                  ProcDbgGetPersRegVal,           1 },
  { "!regpers",         "Set pers reg value",                   ProcDbgSetPersRegVal,           1 },
  { "?time",            "Get system time",                      ProcDbgViewtime,                1 },
  { "!time",            "Set system time",                      ProcDbgSettime,                 1 },
  { "?date",            "Get system date",                      ProcDbgViewdate,                1 },
  { "!date",            "Set system date",                      ProcDbgSetdate,                 1 },
  { "!stopble",         "Enter Permanent Stop with Ble",        ProcDbgEnterStopWBle,           1 },
  { "!stop",            "Enter Permanent Stop",                 ProcDbgEnterStop,               1 },
  { "!connonly",        "Enter Ble Connection Only",            ProcDbgEnterConnOnly,           1 },
//  { "!rebootdefault",   "Reboot with default settings",         ProcDbgRebootDefSettings,       1 },
//  { "!reboot",          "Reboot system",                        ProcDbgReboot,                  1 },
//  { "!shutdown",        "System shutdown",                      ProcDbgShutdown,                1 },
  { "?powerstatus",     "View power info",                      ProcDbgGetPowerInfo,            1 },
  { "!minpower",        "System minimal power",                 ProcDbgMinPower,                1 },
  { "!normpower",       "System normal power",                  ProcDbgNormPower,               1 },
  { "!blinkme",         "Led ping",                             ProcDbgLedBlinkme,              1 },
  { "?bluenrginfo",     "Get BlueNRG info",                     ProcDbgGetBluenrgInfo,          1 },
  { "!bleconintv",      "Set Ble Conn Interval",                ProcDbgSetBleConIntvMode,       1 },
  { "!bluenrgbridge",   "Restart in USB-BlueNRG Bridge",        ProcDbgSetUsbBleBridge,         1 },
  { "?bckdata",         "Get saved data",                       ProcDbgGetBackupData,           1 },
#if USE_CUSTOM_ALGORITHM3
  { "?pedometer",       "Get pedometer data",                   ProcDbgViewPedometer,           1 },
  { "!pedo_start",      "Start pedometer algorithm",            ProcDbgViewPedometerOn,         1 },
  { "!pedo_stop",       "Stop pedometer algorithm",             ProcDbgViewPedometerOff,        1 },
  { "!pedo_reset",      "Reset pedometer algorithm",            ProcDbgViewPedometerRst,        1 },
#endif //USE_CUSTOM_ALGORITHM3
  { "?algostatus",      "View algorithms status",               ProcDbgGetAlgoStatus,           1 },
  { "!eraselics",       "Erase licenses",                       ProcDbgEraseLicenses,           0 },
  { "?lic1",            "View license1 value",                  ProcDbgGetLic1,                 0 },
  { "!lic1_",           "Set license1 value",                   ProcDbgSetLic1,                 0 },
  { "?lic2",            "View license2 value",                  ProcDbgGetLic2,                 0 },
  { "!lic2_",           "Set license2 value",                   ProcDbgSetLic2,                 0 },
  { "?lic3",            "View license3 value",                  ProcDbgGetLic3,                 0 },
  { "!lic3_",           "Set license3 value",                   ProcDbgSetLic3,                 0 },
  { "?hidden",          "View help hidden",                     ProcDbgHelpHidden,              0 },
  { "?hregsession",     "View session reg value (no action)",   ProcDbgGetSessionRegVal,        0 },
  { "!hregsession",     "Set session reg value (no action)",    ProcDbgSetSessionRegVal,        0 },
  { "?hregpers",        "View pers reg value (no action)",      ProcDbgGetPersRegVal,           0 },
  { "!hregpers",        "Set pers reg value (no action)",       ProcDbgSetPersRegVal,           0 },
  { "!dumpaccgyro",     "Dump the accelerometer memory",        ProcDbgDumpAccGyroHidden,       0 },
  { "!dumpmag",         "Dump the magnetometer memory",         ProcDbgDumpMagHidden,           0 },
  { "!dumpprestemp",    "Dump the press and temp memory",       ProcDbgDumpPressTempHidden,     0 },
  { "!rebootaccgyro",   "Reboot Accelerometer and Gyroscope",   ProcDbgRebootAccGyroHidden,     0 },
  { "!rebootmag",       "Reboot just the Magnetometer",         ProcDbgRebootMagHidden,         0 },
  { "!rebootpresstemp", "Reboot just the Press and Temp",       ProcDbgRebootPressTempHidden,   0 },
  { "*rst",             "NULL",                                 ProcDbgDoNotReplyHidden,        0 },
  { "*IDN?",            "Echo WeSU name",                       ProcDbgEchoNameHidden,          0 },
  { "syst:remote",      "NULL",                                 ProcDbgDoNotReplyHidden,        0 },
  { "conf:volt:dc",     "View STC3115 Volt",                    ProcDbgSTC3115VoltHidden,       0 },
  { "conf:curr:dc",     "View STC3115 Current",                 ProcDbgSTC3115CurrentHidden,    0 },
  { "!powerreset",      "Reset STC3115",                        ProcDbgResetPower,              1 },
  { "read?",             "NULL",                                ProcDbgDoNotReplyHidden,        0 },
#if USE_IWDG
  { "!testiwdg",        "Test IWDG",                            ProcDbgTestIwdg,                0 },
#endif //USE_IWDG
  { "?",                "View help",                            ProcDbgHelp,                    1 },
};

UART_HandleTypeDef UartHandle;                                                          //!< Serial port handle

char cTerminalString[TERMINAL_STRING_LENGHT+1] = {0};                                   //!< Buffer to be processed by terminal
  
char cSerialString[SERIAL_STRING_LENGHT+1] = {0};                                       //!< Buffer for serial port

char sDestString[STD_OUT_ARRAY_LENGHT+1];                                               //!< Buffer to store the custom printf output
char sDestString2[STD_OUT_ARRAY_LENGHT+1];                                               //!< Buffer to store the custom printf output
uint8_t bWesuPrintfBleMutex = 1;                                                        //!< Printf on Bletooth low energy mutex
uint8_t bCmdStringsVectLocked = 0;                                                      //!< Bluetooth low energy console mutex

uint32_t nTerminalIndex = 0;                                                            //!< Index to terminal buffer
char cChannel = '\0';                                                                   //!< Console channel: 'B' for Bluetooth low energy, 'U' for serial/usart

/**
  * @}
  */

/** @addtogroup Console_Imported_Variables        Console Imported Variables
  * @{
  */

extern void *ACCELERO_handle;
extern void *GYRO_handle;
extern void *MAGNETO_handle;
extern void *PRESSURE_handle;
extern void *TEMPERATURE_handle;

/**
  * @}
  */  
  
/** @addtogroup Console_Private_Functions
  * @{
  */

/** @brief WeSU sscanf utility function implemetation: scan a string for a hexadecimal value
 * @param B pointer to destination hex value
 * @param S pointer to input string
 * @param N string size
 * @retval 1 if integer has been filled, 0 otherwise
 */
uint8_t SSCANF_WESU_X(uint8_t *B,char*S,uint32_t N)
{
  char tmpstr[3];
  int tmpval;
  while(N--)
  {
    tmpstr[0] = *(uint8_t*)(S++);
    tmpstr[1] = *(uint8_t*)(S++);
    uint8_t nRet = sscanf(tmpstr, "%X", &tmpval);
    if(nRet)
    {
      *(uint8_t*)(B++) = tmpval&0xFF;
    }
    else
    {
      return 0;
    }
  }
  return 1;
}

/** @brief WeSU sscanf utility function implemetation: scan a string for a integer value
 * @param B pointer to destination integer
 * @param S pointer to input string
 * @param N string size
 * @retval 1 if integer has been filled, 0 otherwise
 */
uint8_t SSCANF_WESU_N(uint8_t *B,char*S,uint32_t N)
{
  char tmpstr[3];
  int tmpval;
  while(N--)
  {
    tmpstr[0] = *(uint8_t*)(S++);
    tmpstr[1] = *(uint8_t*)(S++);
    uint8_t nRet = sscanf(tmpstr, "%X", &tmpval);
    if(nRet)
    {
      *(uint8_t*)(B++) = tmpval&0xFF;
    }
    else
    {
      return 0;
    }
  }
  return 1;
}

/** @brief WeSU utility function to left-shift a string by nLen chars
 * @param s String to shift
 * @param nLen maximum number of chars to shift
 * @retval None
 */
void lStringShift(char*s, uint32_t nLen)
{
  while(*(s+nLen))
  {
    *s=*(s+nLen);
    s++;
  }
  *s='\0';
}

/** @brief Check if each character of a string is a hexadecimal digit
 * @param s String to check
 * @param nMaxLen Max number of chars to check
 * @retval None
 */
uint8_t CheckXDigitString(char *s, uint8_t nMaxLen)
{
  uint32_t nLen = strlen(s);
  if(nLen < nMaxLen)
  {
    nMaxLen = nLen;
  }
  
  uint8_t i;
  for( i = 0; i < nMaxLen; i++)
  {
    if(isxdigit(*(s+i)))
    {
    }
    else
    {
      return 1;
    }
  }
  return 0;
}

#if defined (__IAR_SYSTEMS_ICC__)

size_t __write(int Handle, const unsigned char * Buf, size_t Bufsize);
size_t __read(int Handle, unsigned char *Buf, size_t Bufsize);

/** @brief IAR specific low level standard input
 * @param Handle IAR internal handle
 * @param Buf Buffer where to store characters read from stdin
 * @param Bufsize Number of characters to read
 * @retval Number of characters read
 */
size_t __read(int Handle, unsigned char *Buf, size_t Bufsize)
{
  int i;

  if (Handle != 0){
    return -1;
  }

  for(i=0; i<Bufsize; i++)
    Buf[i] = uartReceiveChar();

  return Bufsize;
}

/** @brief IAR specific low level standard output
 * @param Handle IAR internal handle
 * @param Buf Buffer containing characters to be written to stdout
 * @param Bufsize Number of characters to write
 * @retval Number of characters read
 */
size_t __write(int Handle, const unsigned char * Buf, size_t Bufsize)
{
  int i;

  if (Handle != 1 && Handle != 2){
    return -1;
  }

  for(i=0; i< Bufsize; i++)
    uartSendChar(Buf[i]);

  return Bufsize;
}

#elif defined (__CC_ARM)

/**
 * @brief fputc call for standard output implementation
 * @param ch Character to print
 * @param f File pointer
 * @retval Character printed
 */
int fputc(int ch, FILE *f)
{
  return uartSendChar(ch);
}

/** @brief fgetc call for standard input implementation
 * @param f File pointer
 * @retval Character acquired from standard input
 */
int fgetc(FILE *f)
{
  return uartReceiveChar();
}

#elif defined (__GNUC__)

/** @brief putchar call for standard output implementation
 * @param ch Character to print
 * @retval Character printed
 */
int __io_putchar(int ch)
{
  return uartSendChar(ch);
}

/** @brief getchar call for standard input implementation
 * @retval Character acquired from standard input
 */
int __io_getchar(void)
{
  return uartReceiveChar();
}

#else
#error "Toolchain not supported"
#endif


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
  
  int i;
  for(i = 0; i < strlen(sDestString); i++)
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
  
  if(Term_Update((uint8_t*)sDestString, strlen(sDestString))){  SET_BNRG_ERROR_FLAG(); }
  
  bWesuPrintfBleMutex = 1;
  }
  return xRes;
}

uint8_t PRINTF(const char* sFormatString, ...)
{
  uint8_t xRes = 0;
  if(USE_PRINTF)
  {
    va_list ap;
    va_start (ap, sFormatString);
    vsnprintf(sDestString2, STD_OUT_ARRAY_LENGHT, sFormatString, ap);
    va_end (ap);
    if(WeSU_DEBUG_SEVERITY_VERBOSE<=ENABLED_DEBUG_LEVEL_USART)
    {
      int bOldBpVal=ButtonPressed;
      BSP_PB_DeInit(BUTTON_USER);
      consoleInit();
      WESU_PRINTF_USART(sDestString2);
      consoleDeInit();
      BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
      ButtonPressed = bOldBpVal;
    }
    if(WeSU_DEBUG_SEVERITY_VERBOSE<=ENABLED_DEBUG_LEVEL_BLE)
    {
      WESU_PRINTF_BLE(sDestString2);
    }
    /*if(bDebuggerConnected)printf(sDestString2);*/
  }
  return xRes;
}

uint8_t PRINTF_INFO(const char* sFormatString, ...)
{
  uint8_t xRes = 0;
  if(USE_PRINTF)
  {
    va_list ap;
    va_start (ap, sFormatString);
    vsnprintf(sDestString2, STD_OUT_ARRAY_LENGHT, sFormatString, ap);
    va_end (ap);
    if(WeSU_DEBUG_SEVERITY_VERBOSE<=ENABLED_DEBUG_LEVEL_BLE)
    {
      WESU_PRINTF_BLE(sDestString2);
    }
    /*if(bDebuggerConnected)printf(sDestString2);*/
  }
  return xRes;
}

uint8_t DBG_PRINTF(uint16_t L, const char* sFormatString, ...)
{
  uint8_t xRes = 0;
  if(USE_PRINTF)
  {
    if(L<=ENABLED_DEBUG_LEVEL_USART || L<=ENABLED_DEBUG_LEVEL_BLE)
    {
      va_list ap;
      va_start (ap, sFormatString);
      vsnprintf(sDestString2, STD_OUT_ARRAY_LENGHT, sFormatString, ap);
      va_end (ap);
      
      if(L<=ENABLED_DEBUG_LEVEL_USART)
      {
        int bOldBpVal=ButtonPressed;
        BSP_PB_DeInit(BUTTON_USER);
        consoleInit();
        WESU_PRINTF_USART(sDestString2);
        consoleDeInit();
        BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
        ButtonPressed = bOldBpVal;
      }
      if(L<=ENABLED_DEBUG_LEVEL_BLE)
      {
        WESU_PRINTF_BLE(sDestString2);
      }
    }
  }
  return xRes;
}

uint8_t DBG_PRINTF_TERMINAL(uint16_t L, const char* sFormatString, ...)
{
  uint8_t xRes = 0;
  if(USE_PRINTF)
  {
    if(WeSU_DEBUG_TERMINAL_ENABLED)
    {
      va_list ap;
      va_start (ap, sFormatString);
      vsnprintf(sDestString2, STD_OUT_ARRAY_LENGHT, sFormatString, ap);
      va_end (ap);
      if(L<=ENABLED_DEBUG_LEVEL_USART)
      {
        int bOldBpVal=ButtonPressed;
        BSP_PB_DeInit(BUTTON_USER);
        consoleInit();
        WESU_PRINTF_USART(sDestString2);
        consoleDeInit();
        BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
        ButtonPressed = bOldBpVal;
      }
      if(L<=ENABLED_DEBUG_LEVEL_BLE)
      {
        WESU_PRINTF_BLE(sDestString2);
      }
    }
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


/** @brief Get command descriptor index
 * @param pCmd pointer to the command string
 * @retval descriptor index, 0xFFFFFFFF not found
 */
uint32_t GetDescriptorIndex(char* pCmd)
{
	int i;
      for(i=0;i<countof(xCmdStructVect);i++)
      {
        if(0==strncmp(xCmdStructVect[i].cCmdString,pCmd,strlen(xCmdStructVect[i].cCmdString)))
        {
          return i;
        }
      }
      return 0xFFFFFFFF;
}

void ProcDbgGetSysInfo(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("--- GET INFO COMMAND ---\r\n"));
  
  ProcDbgGetFwVersion("");
  ProcDbgGetMcuId("");
  
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "MCU clock %d Hz\r\n", SystemCoreClock);
  
  PrintSystemParameters();
  
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("--- END GET INFO COMMAND ---\r\n"));
}

/** @brief Get power information
 * @param pcInStr Input string
 * @retval None
 */
void ProcDbgGetPowerInfo(char *pcInStr)
{
    if(SESSION_REG(nRsvdDummyRegStc3115) == APP_STC3115_OK)
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nSTC31xx values:");
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "charge %d.%d%%\r\n", SESSION_REG(nChargepercent)/10,SESSION_REG(nChargepercent)%10);
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "voltage %d.%dV\r\n", (int32_t)(SESSION_REG(fBatteryVolt))/1000,(uint32_t)(SESSION_REG(fBatteryVolt))%1000);
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "current %dmA\r\n", (int32_t)(SESSION_REG(fCurrentuA)));
    }
    else
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nSTC31xx ERROR\r\n");
    }
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "errors %d\r\n", LOW_BYTE(SESSION_REG(nRsvdDummyRegStc3115)));
}

/** @brief Get power information
 * @param pcInStr Input string
 * @retval None
 */
void ProcDbgResetPower(char *pcInStr)
{
#if USE_STC3115
  BSP_GG_Reset();
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nSTC3115 RESET\r\n");
#endif //USE_STC3115
}

/** @brief Show help hint
 * @param pcInStr Input string
 * @retval None
 */
void ProcDbgBleDefaultHelp(char *pcInStr)
{
  if(0==strncmp(pcInStr, "B_?", 2))
  {
    PRINTF("Type '?' for help\r\n");
  }
}

/** @brief Show help
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgHelp(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nTerminal HELP:\r\n");
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\n");
  int i;
  for(i=0;i<countof(xCmdStructVect);i++)
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

/** @brief Echo WeSU name
 * @param pcInStr Input string
 * @retval None
 */
void ProcDbgEchoNameHidden(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "WeSU_Hal%.2X%.2X\r\n", HIGH_BYTE(FIRMWARE_VERSION), LOW_BYTE(FIRMWARE_VERSION));
}

/** @brief Facility for commands without echo
 * @param pcInStr Input string
 * @retval None
 */
void ProcDbgDoNotReplyHidden(char *pcInStr)
{
}

/** @brief Voltage measurement
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgSTC3115VoltHidden(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "%d.%.3d\r\n", (int32_t)(SESSION_REG(fBatteryVolt))/1000,(uint32_t)(SESSION_REG(fBatteryVolt))%1000);
}

/** @brief Current measurement
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgSTC3115CurrentHidden(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "%d\r\n", (int32_t)(SESSION_REG(fCurrentuA)));
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "%d.%d\r\n", SESSION_REG(nChargepercent)/10,SESSION_REG(nChargepercent)%10);
}

/** @brief Help function for hidden functions
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgHelpHidden(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nTerminal HELP:\r\n");
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\n");
  int i;
  for(i=0;i<countof(xCmdStructVect);i++)
  {
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, xCmdStructVect[i].cCmdString);
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "  - ");
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, xCmdStructVect[i].cCmdHelp);
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\n");
    }
  }
}

/** @brief Set session register
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgSetSessionRegVal(char *pcInStr)
{
  char * pc = 0;
  uint8_t a = 0;//address
  uint16_t v = 0;//value
  uint8_t n = 0;//nibble
  uint8_t nWithAction = (0 != strncmp(pcInStr, "!h",2));
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Set session reg value\r\n");
  pc=strstr(pcInStr, "R");
  if(pc)
  {
    if(isxdigit(*(pc+1)) && isxdigit(*(pc+2)))
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "reg addr format OK\r\n");
      //reg address is valid, check value
      n = xdigit(*(pc+1));//MSB
      a |= n;
      a = (a<<4);
      n = xdigit(*(pc+2));
      a |= n;
      if(*(pc+3) == 'V')
      {
        if(isxdigit(*(pc+4)) && isxdigit(*(pc+5)) && isxdigit(*(pc+6)) && isxdigit(*(pc+7)))
        {
          DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "reg value format OK\r\n");
          //reg value ok, set register
          n = xdigit(*(pc+4));//MSB
          v |= n;
          v = (v<<4);
          n = xdigit(*(pc+5));
          v |= n;
          v = (v<<4);
          n = xdigit(*(pc+6));
          v |= n;
          v = (v<<4);
          n = xdigit(*(pc+7));//LSB
          v |= n;
          DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "set reg 0x%.2X to value 0x%.4X\r\n",a,v);
          if(nWithAction)
          {
            BSP_SESSIONREGS_WRITE_WACTION(a,(uint8_t*)&v,1);
          }
          else
          {
            BSP_SESSIONREGS_WRITE(a,(uint8_t*)&v,1);
            DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "(No action performed)\r\n");
          }
        }
        else
        {
          pc = 0;
        }
      }
      else
      {
        pc = 0;
      }
    }
    else
    {
      pc = 0;
    }
  }
  if(pc == 0)
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "USAGE: !regsessionRXXVYYYY\r\n");
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "where XX is the reg address\r\n");
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "and YYYY is the reg value\r\n");
  }
}

/**
 * @brief Retrieve stored system data
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgGetBackupData(char *pcInStr)
{
  #if USE_RTC
  WesuBackupData_t d;
  WesuGetBackupData(SYS_PARAMS_BACKUP_POWERON_DONE, &d);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nDisplay poweron saved data for:");
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Date: %s %.2d-%s-%d\r\n",BSP_RTC_GetWeekDayName(d.tm_wday),d.tm_day,BSP_RTC_GetMonthName(d.tm_mon),d.tm_year);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Time: %.2d:%.2d:%.2d\r\n",d.tm_hour,d.tm_min,d.tm_sec);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Ticks: %d \r\n", d.nTicks);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Battery: %d.%dV \r\n", (int32_t)(d.fBatteryVolt)/1000,(uint32_t)(d.fBatteryVolt)%1000);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Steps: %d\r\n", d.nSteps);
  WesuGetBackupData(SYS_PARAMS_BACKUP_SESSION_DONE, &d);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nDisplay session saved data for:\r\n");
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Date: %s %.2d-%s-%d\r\n",BSP_RTC_GetWeekDayName(d.tm_wday),d.tm_day,BSP_RTC_GetMonthName(d.tm_mon),d.tm_year);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Time: %.2d:%.2d:%.2d\r\n",d.tm_hour,d.tm_min,d.tm_sec);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Ticks: %d \r\n", d.nTicks);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Battery: %d.%dV \r\n", (int32_t)(d.fBatteryVolt)/1000,(uint32_t)(d.fBatteryVolt)%1000);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Steps: %d\r\n", d.nSteps);
  #endif //USE_RTC
}

/**
 * @brief Get system time
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgViewtime(char *pcInStr)
{
  #if USE_RTC
  WeSU_GetDateTime((rtc_time_t *)&(SESSION_REG(tm_hour)));
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nRTC config: %s", BSP_RTC_GetRtcConfigAsString(SESSION_REG(tm_rtcConf)));
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nDate: %s %.2d-%s-%d",BSP_RTC_GetWeekDayName(SESSION_REG(tm_wday)),SESSION_REG(tm_day),BSP_RTC_GetMonthName(SESSION_REG(tm_mon)),SESSION_REG(tm_year));
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nTime: %.2d:%.2d:%.2d\r\n",SESSION_REG(tm_hour),SESSION_REG(tm_min),SESSION_REG(tm_sec));
  #endif //USE_RTC
}

/**
 * @brief Get system date
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgViewdate(char *pcInStr)
{
  #if USE_RTC
  WeSU_GetDateTime((rtc_time_t *)&(SESSION_REG(tm_hour)));
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nRTC config: %s", BSP_RTC_GetRtcConfigAsString(SESSION_REG(tm_rtcConf)));
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nDate: %s %.2d-%s-%d",BSP_RTC_GetWeekDayName(SESSION_REG(tm_wday)),SESSION_REG(tm_day),BSP_RTC_GetMonthName(SESSION_REG(tm_mon)),SESSION_REG(tm_year));
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nTime: %.2d:%.2d:%.2d\r\n",SESSION_REG(tm_hour),SESSION_REG(tm_min),SESSION_REG(tm_sec));
  #endif //USE_RTC
}

/**
 * @brief Set system time
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgSettime(char *pcInStr)
{
  #if USE_RTC
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
              memcpy(&t, (rtc_time_t *)&(SESSION_REG(tm_hour)), 8);
              t.tm_hour = h;
              t.tm_min = m;
              t.tm_sec = s;
              t.tm_rtcConf = RTC_AppConfigForced;
              WeSU_SetDateTime(&t);
              DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "RTC set to:");
              WeSU_GetDateTime((rtc_time_t *)&(SESSION_REG(tm_hour)));
              DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nDate: %s %.2d-%s-%d",BSP_RTC_GetWeekDayName(SESSION_REG(tm_wday)),SESSION_REG(tm_day),BSP_RTC_GetMonthName(SESSION_REG(tm_mon)),SESSION_REG(tm_year));
              DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nTime: %.2d:%.2d:%.2d\r\n",SESSION_REG(tm_hour),SESSION_REG(tm_min),SESSION_REG(tm_sec));
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
  #endif //USE_RTC
}

/**
 * @brief Set system date
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgSetdate(char *pcInStr)
{
  #if USE_RTC
  
  char * pc = 0;
  uint8_t d = 0;
  uint8_t m = 0;
  uint8_t y = 0;
  uint8_t w = 0;
  uint8_t n = 0;
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Set RTC date\r\n");
  pc=strstr(pcInStr, "D");
  if(pc)
  {
    if(isdigit(*(pc+1)) && isdigit(*(pc+2)))
    {
      n = digit(*(pc+1));//MSB
      d = n;
      d = (d*10);
      n = digit(*(pc+2));
      d += n;
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
          pc=strstr(pcInStr, "Y");
          if(pc)
          {
            if(isdigit(*(pc+1)) && isdigit(*(pc+2)))
            {
              n = digit(*(pc+1));//MSB
              y = n;
              y = (y*10);
              n = digit(*(pc+2));
              y += n;
              
              pc=strstr(pcInStr, "W");
              if(pc)
              {
                if(isdigit(*(pc+1)) && isdigit(*(pc+2)))
                {
                  n = digit(*(pc+1));//MSB
                  w = n;
                  w = (w*10);
                  n = digit(*(pc+2));
                  w += n;
                  
                  /* Ready to set date */
                  rtc_time_t t;
                  memcpy(&t, (rtc_time_t *)&(SESSION_REG(tm_hour)), 8);
                  t.tm_day = d;
                  t.tm_mon = m;
                  t.tm_year = y;
                  t.tm_wday = w;
                  t.tm_rtcConf = RTC_AppConfigForced;
                  WeSU_SetDateTime(&t);
                  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "RTC set to:");
                  WeSU_GetDateTime((rtc_time_t *)&(SESSION_REG(tm_hour)));
                  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nDate: %s %.2d-%s-%d",BSP_RTC_GetWeekDayName(SESSION_REG(tm_wday)),SESSION_REG(tm_day),BSP_RTC_GetMonthName(SESSION_REG(tm_mon)),SESSION_REG(tm_year));
                  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nTime: %.2d:%.2d:%.2d\r\n",SESSION_REG(tm_hour),SESSION_REG(tm_min),SESSION_REG(tm_sec));
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
    }
    else
    {
      pc = 0;
    }
  }
  if(pc == 0)
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "USAGE: !dateDddMmmYyyWww\r\n");
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "where dd mm yy ww are day, month, year and weekday\r\n");
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "weekday: 1:Monday, 7:Sunday\r\n");
  }
  #endif //USE_RTC
}

/** @brief Get session register
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgGetSessionRegVal(char *pcInStr)
{
  char * pc = 0;
  uint8_t a = 0;//address
  uint16_t v = 0;//value
  uint8_t n = 0;//nibble
  uint8_t nWithAction = (0 != strncmp(pcInStr, "!h",2));
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Get session reg value\r\n");
  pc=strstr(pcInStr, "R");
  if(pc)
  {
    if(isxdigit(*(pc+1)) && isxdigit(*(pc+2)))
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "reg addr format OK\r\n");
      //reg address is valid, get value
      n = xdigit(*(pc+1));//MSB
      a |= n;
      a = (a<<4);
      n = xdigit(*(pc+2));
      a |= n;
      if(nWithAction)
      {
        BSP_SESSIONREGS_READ_WACTION(a,(uint8_t*)&v,1);
      }
      else
      {
        BSP_SESSIONREGS_READ(a,(uint8_t*)&v,1);
        DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "(No action performed)\r\n");
      }
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "reg %.2X: value %.4X\r\n",a,v);
    }
    else
    {
      pc=0;
    }
  }
  if(pc == 0)
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "USAGE: ?regsessionRXX\r\n");
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "where XX is the reg address\r\n");
  }
}

/** @brief Set persistent register value
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgSetPersRegVal(char *pcInStr)
{
  char * pc = 0;
  uint8_t a = 0;
  uint16_t v = 0;
  uint8_t n = 0;
  uint8_t nWithAction = (0 != strncmp(pcInStr, "!h",2));
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Set pers reg value\r\n");
  pc=strstr(pcInStr, "R");
  if(pc)
  {
    if(isxdigit(*(pc+1)) && isxdigit(*(pc+2)))
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "reg addr format OK\r\n");
      //reg address is valid, check value
      n = xdigit(*(pc+1));//MSB
      a |= n;
      a = (a<<4);
      n = xdigit(*(pc+2));
      a |= n;
      if(*(pc+3) == 'V')
      {
        if(isxdigit(*(pc+4)) && isxdigit(*(pc+5)) && isxdigit(*(pc+6)) && isxdigit(*(pc+7)))
        {
          DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "reg value format OK\r\n");
          //reg value ok, set register
          n = xdigit(*(pc+4));//MSB
          v |= n;
          v = (v<<4);
          n = xdigit(*(pc+5));
          v |= n;
          v = (v<<4);
          n = xdigit(*(pc+6));
          v |= n;
          v = (v<<4);
          n = xdigit(*(pc+7));//LSB
          v |= n;
          DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "set reg 0x%.2X to value 0x%.4X\r\n",a,v);
          if(nWithAction)
          {
            BSP_PERSREGS_WRITE_WACTION(a,(uint8_t*)&v,1);
          }
          else
          {
            BSP_PERSREGS_WRITE(a,(uint8_t*)&v,1);
            DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "(No action performed)\r\n");
          }
        }
        else
        {
          pc = 0;
        }
      }
      else
      {
        pc = 0;
      }
    }
    else
    {
      pc = 0;
    }
  }
  if(pc == 0)
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "USAGE: !regpersRXXVYYYY\r\n");
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "where XX is the reg address\r\n");
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "and YYYY is the reg value\r\n");
  }
}

/** @brief Get persistent register value
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgGetPersRegVal(char *pcInStr)
{
  char * pc = 0;
  uint8_t a = 0;
  uint16_t v = 0;
  uint8_t n = 0;
  uint8_t nWithAction = (0 != strncmp(pcInStr, "!h",2));
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Get pers reg value\r\n");
  pc=strstr(pcInStr, "R");
  if(pc)
  {
    if(isxdigit(*(pc+1)) && isxdigit(*(pc+2)))
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "reg addr format OK\r\n");
      //reg address is valid, get value
      n = xdigit(*(pc+1));//MSB
      a |= n;
      a = (a<<4);
      n = xdigit(*(pc+2));
      a |= n;
      if(nWithAction)
      {
        BSP_PERSREGS_READ_WACTION(a,(uint8_t*)&v,1);
      }
      else
      {
        BSP_PERSREGS_READ(a,(uint8_t*)&v,1);
        DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "(No action performed)\r\n");
      }
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "reg %.2X: value %.4X\r\n",a,v);
    }
    else
    {
      pc=0;
    }
  }
  if(pc == 0)
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "USAGE: ?regpersRXX\r\n");
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "where XX is the reg address\r\n");
  }
}

/** @brief Get license
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgEraseLicenses(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Erasing Algorithm1 license\r\n");
  uint8_t pLicense[LICENSE_SIZE+1] = { 0 };
  BSP_LICENSE_Write(ALGORITHM1_LICENSE_NUMBER,(uint8_t*)&pLicense);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Erasing Algorithm2 license\r\n");
  BSP_LICENSE_Write(ALGORITHM2_LICENSE_NUMBER,(uint8_t*)&pLicense);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Erasing AlgorithmFX license\r\n");
  BSP_LICENSE_Write(ALGORITHMFX_LICENSE_NUMBER,(uint8_t*)&pLicense);
}

/** @brief Get license
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgGetAlgoStatus(char *pcInStr)
{
#if USE_CUSTOM_ALGORITHM1
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nMOTION AR license\r\n");
    uint8_t pLicense[LICENSE_SIZE+1];
    BSP_LICENSE_Read(ALGORITHM1_LICENSE_NUMBER,(uint8_t*)&pLicense);
    DBG_PRINTF_LICENSE_CMD(WeSU_TERMINAL_DEBUG_SEVERITY, pLicense, ALGORITHM1_LICENSE_NUMBER+1);
  }

  if(SESSION_REG(xMAR_Status) == 2)
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "MOTION AR not initialized\r\n");
  }
  else if(SESSION_REG(xMAR_Status) == 1)
  {
    if(PROCESS_MOTION_AR_ENABLED())
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "MOTION AR running\r\n");
    }
    else
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "MOTION AR NOT running\r\n");
    }
  }
  else
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "MOTION AR init failed\r\n");
  }
#endif //USE_CUSTOM_ALGORITHM1
#if USE_CUSTOM_ALGORITHM2
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nMOTION CP license\r\n");
    uint8_t pLicense[LICENSE_SIZE+1];
    BSP_LICENSE_Read(ALGORITHM2_LICENSE_NUMBER,(uint8_t*)&pLicense);
    DBG_PRINTF_LICENSE_CMD(WeSU_TERMINAL_DEBUG_SEVERITY, pLicense, ALGORITHM2_LICENSE_NUMBER+1);
  }
  
  if(SESSION_REG(xMCP_Status) == 2)
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "MOTION CP not initialized\r\n");
  }
  else if(SESSION_REG(xMCP_Status) == 1)
  {
    if(PROCESS_MOTION_CP_ENABLED())
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "MOTION CP running\r\n");
    }
    else
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "MOTION CP NOT running\r\n");
    }
  }
  else
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "MOTION CP init failed\r\n");
  }
#endif //USE_CUSTOM_ALGORITHM2

  #if USE_CUSTOM_ALGORITHMFX
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\nMOTION FX license\r\n");
    uint8_t pLicense[LICENSE_SIZE+1];
    BSP_LICENSE_Read(ALGORITHMFX_LICENSE_NUMBER,(uint8_t*)&pLicense);
    DBG_PRINTF_LICENSE_CMD(WeSU_TERMINAL_DEBUG_SEVERITY, pLicense, ALGORITHMFX_LICENSE_NUMBER+1);
  }

  if(SESSION_REG(xMFX_Status) == 2)
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "MOTION FX not initialized\r\n");
  }
  else if(SESSION_REG(xMFX_Status) == 1)
  {
    if(PROCESS_MOTION_FX_ENABLED())
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "MOTION FX running\r\n");
    }
    else
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "MOTION FX NOT running\r\n");
    }
  }
  else
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "MOTION FX init failed\r\n");
  }
#endif //USE_CUSTOM_ALGORITHMFX

}

/** @brief Get license
 * @param pcInStr: pointer to input string
 * @param nLicense: license index
 * @retval None
 */
void ProcDbgGetLic(char *pcInStr, uint8_t nLicense)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Get license %d value\r\n", nLicense+1);
  uint8_t pLicense[LICENSE_SIZE+1];
  BSP_LICENSE_Read(nLicense,(uint8_t*)&pLicense);
  DBG_PRINTF_LICENSE(WeSU_TERMINAL_DEBUG_SEVERITY, pLicense);
}

/** @brief Set license
 * @param pcInStr pointer to input string
 * @param nLicense license index
 * @retval None
 */
void ProcDbgSetLic(char *pcInStr, uint8_t nLicense)
{
  char * pc = 0;
  uint8_t p = 0;
  uint8_t wrong_fmt = 0;
  
  uint8_t pLicense[LICENSE_SIZE+1];
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Set license %d value\r\n", nLicense+1);
  
  pc=strstr(pcInStr, "_");
  if(pc)
  {
    if(pc==(pcInStr + 5))
    {
      wrong_fmt = CheckXDigitString(pc+1, 12);
      
      uint32_t nLen = strlen(pc+1);
      if(nLen<(P_SIZE*2))
      {
        wrong_fmt = 2;
      }
      
      char cTmpString[2*TERMINAL_STRING_LENGHT+1];
      
      strcpy(cTmpString, pc+1);
      
      while(p < (LICENSE_SIZE/P_SIZE) && wrong_fmt == 0 && nLen > P_SIZE*2)
      {
        while(wrong_fmt == 0 && nLen > P_SIZE*2)
        {
          wrong_fmt = CheckXDigitString(cTmpString, 12);
          if(wrong_fmt != 0)
          {
            break;
          }
          if(SSCANF_WESU_X(&pLicense[(p*P_SIZE)],cTmpString,6))
          {
            DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "proc %d\r\n", p);
            char cPrintString[13];
            strncpy(cPrintString, cTmpString, 12);
            cPrintString[12] = '\0';
            DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, cPrintString);
            DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, " proc\r\n", cPrintString);
            p++;
            lStringShift(cTmpString, 2*P_SIZE);
            nLen = strlen(cTmpString);
          }
          else
          {
            wrong_fmt = 5;
          }
        }
      }
      if(p==LICENSE_SIZE/P_SIZE && wrong_fmt == 0)
      {
        DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Update license %d value\r\n", nLicense+1);
        BSP_LICENSE_Write(nLicense,(uint8_t*)&pLicense);
        DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "\r\n");
        DBG_PRINTF_LICENSE(WeSU_TERMINAL_DEBUG_SEVERITY, pLicense);
        ProcDbgGetLic("", nLicense);
        DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "License write successful\r\n");
        if(nLicense == ALGORITHM1_LICENSE_NUMBER)
        {
          ENABLE_PERMANENT_MAR();
          RESET_MAR_INIT_STATE();
        }
        if(nLicense == ALGORITHM2_LICENSE_NUMBER)
        {
          ENABLE_PERMANENT_MCP();
          RESET_MCP_INIT_STATE();
        }
        if(nLicense == ALGORITHMFX_LICENSE_NUMBER)
        {
          ENABLE_PERMANENT_MFX();
          RESET_MFX_INIT_STATE();
        }
      }
      else
      {
        
      }
    }
    else
    {
      wrong_fmt = 3;
    }
  }
  else
  {
    wrong_fmt = 4;
  }
  

  if(wrong_fmt)
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "ERROR: %d\r\n", wrong_fmt);
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "USAGE: !licX_YYYYYYYYYYYY\r\n");
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "where X is the license number\r\n");
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "and Y...Y is the license value\r\n");
  }
}

/** @brief Get 1st license
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgGetLic1(char *pcInStr)
{
  ProcDbgGetLic(pcInStr, 0);
}

/** @brief Get 2nd license
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgGetLic2(char *pcInStr)
{
  ProcDbgGetLic(pcInStr, 1);
}

/** @brief Get 3rd license
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgGetLic3(char *pcInStr)
{
  ProcDbgGetLic(pcInStr, 2);
}

/** @brief Set 1st license
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgSetLic1(char *pcInStr)
{
  ProcDbgSetLic(pcInStr, 0);
}

/** @brief Set 2nd license
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgSetLic2(char *pcInStr)
{
  ProcDbgSetLic(pcInStr, 1);
}

/** @brief Set 3rd license
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgSetLic3(char *pcInStr)
{
  ProcDbgSetLic(pcInStr, 2);
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

/** @brief Empty command
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgEmptyCmd(char *pcInStr)
{
}

/** @brief Get MCU factory ID
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgTestIwdg(char *pcInStr)
{
  APP_BSP_LED_OnPrivate(LED);
  while(2)
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("Stall to test watch dog\r\n"));
    HAL_Delay(500);
  }
}

/** @brief Perform 4 Blinks on the led to locate the board
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgLedBlinkme(char *pcInStr)
{
  uint16_t nTmpLedReg = SESSION_REG(nLedControlMask);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("--- BLINK ME COMMAND ---\r\n"));
  SESSION_REG(nLedControlMask) = 0;
  APP_BSP_LedSmoothBlink(10);APP_BSP_LedSmoothBlink(10);
  APP_BSP_LedSmoothBlink(10);APP_BSP_LedSmoothBlink(10);
  BSP_SESSIONREGS_WRITE_WACTION(LED_CONFIG_REG,(uint8_t*)&nTmpLedReg,1);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("--- END BLINK ME COMMAND ---\r\n"));
}

/** @brief Configure led in high brightness mode
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgLedHigh(char *pcInStr)
{
  SetMaxValue(LED_CONFIG_BRIGHT_MAX);
}

/** @brief Configure led in low brightness mode
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgLedLow(char *pcInStr)
{
  SetMaxValue(LED_CONFIG_BRIGHT_LOW);
}

/** @brief Enter MCU permanent stop mode
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgEnterStop(char *pcInStr)
{
  SESSION_REG(nLPFunction) = WESU_SYS_POWER_PERM_STOP;
  uint16_t nVal = SESSION_REG(nLPFunction);
  BSP_PERSREGS_WRITE(PWR_MODE_CONTROL_REG, (uint8_t*)&nVal, 1);
}

/** @brief Enter MCU permanent stop mode with BlueNRG wake-up capability
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgEnterStopWBle(char *pcInStr)
{
  SESSION_REG(nLPFunction) = WESU_SYS_POWER_PERM_BLE_STOP;
  uint16_t nVal = SESSION_REG(nLPFunction);
  BSP_PERSREGS_WRITE(PWR_MODE_CONTROL_REG, (uint8_t*)&nVal, 1);
}

/** @brief Enter power save mode (MCU active only during connection)
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgEnterConnOnly(char *pcInStr)
{
  SESSION_REG(nLPFunction) = WESU_SYS_POWER_CONN_ONLY;
  uint16_t nVal = SESSION_REG(nLPFunction);
  BSP_PERSREGS_WRITE(PWR_MODE_CONTROL_REG, (uint8_t*)&nVal, 1);
}

/** @brief Reboot the system
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgReboot(char *pcInStr)
{
  SESSION_REG(nLPFunction) = WESU_SYS_POWER_REBOOT;
  uint16_t nVal = SESSION_REG(nLPFunction);
  BSP_PERSREGS_WRITE(PWR_MODE_CONTROL_REG, (uint8_t*)&nVal, 1);
}

/** @brief Reboot the system with default settings 
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgRebootDefSettings(char *pcInStr)
{
  SESSION_REG(nLPFunction) = WESU_SYS_POWER_REBOOT_DEFSETTINGS;
  uint16_t nVal = SESSION_REG(nLPFunction);
  BSP_PERSREGS_WRITE(PWR_MODE_CONTROL_REG, (uint8_t*)&nVal, 1);
}


/**
 * @brief Dump sensor internal memory
 * @param  handle instance handle
 * @retval None
 */
void Sensor_Dump( void *handle, uint32_t nStartAddres, uint32_t nEndAddress)
{
  extern uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
  uint8_t nVal = 0;
  for(int nAddr = nStartAddres; nAddr < nEndAddress; nAddr++)
  {
    if( Sensor_IO_Read(handle, nAddr, &nVal, 1) == 0 )
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "R:0x%.2X -> V:0x%.2X\r\n", nAddr, nVal);
    }
    else
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Error reading sensor\r\n");
      SystemReconfigHw();
      break;
    }
  }
}

/**
 * @brief Dump the Acc memory
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgDumpAccGyroHidden(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, " --- Dump the Accelerometer memory --- \r\n");
  Sensor_Dump(ACCELERO_handle, 0x00, 0x6B);
}

/**
 * @brief Dump the Acc memory
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgDumpMagHidden(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, " --- Dump the Magnetometer memory --- \r\n");
  Sensor_Dump(MAGNETO_handle, 0x00, 0x33);
}

/**
 * @brief Dump the Acc memory
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgDumpPressTempHidden(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, " --- Dump the Pressure/Temp memory --- \r\n");
  Sensor_Dump(PRESSURE_handle, 0x00, 0x3A);
}

/** @brief Reboot the Acc + Gyro on the system
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgRebootAccGyroHidden(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, " --- Reboot the Accelerometer --- \r\n");
  BSP_ACC_GYRO_Sensor_Reset(ACCELERO_handle,LSM6DS3_ACC_GYRO_SW_RESET_RESET_DEVICE);
  BSP_ACC_GYRO_Sensor_Reboot(ACCELERO_handle,LSM6DS3_ACC_GYRO_BOOT_REBOOT_MODE);
  BSP_ACCELERO_DeInit(&ACCELERO_handle);
  SystemReconfigHw();
}

/** @brief Reboot just the Magnetometer on the system
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgRebootMagHidden(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, " --- Reboot the Magnetometer --- \r\n");
  BSP_MAGNETO_Sensor_SoftRST(MAGNETO_handle,LIS3MDL_MAG_SOFT_RST_YES);
  BSP_MAGNETO_Sensor_Reboot(MAGNETO_handle,LIS3MDL_MAG_REBOOT_YES);
  BSP_MAGNETO_DeInit(&MAGNETO_handle);
  SystemReconfigHw();
}

/** @brief Reboot Pressure and Temperature sensor on the system
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgRebootPressTempHidden(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, " --- Reboot the Press and Temp --- \r\n");
  BSP_PRESS_TEMP_Sensor_SwRst_Reboot(PRESSURE_handle);
  BSP_PRESSURE_DeInit(&PRESSURE_handle);
  SystemReconfigHw();
}

/** @brief Shutdown the system (same as battery physical disconnection)
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgShutdown(char *pcInStr)
{
  SESSION_REG(nLPFunction) = WESU_SYS_POWER_SHUTDOWN;
  uint16_t nVal = SESSION_REG(nLPFunction);
  BSP_PERSREGS_WRITE(PWR_MODE_CONTROL_REG, (uint8_t*)&nVal, 1);
}

/** @brief Enter normal power consumption mode
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgNormPower(char *pcInStr)
{
  SESSION_REG(nTimerFrequency) = SESSION_DEFAULT_REG_0x21;
  SESSION_REG(nReadSensorMask) = SESSION_DEFAULT_REG_0x49;
  SESSION_REG(nProcessFwMask) = SESSION_DEFAULT_REG_0x4A;
  SESSION_REG(nLedBlinkMode) = LED_MODE_SMART_BLINKING;
}

/** @brief Enter minimal power consumption mode
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgMinPower(char *pcInStr)
{
  SESSION_REG(nTimerFrequency) = 1;
  SESSION_REG(nReadSensorMask) = 0;
  SESSION_REG(nProcessFwMask) = 0;
  SESSION_REG(nLedBlinkMode) = LED_MODE_OFF;
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("nTimerFrequency = 1.000s\r\n"));
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("nReadSensorMask = Accelerometer/FuelGauge\r\n"));
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("nLedBlinkMode = OFF\r\n"));
}

/** @brief Get WeSU firmware version
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgGetFwVersion(char *pcInStr)
{
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Current Fw version: %d.%d.%.2d\r\n", (0xF0&HIGH_BYTE(FIRMWARE_VERSION))>>4, 0x0F&HIGH_BYTE(FIRMWARE_VERSION), LOW_BYTE(FIRMWARE_VERSION));
  }
}

/** @brief Get BlueNRG info: hw and fw versions
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgGetBluenrgInfo(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---    BlueNRG information    ---\r\n"));
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY,  "HwVer 0x%.2X, FwVer 0x%.4X\r\n", SESSION_REG(xBNRG_HW_ver), SESSION_REG(xBNRG_FW_ver));
}

/** @brief Set BlueNRG connection interval
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgSetUsbBleBridge(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---    Restart in USB-BlueNRG bridged mode   ---\r\n"));
  HAL_Delay(1000);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("--- 3\r\n"));
  HAL_Delay(1000);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("--- 2\r\n"));
  HAL_Delay(1000);
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("--- 1\r\n"));
  HAL_Delay(1000);
  // Check if BLUENRG_BRIDGE_FW_ADDRESS is a RAM address, i.e. BLUENRG_BRIDGE_FW is present in the correct address
  if (((*(__IO uint32_t*)BLUENRG_BRIDGE_FW_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
  {
    //OK, prepare address for restart and reset
    Switch_To_OTA_Service_Manager_Application(BLUENRG_BRIDGE_FW_ADDRESS);
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("--- Restart!\r\n"));
    HAL_Delay(WESU_DELAY_BEFORE_RESET);
    WESU_SYSTEM_RESET();
    return;
  }
  
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("Address not valid\r\n"));
  HAL_Delay(1000);
}

/** @brief Set BlueNRG connection interval
 * @param pcInStr pointer to input string
 * @retval None
 */
void ProcDbgSetBleConIntvMode(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---    SET BLE CONN INTERVAL MODE    ---\r\n"));
  /* https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.peripheral_preferred_connection_parameters.xml */
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, (" Note: The parameter will be multiplied by 1.25\r\n"));
  
  char * pc = 0;
  uint8_t nChars = 0xFF;
  int bleConIntv;
  
  pc=strstr(pcInStr, "=");
  if(pc)
  {
    if(isEOL(*(pc+2)))
    {
      nChars = 1;
    }
    else if(isEOL(*(pc+3)))
    {
      nChars = 2;
    }
    else if(isEOL(*(pc+4)))
    {
      nChars = 3;
    }
    else if(isEOL(*(pc+5)))
    {
      nChars = 4;
    }
    else
    {
      nChars = 0xFF;
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("Parameter too long (max 4 digit)\r\n"));
    }
    if(nChars != 0xFF)
    {
      char tmpstr[5] = { '\0', '\0', '\0', '\0', '\0'};
      tmpstr[0] = *(pc+1);
      if(nChars>1)
      {
        if(isdigit(*(pc+2)))
        {
          tmpstr[1] = *(pc+2);
        }
        else
        {
          nChars = 0xFF;
        }
      }
      if(nChars>2)
      {
        if(isdigit(*(pc+3)))
        {
          tmpstr[2] = *(pc+3);
        }
        else
        {
          nChars = 0xFF;
        }
      }
      if(nChars>3)
      {
        if(isdigit(*(pc+4)))
        {
          tmpstr[3] = *(pc+4);
        }
        else
        {
          nChars = 0xFF;
        }
      }
      if(nChars != 0xFF)
      {
        uint8_t nRet = sscanf(tmpstr, "%d", &bleConIntv);
        if(nRet)
        {
          
        }
        else
        {
          nChars = 0xFF;
          DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("sscanf error\r\n"));
        }
      }
      if(nChars != 0xFF)
      {
        DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "- bleConIntv: %d\r\n", bleConIntv);
        bConnParamUpdate = 1;
        SESSION_REG(nBleConIntv) = bleConIntv;
        DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("--- SET BLE CONN INTERVAL TO: %d ---\r\n"), bleConIntv);
      }
      else
      {
        DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("Format error\r\n"));
      }
    }
    else
    {
      DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("ERROR\r\n"));
    }
        
  }
  if(pc == 0)
  {
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "USAGE: !bleConIntv=NNNN\r\n");
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "where NNNN is the connection interval in ms, divided by 1.25\r\n");
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, "Allowed values: 6 to 3200 (7.5ms to 4000ms\r\n");
  }
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---  END SET BLE CONN INTERVAL MODE  ---\r\n"));
}

#if USE_CUSTOM_ACCEVENT

/**
  * @brief  Pedometer ON in Debug Console
  * @param  *pcInStr 
  * @retval None
  */
void ProcDbgViewPedometerOn(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---  START PEDOMETER ALGORITHM  ---\r\n"));
  uint16_t r = 0;
  BSP_PERSREGS_READ(FW_OPERATING_CAPABILITIES_REG, (uint8_t*)&r, 1);
  r |= ALGORITHM3_BLUEST_MASK;
  BSP_PERSREGS_WRITE(FW_OPERATING_CAPABILITIES_REG, (uint8_t*)&r, 1);
  r=STORE_U8_LE(PEDOMETER_CONTROL_MASK_CHANNEL_DEFAULT_VALUE,PEDOMETER_CONTROL_MASK_SUBSAMPL_DEFAULT_VALUE);
  BSP_PERSREGS_WRITE(PEDOMETER_CONTROL_MASK_REG, (uint8_t*)&r, 1);
  SESSION_REG(nPedometerCtrlMask)         = ALGORITHM_CHANNEL_MASK(SESSION_DEFAULT_REG_0x35);
  SESSION_REG(nPedometerSubSampl)         = ALGORITHM_SUBSAMPL_MASK(SESSION_DEFAULT_REG_0x35);
  SystemReconfigHw();
}

/**
  * @brief  Pedometer OFF in Debug Console
  * @param  *pcInStr 
  * @retval None
  */
void ProcDbgViewPedometerOff(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---  STOP PEDOMETER ALGORITHM  ---\r\n"));
  uint16_t r = 0;
  BSP_PERSREGS_READ(FW_OPERATING_CAPABILITIES_REG, (uint8_t*)&r, 1);
  r &= ~(uint16_t)ALGORITHM3_BLUEST_MASK;
  BSP_PERSREGS_WRITE(FW_OPERATING_CAPABILITIES_REG, (uint8_t*)&r, 1);
  r=0;
  BSP_PERSREGS_WRITE(PEDOMETER_CONTROL_MASK_REG, (uint8_t*)&r, 1);
  SESSION_REG(nPedometerCtrlMask)         = ALGORITHM_CHANNEL_MASK(SESSION_DEFAULT_REG_0x35);
  SESSION_REG(nPedometerSubSampl)         = ALGORITHM_SUBSAMPL_MASK(SESSION_DEFAULT_REG_0x35);
  SystemReconfigHw();
}

/**
  * @brief  Pedometer Reset
  * @param  *pcInStr 
  * @retval None
  */
void ProcDbgViewPedometerRst(char *pcInStr)
{
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("---  RESET PEDOMETER ALGORITHM  ---\r\n"));
  uint16_t r = 0;
  BSP_PERSREGS_READ(FW_OPERATING_CAPABILITIES_REG, (uint8_t*)&r, 1);
  r |= ALGORITHM3_BLUEST_MASK;
  BSP_PERSREGS_WRITE(FW_OPERATING_CAPABILITIES_REG, (uint8_t*)&r, 1);
  r=STORE_U8_LE(PEDOMETER_CONTROL_MASK_CHANNEL_DEFAULT_VALUE,PEDOMETER_CONTROL_MASK_SUBSAMPL_DEFAULT_VALUE);
  BSP_PERSREGS_WRITE(PEDOMETER_CONTROL_MASK_REG, (uint8_t*)&r, 1);
  SESSION_REG(nPedometerCtrlMask)         = ALGORITHM_CHANNEL_MASK(SESSION_DEFAULT_REG_0x35);
  SESSION_REG(nPedometerSubSampl)         = ALGORITHM_SUBSAMPL_MASK(SESSION_DEFAULT_REG_0x35);
  //Reset pedometer counter
  BSP_ACCELERO_Enable_Step_Counter_Reset_Ext( ACCELERO_handle );
  BSP_ACCELERO_Disable_Step_Counter_Reset_Ext( ACCELERO_handle );
  SESSION_REG(nPedoStepCounter) = 0;
  SystemReconfigHw();
}

/**
  * @brief  Pedometer Visible in Debug Console
  * @param  *pcInStr 
  * @retval None
  */
void ProcDbgViewPedometer(char *pcInStr)
{
  extern void SavePedometerData();
  SavePedometerData();
  DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, ("\r\n---    GET PEDOMETER DATA    ---\r\n"));
  uint8_t i;
  for(i = 0; i<7; i++)
  {
    uint8_t d = (SESSION_REG(tm_wday) + 1 + i) % 7;
    if(d==0)d=7;
    uint8_t r = PEDO_SAVE_DATA_0_REG + (2 * (d - 1));
    uint32_t *pDaySteps = (uint32_t*)(PERMREG_STRUCT_GET_ADDR(r));
    uint32_t nDaySteps = *pDaySteps;
    #if USE_RTC
    DBG_PRINTF_TERMINAL(WeSU_TERMINAL_DEBUG_SEVERITY, (" %s%s: %d steps\r\n"), BSP_RTC_GetWeekDayName(d),(d==SESSION_REG(tm_wday))?"(Today)":"", nDaySteps);
    #endif //USE_RTC
  }
}
#endif //USE_CUSTOM_ACCEVENT

/**
  * @}
  */

/** @addtogroup Console_Exported_Functions        Console Exported Functions
  * @{
  */

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

/** @brief Receives a character from serial port
 * @retval Character received
 */
int uartReceiveChar(void)
{
  uint8_t ch;
  HAL_UART_Receive(&UartHandle, &ch, 1, HAL_MAX_DELAY);
  
#if UART_ECHO
  /* Echo character back to console */
  HAL_UART_Transmit(&UartHandle, &ch, 1, HAL_MAX_DELAY);
#endif //UART_ECHO

  /* And cope with Windows */
  if(ch == '\r'){
    uint8_t ret = '\n';
    HAL_UART_Transmit(&UartHandle, &ret, 1, HAL_MAX_DELAY);
  }

  return ch;
}


/** @brief Process terminal command
 * @param pCommand pointer to command buffer
 * @param nFrameLenght Frame lenght
 * @retval None
 */
void ProcessTermCommand(char *pCommand, int nFrameLenght)
{
  uint8_t TempEnabUsart = ENABLED_DEBUG_LEVEL_USART;
  if(cChannel == 'U')
  {
    ENABLED_DEBUG_LEVEL_USART = WeSU_DEBUG_SEVERITY_VERBOSE;
  }
  
#if !UART_MINIMAL_ECHO
  PRINTF("ECHO - ");
  PRINTF(pCommand);
  PRINTF("\r\n");
#endif
  
  /* search cmd in cmds list */
  pCmdHandlerFunc_t pHandle = NULL;
  
  int i;
  for(i=0;i<countof(xCmdStructVect);i++)
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
#if USE_IWDG
    BSP_WD_Refresh();
#endif //USE_IWDG
    
    SUSPEND_SENSORS_READ_ON_INTERRUPT();
    pHandle(pCommand+COMMAND_OFFSET);
    RESUME_SENSORS_READ_ON_INTERRUPT();
    
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
  
  ENABLED_DEBUG_LEVEL_USART = TempEnabUsart;
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
  att_data[data_length] = '\0';
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

/**
  * @}
  */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
