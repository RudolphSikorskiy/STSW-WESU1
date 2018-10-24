/**
  ******************************************************************************
  * @file    Sensors_Read/Inc/wesu_config_examples.h
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Header for wesu_config.c module
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
#ifndef __WESU_CONFIG_H
#define __WESU_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "platform_config.h"

   
/** @addtogroup WeSU_Examples        WeSU Examples 
  * @{
  */

/** @addtogroup WeSU_User_Examples               WeSU User Examples
  * @{
  */

/** @defgroup WeSU_Config_Examples             WeSU Config Examples
  * @{
  */


#define ACCELEROMETER_EXAMPLE           1                                       //!< Enable ACCELEROMETER example
#define GYROSCOPE_EXAMPLE               0                                       //!< Enable GYROSCOPE example
#define MAGNETOMETER_EXAMPLE            0                                       //!< Enable MAGNETOMETER example
#define GAS_GAUGE                       0                                       //!< Enable GAS GAUGE example
#define PRESS_EXAMPLE                   0                                       //!< Enable PRESSURE example
#define TEMPERATURE_EXAMPLE             0                                       //!< Enable TEMPERATURE example

#define nTimerFrequency                 50                                      //!< Set timer frequency for the examples
#define nLedControlMask                 LED_CONFIG_APP_CONTROLLED_MASK          //!< Set Led control for the examples
   
#define APP_BSP_LED_On(L)               if(!LED_CONFIG_USER_ENABLED())BSP_LED_OnGpio(L)         //!< Macro to manage LED_ON command according to nLedControlMask value
#define APP_BSP_LED_Off(L)              if(!LED_CONFIG_USER_ENABLED())BSP_LED_OffGpio(L)        //!< Macro to manage LED_OFF command according to nLedControlMask value
#define APP_BSP_LED_Toggle(L)           if(!LED_CONFIG_USER_ENABLED())BSP_LED_ToggleGpio(L)     //!< Macro to manage LED_TOGGLE command according to nLedControlMask value
#define APP_BSP_LED_GetState(L)         BSP_LED_GetState(L)                                     //!< Macro to get LED state

#define APP_BSP_LED_OnPrivate(L)        BSP_LED_OnGpio(L)                                       //!< Macro to manage LED_ON command 
#define APP_BSP_LED_OffPrivate(L)       BSP_LED_OffGpio(L)                                      //!< Macro to manage LED_OFF command

#define APP_LedSmoothRampDown(V)        LedSmoothRampDownGpio(V)                                //!< Macro to manage smooth LED_OFF command
#define APP_LedSmoothRampUp(V)          LedSmoothRampUpGpio(V)                                  //!< Macro to manage smooth LED_ON command
#define APP_BSP_LedSmoothBlink(V)       LedSmoothBlinkGpio(V)                                   //!< Macro to manage smooth LED_BLINK command

#define WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES    SESSION_REG(WeSU_APP_DEFAULT_SEVERITY)      //!< WeSU application messages default severity

#define STORE_U8_LE(B1, B2)     ((B1) | ((B2)<<8))                                    //!< Little Endian Format
    
/* Define the WESU Name MUST be 7 char long */   
#define WESU_GET_NAME(blename,maxlen)   {\
                                            blename[0] = AD_TYPE_COMPLETE_LOCAL_NAME;\
                                            strncpy((char*)&blename[1], WESU_DEFAULT_NAME, maxlen-1);\
                                        }               //!< GET the WeSU BOARD NAME   

#define MAX_BLUENRG_NAME_LEN_ADV              7                                 //!< WeSU advertising name max lenght
  
/* Define the WESU MAC address */
#define         DEVICE_ID1          (0x1FF80050)                                //!< MCU ID address byte 1
#define         DEVICE_ID2          (0x1FF80054)                                //!< MCU ID address byte 2
#define         DEVICE_ID3          (0x1FF80064)                                //!< MCU ID address byte 3
#define GET_DEVICE_SERIAL_NUMBER(B, N)          {\
                                                  uint32_t deviceserial0, deviceserial1, deviceserial2;\
                                                  deviceserial0 = *(uint32_t*)DEVICE_ID1;\
                                                  deviceserial1 = *(uint32_t*)DEVICE_ID2;\
                                                  deviceserial2 = *(uint32_t*)DEVICE_ID3;\
                                                  deviceserial0 += deviceserial2;\
                                                  B[0] = (0xFF&deviceserial0);\
                                                  B[1] = (0xFF00&deviceserial0)>>8;\
                                                  B[1] ^= (0xFF0000&deviceserial0)>>16;\
                                                  B[2] = (0xFF000000&deviceserial0)>>24;\
                                                  B[2] ^= 0xFF&deviceserial1;\
                                                  B[1] ^= (0xFF00&deviceserial1)>>8;\
                                                  B[2] ^= (0xFF0000&deviceserial1)>>16;\
                                                  B[2] ^= (0xFF000000&deviceserial1)>>24;\
                                                  if(N>3)B[3] = 0xE1;\
                                                  if(N>4)B[4] = 0x80;\
                                                  if(N>5)B[5] = 0x26;\
                                                }                               //!< Calculate BlueNRG public address based on MCU ID

#define WESU_GET_MAC_ADDRESS(bleaddr, maxlen)           GET_DEVICE_SERIAL_NUMBER(bleaddr,maxlen) //!< Get WeSU MAC address: GET_DEVICE_SERIAL_NUMBER() 

    #define USE_CUSTOM_ALGORITHM1       0                                       //!< define to '1' to enable Activity Recognition algorithm usage
#define COPY_ALGORITHM1_BLUEST_CHAR_UUID(uuid_struct)             COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x10,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)    //!< copy AR BlueNRG characteristic UUID
#define ALGORITHM1_LICENSE_NUMBER                               1               //!< AR License Number
#define ALGORITHM1_FREQUENCY                                    16              //!< AR running frequency
#define ALGORITHM1_CHAR_SIZE                                    2+1             //!< AR BlueNRG characteristic size
#define ALGORITHM1_BLUEST_MASK                                  0x10            //!< AR BLUEST MASK identification


extern DrvStatusTypeDef sensorStatusAccelero;                                   
extern DrvStatusTypeDef sensorStatusGyro;                                       
extern DrvStatusTypeDef sensorStatusMag;                                        
extern DrvStatusTypeDef sensorStatusPress;                                      
extern DrvStatusTypeDef sensorStatusTemp;                                         
  


void Switch_To_OTA_Service_Manager_Application(uint32_t imageBase);
void Switch_To_USB_DFU(uint32_t nDfuFlag);

  
void LedManageStart(uint8_t nLedMode);
void LedManageEnd(uint8_t nLedMode); 

void setConnectable(uint16_t nMin, uint16_t nMax);
void BluenrgInitialization();
void Init_BlueNRG_Hw(void);
void Init_BlueNRG_Stack(void);
void Init_BlueNRG_Custom_Services(void);
void Init_PWR_MGMT(void);

void SensorsConfig();

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

#endif /* __WESU_CONFIG_H */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
