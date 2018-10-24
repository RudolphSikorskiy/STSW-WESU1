/**
  ******************************************************************************
  * @file    WESU_DEMO/Inc/wesu_config.h
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

   
/** @addtogroup WeSU_Demo       WeSU Demo
  * @{
  */

/** @addtogroup WeSU_User               WeSU User
  * @{
  */

/** @defgroup WeSU_Config             WeSU Config                
  * @{
  */


/** @defgroup WeSU_Config_Exported_Defines     WeSU Config Exported Defines                
  * @{
  */ 

#define APP_BSP_LED_On(L)               if(!LED_CONFIG_USER_ENABLED())BSP_LED_OnGpio(L)         //!< Macro to manage LED_ON command according to nLedControlMask value
#define APP_BSP_LED_Off(L)              if(!LED_CONFIG_USER_ENABLED())BSP_LED_OffGpio(L)        //!< Macro to manage LED_OFF command according to nLedControlMask value
#define APP_BSP_LED_Toggle(L)           if(!LED_CONFIG_USER_ENABLED())BSP_LED_ToggleGpio(L)     //!< Macro to manage LED_TOGGLE command according to nLedControlMask value
#define APP_BSP_LED_GetState(L)         BSP_LED_GetState(L)                                     //!< Macro to get LED state

#define APP_BSP_LED_OnPrivate(L)        BSP_LED_OnGpio(L)                                       //!< Macro to manage LED_ON command 
#define APP_BSP_LED_OffPrivate(L)       BSP_LED_OffGpio(L)                                      //!< Macro to manage LED_OFF command

#define APP_LedSmoothRampDown(V)        LedSmoothRampDownGpio(V)                                //!< Macro to manage smooth LED_OFF command
#define APP_LedSmoothRampUp(V)          LedSmoothRampUpGpio(V)                                  //!< Macro to manage smooth LED_ON command
#define APP_BSP_LedSmoothBlink(V)       LedSmoothBlinkGpio(V)                                   //!< Macro to manage smooth LED_BLINK command

#define SENSORS_TIMER_DRDY_CTRL_ENABLED                                                         0x0001
#define SENSORS_TIMER_DRDY_CTRL_RUNNING                                                         0x0080
#define SENSORS_TIMER_DRDY_CTRL_ENABLED_AND_RUNNING                                             (SENSORS_TIMER_DRDY_CTRL_ENABLED | SENSORS_TIMER_DRDY_CTRL_RUNNING)

#define ENABLE_SENSORS_READ_ON_INTERRUPT()                                                      do{/*SensorsPrintInitStatus();*/SensorAxes_t x;BSP_GYRO_Get_Axes(GYRO_handle, &x);SESSION_REG(nTimerDrdyCtrlMask) |= SENSORS_TIMER_DRDY_CTRL_ENABLED;}while(0)
#define DISABLE_SENSORS_READ_ON_INTERRUPT()                                                     do{SESSION_REG(nTimerDrdyCtrlMask) &= ~(SENSORS_TIMER_DRDY_CTRL_ENABLED);}while(0)

#define RESUME_SENSORS_READ_ON_INTERRUPT()                                                      do{/*SensorsPrintInitStatus();*/SensorAxes_t x;BSP_GYRO_Get_Axes(GYRO_handle, &x);SESSION_REG(nTimerDrdyCtrlMask) |= SENSORS_TIMER_DRDY_CTRL_RUNNING;}while(0)
#define SUSPEND_SENSORS_READ_ON_INTERRUPT()                                                     do{SESSION_REG(nTimerDrdyCtrlMask) &= ~(SENSORS_TIMER_DRDY_CTRL_RUNNING);}while(0)

#define WESU_SYS_MODE_GO_IN_RUN         0x01                                                    //!< WesuSysChangeRunMode param going in RUNNING mode: check before user application
#define WESU_SYS_MODE_GO_IN_STOP        0x02                                                    //!< WesuSysChangeRunMode param going in STOPPING mode: check after user application

#define WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES    SESSION_REG(WeSU_APP_DEFAULT_SEVERITY)      //!< WeSU application messages default severity

  #define LED_INTERVAL_DISCONNECTED     *LED_INTERVAL_DISCONNECTED_REGADDR                      //!< led blinking interval when disconnected
  #define LED_INTERVAL_DISCONNECTED_ON  *LED_INTERVAL_DISCONNECTED_ON_REGADDR                   //!< on-phase when disconnected
  #define LED_INTERVAL_CONNECTED        *LED_INTERVAL_CONNECTED_REGADDR                         //!< led blinking interval when connected
  #define LED_INTERVAL_CONNECTED_ON     *LED_INTERVAL_CONNECTED_ON_REGADDR                      //!< on-phase when connected

#define STORE_U8_LE(B1, B2)     ((B1) | ((B2)<<8))                                    //!< Little Endian Format
    
/* Define the WESU Name MUST be 7 char long */   
#define WESU_GET_NAME(blename,maxlen)   {\
                                          strncpy((char*)&blename[0], (char*)BLE_LOC_NAME_REGADDR, maxlen-1);\
                                          if(blename[0] != AD_TYPE_COMPLETE_LOCAL_NAME)\
                                          {\
                                            blename[0] = AD_TYPE_COMPLETE_LOCAL_NAME;\
                                            strncpy((char*)&blename[1], WESU_DEFAULT_NAME, maxlen-1);\
                                          }\
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

#define WESU_GET_MAC_ADDRESS(bleaddr, maxlen)           if(memcmp(BLE_PUB_ADDR_REGADDR,WesuAddrDefault,6)==0){GET_DEVICE_SERIAL_NUMBER(bleaddr,maxlen);}else{memcpy(bleaddr,BLE_PUB_ADDR_REGADDR,maxlen);}      //!< Get WeSU MAC address: check BLE_PUB_ADDR_REGADDR for a custom address, GET_DEVICE_SERIAL_NUMBER() otherwise

#ifndef BLUENRG_MS
//  #define BLUENRG_MS      1  //this define must be _only_ in compiler preprocessor
#endif //BLUENRG_MS

#define ENABLE_SPI_FIX                          //!< define to enable SPI FIX on BlueNRG
#define USE_UPDATE_CONNECTION_PARAMETER 1       //!< define to enable conn params update
#define USE_QUATERNION_FLOAT            0       //!< define to '1' to enable Quaternion float usage

    #define USE_CUSTOM_ALGORITHM1       1                       //!< define to '1' to enable Activity Recognition algorithm usage
#define COPY_ALGORITHM1_BLUEST_CHAR_UUID(uuid_struct)             COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x10,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)    //!< copy AR BlueNRG characteristic UUID
#define ALGORITHM1_LICENSE_NUMBER                               1               //!< AR License Number
#define ALGORITHM1_FREQUENCY                                    16              //!< AR running frequency
#define ALGORITHM1_CHAR_SIZE                                    2+1             //!< AR BlueNRG characteristic size
#define ALGORITHM1_BLUEST_MASK                                  0x10            //!< AR BLUEST MASK identification

#define SET_MAR_INIT_STATE()                                    SESSION_REG(nMarInitState)++        //!< AR initialization state
#define RESET_MAR_INIT_STATE()                                  SESSION_REG(nMarInitState)=0        //!< AR initialization state
#define GET_MAR_INIT_STATE()                                    SESSION_REG(nMarInitState)          //!< AR initialization state
#define DISABLE_PERMANENT_MAR()                                 {uint16_t nFwMsk=0;BSP_PERSREGS_READ(FW_OPERATING_CAPABILITIES_REG,(uint8_t*)&nFwMsk,1);CLEAR_FLAG_16(nFwMsk,ALGORITHM1_BLUEST_MASK );BSP_PERSREGS_WRITE_WACTION(FW_OPERATING_CAPABILITIES_REG,(uint8_t*)&nFwMsk,1);}   //!< Disable AR
#define ENABLE_PERMANENT_MAR()                                  {uint16_t nFwMsk=0;BSP_PERSREGS_READ(FW_OPERATING_CAPABILITIES_REG,(uint8_t*)&nFwMsk,1);SET_FLAG_16(nFwMsk,ALGORITHM1_BLUEST_MASK );BSP_PERSREGS_WRITE_WACTION(FW_OPERATING_CAPABILITIES_REG,(uint8_t*)&nFwMsk,1);}     //!< Enable AR

    #define USE_CUSTOM_ALGORITHM2       1                                       //!< define to '1' to enable Carry Position algorithm usage
#define COPY_ALGORITHM2_BLUEST_CHAR_UUID(uuid_struct)             COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x08,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)    //!< copy CP BlueNRG characteristic UUID
#define ALGORITHM2_LICENSE_NUMBER                               2               //!< CP License Number
#define ALGORITHM2_FREQUENCY                                    50              //!< CP running frequency
#define ALGORITHM2_CHAR_SIZE                                    2+1             //!< CP BlueNRG characteristic size
#define ALGORITHM2_BLUEST_MASK                                  0x08            //!< CP BLUEST MASK identification

#define SET_MCP_INIT_STATE()                                    SESSION_REG(nMcpInitState)++        //!< CP initialization state
#define RESET_MCP_INIT_STATE()                                  SESSION_REG(nMcpInitState)=0        //!< CP initialization state
#define GET_MCP_INIT_STATE()                                    SESSION_REG(nMcpInitState)          //!< CP initialization state
#define DISABLE_PERMANENT_MCP()                                 {uint16_t nFwMsk=0;BSP_PERSREGS_READ(FW_OPERATING_CAPABILITIES_REG,(uint8_t*)&nFwMsk,1);CLEAR_FLAG_16(nFwMsk,ALGORITHM2_BLUEST_MASK );BSP_PERSREGS_WRITE_WACTION(FW_OPERATING_CAPABILITIES_REG,(uint8_t*)&nFwMsk,1);}   //!< Disable AR
#define ENABLE_PERMANENT_MCP()                                  {uint16_t nFwMsk=0;BSP_PERSREGS_READ(FW_OPERATING_CAPABILITIES_REG,(uint8_t*)&nFwMsk,1);SET_FLAG_16(nFwMsk,ALGORITHM2_BLUEST_MASK );BSP_PERSREGS_WRITE_WACTION(FW_OPERATING_CAPABILITIES_REG,(uint8_t*)&nFwMsk,1);}     //!< Enable AR

      #define USE_CUSTOM_ALGORITHMFX     1                      //!< define to '1' to enable Motion FX algorithm usage                              
//#define COPY_QUATERNIONS_BLUEST_CHAR_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST FX characteristic
#define ALGORITHMFX_LICENSE_NUMBER                              0                                   //!< FX License Number
#define MOTION_FX_BLUEST_MASK                                   0x0100                  //!< Motion FX BLUEST MASK identification
#define SET_MFX_INIT_STATE()                                    SESSION_REG(nMfxInitState)++        //!< FX initialization state
#define RESET_MFX_INIT_STATE()                                  SESSION_REG(nMfxInitState)=0        //!< FX initialization state
#define GET_MFX_INIT_STATE()                                    SESSION_REG(nMfxInitState)          //!< FX initialization state
#define DISABLE_PERMANENT_MFX()                                 {uint16_t nFwMsk=0;BSP_PERSREGS_READ(FW_OPERATING_CAPABILITIES_REG,(uint8_t*)&nFwMsk,1);CLEAR_FLAG_16(nFwMsk,MOTION_FX_BLUEST_MASK );BSP_PERSREGS_WRITE_WACTION(FW_OPERATING_CAPABILITIES_REG,(uint8_t*)&nFwMsk,1);}   //!< Disable FX
#define ENABLE_PERMANENT_MFX()                                  {uint16_t nFwMsk=0;BSP_PERSREGS_READ(FW_OPERATING_CAPABILITIES_REG,(uint8_t*)&nFwMsk,1);SET_FLAG_16(nFwMsk,MOTION_FX_BLUEST_MASK );BSP_PERSREGS_WRITE_WACTION(FW_OPERATING_CAPABILITIES_REG,(uint8_t*)&nFwMsk,1);}     //!< Enable FX

//    #define USE_CUSTOM_ALGORITHM1                       0                       //!< define to '1' to enable algorithm1 usage
//#define COPY_ALGORITHM1_BLUEST_CHAR_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x00,,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00)   //!< copy algorithm1 BlueNRG characteristic UUID
//#define ALGORITHM1_CHAR_SIZE                            2+1                     //!< algorithm1 BlueNRG characteristic size
//#define ALGORITHM1_BLUEST_MASK                          0x0000                  //!< algorithm1 BLUEST MASK identification

//    #define USE_CUSTOM_ALGORITHM2                       0                       //!< define to '1' to enable algorithm2 usage
//#define COPY_ALGORITHM2_BLUEST_CHAR_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x00,,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00)   //!< copy algorithm2 BlueNRG characteristic UUID
//#define ALGORITHM2_CHAR_SIZE                            2+1                     //!< algorithm2 BlueNRG characteristic size
//#define ALGORITHM2_BLUEST_MASK                          0x0000                  //!< algorithm2 BLUEST MASK identification

    #define USE_CUSTOM_ALGORITHM3                       0                       //!< define to '1' to enable pedometer algorithm usage
#define COPY_ALGORITHM3_BLUEST_CHAR_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)    //!< copy pedometer BlueNRG characteristic UUID
#define ALGORITHM3_CHAR_SIZE                            2+4+2                   //!< pedometer BlueNRG characteristic size
#define ALGORITHM3_BLUEST_MASK                          0x0001                  //!< pedometer BLUEST MASK identification
#define ALGORITHM3_FREQUENCY                            1                       //!< pedometer running frequency

    #define USE_CUSTOM_ALGORITHM4                       0                       //!< define to '1' to enable freefall algorithm usage
#define COPY_ALGORITHM4_BLUEST_CHAR_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x02,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)    //!< copy pedometer BlueNRG characteristic UUID
#define ALGORITHM4_CHAR_SIZE                            2+2                     //!< freefall BlueNRG characteristic size
#define ALGORITHM4_BLUEST_MASK                          0x0200                  //!< freefall BLUEST MASK identification
#define ALGORITHM4_FREQUENCY                            5                       //!< freefall running frequency

    #define USE_CUSTOM_ACCEVENT                         1                       //!< define to '1' to enable accelerometer HW algorithm usage
#define COPY_ACCEVENT_BLUEST_CHAR_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x00,0x00,0x04,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST LSM6DS3 events characteristic
#define ACCEVENT_CHAR_SIZE                              2+3                     //!< ACCEVENT BlueNRG characteristic size 1 Byte Accevent + 2 Bytes HW Pedometer 
#define ACCEVENT_BLUEST_MASK                            0x400                   //!< Accevent BLUEST MASK identification
#define ACCEVENT_FREQUENCY                              1                       //!< Accevent running frequency

#define ACCEVENT_FEATURE_ENABLED(F)                     ((F & SESSION_REG(nAccEventConf)) != 0) //!< Accevent Features Enabling
#define ACCEVENT_FEATURE_6DORIENTATION                 (ACC_6D_OR_TOP | \
                                                        ACC_6D_OR_LEFT | \
                                                        ACC_6D_OR_BOTTOM | \
                                                        ACC_6D_OR_RIGTH | \
                                                        ACC_6D_OR_UP | \
                                                        ACC_6D_OR_DOWN)         //!< Accevent 6D Orientations Feature

#define ACCEVENT_FEATURE_FREE_FALL                      ACC_FREE_FALL           //!< Accevent Free Fall Feature
#define ACCEVENT_FEATURE_DOUBLE_TAP                     ACC_DOUBLE_TAP          //!< Accevent Double Tap Feature
#define ACCEVENT_FEATURE_SINGLE_TAP                     ACC_SINGLE_TAP          //!< Accevent Single Tap Feature
#define ACCEVENT_FEATURE_WAKE_UP                        ACC_WAKE_UP             //!< Accevent Wake Up Feature                   
#define ACCEVENT_FEATURE_TILT                           ACC_TILT                //!< Accevent Tilt Feature

#define ACCEVENT_FEATURE_PEDOMETER                      0x100                   //!< Accevent Pedometer Feature
  
    #define USE_CUSTOM_GP_ALGORITHM                       0                       //!< define to '1' to enable GP_ALGORITHM usage
#define COPY_GP_ALGORITHM_BLUEST_SERVICE_UUID(uuid_struct)           COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x03,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST Sensors service
#define COPY_GP_ALGORITHM_BLUEST_CHAR_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x00,0x03,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)    //!< copy GP_ALGORITHM BlueNRG characteristic UUID
#define GP_ALGORITHM_CHAR_SIZE                            2+1                     //!< GP_ALGORITHM BlueNRG characteristic size
#define GP_ALGORITHM_BLUEST_MASK                          0x8000                  //!< GP_ALGORITHM BLUEST MASK identification
#define GP_ALGORITHM_FREQUENCY                            2                       //!< GP_ALGORITHM running frequency
#define GP_ALGORITHM_THS1_DEFAULT                               650
#define GP_ALGORITHM_THS2_DEFAULT                               800

#define USE_SLEEP_INFO                                  1                       //!< define to '1' to use sleep infos
#define RESET_AFTER_BUTTON_WAKEUP                       0                       //!< define to '1' to reset after button wake up

#define PERMREG_NEW_OPERATION_COMPLETE_WITH_ACK         2                       //!< WeSU register operation completed, send acknowledgement
#define PERMREG_NEW_OPERATION_COMPLETE                  1                       //!< WeSU register operation completed, no acknowledgement needed
#define PERMREG_NO_OPERATION                            0                       //!< WeSU register no ongoing operation

/* CONTROL BYTE */
#define PERMREG_PENDING_OPERATION               0x80                            //!< pending operation in reading/writing registers
#define PERMREG_EEPROM_OPERATION                0x40                            //!< operation will be done in EEPROM registers (persistent)
#define PERMREG_RAM_OPERATION                   0x00                            //!< operation will be done in RAM registers (session)
#define PERMREG_WRITE_OPERATION                 0x20                            //!< operation will perform a write operation on the selcted register(s)
#define PERMREG_READ_OPERATION                  0x00                            //!< operation will perform a read operation on the selcted register(s)
#define PERMREG_ERROR_CONDITION                 0x10                            //!< an error occurred during write/read operation
#define PERMREG_ACK_REQUIRED                    0x08                            //!< an acknowledge is required at the end of the current operation
#define PERMREG_RESERVED_BIT_2                  0x04                            //!< reserved bit 2
#define PERMREG_RESERVED_BIT_1                  0x02                            //!< reserved bit 1
#define PERMREG_RESERVED_BIT_0                  0x01                            //!< reserved bit 0


/* ERRORS */
#define PERMREG_NO_ERROR_CODE                   0                               //!< no error in last operation
#define PERMREG_ERROR_LENGHT                    1                               //!< lenght error
#define PERMREG_ERROR_WRONG_FORMAT              2                               //!< wrong format error
#define PERMREG_ERROR_NOT_IMPLEMENTED           3                               //!< error: not implemented
#define PERMREG_ERROR_ACTION_NOT_ALLOWED        4                               //!< error: action not allowed on the selected register(s)
#define PERMREG_ERROR_REG_IS_READ_ONLY          5                               //!< error: register is readonly
#define PERMREG_ERROR_NOT_ALLOWED               6                               //!< error: not allowed
#define PERMREG_ERROR_MULTIFRAME_ERR            7                               //!< error: multi-fragment error (not implemented, RFU)

#define PERMREG_MAX_LENGHT                      (64 + 4 + 4)                    //!< maximum frame lenght
#define PERMREG_MAX_PAYLOAD_LENGHT              (PERMREG_MAX_LENGHT - 4)        //!< maximum payload lenght

/**
 * @}
 */   
   
/** @defgroup WeSU_Config_Exported_Constants     WeSU Config Exported Constants                
  * @{
  */ 
extern const uint16_t nGlobalConfStruct[];       //!< Persistent Registers Global Struct

/**
 * @}
 */   

/** @defgroup WeSU_Config_Private_Functions     WeSU Config Private Functions              
  * @{
  */ 

uint8_t ReadOnly(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t CopyToSession(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t DummyRegChange(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t LedControlMaskChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t RedLedControlMaskChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t PwrModeControlChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t RtcConfChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
void Switch_To_OTA_Service_Manager_Application(uint32_t imageBase);
void Switch_To_USB_DFU(uint32_t nDfuFlag);
uint8_t TxPwrLvlChange(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t MotionFxCtrlChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t TimerFreqChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t FXMagCalibChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t PedometerCtrlChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t BleConIntvChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t HwConfigChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t FwConfigChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t AccFsChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t AccOdrChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t GyroFsChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t GyroOdrChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t MagFsChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t MagOdrChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t AccEventConfChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t PressOdrChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t PwrModeAltRegChange(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t TimerDrdyCtrlChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);

/**
 * @}
 */

/** @defgroup WeSU_Config_Exported_Functions     WeSU Config Exported Functions              
  * @{
  */   
  
void LedManageStart(uint8_t nLedMode);
void LedManageEnd(uint8_t nLedMode); 

void setConnectable(uint16_t nMin, uint16_t nMax, uint8_t boardSleeping);
void BluenrgInitialization();
void Init_BlueNRG_Hw(void);
void Init_BlueNRG_Stack(void);
void Init_BlueNRG_Custom_Services(void);
void Init_PWR_MGMT(void);

void DebugConfiguration(void);
uint32_t WesuGetSystemStartup();
void RegistersInitialization(void);
uint8_t AdvIntvChange(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);
uint8_t ProcessRegCommand(RegFrameStruct_t *pCommand, uint8_t nFrameLenght);

void SystemReconfigHw();

void PrintSystemParameters();
void SystemGotoShutdown(void);
void SystemGotoFullRunMode(void);
void SystemGotoFullRunWfiMode(void);
void SystemGotoLowPowerRunMode(void);
void SystemGotoLowPowerRunWithAlgos(void);
void SystemGotoLowPowerRunModeJustAcc(void);
void SystemGotoLowPowerMode();
void SystemGotoRunSleepMode();
void SystemGotoRunLowPowerSleepMode();
void SystemGotoPermanentStopMode(void);
void SystemGotoPermanentStopBleMode(void);
void SystemGotoPermanentStopBlePmMode(void);

void SensorsPrintInitStatus();
void SensorsConfigInit(void);
void SensorsConfigFullRun(void);
void SensorsConfigLowPowerRunWithAlgos(void); 
void SensorsConfigJustAcc(void);
void SensorsConfigLowPowerRun(void);


void SystemGotoRebootMode();
void SystemGotoRebootDefSettingsMode();
void WesuSysChangeRunMode(uint8_t nMode);
void WesuSaveSystemParameters();
void WesuBackupSystemParameters(uint32_t nMode);
void WesuGetBackupData(uint32_t nMode, WesuBackupData_t *d);

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

#endif /* __WESU_CONFIG_H */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
