/**
  ******************************************************************************
  * @file    wesu.h
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Header for wesu.c module
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
#ifndef __WESU_H
#define __WESU_H

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#ifdef USE_REGS
#include "wesu_regs.h"
#endif //USE_REGS

/** @addtogroup BSP             BSP
  * @{
  */

/** @addtogroup STEVAL-WESU1    STEVAL-WESU1
  * @{
  */


/** @addtogroup PLATFORM        PLATFORM
  * @{
  */

/** @addtogroup WeSU_LOW_LEVEL  WeSU_LOW_LEVEL
  * @{
  */


/** @defgroup WeSU_LOW_LEVEL_Private_Types WeSU_LOW_LEVEL_Private_Types
  * @{
  */

/**
  * @brief  Led_Typedef
  */
typedef enum
{
  LED = 0,
  RED_LED = 1
}Led_TypeDef;

/**
  * @brief  Button_TypeDef
  */
typedef enum
{
  BUTTON_USER = 0
}Button_TypeDef;

/**
  * @brief  ButtonMode_TypeDef
  */
typedef enum
{
  BUTTON_MODE_GPIO = 0,         //!< GPIO MODE
  BUTTON_MODE_EXTI = 1          //!< EXTI MODE
}ButtonMode_TypeDef;

/**
  * @brief  UsbPwrMode_TypeDef
  */
typedef enum
{
  USB_PWR_MODE_GPIO = 0,        //!< GPIO MODE
  USB_PWR_MODE_EXTI = 1         //!< EXTI MODE
}UsbPwrMode_TypeDef;

/**
  * @}
  */

/** @defgroup WeSU_LOW_LEVEL_Exported_Constants WeSU_LOW_LEVEL_Exported_Constants
  * @{
  */

/**
  * @brief  Define for WeSU board
  */
#if !defined (USE_WESU)
 #error "missing USE_WESU define"
#endif

#ifndef FIRMWARE_VERSION
 #error "missing FIRMWARE_VERSION define"
#endif

/**
  * @}
  */


#ifdef USE_REGS
  #define SESSION_REG(V)      GlobalSessionStruct->V          //!< MACRO to assign GlobalSessionStruct
#else //USE_REGS
  #define SESSION_REG(V)      V          //!< MACRO to assign GlobalSessionStruct
#endif //USE_REGS

/* EEPROM address for EEPROM REGISTERS */
#define USB_DFU_MEM_INFO                                        (uint32_t)(EEPROM_BASE_ADDRESS+0xFF0)                   //!< EEPROM address for USB-DFU flag
#define NEW_APP_MEM_INFO                                        (uint32_t)(EEPROM_BASE_ADDRESS+0xFF8)                   //!< EEPROM address for application started by RESET MANAGER
#define PERMREG_STRUCT_START_ADDRESS                            (EEPROM_BASE_ADDRESS+0x1000)                            //!< PERSISTENT Regs Start Address
#define PERMREG_STRUCT_BCK_START_ADDRESS                        (EEPROM_BASE_ADDRESS+0x1400)                            //!< PERSISTENT Regs Backup Start Address
#define PRODUCTION_DATA_START_ADDRESS                           (EEPROM_BASE_ADDRESS+0x2F00)                            //!< Production Data Start Address
#define LICENSE_START_ADDRESS                                   (EEPROM_BASE_ADDRESS+0x1800)                            //!< License Start Address
#define ROM_VARS                                                                                                        //!< use compiler assigned address for configuration struct

#define TEST_ID_EEPROM_ADDRESS                                  0x08082E00                                              //!< EEPROM production data
#define TEST_DATETIME_EEPROM_ADDRESS                            0x08082EE0                                              //!< EEPROM production data
#define TEST_RESULT_EEPROM_ADDRESS                              0x08082EF0                                              //!< EEPROM production data

/* Ram address for session registers */
#define SESSIONREG_STRUCT_START_ADDRESS                         0x20013000                                              //!< SESSION REGISTERS Start Address

/* WeSU memory organization */
#define FLASH_BASE_ADDRESS                                      0x08000000                                              //!< STM32 Internal flash memory base address
#define EEPROM_BASE_ADDRESS                                     0x08080000                                              //!< STM32 Internal EEPROM base address

#define OTA_FW_ADDRESS                                          (FLASH_BASE_ADDRESS + 0x3000 + 0x800)                   //!< Firmware base address for BlueNRG OTA application

#define BLUENRG_BRIDGE_FW_ADDRESS                               (FLASH_BASE_ADDRESS + 0x8000)                          //!< Firmware base address for USB-BlueNRG bridge application (see \ref RESTART_BLUENRG_BRIDGE_MODE)

#define APP_FW_ADDRESS                                          (FLASH_BASE_ADDRESS + 0x20000)                          //!< Fw base address for WeSU application

#define ACC_READ                                                SESSION_REG(acceleration)                               //!< Acceleration Value in the Session Registers
#define GYRO_READ                                               SESSION_REG(angular_rate)                               //!< Angular Rate Value in the Session Registers
#define MAG_READ                                                SESSION_REG(magnetic_field)                             //!< Magnetic_field Value in the Session Registers
#define PRESS_READ                                              SESSION_REG(pressure)                                   //!< Pressure Value in the Session Registers
#define TEMP_READ                                               SESSION_REG(temperature)                                //!< Temperature Value in the Session Registers
#define BATT_V_READ                                             SESSION_REG(fBatteryVolt)                               //!< Battery Voltage Value in the Session Registers
#define BATT_I_READ                                             SESSION_REG(fCurrentuA)                                 //!< Battery Current Value in the Session Registers
#define BATT_SOC_READ                                           SESSION_REG(nChargepercent)                             //!< Status of Charge % Value in the Session Registers
#define BATT_STATUS_READ                                        SESSION_REG(nPowerMode)                                 //!< Battery Status Value in the Session Registers

#define ENABLED_DEBUG_LEVEL_USART                               SESSION_REG(WeSU_DEBUG_SEVERITY_usart)      //!< Usart messages debug severity 
#define ENABLED_DEBUG_LEVEL_BLE                                 SESSION_REG(WeSU_DEBUG_SEVERITY_ble)        //!< BlueNRG messages debug severity 



/** @defgroup WeSU_LOW_LEVEL_Exported_Variables WeSU_LOW_LEVEL_Exported_Variables
  * @{
  */
extern uint16_t  TimeStamp;

extern AxesRaw_TypeDef ACC_Value;
extern AxesRaw_TypeDef GYR_Value;
extern AxesRaw_TypeDef MAG_Value;
extern int32_t PRESS_Value;
extern int32_t TEMP_Value;
extern uint16_t BATT_V_Value;
extern uint16_t BATT_I_Value;
extern uint16_t BATT_SOC_Value;
extern uint8_t BATT_STATUS_Value;


/**
  * @}
  */ 


/** @defgroup WeSU_LOW_LEVEL_Exported_Types WeSU_LOW_LEVEL_Exported_Types
  * @{
  */


#define USE_RANDOM_ADDRESS_ALWAYS               0x00FF          //!< BlueNRG public address set to RANDOM
#define USE_RANDOM_ADDRESS_IF_DEFAULT           0x0001          //!< BlueNRG public address set to RANDOM if address is not set by user
#define USE_USB_ADDRESS_IF_DEFAULT              0x0000          //!< BlueNRG public address set to USB address if address is not set by user

#define WESU_DBG_WFI()                          if(SESSION_REG(nRsvdDummyRegDebug))__WFI()  //!< WFI Instruction
#define WESU_SYSTEM_RESET()                     HAL_NVIC_SystemReset()                          //!< System Reset WESU
#define WESU_DELAY_BEFORE_RESET                 1000                                            //!< Delay in ms before System_Reset


#define LPCFG_CHANGE_MSI_CLK                    0x00000001      //!< LPCFG to change the MSI CLK
#define LPCFG_SWITCH_TO_HSI_CLK                 0x00000002      //!< LPCFG to switch to HSI CLK
#define LPCFG_SYS_TIMER_ENTER_SLEEP             0x00000004      //!< LPCFG to switch Systick Timer Prescaler when occurs
#define LPCFG_USR_TIMER_ENTER_SLEEP             0x00000008      //!< LPCFG to switch User Timer Prescaler when occurs
#define LPCFG_TIMERS_SHUTDOWN                   0x00000010      //!< LPCFG to shutdown all the timers when the system go to permanent STOP
#define LPCFG_LED_OFF                           0x00000020      //!< LPCFG to switch off the LED using the application command
#define LPCFG_GPIO_AN_IN                        0x00000040      //!< LPCFG to force GPIO in ANALOG Input
#define LPCFG_SWITCH_TO_HSE_CLK                 0x00000080      //!< LPCFG to switch to HSE CLK
#define LPCFG_LED_OFF_FORCED                    0x00000100      //!< LPCFG to switch off the LED using the low level BSP command
#define LPCFG_CONFIG_BLE                        0x00000200      //!< LPCFG to config BLE
#define LPCFG_MCU_ENTER_LP_RUN                  0x00080000      //!< LPCFG to control the action when MCU goes in LP RUN
#define LPCFG_MCU_ENTER_SLEEP                   0x00100000      //!< LPCFG to control the action when MCU goes in SLEEP
#define LPCFG_MCU_ENTER_LP_SLEEP                0x00200000      //!< LPCFG to control the action when MCU goes in LP SLEEP
#define LPCFG_MCU_ENTER_STDBY                   0x00400000      //!< LPCFG to control the action when MCU goes in STDBY
#define LPCFG_MCU_ENTER_STOP                    0x00800000      //!< LPCFG to control the action when MCU goes in STOP
#define LPCFG_MCU_ENTER_STOP_RESET              0x01000000      //!< LPCFG to control the action when MCU goes in STOP and force a RESET after
#define LPCFG_MCU_ENTER_STOP_W_BLE              0x02000000      //!< LPCFG to control the system recovering from STOP after the BLE reconnection
#define LPCFG_MCU_ENTER_STOP_W_BLE_RESET        0x04000000      //!< LPCFG to control the system recovering from STOP after the BLE reconnection and force a RESET after

#define WESU_SYS_POWER_UNKNOWN                  0xFF            //!< Check if current power state machine index is undefined
#define WESU_SYS_POWER_RECONFIG_HW              0xF0            //!< Trigger hardware reconfiguration
#define WESU_SYS_POWER_PERM_STOP                0xFE            //!< Power state machine to go in STOP
#define WESU_SYS_POWER_PERM_BLE_STOP            0xFD            //!< Power state machine to go in STOP but recoverable by BLE
#define WESU_SYS_POWER_REBOOT                   0xFC            //!< Power state machine to REBOOT the System
#define WESU_SYS_POWER_REBOOT_DEFSETTINGS       0xFB            //!< Power state machine to REBOOT loading the default registers settings
#define WESU_SYS_POWER_SHUTDOWN                 0xFA            //!< Power state machine to disconnect battery and shutdown the system
#define WESU_SYS_POWER_PERM_PM_BLE_STOP         0xF9            //!< Power state machine to go in STOP with pedometer running and weakable by BLE
#define WESU_SYS_POWER_FULLRUN                  0x00            //!< Power state machine running
#define WESU_SYS_POWER_FULLRUN_WFI              0x01            //!< Power state machine running with wfi instruction

/* The following masks are not currently implemented */
#define WESU_SYS_POWER_LOWPOWER_RUN             0x0F            //!< Power state machine to go in LOWPOWER (LOW POWER by App)
#define WESU_SYS_POWER_LOWPOWER_RUN_WITH_ALGO   0x02            //!< Power state machine to go in LOWPOWER_RUN_WITH_ALGO (LOW POWER RUN AR CP AE by App)
#define WESU_SYS_POWER_LOWPOWER_RUN_JUST_ACC    0x03            //!< Power state machine to go in LOWPOWER_RUN_JUST_ACC  (LOW POWER RUN AE by App)
#define WESU_SYS_POWER_SLEEP                    0x04            //!< Power state machine to go in SLEEP
#define WESU_SYS_POWER_LOWPOWER_SLEEP           0x05            //!< Power state machine to go in LOWPOWER_SLEEP
#define WESU_SYS_POWER_LOWPOWER                 0x10            //!< Power state machine to go in LOWPOWER_RUN (LOW POWER by App)
#define WESU_SYS_POWER_STOP                     0x50            //!< Power state machine to go in STOP
#define WESU_SYS_POWER_CONN_ONLY                0x80            //!< Power state machine to go in STOP when Bluenrg connection not active

/* The following masks are not currently implemented */

#define POWER_MODE_STOPPING(P)                  (HIGH_NIBBLE(P) == 0xF)         //!< Macro to check if parameter 'P' is a stopping power mode (High nibble is 0xF)

#define LOAD_RUNNING_nLPFunction()               SESSION_REG(nLPFunction) = *PWR_MODE_CONTROL_REGADDR;if(POWER_MODE_STOPPING(SESSION_REG(nLPFunction))){SESSION_REG(nLPFunction) = *PWR_MODE_CONTROL_REGADDR_DEFAULT;}  //!< Macro to load a 'running' power mode (used when exiting from low power)

#define SYSTEM_STARTUP_FLAG_CLOSE_SESSION       0x80000000                      //!< Flag to identify the startup flag for Power-on reset
#define SYSTEM_STARTUP_FLAG_POWERON             0x08000000                      //!< Flag to identify the startup flag for Power-on reset
#define SYSTEM_STARTUP_FLAG_RESET_PIN           0x04000000                      //!< Flag to identify the startup flag for External reset pin
#define SYSTEM_STARTUP_FLAG_SOFT_RESET          0x02000000                      //!< Flag to identify the startup flag for software reset
#define SYSTEM_STARTUP_FLAG_IWDG_RESET          0x01000000                      //!< Flag to identify the startup flag for indipendent watchdog reset

#define SYSTEM_STARTUP_FLAG_GO_LP               0x00800000                      //!< Flag to identify the startup flag for Power-on reset

#define SYS_PARAMS_BACKUP_SESSION_DONE          0x00000001                      //!< Flag store backup paramters in last session
#define SYS_PARAMS_BACKUP_POWERON_DONE          0x00000002                      //!< Flag store backup paramters in last power on

#define WESU_GET_BUTTON_PRESS_MODE              *BUTTON_PRESS_MODE_REGADDR      //!< Power mode to be entered when button is pressed
#define WESU_GET_CONN_ONLY_MODE                 *CONN_ONLY_MODE_REGADDR         //!< Power mode to be entered on timeout when power mode is WESU_SYS_POWER_CONN_ONLY
#define WESU_GET_AUTOSLEEP_MODE                 *AUTOSLEEP_MODE_REGADDR         //!< Power mode to be entered on timeout when AUTOSLEEP_TIME_REG is set

#define DELAY_BEFORE_LOWPOWER_CONN_ONLY         5000                            //!< Delay after last connection to go in low power when power mode is \ref WESU_SYS_POWER_CONN_ONLY

#define POWER_MODE_ON_USB_CABLE                 WESU_SYS_POWER_FULLRUN
#define POWER_MODE_ON_BATTERY                   WESU_SYS_POWER_LOWPOWER_RUN
#define POWER_MODE_ON_LOW_BATTERY               WESU_SYS_POWER_CONN_ONLY
#define POWER_MODE_ON_VERY_LOW_BATTERY          WESU_SYS_POWER_PERM_STOP
    
#define DRDY_CONTROL_ENABLE                           0x00FF
#define TIMER_CONTROL_ENABLE                          0x0001

#define BATTERY_OK                              0
#define BATTERY_LOW                             1
#define BATTERY_VERY_LOW                        2
    
/* LED Control */
#define LED_CONFIG_USER_ENABLED_MASK            0x10    //!< user -> controlled by smartphone
#define LED_CONFIG_APP_CONTROLLED_MASK          0x00    //!< app  -> controlled by firmaware
#define LED_CONFIG_USER_LED_ON_MASK             0x01    //!< To configure the LED on Mask
#define LED_CONFIG_USER_LED_OFF_MASK            0x02    //!< To configure the LED off Mask
#define LED_CONFIG_BRIGHT_MAX                   0x0C00  //!< To configure the LED Max Brightness
#define LED_CONFIG_BRIGHT_LOW                   0x0400  //!< To configure the LED Min Brightness

#define LED_CONFIG_USER_ENABLED()               (LED_CONFIG_USER_ENABLED_MASK & SESSION_REG(nLedControlMask))       //!< Check if led control is user -> controlled by smartphone
#define LED_CONFIG_USER_CMD_ON()                (LED_CONFIG_USER_LED_ON_MASK & SESSION_REG(nLedControlMask))        //!< Check if led control is To configure the LED on 
#define LED_CONFIG_USER_CMD_OFF()               (LED_CONFIG_USER_LED_OFF_MASK & SESSION_REG(nLedControlMask))       //!< Check if led control is To configure the LED off 
#define RED_LED_CONFIG_USER_ENABLED()           (LED_CONFIG_USER_ENABLED_MASK & SESSION_REG(nRedLedControlMask))    //!< Check if led control is user -> controlled by smartphone
#define RED_LED_CONFIG_USER_CMD_ON()            (LED_CONFIG_USER_LED_ON_MASK & SESSION_REG(nRedLedControlMask))     //!< Check if led control is To configure the LED on 
#define RED_LED_CONFIG_USER_CMD_OFF()           (LED_CONFIG_USER_LED_OFF_MASK & SESSION_REG(nRedLedControlMask))    //!< Check if led control is To configure the LED off 
#define SWITCH_LED(L)                           //!< Not implemented

//nReadSensorMask defines
#define ACC_ENABLED_MASK_                       HW_CAP_ACCEL                                                            //!< Accerometer Enabling Mask
#define GYRO_ENABLED_MASK_                      HW_CAP_GYRO                                                             //!< Gyroscope Enabling Mask
#define MAG_ENABLED_MASK_                       HW_CAP_MAGN                                                             //!< Magnetometer Enabling Mask
#define PRESS_ENABLED_MASK_                     HW_CAP_PRESS                                                            //!< Pressure Sensor Enabling Mask
#define FUEL_GAUGE_ENABLED_MASK_                HW_CAP_GG                                                               //!< Gas Gauge Enabling Mask
#define TEMP_ENABLED_MASK_                      HW_CAP_TEMPERATURE                                                      //!< Temperature sensor Enabling Mask

#define AHRS_ENABLED_MASK_                      0x0080                                                                  //!< Sensor Fusion Algoritm Enabling Mask
#define MOTION_FX_ENABLED_MASK_                 0x0100                                                                  //!< Sensor Fusion FX Enabling Mask


#define READ_SENSORS_ACC_ENABLED()              (ACC_ENABLED_MASK_ & SESSION_REG(nReadSensorMask))                  //!< Accerometer Enabling Macro
#define READ_SENSORS_GYRO_ENABLED()             (GYRO_ENABLED_MASK_ & SESSION_REG(nReadSensorMask))                 //!< Gyroscope Enabling Macro
#define READ_SENSORS_MAG_ENABLED()              (MAG_ENABLED_MASK_ & SESSION_REG(nReadSensorMask))                  //!< Magnetometer Enabling Macro
#define READ_SENSORS_PRESS_ENABLED()            (PRESS_ENABLED_MASK_ & SESSION_REG(nReadSensorMask))                //!< Pressure Sensor Enabling Macro
#define READ_SENSORS_FUEL_GAUGE_ENABLED()       (FUEL_GAUGE_ENABLED_MASK_ & SESSION_REG(nReadSensorMask))           //!< Gas Gauge Enabling Macro
#define READ_SENSORS_TEMP_ENABLED()             (TEMP_ENABLED_MASK_ & SESSION_REG(nReadSensorMask))                 //!< Temperature sensor Enabling Macro

#define CALIBRATION_ACC_ENABLED()               (ACC_ENABLED_MASK_ & (*CALIBRATION_HW_REGADDR))                         //!< Accerometer calibration enabling Macro
#define CALIBRATION_GYRO_ENABLED()              (GYRO_ENABLED_MASK_ & (*CALIBRATION_HW_REGADDR))                        //!< Gyroscope calibration enabling Macro
#define CALIBRATION_MAG_ENABLED()               (MAG_ENABLED_MASK_ & (*CALIBRATION_HW_REGADDR))                         //!< Magnetometer calibration enabling Macro
#define CALIBRATION_PRESS_ENABLED()             (PRESS_ENABLED_MASK_ & (*CALIBRATION_HW_REGADDR))                       //!< Pressure Sensor calibration enabling Macro
#define CALIBRATION_FUEL_GAUGE_ENABLED()        (FUEL_GAUGE_ENABLED_MASK_ & (*CALIBRATION_HW_REGADDR))                  //!< Gas Gauge calibration enabling Macro
#define CALIBRATION_TEMP_ENABLED()              (TEMP_ENABLED_MASK_ & (*CALIBRATION_HW_REGADDR))                        //!< Temperature sensor calibration enabling Macro

#define FUEL_GAUGE_EXEC_INTERVAL                8000            //!< Time Interval for FUEL GAUGE task execution (in ms)

#define PROCESS_PEDOMETER_ENABLED()             (ALGORITHM3_BLUEST_MASK & SESSION_REG(nProcessFwMask))                      //!< Pedometer Algoritm Enabling Macro

//#define PROCESS_FREEFALL_ENABLED()              (ALGORITHM4_BLUEST_MASK & SESSION_REG(nProcessFwMask))                      //!< Free fall Algoritm Enabling Macro
#define PROCESS_ACCEVENT_ENABLED()              (ACCEVENT_BLUEST_MASK & SESSION_REG(nProcessFwMask))                        //!< AccEvent Algoritm Enabling Macro
   
#define PROCESS_GP_ALGORITHM_ENABLED()              (GP_ALGORITHM_BLUEST_MASK & SESSION_REG(nProcessFwMask))                      //!< GP_ALGORITHM Enabling Macro

#define OUTPUT_ON_BLE_ACC_ENABLED()             ALGORITHM_CHANNEL_BLE(nRawMotionCtrlMask)                                       //!< Check if Accelerometer read is transmitted on BlueNRG
#define OUTPUT_ON_BLE_GYRO_ENABLED()            ALGORITHM_CHANNEL_BLE(nRawMotionCtrlMask)                                       //!< Check if Gyroscope read is transmitted on BlueNRG
#define OUTPUT_ON_BLE_MAG_ENABLED()             ALGORITHM_CHANNEL_BLE(nRawMotionCtrlMask)                                       //!< Check if Magnetometer read is transmitted on BlueNRG
#define OUTPUT_ON_BLE_MEMS_ENABLED()            ALGORITHM_CHANNEL_BLE(nRawMotionCtrlMask)                                       //!< Check if MEMS sensors read is transmitted on BlueNRG
#define OUTPUT_ON_BLE_PRESS_ENABLED()           ALGORITHM_CHANNEL_BLE(nPressCtrlMask)                                           //!< Check if pressure read is transmitted on BlueNRG
#define OUTPUT_ON_BLE_FUEL_GAUGE_ENABLED()      ALGORITHM_CHANNEL_BLE(nPwrCtrlMask)                                             //!< Check if Fuel gauge read is transmitted on BlueNRG
#define OUTPUT_ON_BLE_TEMP_ENABLED()            ALGORITHM_CHANNEL_BLE(nTemperatureCtrlMask)                                     //!< Check if temperature read is transmitted on BlueNRG

#define OUTPUT_ON_BLE_PEDOMETER_ENABLED()       ALGORITHM_CHANNEL_BLE(nPedometerCtrlMask)                                       //!< Check if pedometer read is transmitted on BlueNRG
#define OUTPUT_ON_BLE_ACCEVENT_ENABLED()        ALGORITHM_CHANNEL_BLE(nAccEventCtrlMask)                                       //!< Check if pedometer read is transmitted on BlueNRG
#define OUTPUT_ON_BLE_GP_ALGORITHM_ENABLED()        ALGORITHM_CHANNEL_BLE(nGP_ALGORITHMCtrlMask)                                       //!< Check if GP_ALGORITHM read is transmitted on BlueNRG

#define OUTPUT_ON_BLE_MOTION_AR_ENABLED()       ALGORITHM_CHANNEL_BLE(nMotionArCtrlMask)                                       //!< Check if MotionAr read is transmitted on BlueNRG
#define PROCESS_MOTION_AR_ENABLED()             (ALGORITHM1_BLUEST_MASK & SESSION_REG(nProcessFwMask))                      //!< MotionAr Algoritm Enabling Macro
#define OUTPUT_ON_BLE_MOTION_CP_ENABLED()       ALGORITHM_CHANNEL_BLE(nMotionCpCtrlMask)                                       //!< Check if MotionCp read is transmitted on BlueNRG
#define PROCESS_MOTION_CP_ENABLED()             (ALGORITHM2_BLUEST_MASK & SESSION_REG(nProcessFwMask))                      //!< MotionCp Algoritm Enabling Macro

#define OUTPUT_ON_BLE_MOTION_FX_ENABLED()       ALGORITHM_CHANNEL_BLE(nMotionFxCtrlMask)                                       //!< Check if MotionFx read is transmitted on BlueNRG
#define PROCESS_MOTION_FX_ENABLED()             (MOTION_FX_ENABLED_MASK_ & SESSION_REG(nProcessFwMask))                      //!< MotionFx Algoritm Enabling Macro


//channel and subsampling

//#define OUTPUT_PWM_CHANNEL                      0x01                                                                            //!< Output channel is PWM (RFU)
#define OUTPUT_USART_CHANNEL                    0x02                                                                            //!< Output channel is USART (RFU)
#define OUTPUT_BLE_CHANNEL                      0x04                                                                            //!< Output channel is BlueNRG
//#define OUTPUT_USB_CHANNEL                      0x08                                                                            //!< Output channel is USB (Not implemented in WeSU)
#define OUTPUT_RAM_CHANNEL                      0x80                                                                            //!< Output channel is RAM (Update value on Session registers)

#define ALGORITHM_CHANNEL_MASK(R)               ((R)&0xFF)                                                                      //!< Output channel mask in configuration register
#define ALGORITHM_SUBSAMPL_MASK(R)              (((R)&0xFF00)>>8)                                                               //!< Subsampling channel mask in configuration register
#define ALGORITHM_CHANNEL_RAM(R)                (SESSION_REG(R)&OUTPUT_RAM_CHANNEL)                                         //!< see \ref OUTPUT_RAM_CHANNEL
#define ALGORITHM_CHANNEL_BLE(R)                (SESSION_REG(R)&OUTPUT_BLE_CHANNEL)                                         //!< see \ref OUTPUT_BLE_CHANNEL
#define ALGORITHM_CHANNEL_USART(R)              (SESSION_REG(R)&OUTPUT_USART_CHANNEL)                                       //!< see \ref OUTPUT_USART_CHANNEL
//#define ALGORITHM_CHANNEL_PWM(R)                (SESSION_REG(R)&OUTPUT_PWM_CHANNEL)                                         //!< see \ref OUTPUT_PWM_CHANNEL
//#define ALGORITHM_CHANNEL_USB(R)                (SESSION_REG(R)&OUTPUT_USB_CHANNEL)                                         //!< see \ref OUTPUT_USB_CHANNEL
#define ALGORITHM_ENABLED(R)                    ((SESSION_REG(R)&0xFF)!=0)                                                  //!< Algorithm is enabled if one output channel is enabled. Use \ref OUTPUT_RAM_CHANNEL if no output interface is needed

#define CHECK_SUBSAMPLING(S)                    (TimeStamp%SESSION_REG(S)==0)                                               //!< Check subsamplng for the requested sensor

#define APP_ACC_SENSITIVITY         1.0f                                        //!< Mobile APP sensitivity for accelerometer data
#define APP_GYRO_SENSITIVITY        100.0f                                      //!< Mobile APP sensitivity for gyroscope data
#define APP_MAG_SENSITIVITY         1.0f                                        //!< Mobile APP sensitivity for magnetometer data

#define ACC_FullScale_DEFAULT                     4                             //!< Acc_FS Default value
#define ACC_OutputDataRate_DEFAULT                104 //LSM6DS3_ACC_GYRO_ODR_XL_104Hz //!< Acc_ODR Default value
#define ACC_OutputDataRate_ALGOS                  52  //LSM6DS3_ACC_GYRO_ODR_XL_52Hz  //!< Acc_ODR Algos value just for AR - CP - and Acc Event

#define ACC_FullScale_LOWPOWER                    2                             //!< Acc_FS Low Power value
#define ACC_OutputDataRate_LOWPOWER               LSM6DS3_ACC_GYRO_ODR_XL_13Hz  //!< Acc_ODR Low Power value

#define GYRO_FullScale_DEFAULT                    2000                          //!< Gyro_FS Default value
#define GYRO_OutputDataRate_DEFAULT               104                           //!< Gyro_ODR Default value
#define GYRO_OutputDataRate_ALGOS                 52  //LSM6DS3_ACC_GYRO_ODR_XL_52Hz  //!< Acc_ODR Algos value just for AR - CP - and Acc Event

#define GYRO_FullScale_LOWPOWER                   245 //LSM6DS3_ACC_GYRO_ODR_XL_104Hz //!< Gyro_FS Low Power value
#define GYRO_OutputDataRate_LOWPOWER              LSM6DS3_ACC_GYRO_ODR_G_13Hz   //!< Gyro_ODR Low Power value

#define MAG_FullScale_DEFAULT                     LIS3MDL_MAG_FS_8Ga            //!< Mag_FS Default value
#define MAG_OutputDataRate_DEFAULT                80                            //!< Mag_ODR Default value

#define MAG_FullScale_LOWPOWER                    LIS3MDL_MAG_FS_4Ga            //!< Mag_FS Low Power value
#define MAG_OutputDataRate_LOWPOWER               LIS3MDL_MAG_DO_5Hz            //!< Mag_ODR Low Power value

#define ACCEVENT_CONF_DEFAULT                     (ACCEVENT_FEATURE_6DORIENTATION | ACCEVENT_FEATURE_FREE_FALL | ACCEVENT_FEATURE_DOUBLE_TAP | ACCEVENT_FEATURE_SINGLE_TAP | ACCEVENT_FEATURE_TILT | ACCEVENT_FEATURE_WAKE_UP | ACCEVENT_FEATURE_PEDOMETER)
    
#define PRESS_TEMP_OutputDataRate_DEFAULT         25                            //!< Press_ODR Default value
#define PRESS_TEMP_OutputDataRate_LOWPOWER        1                             //!< Press_ODR Low Power value

#define ACC_FullScale_Macro                     *ACC_FullScale_REGADDR                  //!< Acc_FS Default value, read from persistent register
#define ACC_OutputDataRate_Macro                *ACC_OutputDataRate_REGADDR             //!< Acc_ODR Default value, read from persistent register

#define GYRO_FullScale_Macro                    *GYRO_FullScale_REGADDR                 //!< Gyro_FS Default value, read from persistent register
#define GYRO_OutputDataRate_Macro               *GYRO_OutputDataRate_REGADDR            //!< Gyro_ODR Default value, read from persistent register

#define MAG_FullScale_Macro                     *MAG_FullScale_REGADDR                  //!< Mag_FS Default value, read from persistent register
#define MAG_OutputDataRate_Macro                *MAG_OutputDataRate_REGADDR             //!< Mag_ODR Default value, read from persistent register

#define PRESS_TEMP_OutputDataRate_Macro         *PRESS_TEMP_OutputDataRate_REGADDR      //!< Press_ODR Default value, read from persistent register

/** 
  * @brief Define for WeSU board  
  */ 
  
    #define HW_ID                       0x01                                    //!< HW_ID advertising data value for STEVAL-WESU1
    #define USE_BUTTON                  1                                       //!< define to '1' to enable button usage
    #define USE_AUX_PERIPHERAL_USART    1                                       //!< define to '1' to enable usart usage
    #define USE_AUX_PERIPHERAL_PWM      1                                       //!< define to '1' to enable PWM usage
    #define USE_AUX_PERIPHERAL_ADC      1                                       //!< define to '1' to enable ADC usage
    #define USE_AUX_ALGORITHMS          1                                       //!< define to '1' to enable Algorithms usage
    #define USE_STC3115                 1                                       //!< define to '1' to enable STC3115 usage
    #define USE_USB_PWR                 1                                       //!< define to '1' to enable USB power usage
    #define USE_USB_POWER_CONFIG        1                                       //!< define to '1' to enable USB power configuration usage
    #define USE_USB_PLUG_WAKEUP         1                                       //!< define to '1' to enable USB plug wakeup usage
    #define USE_SPI_SENSORS             1                                       //!< define to '1' to enable SPI sensors usage
    #define USE_ACC_IT                  1                                       //!< define to '1' to enable accelerometer interrupt pin usage
    #define USE_GYRO_IT                 1                                       //!< define to '1' to enable gyroscope interrupt pin usage
    #define USE_MAG_IT                  0                                       //!< define to '1' to enable magnetometer interrupt pin usage
    #define USE_PRESS_IT                0                                       //!< define to '1' to enable pressure interrupt pin usage
    #define USE_TEMP_IT                 0                                       //!< define to '1' to enable temperature interrupt pin usage
    #define USE_TIMER                   0

#define FROM_MG_TO_G    0.001                                                   //!< Scale factor. It is used to scale acceleration from mg to g

/** @defgroup WeSU_LED LED Constants
  * @{
  */  
    #define LEDn                        2                       //!< LED Instance

    #define LED_GPIO_PORT               GPIOB                   //!< LED PORT
    #define LED_GPIO_PIN                GPIO_PIN_0              //!< LED PIN PB.0
    #define LED_GPIO_MODE               GPIO_MODE_OUTPUT_PP         //!< LED PIN MODE
    #define LED_GPIO_PULL               GPIO_NOPULL             //!< LED PIN PUSH PULL
    #define LED_GPIO_SPEED              GPIO_SPEED_VERY_LOW     //!< LED PIN SPEED
    #define LED_GPIO_OFF_PIN_STATUS     GPIO_PIN_RESET
    #define LED_GPIO_ON_PIN_STATUS      GPIO_PIN_SET

    #define LED_GPIO_CLK_ENABLE()       __GPIOB_CLK_ENABLE()    //!< LED ENABLE
    #define LED_GPIO_CLK_DISABLE()      __GPIOB_CLK_DISABLE()   //!< LED DISABLE         
    #define LED_GPIO_TIMER              TIM3                    //!< LED PIN WHEN USED AS TIMER 
    #define LED_GPIO_TIMER_ENABLE       __TIM3_CLK_ENABLE()     //!< LED TIMER ENABLING
    #define LED_GPIO_TIMER_DISABLE      __TIM3_CLK_DISABLE()    //!< LED TIMER DISABLING
    #define LED_GPIO_TIMER_AF           GPIO_AF2_TIM3           //!< LED TIMER AF PIN

    #define LED_DMA_ENABLE              __DMA1_CLK_ENABLE();    //!< LED DMA ENABLING
    #define LED_DMA_DISABLE             __DMA1_CLK_DISABLE();   //!< LED DMA DISABLING
  
    #define LEDx_GPIO_CLK_ENABLE(__INDEX__)    do{      \
                                                if((__INDEX__) == LED) LED_GPIO_CLK_ENABLE(); \
                                                if((__INDEX__) == RED_LED) RED_LED_GPIO_CLK_ENABLE(); \
                                                }while(0)       //!< LED CLK MACRO ENABLING
    #define LEDx_GPIO_CLK_DISABLE(__INDEX__)    do{     \
                                                if((__INDEX__) == LED) LED_GPIO_CLK_DISABLE(); \
                                                if((__INDEX__) == RED_LED) RED_LED_GPIO_CLK_DISABLE(); \
                                                }while(0)       //!< LED CLK MACRO DISABLING
                                                   
    #define RED_LED_GPIO_PIN                    CHARGER_GPIO_CHG_PIN
    #define RED_LED_GPIO_PORT                   CHARGER_GPIO_CHG_PORT
    #define RED_LED_GPIO_MODE                   GPIO_MODE_OUTPUT_OD            //!< LED PIN PUSH PULL
    #define RED_LED_GPIO_OFF_PIN_STATUS         GPIO_PIN_SET
    #define RED_LED_GPIO_ON_PIN_STATUS          GPIO_PIN_RESET
    #define RED_LED_GPIO_CLK_ENABLE()           CHARGER_GPIO_CHG_CLK_ENABLE()    //!< LED ENABLE
    #define RED_LED_GPIO_CLK_DISABLE()          CHARGER_GPIO_CHG_CLK_DISABLE()   //!< LED DISABLE         
    
/**
  * @}
  */ 

/** @defgroup WeSU_BUTTON BUTTON Constants
  * @{
  */
    #define BUTTONn                     1                       //!< Button Instance

    #define BUTTON_GPIO_PORT            GPIOA                   //!< BUTTON PORT
    #define BUTTON_GPIO_PIN             GPIO_PIN_2              //!< BUTTON PIN PA.2
    #define BUTTON_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE()    //!< BUTTON ENABLING
    #define BUTTON_GPIO_CLK_DISABLE()   __GPIOA_CLK_DISABLE()   //!< BUTTON DISABLING
    #define BUTTON_EXTI_IRQn            EXTI2_IRQn              //!< BUTTON IRQ Handler

    #define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do{if((__INDEX__) == 0) BUTTON_GPIO_CLK_ENABLE(); \
                                                    }while(0)   //!< BUTTON CLK MACRO ENABLING
    #define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)   do{if((__INDEX__) == 0) BUTTON_GPIO_CLK_DISABLE(); \
                                                    }while(0)   //!< BUTTON CLK MACRO DISABLING

    #define USB_PWR_GPIO_PORT           GPIOC                   //!< USB POWER SUPPLY Monitor PORT
    #define USB_PWR_GPIO_PIN            GPIO_PIN_5              //!< USB POWER SUPPLY Monitor PIN PC.5
    #define USB_PWR_GPIO_CLK_ENABLE()   __GPIOC_CLK_ENABLE()    //!< USB POWER SUPPLY Monitor ENABLING
    #define USB_PWR_GPIO_CLK_DISABLE()  __GPIOC_CLK_DISABLE()   //!< USB POWER SUPPLY Monitor DISABLING
    #define USB_PWR_EXTI_IRQn           EXTI9_5_IRQn            //!< USB POWER SUPPLY IRQ Handler
/**
  * @}
  */

                                                      /** @addtogroup WeSU_MAIN_Exported_Defines     WeSU_MAIN Exported Defines
  * @{
  */

#define USARTx                           USART2                                 //!< WeSU usart peripheral (see datasheet, expansion connector)
#define USARTx_CLK_ENABLE()              __USART2_CLK_ENABLE()                  //!< usart clock enable
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()                   //!< usart gpio clock enable
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()                   //!< usart gpio clock enable

#define USARTx_FORCE_RESET()             __USART2_FORCE_RESET()                 //!< usart force reset
#define USARTx_RELEASE_RESET()           __USART2_RELEASE_RESET()               //!< usart release reset

#define USARTx_TX_PIN                    GPIO_PIN_2                             //!< usart TX pin
#define USARTx_TX_GPIO_PORT              GPIOA                                  //!< usart TX port
#define USARTx_TX_AF                     GPIO_AF7_USART2                        //!< usart TX alternate function
#define USARTx_RX_PIN                    GPIO_PIN_3                             //!< usart TX pin
#define USARTx_RX_GPIO_PORT              GPIOA                                  //!< usart TX port
#define USARTx_RX_AF                     GPIO_AF7_USART2                        //!< usart TX alternate function
#define USARTx_IRQn                      USART2_IRQn                            //!< usart IRQn
#define COM_BAUDRATE                    115200                                  //!< usart baudrate

 
/* Definition for TIMx clock resources : Timer used for user process*/
#define TIM_process                       TIM2                                  //!< Timer Process Instance
#define TIM_process_CLK_ENABLE           __TIM2_CLK_ENABLE                      //!< Timer Process CLK ON 
#define TIM_process_CLK_DISABLE          __TIM2_CLK_DISABLE                     //!< Timer Process CLK OFF                              
/* Definition for TIMx's NVIC */
#define TIM_process_IRQn                 TIM2_IRQn                              //!< Timer Process IRQn 
#define TIM_process_IRQHandler           TIM2_IRQHandler                        //!< Timer Process IRQ handler    
/**
 * @}
 */
                                                      
#define EXEC_WITH_INTERVAL(a,i)   {static uint32_t t=(0xFFFFFFFF-i)+1;if(HAL_GetTick()>=(t+i)){t=HAL_GetTick();{a;}}}  //!< Execute application 'a' with interval 'i' milliseconds

#define LICENSE_SIZE                                            48                                                      //!< License size
#define LICENSE_INIT_MAX_TRIES                                  3                                                       //!< Max retry for license initialization


#define LOW_BYTE(B)                                             (B&0xFF)                                        //!< Macro to get lower byte
#define HIGH_BYTE(B)                                            ((B&0xFF00)>>8)                                 //!< Macro to get higher byte
#define SET_LOW_BYTE(V16, V8)                                   V16 = (((V16)&0xFF00) | (0xFF&(V8)))                //!< Macro to set lower byte
#define SET_HIGH_BYTE(V16, V8)                                  V16 = (((V16)&0xFF) | ((0xFF&(V8))<<8))             //!< Macro to Set higher byte

#define LOW_NIBBLE(B)                                           (B&0x0F)                                        //!< Macro to get lower nibble
#define HIGH_NIBBLE(B)                                          ((B&0xF0)>>4)                                   //!< Macro to get higher nibble
#define SET_LOW_NIBBLE(V8, V4)                                  V8 = (((V8)&0xF0) | (0xF&(V4)))                     //!< Macro to set lower nibble
#define SET_HIGH_NIBBLE(V8, V4)                                 V8 = (((V8)&0xF) | ((0xF&(V4))<<4))                 //!< Macro to set higher nibble

#define HEXH(b)                                                 (HIGH_NIBBLE(b)<=9)?('0'+HIGH_NIBBLE(b)):('A'+(HIGH_NIBBLE(b)-0x0A))    //!< Macro to get higher nibble in hex 'char' value
#define HEXL(b)                                                 (LOW_NIBBLE(b)<=9)?('0'+LOW_NIBBLE(b)):('A'+LOW_NIBBLE(b))              //!< Macro to get lower byte address

#define CLEAR_FLAG_8(W8, F8)                                    W8 &= ~((uint8_t)(F8))                          //!< Macro to clear the F8 flag asserted in the given W8 word
#define CLEAR_FLAG_16(W16, F16)                                 W16 &= ~((uint16_t)(F16))                       //!< Macro to clear the F16 flag asserted in the given W16 word
#define CLEAR_FLAG_32(W32, F32)                                 W32 &= ~((uint32_t)(F32))                       //!< Macro to clear the F32 flag asserted in the given W32 word

#define SET_FLAG_8(W8, F8)                                      W8 |= ((uint8_t)(F8))                          //!< Macro to set the F8 flag asserted in the given W8 word
#define SET_FLAG_16(W16, F16)                                   W16 |= ((uint16_t)(F16))                       //!< Macro to set the F16 flag asserted in the given W16 word
#define SET_FLAG_32(W32, F32)                                   W32 |= ((uint32_t)(F32))                       //!< Macro to set the F32 flag asserted in the given W32 word

#define WESU_L_V2_HW_CAPABILITIES                               (HW_CAP_ACCEL | HW_CAP_GYRO | HW_CAP_MAGN | HW_CAP_PRESS | HW_CAP_TEMPERATURE | HW_CAP_GG)   //!< WESU hardware capabilities

#define WESU_L_V2_FW_CAPABILITIES                               (FW_CAP_FX)                                                             //!< WESU firmware capabilities (algorithms)

#define DEFAULT_BLE_TX_POWER_LEVEL                              5                                                                       //!< Default bluenrg transmit power level
#define DEFAULT_BLE_TX_HIGH_POWER                               1                                                                       //!< Default bluenrg 'high power' setting
#define DEFAULT_BLE_TX_POWER_VALID_MASK                         0x08                                                                    //!< Default bluenrg power valid mask

#define DEFAULT_CALIBRATION                                     FX_CALIBRATION//(INEMO_CALIBRATION | FX_CALIBRATION)

#define AUTOSLEEP_TIME_DEFAULT_REG_VALUE                        0                                                                       //!< Default autosleep timer value

#define PWR_FEATURE_MASK_CHANNEL_DEFAULT_VALUE                  (OUTPUT_RAM_CHANNEL | OUTPUT_BLE_CHANNEL)                               //!< Default output channel for hardware feature 1
#define PWR_FEATURE_MASK_SUBSAMPL_DEFAULT_VALUE                 PERSISTENT_DEFAULT_REG_0x21                                             //!< Default subsampling value for hardware feature 1
#define TEMPERATURE_FEATURE_MASK_CHANNEL_DEFAULT_VALUE          (OUTPUT_RAM_CHANNEL | OUTPUT_BLE_CHANNEL)                               //!< Default output channel for hardware feature 2
#define TEMPERATURE_FEATURE_MASK_SUBSAMPL_DEFAULT_VALUE         4                                                                       //!< Default subsampling value for hardware feature 2
#define PRESSURE_FEATURE_MASK_CHANNEL_DEFAULT_VALUE             (OUTPUT_RAM_CHANNEL | OUTPUT_BLE_CHANNEL)                               //!< Default output channel for hardware feature 3
#define PRESSURE_FEATURE_MASK_SUBSAMPL_DEFAULT_VALUE            4                                                                       //!< Default subsampling value for hardware feature 3
#define RAWMOTION_FEATURE_MASK_CHANNEL_DEFAULT_VALUE            (OUTPUT_RAM_CHANNEL | OUTPUT_BLE_CHANNEL)                               //!< Default output channel for hardware feature 4
#define RAWMOTION_FEATURE_MASK_SUBSAMPL_DEFAULT_VALUE           1                                                                       //!< Default subsampling value for hardware feature 4

#define MOTION_FX_CONTROL_MASK_CHANNEL_DEFAULT_VALUE            (OUTPUT_RAM_CHANNEL | OUTPUT_BLE_CHANNEL)                               //!< Default output channel for MotionFX
#define MOTION_FX_CONTROL_MASK_SUBSAMPL_DEFAULT_VALUE            1                                                                        //!< Default subsampling value for MotionFX

#define PEDOMETER_CONTROL_MASK_CHANNEL_DEFAULT_VALUE            (OUTPUT_RAM_CHANNEL | OUTPUT_BLE_CHANNEL)                               //!< Default output channel for pedometer
#define PEDOMETER_CONTROL_MASK_SUBSAMPL_DEFAULT_VALUE           (PERSISTENT_DEFAULT_REG_0x21/ALGORITHM3_FREQUENCY)                        //!< Default subsampling value for pedometer

#define FREEFALL_CONTROL_MASK_CHANNEL_DEFAULT_VALUE             (OUTPUT_RAM_CHANNEL | OUTPUT_BLE_CHANNEL)                               //!< Default output channel for freefall
#define FREEFALL_CONTROL_MASK_SUBSAMPL_DEFAULT_VALUE            (PERSISTENT_DEFAULT_REG_0x21/ALGORITHM4_FREQUENCY)                        //!< Default subsampling value for freefall

#define ACCEVENT_CONTROL_MASK_CHANNEL_DEFAULT_VALUE             (OUTPUT_RAM_CHANNEL | OUTPUT_BLE_CHANNEL)                               //!< Default output channel for freefall
#define ACCEVENT_CONTROL_MASK_SUBSAMPL_DEFAULT_VALUE            (PERSISTENT_DEFAULT_REG_0x21/ACCEVENT_FREQUENCY)                       //!< Default subsampling value for freefall

#define GP_ALGORITHM_CONTROL_MASK_CHANNEL_DEFAULT_VALUE         (OUTPUT_RAM_CHANNEL | OUTPUT_BLE_CHANNEL)                               //!< Default output channel for GP_ALGORITHM
#define GP_ALGORITHM_CONTROL_MASK_SUBSAMPL_DEFAULT_VALUE         1                                                                      //!< Default subsampling value for GP_ALGORITHM

#define MOTION_AR_CONTROL_MASK_CHANNEL_DEFAULT_VALUE             (OUTPUT_RAM_CHANNEL | OUTPUT_BLE_CHANNEL)                               //!< Default output channel for MOTION_AR
#define MOTION_AR_CONTROL_MASK_SUBSAMPL_DEFAULT_VALUE            PERSISTENT_DEFAULT_REG_0x21/ALGORITHM1_FREQUENCY                        //!< Default subsampling value for MOTION_AR

#define MOTION_CP_CONTROL_MASK_CHANNEL_DEFAULT_VALUE             (OUTPUT_RAM_CHANNEL | OUTPUT_BLE_CHANNEL)                               //!< Default output channel for MOTION_CP
#define MOTION_CP_CONTROL_MASK_SUBSAMPL_DEFAULT_VALUE            PERSISTENT_DEFAULT_REG_0x21/ALGORITHM2_FREQUENCY                        //!< Default subsampling value for MOTION_CP

#define PIN_ALTERNATE_FN_DEFAULT                                0x00                                                                    //!< Default alternate function for configurable pins

#define HW_CAP_ACCEL                                            0x80                                                                    //!< Hardware capability BLUEST mask for accelerometer
#define HW_CAP_GYRO                                             0x40                                                                    //!< Hardware capability BLUEST mask for gyroscope
#define HW_CAP_MAGN                                             0x20                                                                    //!< Hardware capability BLUEST mask for magnetometer
#define HW_CAP_PRESS                                            0x10                                                                    //!< Hardware capability BLUEST mask for pressure
#define HW_CAP_HUMIDITY                                         0x08                                                                    //!< Hardware capability BLUEST mask for humidity
#define HW_CAP_TEMPERATURE                                      0x04                                                                    //!< Hardware capability BLUEST mask for temperature
#define HW_CAP_GG                                               0x02                                                                    //!< Hardware capability BLUEST mask for Gas gausge
#define HW_CAP_RFID                                             0x01                                                                    //!< Hardware capability BLUEST mask for RFID

#define FW_CAP_FX                                             	0x0100                                                                  //!< Firmware capability BLUEST mask for MotionFX

#define WeSU_DEBUG_DEFAULT_VALUE                                (WeSU_DEBUG_MAIN_MASK | WeSU_DEBUG_WESU_CONFIG_MASK | WeSU_DEBUG_TERMINAL_MASK | WeSU_DEBUG_ALGORITHMS_MASK | WeSU_DEBUG_BLUENRG_MASK /*| WeSU_DEBUG_PWR_MGNT_MASK*/ )

#define WeSU_IWDG_DEFAULT_RELOAD_VALUE                          5000

#define DEFAULT_TIMER_FREQUENCY                                 100                                                                      //!< Firmware capability BLUEST mask for pedometer

#define HW_OPERATING_CAPABILITIES_DEFAULT_REG_VALUE             0xFFFF                                                                  //!< Operating Hardware capabilities control mask: (all sensors enabled by default)
#define FW_OPERATING_CAPABILITIES_DEFAULT_REG_VALUE             0xFFFF                                                                  //!< Operating Firmware capabilities control mask: (all algorithms enabled by default)

#define CALIBRATION_HW_DEFAULT_REG_VALUE                        0xFFFF                                                                  //!< Hardware calibration enable register: (all sensors enabled by default)
#define CALIBRATION_FW_DEFAULT_REG_VALUE                        0xFFFF                                                                  //!< Firmware calibration enable register: (all algorithms enabled by default)

#define LED_MODE_DEFAULT                                        LED_MODE_SMART_BLINKING                                                 //!< Default led mode bliking

#define LED_INTERVAL_DISCONNECTED_DEFAULT_VALUE     2000                                                                                //!< Default value for led led blinking interval when disconnected
#define LED_INTERVAL_DISCONNECTED_ON_DEFAULT_VALUE  20                                                                                   //!< Default value for led on-phase when disconnected
#define LED_INTERVAL_CONNECTED_DEFAULT_VALUE        500                                                                                 //!< Default value for led led blinking interval when connected
#define LED_INTERVAL_CONNECTED_ON_DEFAULT_VALUE     20                                                                                  //!< Default value for led on-phase when connected
                                                      
#define BLE_CON_INTV_DEFAULT_REG_VALUE                          0x0014                                                                  //!< BlueNRG default value for connection interval
#define BLE_ADV_INTV_RUN_DEFAULT                                0x0080                                                                  //!< BlueNRG default value for advertising interval in run mode
#define BLE_ADV_INTV_SLEEP_DEFAULT                              0x0800                                                                  //!< BlueNRG default value for advertising interval in sleep/lowpower mode

#define HOURS_DEFAULT                                           8                                                                       //!< default timestamp for RTC
#define MINUTES_DEFAULT                                         0                                                                       //!< default timestamp for RTC
#define SECONDS_DEFAULT                                         0                                                                       //!< default timestamp for RTC
#define DAY_DEFAULT                                             3                                                                       //!< default timestamp for RTC
#define MONTH_DEFAULT                                           RTC_MONTH_OCTOBER                                                       //!< default timestamp for RTC
#define YEAR_DEFAULT                                            16                                                                      //!< default timestamp for RTC
#define WEEKDAY_DEFAULT                                         RTC_WEEKDAY_MONDAY                                                      //!< default timestamp for RTC
#define RTC_CONFIG_DEFAULT                                      RTC_AppConfigDefault                                                    //!< default configuration for RTC

#define INEMO_CALIBRATION                                       0x0001                                                                  //!< iNEMO calibration procedure for magnetometer
#define FX_CALIBRATION                                          0x0003                                                                  //!< FX calibration procedure for magnetometer

#define BLE_DEBUG_CONFIG_DEFAULT_VALUE                          WeSU_DEBUG_SEVERITY_CRITICAL                                            //!< Default minimal severity for debug messages on BlueNRG
#define USART_DEBUG_CONFIG_DEFAULT_VALUE                        WeSU_DEBUG_SEVERITY_VERBOSE                                             //!< Default minimal severity for debug messages on USART

#define BUTTON_PRESS_MODE_DEFAULT_VALUE                         WESU_SYS_POWER_PERM_BLE_STOP                                            //!< Default power mode to be entered when pressing button
#define CONN_ONLY_MODE_DEFAULT_VALUE                            WESU_SYS_POWER_PERM_PM_BLE_STOP                                         //!< Default power mode to be entered on timeout when power mode is WESU_SYS_POWER_CONN_ONLY
#define AUTOSLEEP_MODE_DEFAULT_VALUE                            WESU_SYS_POWER_PERM_BLE_STOP                                            //!< Default power mode to be entered on timeout when AUTOSLEEP_TIME_REG is set

#define BLE_PUB_ADDR_DEFAULT_5                                  0x26                                                                    //!< Default BlueNRG public address
#define BLE_PUB_ADDR_DEFAULT_4                                  0x80                                                                    //!< Default BlueNRG public address
#define BLE_PUB_ADDR_DEFAULT_3                                  0xE1                                                                    //!< Default BlueNRG public address
#define BLE_PUB_ADDR_DEFAULT_2                                  0x00                                                                    //!< Default BlueNRG public address
#define BLE_PUB_ADDR_DEFAULT_1                                  0x00                                                                    //!< Default BlueNRG public address
#define BLE_PUB_ADDR_DEFAULT_0                                  0x00                                                                    //!< Default BlueNRG public address

#define WESU_DEFAULT_NAME                                       "WeSU"                                                                 //!< Default WeSU board name

#define LED_MODE_SENSORS_NOT_CALIBRATED                         0xFF                                                                    //!< Led blinking mode when sensors are not calibrated
#define LED_MODE_SMART_BLINKING                                 0                                                                       //!< Led smart blinking mode for power saving
#define LED_MODE_TOGGLE_TM_EXP_EVENT                            1                                                                       //!< Led blinking mode: toggle when timer expired
#define LED_MODE_MEASURE_TIMER_EXP_EVENT_PROC                   2                                                                       //!< Led blinking mode: measure processing time for timer expred event
#define LED_MODE_OFF                                            3                                                                       //!< Led mode: OFF
#define LED_MODE_OFF2                                           4                                                                       //!< Led mode: OFF
#define LED_MODE_MEASURE_SENSORS_READ                           5                                                                       //!< Led blinking mode: measure sensors read time
#define LED_MODE_MEASURE_AHRS                                   6                                                                       //!< Led blinking mode: measure algorithm processing time
#define LED_MODE_MEASURE_ALGORITHM1                             7                                                                       //!< Led blinking mode: measure algorithm processing time
#define LED_MODE_MEASURE_ALGORITHM2                             8                                                                       //!< Led blinking mode: measure algorithm processing time
#define LED_MODE_MEASURE_ALGORITHM3                             9                                                                       //!< Led blinking mode: measure algorithm processing time
#define LED_MODE_MEASURE_ALGORITHM4                            10                                                                       //!< Led blinking mode: measure algorithm processing time
#define LED_MODE_MEASURE_GP_ALGORITHM                          11                                                                       //!< Led blinking mode: measure algorithm processing time
#define LED_MODE_MEASURE_BLUENRG_UPDATE                        12                                                                       //!< Led blinking mode: measure BlueNRG update time

#define PERMREG_START_DUMMY_REG                                   0xF0                                                                    //!< Start dummy section registers
#define PERMREG_DUMMY_ALT_POWER_MODE_REG                          0xF2                                                                    //!< Dummy register for stopping power mode(s)

#define RESTART_BLUENRG_BRIDGE_MODE                             0x0003                                                                  //!< Restart system in USB-BlueNRG bridge mode for use with BlueNRG DK GUI tool
#define RESTART_DFU_OTA_MODE                                    0x0002                                                                  //!< Restart system in DFU OTA mode
#define RESTART_DFU_USB_MODE                                    0x0001                                                                  //!< Restart system in USB DFU mode
#define RESTART_NORMAL_MODE                                     0x0000                                                                  //!< Set system normal application mode

void BSP_EEPROM_WriteBuffer(uint32_t pWriteAddress, uint32_t pBuf, uint32_t nBufLen);
void BSP_EEPROM_ReadBuffer(uint32_t pReadAddress, uint32_t pBuf, uint32_t nBufLen);

void BSP_WD_Init();
void BSP_WD_Refresh();

/** @addtogroup WeSU_LOW_LEVEL_Exported_Functions       WeSU_LOW_LEVEL_Exported_Functions
  * @{
  */



/** @addtogroup WeSU_LOW_LEVEL_LED_Functions_Prototype  WeSU_LOW_LEVEL_LED_Functions_Prototype
  * @{
  */

  void  BSP_LED_InitGpio(Led_TypeDef Led);
  void  BSP_LED_OnGpio(Led_TypeDef Led);
  void  BSP_LED_OffGpio(Led_TypeDef Led);
  void  BSP_LED_ToggleGpio(Led_TypeDef Led);

  void  BSP_LED_InitDma(Led_TypeDef Led);
  void  BSP_LED_OnDma(Led_TypeDef Led);
  void  BSP_LED_OffDma(Led_TypeDef Led);
  void  BSP_LED_ToggleDma(Led_TypeDef Led);

  uint8_t       BSP_LED_GetState(Led_TypeDef Led);

  void  SetMaxValue(uint32_t nMaxVal);

  void LedSmoothRampUpGpio(uint8_t nSpeed);
  void LedSmoothRampDownGpio(uint8_t nSpeed);
  void LedSmoothBlinkGpio(uint8_t nSpeed);

  void LedSmoothRampUpDma(uint8_t nSpeed);
  void LedSmoothRampDownDma(uint8_t nSpeed);
  void LedSmoothBlinkDma(uint8_t nSpeed);


/**
  * @}
  */

/** @addtogroup WeSU_BUTTON_Functions WeSU_BUTTON_Functions_Prototype
  * @{
  */

  void          BSP_PB_DeInit(Button_TypeDef Button);
  void          BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
  uint32_t      BSP_PB_GetState(Button_TypeDef Button);

  void          BSP_PwrUsbMonitor_DeInit();
  void          BSP_PwrUsbMonitor_Init(UsbPwrMode_TypeDef UsbPwrMode);
  uint32_t      BSP_PwrUsbMonitor_GetState();
  void          BSP_DbgDisable();




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

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __WESU_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
