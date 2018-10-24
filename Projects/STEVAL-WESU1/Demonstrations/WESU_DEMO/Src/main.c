/**
  ******************************************************************************
  * @file    WESU_DEMO/Src/main.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Main program body
  *****************************************************************************
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
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include "main.h"
#include "low_power.h"
#include "BlueST_Protocol.h"
#include "wesu_config.h"
#include "algorithms.h"


   
/** @addtogroup WeSU_Demo       WeSU Demo
  * @{
  */

/** @addtogroup WeSU_User               WeSU User
  * @{
  */


/** @addtogroup WeSU_MAIN               WeSU MAIN
  * @{
  * @brief WeSU DEMO Main
  */   
   
/** @defgroup WeSU_MAIN_Imported_Variables WeSU_MAIN Imported Variables
 * @{
 *
 */
extern volatile uint8_t set_connectable;
extern volatile int bIsConnected;
extern uint8_t calibIndex;         // run calibration @ 25Hz       
extern unsigned char isCal;                                        
extern unsigned char isStartFxCalib;                               
static int32_t CounterFX     =0;
extern uint8_t xCurSmFunc;
/**
 * @}
 */

/** @defgroup WeSU_MAIN_Private_Variables WeSU_MAIN Private Variables
 * @{
 */

FlagStatus LpsDataReady=RESET;                                                  //!< LPS25HB sensor data-ready flag
FlagStatus LsmXGInt1Ready=RESET;                                                //!< LSM6DS3 sensor INT1 flag
FlagStatus LsmXGInt2Ready=RESET;                                                //!< LSM6DS3 sensor INT2 flag
FlagStatus LisIntReady=RESET;                                                   //!< LIS3MDL sensor INT flag
FlagStatus LsmDrdyMReady=RESET;                                                 //!< LIS3MDL sensor data-ready flag
FlagStatus GasGaugeInterrupt=RESET;                                             //!< STC3115 interrupt flag

RTC_HandleTypeDef RtcHandle;                                                    //!< RTC HANDLE 

void *ACCELERO_handle = NULL;                                                   //!< ACCELEROMETER HANDLE 
void *GYRO_handle = NULL;                                                       //!< GYROSCOPE HANDLE 
void *MAGNETO_handle = NULL;                                                    //!< MAGNETOMETER HANDLE 
void *PRESSURE_handle = NULL;                                                   //!< PRESSURE HANDLE 
void *TEMPERATURE_handle = NULL;                                                //!< TEMPERATURE HANDLE 

uint32_t nLastConnectionTick = 0;                                               //!< store last connection tick
  
int UsbConnected=0;                                                             //!< usb power connection status

osxMFX_calibFactor magOffset;                             

SensorAxes_t quat_axes[3];

/**
 * @}
 */

/* Define How Many quaterions you want to trasmit (from 1 to 3) */
#define SEND_N_QUATERNIONS 3
#ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifdef RTC_CLOCK_SOURCE_LSI
  #define RTC_ASYNCH_PREDIV  0x7F       
  #define RTC_SYNCH_PREDIV   0x0130
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
  #define RTC_ASYNCH_PREDIV  0x7F
  #define RTC_SYNCH_PREDIV   0x00FF
#endif

#define RTC_ASYNCH_PREDIV_LSI  0x7F
#define RTC_SYNCH_PREDIV_LSI   0x0130
#define RTC_ASYNCH_PREDIV_LSE  0x7F
#define RTC_SYNCH_PREDIV_LSE   0x00FF

#endif /* DOXYGEN_SHOULD_SKIP_THIS */


/** @defgroup WeSU_MAIN_Exported_Variables WeSU_MAIN_Exported_Variables
 * @{
 */

uint8_t bDebuggerConnected = 0;                                                 //!< This facility (bDebuggerConnected) enables the output on IAR Terminal I/O 

int ButtonPressed=FALSE;                                                        //!< user push button pressed event
int LowPowerFlag=FALSE;                                                         //!< Flag to put system in low power mode

AxesRaw_TypeDef ACC_Value;                                                      //!< Prepare data for BLUEST accelerometer characteristic update
AxesRaw_TypeDef GYR_Value;                                                      //!< Prepare data for BLUEST gyroscope characteristic update
AxesRaw_TypeDef MAG_Value;                                                      //!< Prepare data for BLUEST magnetometer characteristic update
int32_t PRESS_Value;                                                            //!< Prepare data for BLUEST pressure characteristic update
int32_t TEMP_Value;                                                             //!< Prepare data for BLUEST temperature characteristic update
uint16_t BATT_V_Value;                                                          //!< Prepare data for BLUEST battery voltage characteristic update
uint16_t BATT_I_Value;                                                          //!< Prepare data for BLUEST battery current characteristic update
uint16_t BATT_SOC_Value;                                                        //!< Prepare data for BLUEST battery state of charge characteristic update
uint8_t BATT_STATUS_Value;                                                      //!< Prepare data for BLUEST battery status characteristic update
TIM_HandleTypeDef processTimHandle;                                             //!<Process timer handle
uint32_t TkUser;

/**
 * @}
 */

/** @defgroup WeSU_MAIN_Private_Define WeSU_MAIN Private Define
 * @{
 */

#define STRING_FIRMWARE_VERSION { 'F', 'W', '_', 'V', 'E', 'R', HEXH(HIGH_BYTE(FIRMWARE_VERSION)), HEXL(HIGH_BYTE(FIRMWARE_VERSION)), HEXH(LOW_BYTE(FIRMWARE_VERSION)), HEXL(LOW_BYTE(FIRMWARE_VERSION)), 'W', 'e', 'S', 'U', '_', 'D', 'E', 'M', 'O', 0x00 }
//!< firmware version description
/**
 * @}
 */

             
/** @addtogroup WeSU_MAIN_Exported_Constants     WeSU_MAIN Exported Constants               
  * @{
  */
#if defined (__IAR_SYSTEMS_ICC__)
        const char nFwVer[]@(0x08000000+VECT_TAB_OFFSET+0x200) = STRING_FIRMWARE_VERSION;
        _Pragma("required=nFwVer")
#elif defined (__CC_ARM)
#pragma Ono_remove_unused_constdata
        const char nFwVer[] = STRING_FIRMWARE_VERSION;
#elif defined (__GNUC__)
        const char nFwVer[] = STRING_FIRMWARE_VERSION;
#else
      #error "Toolchain not supported"
#endif

/**
 * @}
 */
        
/** @addtogroup WeSU_MAIN_Exported_Functions
 * @{
 */

/**
 * @brief  Splits a float into two integer values (Integer/Decimal)
 * @param in: Input float value
 * @param out_int: Output integer part
 * @param out_dec: Output decimal part
 * @param dec_prec: number of digits for decimal part
 * @retval None
 */
void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec)
{
    *out_int = (int32_t)in;
    in = in - (float)(*out_int);
    *out_dec = (int32_t)trunc(in*pow(10,dec_prec));
}

/**
 * @}
 */         
        
        
/** @defgroup WeSU_MAIN_Private_Functions       WeSU_MAIN Private Functions
 * @{
 */
        
int main(void);  
void SystemClock_Config_MSI_2MHz(void);
void SystemClock_Config_MSI_4MHz(void);
void SystemClock_Config_HSI_12MHz(void);
void SystemClock_Config_HSE_18MHz(void);
void SystemClock_Config_HSE_24MHz(void);
void SystemClock_Config_HSI_32MHz(void);
void SystemClock_Config_RTC_HSE32MHz(void);

void (*SystemClock_Config_APP_STARTUP)  (void) = SystemClock_Config_RTC_HSE32MHz;       //!< pointer to startup clock configuration function

void Error_Handler(void);                                                        //!< Error handler function

void HAL_DelayUntil(uint32_t Delay, uint32_t *nPreviousWakeTime);

static void RTC_TimeStampConfigDefault(void);
static void RTC_Config(void);

static inline void User_Init();
static inline void User_Process(void);
static inline void Read_Sensors();
static inline void CheckSensorsCallbacks();
static inline void ManageBleConnection();
#if USE_TIMER 
void MX_TIM_process_Init(void);
#endif

/**
 * @brief  User Initialization
 * @retval None
 */
void User_Init()
{
  RegistersInitialization();
  
  /* Initialize WatchDog */
#if USE_IWDG
  if(!LowPowerFlag)
  {
    BSP_WD_Init();
  }
#endif //USE_IWDG
  
  /* Initialize LEDs */
  BSP_LED_InitGpio(LED);
  BSP_LED_InitGpio(RED_LED);
  APP_BSP_LED_On(LED);
  
  /* Initialize button */
  PRINTF("BSP_PB_Init\r\n");
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  
  /* Initialize RTC */
#if USE_RTC
  RTC_Config();
  WesuRtcConfig((pConfigRtcDefault)RTC_TimeStampConfigDefault);
  WeSU_GetDateTime((rtc_time_t *)&(SESSION_REG(tm_hour)));
#endif //USE_RTC
  
  /* Initialize Usb power */
  DBG_PRINTF_MAIN(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "BSP_PwrUsbMonitor_Init\r\n");
  BSP_PwrUsbMonitor_Init(USB_PWR_MODE_EXTI);
  UsbConnected = BSP_PwrUsbMonitor_GetState();
  
  DBG_PRINTF_MAIN(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nWeSU_Hal Sys startup\r\n");
  
  DBG_PRINTF_MAIN(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nBluenrgInitialization\r\n");
  BluenrgInitialization();
  
  if(SESSION_REG(bRebootSensors)==0)
  {
    SensorsConfigInit();
    if((SESSION_REG(bRebootSensors) & (ACC_ENABLED_MASK_|GYRO_ENABLED_MASK_)) != (ACC_ENABLED_MASK_|GYRO_ENABLED_MASK_))
    {
      extern void ProcDbgRebootAccGyroHidden(char *pcInStr);
      ProcDbgRebootAccGyroHidden("");
      SESSION_REG(bRebootSensors) |= (ACC_ENABLED_MASK_|GYRO_ENABLED_MASK_);
    }
    if((SESSION_REG(bRebootSensors) & (MAG_ENABLED_MASK_)) != (MAG_ENABLED_MASK_))
    {
      extern void ProcDbgRebootMagHidden(char *pcInStr);
      ProcDbgRebootMagHidden("");
      SESSION_REG(bRebootSensors) |= (MAG_ENABLED_MASK_);
    }
    if((SESSION_REG(bRebootSensors) & (PRESS_ENABLED_MASK_|TEMP_ENABLED_MASK_)) != (PRESS_ENABLED_MASK_|TEMP_ENABLED_MASK_))
    {
      extern void ProcDbgRebootPressTempHidden(char *pcInStr);
      ProcDbgRebootPressTempHidden("");
      SESSION_REG(bRebootSensors) |= (PRESS_ENABLED_MASK_|TEMP_ENABLED_MASK_);
    }
  }

  DBG_PRINTF_MAIN(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nSensorsConfigFullRun\r\n");
  SensorsConfigFullRun();
  
  DBG_PRINTF_MAIN(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nInit_PWR_MGMT\r\n");
  Init_PWR_MGMT();
  
  SUSPEND_SENSORS_READ_ON_INTERRUPT();
  uint8_t status = 0;
  if(BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status) == COMPONENT_OK && status == 1)
  {
    SensorAxes_t x;
    BSP_ACCELERO_Get_Axes(ACCELERO_handle, &x);
    ACC_READ.AXIS_X = x.AXIS_X;
    ACC_READ.AXIS_Y = x.AXIS_Y;
    ACC_READ.AXIS_Z = x.AXIS_Z;
  }
  if(BSP_GYRO_IsInitialized(GYRO_handle, &status) == COMPONENT_OK && status == 1)
  {
    SensorAxes_t x;
    BSP_GYRO_Get_Axes(GYRO_handle, &x);
    GYRO_READ.AXIS_X = x.AXIS_X;
    GYRO_READ.AXIS_Y = x.AXIS_Y;
    GYRO_READ.AXIS_Z = x.AXIS_Z;
  }
  
  if(BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) == COMPONENT_OK && status == 1)
  {
    SensorAxes_t x;
    BSP_MAGNETO_Get_Axes(MAGNETO_handle, &x);
    MAG_READ.AXIS_X = x.AXIS_X;
    MAG_READ.AXIS_Y = x.AXIS_Y;
    MAG_READ.AXIS_Z = x.AXIS_Z;
  }
  
  if(BSP_PRESSURE_IsInitialized(PRESSURE_handle, &status) == COMPONENT_OK && status == 1)
  {
    float P_Value;     
    BSP_PRESSURE_Get_Press(PRESSURE_handle, &P_Value);
    int32_t decPart, intPart;
    floatToInt(P_Value, &intPart, &decPart, 2);
    PRESS_READ = intPart*100+decPart;
  }
  
  if(BSP_TEMPERATURE_IsInitialized(TEMPERATURE_handle, &status) == COMPONENT_OK && status == 1)
  {
    float T_Value;
    BSP_TEMPERATURE_Get_Temp(TEMPERATURE_handle, &T_Value);
    int32_t decPart, intPart;
    floatToInt(T_Value, &intPart, &decPart, 1);
    TEMP_READ = intPart*10+decPart;
  }
  RESUME_SENSORS_READ_ON_INTERRUPT();

  DBG_PRINTF_MAIN(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nAlgorithms_Init\r\n");
  Algorithms_Init(1);
  uint16_t nTmpLedReg = SESSION_REG(nLedControlMask);
  BSP_SESSIONREGS_WRITE_WACTION(LED_CONFIG_REG,(uint8_t*)&nTmpLedReg,1);
}


/**
  * @brief  Main program.
  *  You can use ST WeSU APP on Android or iOS device to display sensor and algorithms data on customized plot or DEMO.
  *  Warning for Android users: activate the "clear device cache" option on the Android APP Menu if you switch the firmware 
  *  between STEVAL-WESU1 demostration and examples
  * @retval None
  */
int main(void)
{
  HAL_Init();
  
  /* Configure the System clock */
  SystemClock_Config_APP_STARTUP = SystemClock_Config_RTC_HSE32MHz;
  SystemClock_Config_APP_STARTUP();
  
  /* Configure System debug */
  DebugConfiguration();
  
  /* Init system */
  User_Init();
  
  DBG_PRINTF_MAIN(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nStart MAIN LOOP\r\nWESU_FW %s\r\n", nFwVer);
  
  if(SESSION_REG(nWakeSource) & SYSTEM_STARTUP_FLAG_GO_LP)
  {
    DBG_PRINTF_MAIN(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nSYSTEM_STARTUP_FLAG_GO_LP flag is set.\r\n");
  }
  
#if USE_TIMER 
  DBG_PRINTF_MAIN(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nMX_TIM_process_Init()\r\n");
  MX_TIM_process_Init();
#endif
  
  /* Infinite loop */
  while (1)
  {
    
#if (USE_TIMER == 0)
    HCI_Process();
#endif  
    
    static uint32_t t=0;
    if(HAL_GetTick()>=(t+(1000 / (SESSION_REG(nTimerFrequency)))))
    {
      t=HAL_GetTick();
#if USE_TIMER
      if(USE_FX_MAG_CALIBRATION)
      {
        FxMagCalibration();
      }
#else //USE_TIMER
      {
        TkUser=HAL_GetTick();
        User_Process();
#if USE_IWDG
        BSP_WD_Refresh();
#endif //USE_IWDG
      }
#endif //USE_TIMER
    }
    
    HAL_Delay(1);
  }     //While
}

/**
 * @brief Read_Sensors function reads the values of MEMS sensors
 * @retval none
 */
void Read_Sensors()
{
   if(READ_SENSORS_ACC_ENABLED())
  { 
    SensorAxes_t x;
    BSP_ACCELERO_Get_Axes(ACCELERO_handle, &x);
    ACC_READ.AXIS_X = x.AXIS_X;
    ACC_READ.AXIS_Y = x.AXIS_Y;
    ACC_READ.AXIS_Z = x.AXIS_Z;
  }
  
  if(READ_SENSORS_GYRO_ENABLED())
  { 
    SensorAxes_t x;
    BSP_GYRO_Get_Axes(GYRO_handle, &x);
    GYRO_READ.AXIS_X = x.AXIS_X;
    GYRO_READ.AXIS_Y = x.AXIS_Y;
    GYRO_READ.AXIS_Z = x.AXIS_Z;
  }
  
  if(READ_SENSORS_MAG_ENABLED())
  { 
    SensorAxes_t x;
    BSP_MAGNETO_Get_Axes(MAGNETO_handle, &x);
    MAG_READ.AXIS_X = x.AXIS_X;
    MAG_READ.AXIS_Y = x.AXIS_Y;
    MAG_READ.AXIS_Z = x.AXIS_Z;
  }
  
  if(READ_SENSORS_PRESS_ENABLED())
  {
    float P_Value;     
    BSP_PRESSURE_Get_Press(PRESSURE_handle, &P_Value);
    int32_t decPart, intPart;
    floatToInt(P_Value, &intPart, &decPart, 2);
    PRESS_READ = intPart*100+decPart;
  }
  
  if(READ_SENSORS_TEMP_ENABLED())
  {
    float T_Value;     
    BSP_TEMPERATURE_Get_Temp(TEMPERATURE_handle, &T_Value);
    int32_t decPart, intPart;
    floatToInt(T_Value, &intPart, &decPart, 1);
    TEMP_READ = intPart*10+decPart;
  } 
  if(READ_SENSORS_FUEL_GAUGE_ENABLED())
  {
#if USE_STC3115
    {
      static uint32_t t=0;
      if(HAL_GetTick()>=(t+FUEL_GAUGE_EXEC_INTERVAL))
      {
        t=HAL_GetTick();
        {
          SESSION_REG(nReadPower)++;
          if (BSP_GG_Task() == GG_OK)
          { 
            BATT_V_READ         = BSP_GG_GetVoltage();
            BATT_I_READ         = BSP_GG_GetCurrent();
            BATT_SOC_READ       = BSP_GG_GetSOC();
            
            DBG_PRINTF_PWR_MGNT(WeSU_DEBUG_SEVERITY_VERBOSE, "\r\nSTC31xx read:");
            DBG_PRINTF_PWR_MGNT(WeSU_DEBUG_SEVERITY_VERBOSE, "charge %d.%d%%\r\n", SESSION_REG(nChargepercent)/10,SESSION_REG(nChargepercent)%10);
            DBG_PRINTF_PWR_MGNT(WeSU_DEBUG_SEVERITY_VERBOSE, "voltage %d.%dV\r\n", (int32_t)(SESSION_REG(fBatteryVolt))/1000,(uint32_t)(SESSION_REG(fBatteryVolt))%1000);
            DBG_PRINTF_PWR_MGNT(WeSU_DEBUG_SEVERITY_VERBOSE, "current %dmA\r\n", (int32_t)(SESSION_REG(fCurrentuA)));  
            
          }
          else /* GG_TASK Error */
          { 
            DBG_PRINTF_PWR_MGNT(WeSU_DEBUG_SEVERITY_VERBOSE, "\r\nSTC31xx read FAIL\r\n");
          }
        } 
      }
    }
#endif //USE_STC3115    
    BATT_STATUS_READ    = BSP_CHARGER_GetState();
    BSP_BatteryChargerTask();
  }
}

void UsbMonitorCallback(uint8_t usbstate)
{
  if(usbstate != USB_CABLE_NOT_CONNECTED)
  {
    SESSION_REG(nLPFunction) = POWER_MODE_ON_USB_CABLE;
    SWITCH_LED(LED_WHITE);
  }
  else
  {
    SESSION_REG(nLPFunction) = POWER_MODE_ON_BATTERY;
    SWITCH_LED(LED_WHITE);
  }
}

void LowBatteryCallback(uint8_t lowbattstate, uint16_t battsoc)
{
  if(lowbattstate == BATTERY_OK)
  {
    SESSION_REG(nLPFunction) = POWER_MODE_ON_BATTERY;
    SWITCH_LED(LED_WHITE);
  }
  if(lowbattstate == BATTERY_LOW)
  {
    SESSION_REG(nLPFunction) = POWER_MODE_ON_LOW_BATTERY;
    SWITCH_LED(LED_WHITE_RED);
  }
  if(lowbattstate == BATTERY_VERY_LOW)
  {
    SESSION_REG(nLPFunction) = POWER_MODE_ON_VERY_LOW_BATTERY;
    SWITCH_LED(LED_RED);
  }
}

void CheckSensorsCallbacks()
{
  //check usb cable connection and trigger related callback
  uint8_t bnewUsbCableCondition = (BSP_PwrUsbMonitor_GetState() != USB_CABLE_NOT_CONNECTED);
  if(bnewUsbCableCondition != SESSION_REG(UsbCableCondition))
  {
    SESSION_REG(UsbCableCondition) = bnewUsbCableCondition;
#if USE_USB_MONITOR_CALLBACK
    UsbMonitorCallback(bnewUsbCableCondition);
#endif //USE_USB_MONITOR_CALLBACK
  }
  
  //check battery level and trigger related callback
  if(BATT_V_Value != 0)
  {
    uint8_t bNewLowBatteryStatus = (BATT_SOC_Value <= *BATTERY_VERY_LOW_THRESHOLD_REGADDR)?BATTERY_VERY_LOW:(BATT_SOC_Value <= *BATTERY_LOW_THRESHOLD_REGADDR)?BATTERY_LOW:BATTERY_OK;
    if(bNewLowBatteryStatus != SESSION_REG(LowBatteryStatus))
    {
      SESSION_REG(LowBatteryStatus) = bNewLowBatteryStatus;
#if USE_LOW_BATTERY_CALLBACK
      LowBatteryCallback(bNewLowBatteryStatus, BATT_SOC_Value);
#endif //USE_LOW_BATTERY_CALLBACK
    }
  }
}

/**
 * @brief BLE Connection Management
 * @retval none
 */
void ManageBleConnection()
{
  if(set_connectable)
  {
    uint16_t ai = *BLE_ADV_INTV_RUN_REGADDR;
    
    if(AdvIntvChange(0,0,(uint8_t*)BLE_ADV_INTV_RUN_REGADDR))
    {
      //default values
      ai=0x0020;
    }
    
    DBG_PRINTF_MAIN(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "setConnectable()\r\n");
    setConnectable(ai, ai, 0);
    set_connectable = FALSE;
  }
  
  UpdateConnectionParameters();
  
  if(bIsConnected)
  {
    nLastConnectionTick = HAL_GetTick();
    TimeStamp++;
    //      TimeStamp = HAL_GetTick()&0xFFFF;
    
    if((READ_SENSORS_ACC_ENABLED() || READ_SENSORS_GYRO_ENABLED() || READ_SENSORS_MAG_ENABLED()) && (OUTPUT_ON_BLE_MEMS_ENABLED())) //if(OUTPUT_ON_BLE_ACC_ENABLED() || OUTPUT_ON_BLE_GYRO_ENABLED() || OUTPUT_ON_BLE_MAG_ENABLED())
    {
      if(CHECK_SUBSAMPLING(nRawMotionSubSampl))
      {
        ACC_Value.AXIS_X = (int16_t)((float)ACC_READ.AXIS_X/APP_ACC_SENSITIVITY);
        ACC_Value.AXIS_Y = (int16_t)((float)ACC_READ.AXIS_Y/APP_ACC_SENSITIVITY);
        ACC_Value.AXIS_Z = (int16_t)((float)ACC_READ.AXIS_Z/APP_ACC_SENSITIVITY);
        GYR_Value.AXIS_X = (int16_t)((float)GYRO_READ.AXIS_X/APP_GYRO_SENSITIVITY);
        GYR_Value.AXIS_Y = (int16_t)((float)GYRO_READ.AXIS_Y/APP_GYRO_SENSITIVITY);
        GYR_Value.AXIS_Z = (int16_t)((float)GYRO_READ.AXIS_Z/APP_GYRO_SENSITIVITY);
        MAG_Value.AXIS_X = (int16_t)((float)MAG_READ.AXIS_X/APP_MAG_SENSITIVITY);
        MAG_Value.AXIS_Y = (int16_t)((float)MAG_READ.AXIS_Y/APP_MAG_SENSITIVITY);
        MAG_Value.AXIS_Z = (int16_t)((float)MAG_READ.AXIS_Z/APP_MAG_SENSITIVITY);
        if(AccGyroMag_Update(&ACC_Value, &GYR_Value, &MAG_Value) != BLE_STATUS_SUCCESS)
        {
          SESSION_REG(nErrorsBlueNRGRwMot)++;
          SET_BNRG_ERROR_FLAG();
        }
      }
    }
    
    if(READ_SENSORS_FUEL_GAUGE_ENABLED() && OUTPUT_ON_BLE_FUEL_GAUGE_ENABLED())
    {
      BATT_V_Value      = (uint16_t) BATT_V_READ;
      BATT_I_Value      = (uint16_t) BATT_I_READ;
      BATT_SOC_Value    = (uint16_t) BATT_SOC_READ;
      BATT_STATUS_Value = (uint8_t) BATT_STATUS_READ;
      if(CHECK_SUBSAMPLING(nPwrSubSampl))
      {
        if(Pwr_Update(BATT_SOC_Value, BATT_V_Value, BATT_I_Value, BATT_STATUS_Value) != BLE_STATUS_SUCCESS)
        {
          SESSION_REG(nErrorsBlueNRGPwr)++;
          SET_BNRG_ERROR_FLAG();
        }
      }
    }
    
    if(READ_SENSORS_TEMP_ENABLED() && OUTPUT_ON_BLE_TEMP_ENABLED())
    {
      if(CHECK_SUBSAMPLING(nTemperatureSubSampl))
      {
        TEMP_Value = (int32_t)(TEMP_READ);
        if(Temp_Update(TEMP_Value))
        {
          SET_BNRG_ERROR_FLAG();
        }
      }
    }
    
    if(READ_SENSORS_PRESS_ENABLED() && OUTPUT_ON_BLE_PRESS_ENABLED())
    {
      if(CHECK_SUBSAMPLING(nPressSubSampl))
      {
        PRESS_Value = (int32_t)(PRESS_READ);
        extern uint32_t ConnectionPres;
        if(ConnectionPres)
        if(Press_Update(PRESS_Value) != BLE_STATUS_SUCCESS)
        {
          SESSION_REG(nErrorsBlueNRGPrs)++;
          SET_BNRG_ERROR_FLAG();
        }
      }
    }
    
#if USE_CUSTOM_ALGORITHMFX    
    if(PROCESS_MOTION_FX_ENABLED() && OUTPUT_ON_BLE_MOTION_FX_ENABLED()){
      if(CHECK_SUBSAMPLING(nMotionFxSubSampl))
      {
        CounterFX++;
        osxMFX_output *MotionFX_Engine_Out = MotionFX_manager_getDataOUT();
    int32_t QuaternionNumber = (CounterFX>SEND_N_QUATERNIONS) ? (SEND_N_QUATERNIONS-1) : (CounterFX-1);
        if(MotionFX_Engine_Out->quaternion_9X[3] < 0){
              quat_axes[QuaternionNumber].AXIS_X = (int)(MotionFX_Engine_Out->quaternion_9X[0] * (-10000));
              quat_axes[QuaternionNumber].AXIS_Y = (int)(MotionFX_Engine_Out->quaternion_9X[1] * (-10000));
              quat_axes[QuaternionNumber].AXIS_Z = (int)(MotionFX_Engine_Out->quaternion_9X[2] * (-10000));
           } else {
              quat_axes[QuaternionNumber].AXIS_X = (int)(MotionFX_Engine_Out->quaternion_9X[0] * 10000);
              quat_axes[QuaternionNumber].AXIS_Y = (int)(MotionFX_Engine_Out->quaternion_9X[1] * 10000);
              quat_axes[QuaternionNumber].AXIS_Z = (int)(MotionFX_Engine_Out->quaternion_9X[2] * 10000);     
           }
        if(CounterFX==3){
         if(Quat_Update(quat_axes) != BLE_STATUS_SUCCESS) 
        {
          SESSION_REG(nErrorsBlueNRGFX)++;
          SET_BNRG_ERROR_FLAG();
        }
        CounterFX=0;
        }
      }
     }
    #endif //USE_CUSTOM_ALGORITHMFX

#if USE_CUSTOM_ALGORITHM1
    if(PROCESS_MOTION_AR_ENABLED() && OUTPUT_ON_BLE_MOTION_AR_ENABLED())
    {
      if(CHECK_SUBSAMPLING(nMotionArSubSampl))
      {
        {
          if(Algorithm1_Update(SESSION_REG(xMAR_Value)) != BLE_STATUS_SUCCESS)
          {
            SESSION_REG(nErrorsBlueNRGRwMot)++;
            SET_BNRG_ERROR_FLAG();
          }
        }
      }
    }
#endif //USE_CUSTOM_ALGORITHM1

#if USE_CUSTOM_ALGORITHM2
    if(PROCESS_MOTION_CP_ENABLED() && OUTPUT_ON_BLE_MOTION_CP_ENABLED())
    {
      if(CHECK_SUBSAMPLING(nMotionCpSubSampl))
      {
        {
          if(Algorithm2_Update(SESSION_REG(xMCP_Value)) != BLE_STATUS_SUCCESS)
          {
            SESSION_REG(nErrorsBlueNRGRwMot)++;
            SET_BNRG_ERROR_FLAG();
          }
        }
      }
    }
#endif //USE_CUSTOM_ALGORITHM2
    
#if USE_CUSTOM_ALGORITHM3
    if(PROCESS_PEDOMETER_ENABLED() && OUTPUT_ON_BLE_PEDOMETER_ENABLED())
    {
      if(CHECK_SUBSAMPLING(nPedometerSubSampl))
      {
        {
          if(Algorithm3_Update(SESSION_REG(nPedoStepCounter),0) != BLE_STATUS_SUCCESS) 
          {
            SESSION_REG(nErrorsBlueNRGRwMot)++;
            SET_BNRG_ERROR_FLAG();
          }
        }
      }
    }
#endif //USE_CUSTOM_ALGORITHM3
    
#if USE_CUSTOM_ALGORITHM4
    
#endif //USE_CUSTOM_ALGORITHM4    
            
#if USE_CUSTOM_ACCEVENT
    if((PROCESS_ACCEVENT_ENABLED() && OUTPUT_ON_BLE_ACCEVENT_ENABLED()) || \
    (ACCEVENT_FEATURE_ENABLED(ACCEVENT_FEATURE_PEDOMETER)))
    {
      static uint32_t nPrevPedoStepCounter = 0xFFFFFFFF;
      if(CHECK_SUBSAMPLING(nAccEventSubSampl))
          if(SESSION_REG(nAccEventStatus) || (nPrevPedoStepCounter != SESSION_REG(nPedoStepCounter)) )
          {
            {
              if(AccEvent_Update(SESSION_REG(nAccEventStatus),SESSION_REG(nPedoStepCounter)) != BLE_STATUS_SUCCESS)
              {
                SESSION_REG(nErrorsBlueNRGRwMot)++;
                SET_BNRG_ERROR_FLAG();
              }
              SESSION_REG(nAccEventStatus)= 0; //reset events after notification
              nPrevPedoStepCounter = SESSION_REG(nPedoStepCounter);
            }
          }
    }
#endif //USE_CUSTOM_ACCEVENT

#if USE_CUSTOM_GP_ALGORITHM
    if(PROCESS_GP_ALGORITHM_ENABLED() && OUTPUT_ON_BLE_GP_ALGORITHM_ENABLED())
    {
      if(CHECK_SUBSAMPLING(nGP_ALGORITHMSubSampl))
      {
        {
          if(GP_ALGORITHM_Update(SESSION_REG(nGP_ALGORITHMStatus)) != BLE_STATUS_SUCCESS)
          {
            SESSION_REG(nErrorsBlueNRGRwMot)++;
            SET_BNRG_ERROR_FLAG();
          }
        }
      }
    }
#endif //USE_CUSTOM_GP_ALGORITHM
    
  }// if(bIsConnected)
  else
  {
    //not connected
  }

}

/**
 * @brief Main User Process
 * @retval none
 */
void User_Process(void)
{
  SESSION_REG(nTicks) = HAL_GetTick();
  SESSION_REG(nUserProcessCount)++;
#if USE_RTC
  WeSU_GetDateTime((rtc_time_t *)&(SESSION_REG(tm_hour)));
#endif //USE_RTC

    static uint32_t ticksPrintSysParams=0;
    if(HAL_GetTick()>=ticksPrintSysParams+30000)
    {
      ticksPrintSysParams=HAL_GetTick();
    PrintSystemParameters();
  }
  
  WesuSysChangeRunMode(WESU_SYS_MODE_GO_IN_RUN);
  
  LedManageStart(SESSION_REG(nLedBlinkMode));
  
  if (SESSION_REG(nTimerDrdyCtrlMask) != (SENSORS_TIMER_DRDY_CTRL_ENABLED_AND_RUNNING))
  {     
    Algorithms_Hw_Process();
    if(LsmXGInt1Ready)
      {
        Algorithms_Hw_Process_INT1();
        LsmXGInt1Ready = RESET;
      }
    Read_Sensors();     
  }
  
  CheckSensorsCallbacks();
  
  Algorithms_Process();
  
  ManageBleConnection();
  
  ManageTerminal();
  
  LedManageEnd(SESSION_REG(nLedBlinkMode));
  
  if(ButtonPressed)
  {
    PRINTF("ButtonPressed\r\n");
    ButtonPressed = FALSE;
    SESSION_REG(nLPFunction) = WESU_GET_BUTTON_PRESS_MODE;
  }
  if(LowPowerFlag)
  {
    PRINTF("LowPowerFlag\r\n");
    SESSION_REG(nLPFunction) = LowPowerFlag;
    LowPowerFlag = FALSE;
  }
  if(BSP_PB_GetState(BUTTON_USER))
  {
    PRINTF("BSP_PB_GetState\r\n");
    APP_BSP_LED_On(LED);
    ButtonPressed = FALSE;
    SESSION_REG(nLPFunction) = WESU_GET_BUTTON_PRESS_MODE;
  }
  
  if(!bIsConnected)
  {
    if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_CONN_ONLY)
    {
      if(HAL_GetTick() > nLastConnectionTick + DELAY_BEFORE_LOWPOWER_CONN_ONLY)
      {
        nLastConnectionTick = HAL_GetTick();
        APP_LedSmoothRampDown(30);
        SESSION_REG(nLPFunction) = WESU_GET_CONN_ONLY_MODE;
        PRINTF("DELAY_BEFORE_LOWPOWER_CONN_ONLY\r\n");
      }
    }
    if(*AUTOSLEEP_TIME_REGADDR)
    {
      if(HAL_GetTick() > nLastConnectionTick + (*AUTOSLEEP_TIME_REGADDR))
      {
        nLastConnectionTick = HAL_GetTick();
        APP_LedSmoothRampDown(30);
        SESSION_REG(nLPFunction) = WESU_GET_AUTOSLEEP_MODE;
        PRINTF("AUTOSLEEP_TIME_REGADDR\r\n");
      }
    }
  }
  
  WesuSysChangeRunMode(WESU_SYS_MODE_GO_IN_STOP);
  
  if(SESSION_REG(bLP))
  {
    if(bWdgConfigured)
    {
      if(SESSION_REG(nLPMask) & (LPCFG_MCU_ENTER_STOP_RESET | LPCFG_MCU_ENTER_STOP_W_BLE | LPCFG_MCU_ENTER_STOP_W_BLE_RESET) )
      {
        SESSION_REG(nWakeSource) = SYSTEM_STARTUP_FLAG_CLOSE_SESSION | SYSTEM_STARTUP_FLAG_GO_LP;
        WESU_SYSTEM_RESET();
      }
    }
    
    PrepareLowPowerMode();
    
    if(SESSION_REG(nLPMask)&LPCFG_MCU_ENTER_SLEEP)
    {
      BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
      SESSION_REG(nWakeSource) = 0;
      
      int32_t t_sleep;
      t_sleep = HAL_GetTick() - TkUser;
      
      if (t_sleep < 1000)
      {
        HAL_SYSTICK_Config((1000 - t_sleep)*(SystemCoreClock /1000));
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
        while((t_sleep--)) {HAL_IncTick();}
      } 

      HAL_SYSTICK_Config(SystemCoreClock/1000);

      SESSION_REG(bLP) = 0;
    }
    
    if(SESSION_REG(nLPMask)&LPCFG_MCU_ENTER_LP_SLEEP)
    {
      BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
      SESSION_REG(nWakeSource) = 0;
      
      int32_t t_sleep;
      t_sleep = HAL_GetTick() - TkUser;
      
      if (t_sleep < 1000)
      {
        HAL_SYSTICK_Config((1000 - t_sleep)*(SystemCoreClock /1000));
        HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
        while((t_sleep--)) {HAL_IncTick();}
      } 
      
      HAL_SYSTICK_Config((SystemCoreClock /1000));
      
      SESSION_REG(bLP) = 0;
    }
    
  
    if(SESSION_REG(nLPMask)&LPCFG_MCU_ENTER_STOP_RESET)
    {
      PRINTF("LPCFG_MCU_ENTER_STOP_RESET\r\n");
      PRINTF("MCU LOW POWER MODE\r\n");
      BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
      SESSION_REG(nWakeSource) = SYSTEM_STARTUP_FLAG_CLOSE_SESSION | SYSTEM_STARTUP_FLAG_GO_LP;
#if USE_IWDG
      if(bWdgConfigured)WESU_SYSTEM_RESET();
#endif //USE_IWDG
      HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
      WESU_SYSTEM_RESET();
    }
    if(SESSION_REG(nLPMask)&LPCFG_MCU_ENTER_STOP_W_BLE)
    {
      PRINTF("LPCFG_MCU_ENTER_STOP_W_BLE\r\n");
      PRINTF("MCU LOW POWER MODE\r\n");
      BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
      BNRG_IRQ_CLK_ENABLE();
      set_irq_as_input();
      SESSION_REG(nWakeSource) = SYSTEM_STARTUP_FLAG_CLOSE_SESSION | SYSTEM_STARTUP_FLAG_GO_LP;
#if USE_IWDG
      if(bWdgConfigured)WESU_SYSTEM_RESET();
#endif //USE_IWDG
      HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
      if(SESSION_REG(nWakeSource) & BUTTON_GPIO_PIN)
      {
#if RESET_AFTER_BUTTON_WAKEUP | USE_IWDG
        SESSION_REG(nWakeSource) &= ~SYSTEM_STARTUP_FLAG_GO_LP;
        WESU_SYSTEM_RESET();
#endif //RESET_AFTER_BUTTON_WAKEUP
      }
      LOAD_RUNNING_nLPFunction();
      HCI_Process();
      if(!bIsConnected)
      {
        set_connectable = 1;
      }
    }
    if(SESSION_REG(nLPMask)&LPCFG_MCU_ENTER_STOP_W_BLE_RESET)
    {
      PRINTF("LPCFG_MCU_ENTER_STOP_W_BLE_RESET\r\n");
      PRINTF("MCU LOW POWER MODE\r\n");
      BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
      BNRG_IRQ_CLK_ENABLE();
      set_irq_as_input();
      SESSION_REG(nWakeSource) = SYSTEM_STARTUP_FLAG_CLOSE_SESSION | SYSTEM_STARTUP_FLAG_GO_LP;
#if USE_IWDG
      if(bWdgConfigured)WESU_SYSTEM_RESET();
#endif //USE_IWDG
      HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
      WESU_SYSTEM_RESET();
    }
    if(SESSION_REG(bLP) != 0)
    {
      RestoreRunMode();
      nLastConnectionTick = HAL_GetTick();
    }
  }
  SESSION_REG(nTicks) = HAL_GetTick();
}

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config_HSI_12MHz(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  HAL_RCC_DeInit();
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

/** System Clock Configuration
*/
void SystemClock_Config_HSE_18MHz(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  HAL_RCC_DeInit();
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

/** System Clock Configuration
*/
void SystemClock_Config_HSE_24MHz(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

/** System Clock Configuration
*/
void SystemClock_Config_MSI_2MHz(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  HAL_RCC_DeInit();
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}


/** System Clock Configuration
*/
void SystemClock_Config_MSI_4MHz(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

/** System Clock Configuration
*/
void SystemClock_Config_RTC_HSE32MHz(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
#if USE_RTC
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
#endif //USE_RTC

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  HAL_RCC_DeInit();
  
#if USE_RTC
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
#else //USE_RTC
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
#endif //USE_RTC
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
#if USE_RTC
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
#endif //USE_RTC
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

#if USE_RTC
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
#endif //USE_RTC
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLLMUL                         = 6
  *            PLLDIV                         = 3
  *            Flash Latency(WS)              = 1
  * @retval None
  */
void SystemClock_Config_HSI_32MHz(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  
  /**
  * Enable clock on PWR block
  * This is used to setup registers when entering low power mode
  */
  __PWR_CLK_ENABLE();
  
  /**
   * Set voltage scaling range
   * The voltage scaling allows optimizing the power consumption when the device is 
   * clocked below the maximum system frequency, to update the voltage scaling value 
   * regarding system frequency refer to product datasheet.
   */
#if (HCLK_32MHZ == 1)
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};
  
  /**
   *  Enable HSI oscillator and configure the PLL to reach the max system frequency 
   *  (32MHz) when using HSI oscillator as PLL clock source.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6; //USB support
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3; 

  
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /**
  *  Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers.
  *  The SysTick 1 msec interrupt is required for the HAL process (Timeout management); by default
  *  the configuration is done using the HAL_Init() API, and when the system clock configuration
  *  is updated the SysTick configuration will be adjusted by the HAL_RCC_ClockConfig() API.
  */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;

  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
  
#elif (HCLK_24MHZ == 1)

//  RCC_OscInitTypeDef RCC_OscInitStruct;
//  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

#else
/**
  * Reset value is Range 2
  */
  
  /**
  *  Enable HSI oscillator and configure the system at 16MHz
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
  
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /**
  *  Configure the HCLK, PCLK1 and PCLK2 clocks dividers  to get 8Mhz.
  *  The SysTick 1 msec interrupt is required for the HAL process (Timeout management); by default
  *  the configuration is done using the HAL_Init() API, and when the system clock configuration
  *  is updated the SysTick configuration will be adjusted by the HAL_RCC_ClockConfig() API.
  */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
  
#endif /* (HCLK_32MHZ == 1) */
}

/**
 * @brief  Configures the current time and date
 * @retval None
 */
static void RTC_TimeStampConfigDefault(void)
{
  rtc_time_t t;
  memcpy(&t, (rtc_time_t *)RTC_CONF_0_REGADDR, 8);
  switch(t.tm_rtcConf)
  {
  case RTC_AppConfigDefault:
    break;
  case RTC_AppConfigDefaultRunning:
    t.tm_rtcConf = RTC_AppConfigRestored;
    break;
  case RTC_AppConfigRestored:
    break;
  case RTC_AppConfigRestoredRunning:
    t.tm_rtcConf = RTC_AppConfigRestored;
    break;
  case RTC_AppConfigUser:
    t.tm_rtcConf = RTC_AppConfigRestored;
    break;
  case RTC_AppConfigUserRunning:
    t.tm_rtcConf = RTC_AppConfigRestored;
    break;
  case RTC_AppConfigForced:
    t.tm_rtcConf = RTC_AppConfigRestored;
    break;
  default:
    t.tm_rtcConf = RTC_AppConfigInvalid;
    break;
  }

  if(t.tm_rtcConf != RTC_AppConfigInvalid)
  {
    WeSU_SetDateTime(&t);
  }
}

/**
 * @brief  Configures the RTC
 * @retval None
 */
static void RTC_Config(void)
{
  /*##-1- Configure the RTC peripheral #######################################*/
  RtcHandle.Instance = RTC;
  
  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
  - Hour Format    = Format 12
  - Asynch Prediv  = Value according to source clock
  - Synch Prediv   = Value according to source clock
  - OutPut         = Output Disable
  - OutPutPolarity = High Polarity
  - OutPutType     = Open Drain */
  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_12;
  RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSE;
  RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSE;
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  
  if(HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


/**
 * @brief  TIM_process init function.
 * @param  None
 * @retval None
 * @details This function intialize the Timer used to syncronize the user process.
 */
void MX_TIM_process_Init(void)
{
  #define PERIOD_100HZ  9
  #define PERIOD_25HZ  39
  #define PRESCALER_100HZ     31999
  
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  processTimHandle.Instance = TIM_process;
  processTimHandle.Init.Prescaler = PRESCALER_100HZ;
  processTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  processTimHandle.Init.Period = PERIOD_100HZ;
  processTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&processTimHandle);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&processTimHandle, &sClockSourceConfig);

  HAL_TIM_OC_Init(&processTimHandle);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&processTimHandle, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&processTimHandle, &sConfigOC, TIM_CHANNEL_1);
  
  HAL_TIM_Base_Start_IT(&processTimHandle);
}

/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * @note In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where uwTick
  *       is incremented.
  * @note This is a user file implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay)
  {
    WESU_DBG_WFI();
  }
}

/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * @note This is a user file implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @param nPreviousWakeTime: specifies the previous wake-up time of the 'task'
  * @retval None
  */
void HAL_DelayUntil(uint32_t Delay, uint32_t *nPreviousWakeTime)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  
  if(*nPreviousWakeTime + Delay > tickstart)
  {
    while((HAL_GetTick()) < (Delay + *nPreviousWakeTime))
    {
      WESU_DBG_WFI();
    }
  }
  *nPreviousWakeTime += Delay;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

/**
 * @brief  EXTI line detection callback.
 * @param  Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t Pin)
{
  if(SESSION_REG(bLP)){SESSION_REG(nWakeSource) |= Pin;}
  switch(Pin)
  {
    case BNRG_EXTI_PIN:
      if(SESSION_REG(bLP))RestoreRunMode();
      #if (USE_TIMER == 0)
        HCI_Isr();
      #endif 
    break;
  case BUTTON_GPIO_PIN:
    if(SESSION_REG(bLP))break;
    ButtonPressed = TRUE;
    break;
  case USB_PWR_GPIO_PIN:
    if(SESSION_REG(bLP))break;
    UsbConnected = BSP_PwrUsbMonitor_GetState();
    break;
  case IMU_6AXES_INT1_PIN:
    {
      LsmXGInt1Ready = SET;
    }
    break;
    
  case IMU_6AXES_INT2_PIN:     /* set the LSM6DS3 INT2 Ready flag */
    LsmXGInt2Ready = SET;
    if((xCurSmFunc!=WESU_SYS_POWER_UNKNOWN) && (SESSION_REG(nTimerDrdyCtrlMask) == SENSORS_TIMER_DRDY_CTRL_ENABLED_AND_RUNNING))
    {
      Algorithms_Hw_Process();
      if(LsmXGInt1Ready)
      {
        Algorithms_Hw_Process_INT1();
        LsmXGInt1Ready = RESET;
      }
      Read_Sensors();
      FX_Process_High();
      LsmXGInt2Ready = RESET;
    }

    break;
    
  case MAGNETO_INT_PIN:       /* set the  INT_M Ready flag */
      LisIntReady = SET;
    break;
    
  case MAGNETO_DRDY_PIN:      /* set the  DRDY_M Ready flag */
      LsmDrdyMReady = SET;  
    break;
  case GG_INT_PIN:            /* manage gas gauge interrupt */
    GasGaugeInterrupt = SET;
    break;
  default:
      //not handled
//    while(1);
    break;
  }
}

/**
  * @brief  Period elapsed callback 
  * @param  htim: pointer to a TIM_HandleTypeDef structure that contains
  *                the configuration information for TIM module.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{      
  if(htim->Instance==TIM_process)
  {
  #if USE_TIMER 
    HCI_Isr();
    HCI_Process();
    
    TkUser=HAL_GetTick();
    {
      User_Process();
    #if USE_IWDG
        BSP_WD_Refresh();
    #endif //USE_IWDG
    }
  #endif //USE_TIMER
  }
}

/**
 * @}
 */


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  
  }
}
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

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
