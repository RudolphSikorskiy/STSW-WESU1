/**
  ******************************************************************************
  * @file    Sensors_Read/Src/main_examples.c
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
#include "main_examples.h"
#include "BlueST_Protocol_examples.h"
#include "wesu_config_examples.h"
#include "algorithms_examples.h"



/** @addtogroup WeSU_Examples        WeSU Examples
  * @{
  */

/** @addtogroup WeSU_User_Examples               WeSU User Examples
  * @{
  */


/** @addtogroup WeSU_MAIN_Examples               WeSU MAIN Examples
  * @{
  * @brief WeSU DEMO Main
  */

  extern volatile uint8_t set_connectable;
  extern volatile int bIsConnected;


  int32_t pressure;                     //!<   Pressure output            (mbar) (reg64->reg65)
  AxesRawFloat_TypeDef acceleration;    //!<   Accelerometer output       (XXXXXX) (reg66->reg6B)
  AxesRawFloat_TypeDef angular_rate;    //!<   Gyroscope output           (XXXXXX) (reg6C->reg71)
  AxesRawFloat_TypeDef magnetic_field;  //!<   Magnetometer output        (XXXXXX) (reg72->reg77)
  int32_t temperature;                  //!<   Temperature                (XXXXXX) (reg78->reg79)

  uint32_t nReadPower;                  //!<   number of power read (reg56->reg57)
  uint16_t nChargepercent;              //!<  Battery charge percentage (0.1%)
  float fBatteryVolt;                   //!<  Battery Voltage (reg04->reg05)
  float fCurrentuA;                     //!<  Battery Current (uA)(in >0 /out <0) (reg06->reg07)
  uint8_t nPowerMode;                   //!<  LSB: Battery power (charging/discharging/battery/low battery) MSB: RFU

  uint16_t nReadSensorMask;                                                     //!<  Read sensors mask
  uint16_t nProcessFwMask;                                                      //!<  Process algorithms mask

  uint16_t WeSU_DEBUG_SEVERITY_ble = WeSU_DEBUG_SEVERITY_CRITICAL;              //!<  Configured severity level for BLE
  uint16_t WeSU_DEBUG_SEVERITY_usart = WeSU_DEBUG_SEVERITY_VERBOSE;             //!<  Configured severity level for USART

  FlagStatus LpsDataReady=RESET;                                                //!< LPS25HB sensor data-ready flag
  FlagStatus LsmXGInt1Ready=RESET;                                              //!< LSM6DS3 sensor INT1 flag
  FlagStatus LsmXGInt2Ready=RESET;                                              //!< LSM6DS3 sensor INT2 flag
  FlagStatus LisIntReady=RESET;                                                 //!< LIS3MDL sensor INT flag
  FlagStatus LsmDrdyMReady=RESET;                                               //!< LIS3MDL sensor data-ready flag
  FlagStatus GasGaugeInterrupt=RESET;                                           //!< STC3115 interrupt flag

  RTC_HandleTypeDef RtcHandle;                                                  //!< RTC HANDLE

  void *ACCELERO_handle = NULL;                                                 //!< ACCELEROMETER HANDLE
  void *GYRO_handle = NULL;                                                     //!< GYROSCOPE HANDLE
  void *MAGNETO_handle = NULL;                                                  //!< MAGNETOMETER HANDLE
  void *PRESSURE_handle = NULL;                                                 //!< PRESSURE HANDLE
  void *TEMPERATURE_handle = NULL;                                              //!< TEMPERATURE HANDLE

  uint32_t nLastConnectionTick = 0;                                             //!< store last connection tick
  int UsbConnected=0;                                                           //!< usb power connection status


int main(void);
void SystemClock_Config_HSI_32MHz(void);
void SystemClock_Config_RTC_HSE32MHz(void);
void SystemClock_Config_HSI_12MHz(void);
void SystemClock_Config_HSE_18MHz(void);
void SystemClock_Config_MSI_2MHz(void);

void (*SystemClock_Config_APP_STARTUP)  (void) = SystemClock_Config_RTC_HSE32MHz;       //!< pointer to startup clock configuration function

void Error_Handler(void);                                                        //!< Error handler function

void HAL_DelayUntil(uint32_t Delay, uint32_t *nPreviousWakeTime);

static void RTC_TimeStampConfigDefault(void);
static void RTC_Config(void);

static inline void User_Init();
static inline void User_Process(void);
static inline void Read_Sensors();
static inline void ManageBleConnection();
static inline void LoadExamplesMasks();

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

//uint8_t ENABLED_DEBUG_LEVEL_USART = WeSU_DEBUG_SEVERITY_CRITICAL;               //!< Usart messages debug severity
//uint8_t ENABLED_DEBUG_LEVEL_BLE = WeSU_DEBUG_SEVERITY_CRITICAL;                 //!< BlueNRG messages debug severity

uint8_t bDebuggerConnected = 0;                                                 //!< This facility (bDebuggerConnected) enables the output on IAR Terminal I/O

int ButtonPressed=0;                                                            //!< user push button pressed event

AxesRaw_TypeDef ACC_Value;                                                      //!< Prepare data for BLUEST accelerometer characteristic update
AxesRaw_TypeDef GYR_Value;                                                      //!< Prepare data for BLUEST gyroscope characteristic update
AxesRaw_TypeDef MAG_Value;                                                      //!< Prepare data for BLUEST magnetometer characteristic update
int32_t PRESS_Value;                                                            //!< Prepare data for BLUEST pressure characteristic update
int32_t TEMP_Value;                                                             //!< Prepare data for BLUEST temperature characteristic update
uint16_t BATT_V_Value;                                                          //!< Prepare data for BLUEST battery voltage characteristic update
uint16_t BATT_I_Value;                                                          //!< Prepare data for BLUEST battery current characteristic update
uint16_t BATT_SOC_Value;                                                        //!< Prepare data for BLUEST battery state of charge characteristic update
uint8_t BATT_STATUS_Value;                                                      //!< Prepare data for BLUEST battery status characteristic update



#define STRING_FIRMWARE_VERSION { 'F', 'W', '_', 'V', 'E', 'R', HEXH(HIGH_BYTE(FIRMWARE_VERSION)), HEXL(HIGH_BYTE(FIRMWARE_VERSION)), HEXH(LOW_BYTE(FIRMWARE_VERSION)), HEXL(LOW_BYTE(FIRMWARE_VERSION)), 'W', 'e', 'S', 'U', '_', 'A', 'C', 'C', 0x00 }
//!< firmware version description


#if defined (__IAR_SYSTEMS_ICC__)
        const char nFwVer[]@(0x08000000+VECT_TAB_OFFSET+0x200) = STRING_FIRMWARE_VERSION;
        _Pragma("required=nFwVer")
#elif defined (__CC_ARM)
	//__attribute__((section ("RomFixReg")))
#pragma Ono_remove_unused_constdata
        const char nFwVer[] = STRING_FIRMWARE_VERSION;
#elif defined (__GNUC__)
        const char nFwVer[] = STRING_FIRMWARE_VERSION;
#else
      #error "Toolchain not supported"
#endif

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
  SystemClock_Config_APP_STARTUP();

  /* Init system */
  User_Init();

  DBG_PRINTF(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nStart MAIN LOOP\r\nWESU_FW %s\r\n", nFwVer);
  APP_BSP_LED_Off(LED);

  /* Infinite loop */
  while (1)
  {
    HCI_Process();
    {
      static uint32_t t=0;
      if(HAL_GetTick()>=(t+(1000 / (nTimerFrequency))))
      {
        t=HAL_GetTick();
        {
          User_Process();
        }
      }
    }

    HAL_Delay(1);
  }
}

/**
 * @brief  User Initialization
 * @retval None
 */
void User_Init()
{
  LoadExamplesMasks();

  /* Initialize LEDs */
  BSP_LED_InitGpio(LED);
  APP_BSP_LED_On(LED);

  /* Initialize button */
  PRINTF("BSP_PB_Init\r\n");
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize RTC */
  RTC_Config();
  WesuRtcConfig((pConfigRtcDefault)RTC_TimeStampConfigDefault);

  /* Initialize Usb power */
  PRINTF("BSP_PwrUsbMonitor_Init\r\n");
  BSP_PwrUsbMonitor_Init(USB_PWR_MODE_EXTI);
  UsbConnected = BSP_PwrUsbMonitor_GetState();

  DBG_PRINTF(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nWeSU_Hal Sys startup\r\n");

  DBG_PRINTF(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nBluenrgInitialization\r\n");
  BluenrgInitialization();

  DBG_PRINTF(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nSensorsConfigFullRun\r\n");
  SensorsConfig();

  DBG_PRINTF(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nInit_PWR_MGMT\r\n");
  Init_PWR_MGMT();

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

  DBG_PRINTF(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nAlgorithms_Init\r\n");
  Algorithms_Init(1);
  APP_LedSmoothRampUp(50);
  ButtonPressed = 0;
}

/**
 * @brief Main User Process
 * @retval none
 */
void User_Process(void)
{
  {
    static uint32_t ticksPrintSysParams=0;
    if(HAL_GetTick()>=ticksPrintSysParams+30000)
    {
      ticksPrintSysParams=HAL_GetTick();
      {
        //Print system parameters: ticks, rtc time, errors...
        rtc_time_t t;
        WeSU_GetDateTime(&t);
        DBG_PRINTF(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nTicks: %d ", HAL_GetTick());
        DBG_PRINTF(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nDate: %s %.2d-%s-%d",BSP_RTC_GetWeekDayName(t.tm_wday),t.tm_day,BSP_RTC_GetMonthName(t.tm_mon),t.tm_year);
        DBG_PRINTF(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nTime: %.2d:%.2d:%.2d\r\n",t.tm_hour,t.tm_min,t.tm_sec);
      }
    }
  }

  LedManageStart(0);

  Read_Sensors();

  Algorithms_Process();

  ManageBleConnection();

  ManageTerminal();

  LedManageEnd(0);

  if(ButtonPressed)
  {
    ButtonPressed = FALSE;
    /* Take button pressed action...*/
    /* For example, board standby or shutdown */
    APP_BSP_LedSmoothBlink(20);
    APP_BSP_LedSmoothBlink(20);
    APP_LedSmoothRampDown(30);

    /*Sensors Shutdown*/
    BSP_GG_PowerSavingMode();

    BSP_PRESSURE_Sensor_Disable( PRESSURE_handle );
    BSP_TEMPERATURE_Sensor_Enable( TEMPERATURE_handle );

    BSP_ACCELERO_Sensor_Disable( ACCELERO_handle );

    BSP_GYRO_Sensor_Disable( GYRO_handle );

    BSP_MAGNETO_Sensor_Disable( MAGNETO_handle );

    BluenrgInitialization();

    PRINTF("\r\nsetConnectable()");
    setConnectable(0x0800, 0x0800);
    HCI_Process();

    if(BSP_PB_GetState(BUTTON_USER))
    {
      /* If the pushbutton is still pressed, then shutdown the board */
      APP_BSP_LED_On(LED);
      HAL_Delay(500);
      BSP_GG_Stop();
      BSP_PWMGT_BoardShutdown();
      while(1)
      {
        APP_BSP_LedSmoothBlink(50);
      }
    }

    BSP_PB_DeInit(BUTTON_USER);

    /* Configure all GPIO as analog to reduce current consumption on non used IOs */
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* Enable GPIOs clock */
    __GPIOA_CLK_ENABLE();__GPIOB_CLK_ENABLE();__GPIOC_CLK_ENABLE();__GPIOD_CLK_ENABLE();__GPIOE_CLK_ENABLE();__GPIOH_CLK_ENABLE();__GPIOF_CLK_ENABLE();__GPIOG_CLK_ENABLE();

    /* Configure the GPIO_LED pin */

    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
    GPIO_InitStruct.Pin = GPIO_PIN_All;

    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Pin &= ~( GPIO_PIN_13 | GPIO_PIN_14 );
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __GPIOA_CLK_DISABLE();__GPIOB_CLK_DISABLE();__GPIOC_CLK_DISABLE();__GPIOD_CLK_DISABLE();__GPIOE_CLK_DISABLE();__GPIOH_CLK_DISABLE();__GPIOF_CLK_DISABLE();__GPIOG_CLK_DISABLE();

    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
    BNRG_IRQ_CLK_ENABLE();
    set_irq_as_input();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    if( ButtonPressed == 1 )
    {
      WESU_SYSTEM_RESET();
    }
    HCI_Process();
    if(!bIsConnected)
    {
      set_connectable = 1;
    }
  }
}

/**
 * @brief Load Examples masks, based on XXXX_EXAMPLE defines: ACCELEROMETER_EXAMPLE, GYROSCOPE_EXAMPLE, MAGNETOMETER_EXAMPLE, GAS_GAUGE, PRESS_EXAMPLE, TEMPERATURE_EXAMPLE and USE_CUSTOM_ALGORITHM1
 * @retval none
 */
void LoadExamplesMasks()
{
#if ACCELEROMETER_EXAMPLE
    nReadSensorMask |= HW_CAP_ACCEL;
#endif //ACCELEROMETER_EXAMPLE
#if GYROSCOPE_EXAMPLE
    nReadSensorMask |= HW_CAP_GYRO;
#endif //GYROSCOPE_EXAMPLE
#if MAGNETOMETER_EXAMPLE
    nReadSensorMask |= HW_CAP_MAGN;
#endif //MAGNETOMETER_EXAMPLE
#if PRESS_EXAMPLE
    nReadSensorMask |= HW_CAP_PRESS;
#endif //PRESS_EXAMPLE
#if TEMPERATURE_EXAMPLE
    nReadSensorMask |= HW_CAP_TEMPERATURE;
#endif //TEMPERATURE_EXAMPLE
#if GAS_GAUGE
    nReadSensorMask |= HW_CAP_GG;
#endif //GAS_GAUGE

#if USE_CUSTOM_ALGORITHM1
    nProcessFwMask |= ALGORITHM1_BLUEST_MASK;
#endif /* USE_CUSTOM_ALGORITHM1 */
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

    BATT_STATUS_READ    = BSP_CHARGER_GetState();
    BSP_BatteryChargerTask();
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
    uint16_t ai = 0;

    {
      //default values
      ai=0x0020;
    }

    PRINTF("\r\nsetConnectable()");
    setConnectable(ai, ai);
    set_connectable = FALSE;
  }

  if(bIsConnected)
  {
    nLastConnectionTick = HAL_GetTick();
    TimeStamp++;
    //      TimeStamp = HAL_GetTick()&0xFFFF;

    if(ACCELEROMETER_EXAMPLE || GYROSCOPE_EXAMPLE || MAGNETOMETER_EXAMPLE )
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
        }
    }

    if(GAS_GAUGE)
    {
      BATT_V_Value      = (uint16_t) BATT_V_READ;
      BATT_I_Value      = (uint16_t) BATT_I_READ;
      BATT_SOC_Value    = (uint16_t) BATT_SOC_READ;
      BATT_STATUS_Value = (uint8_t) BATT_STATUS_READ;
      if(CHECK_SUBSAMPLING(nTimerFrequency))//once per second
        if(Pwr_Update(BATT_SOC_Value, BATT_V_Value, BATT_I_Value, BATT_STATUS_Value) != BLE_STATUS_SUCCESS)
        {
        }
    }

      if(TEMPERATURE_EXAMPLE)
      {
        TEMP_Value = (int32_t)(TEMP_READ);
        if(Temp_Update(TEMP_Value))
        {
        }
      }

    if(PRESS_EXAMPLE)
    {
        PRESS_Value = (int32_t)(PRESS_READ);
        if(Press_Update(PRESS_Value) != BLE_STATUS_SUCCESS)
        {
        }
    }

#if USE_CUSTOM_ALGORITHM1
    {
        if(CHECK_SUBSAMPLING(nTimerFrequency))
        {
          extern uint8_t algorithm1_value;
          if(Algorithm1_Update(SESSION_REG(algorithm1_value)) != BLE_STATUS_SUCCESS)
          {
          }
        }
    }
#endif //USE_CUSTOM_ALGORITHM1

  }// if(bIsConnected)
  else
  {
    //not connected
//    APP_BSP_LED_Off(LED);
  }
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
void SystemClock_Config_MSI_2MHz(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
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
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable HSE Oscillator and Activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Set Voltage scale1 as MCU will run at 32MHz */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief  Configures the current time and date
 * @retval None
 */
static void RTC_TimeStampConfigDefault(void)
{
  rtc_time_t t = {HOURS_DEFAULT, MINUTES_DEFAULT, SECONDS_DEFAULT,DAY_DEFAULT, MONTH_DEFAULT, YEAR_DEFAULT, WEEKDAY_DEFAULT, RTC_CONFIG_DEFAULT};
//  memcpy(&t, (rtc_time_t *)RTC_CONF_0_REGADDR, 8);
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
    //WESU_DBG_WFI();
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
      //WESU_DBG_WFI();
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
  switch(Pin)
  {
    case BNRG_EXTI_PIN:
      HCI_Isr();
    break;
  case BUTTON_GPIO_PIN:
    ButtonPressed = 1;
    break;
  case USB_PWR_GPIO_PIN:
    UsbConnected = BSP_PwrUsbMonitor_GetState();
    break;
  case IMU_6AXES_INT1_PIN:
    {
      LsmXGInt1Ready = SET;
    }
    break;

  case IMU_6AXES_INT2_PIN:     /* set the LSM6DS3 INT2 Ready flag */
      LsmXGInt2Ready = SET;
    break;

  case MAGNETO_INT_PIN:       /* set the  INT_M Ready flag */
      LisIntReady = SET;
    break;

  case MAGNETO_DRDY_PIN:      /* set the  DRDY_M Ready flag */
      LsmDrdyMReady = SET;
    break;
  case GG_INT_PIN:            /* TODO:FS: manage gas gauge interrupt */
    GasGaugeInterrupt = SET;
    break;
  default:
      //not handled
//    while(1);
    break;
  }
}


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

static int myerr = 0;

int*__errno()
{
	return &myerr;
	}

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

/**
 * @}
 */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
