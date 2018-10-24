/**
  ******************************************************************************
  * @file    WESU_DEMO/Src/wesu_config.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   WeSU Config Application API
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

#include "BlueST_Protocol.h"
#include "wesu_config.h"
#include "wesu_charger.h"
#include "wesu_sensors.h"
#include "algorithms.h"



/** @addtogroup WeSU_Demo       WeSU Demo
  * @{
  */

/** @addtogroup WeSU_User               WeSU User
  * @{
  */

/** @addtogroup WeSU_Config
  * @{
  * @brief WeSU Configuration customized for the DEMO
  */


/** @defgroup WeSU_Config_Private_Defines    WeSU Config Private Defines
  * @{
  */

#define PERMREG_CONTROL_BYTE              (pPointerToFrame->nControlByte)         //!< frame header: control byte
#define PERMREG_REG_ADDR                  (pPointerToFrame->nRegAddress)          //!< frame header: register address
#define PERMREG_LENGHT                    (pPointerToFrame->nPayloadLenght)       //!< frame header: payload lenght
#define PERMREG_ERROR_CODE                (pPointerToFrame->nErrorCode)           //!< frame header: error code
#define PERMREG_PAYLOAD_POINTER           (pPointerToFrame->pPayload)             //!< frame payload

#define PERMREG_SET_CONTROL_BIT(BITMASK,bitval)          PERMREG_CONTROL_BYTE&=~BITMASK;PERMREG_CONTROL_BYTE|=((bitval!=0x00)*BITMASK)        //!< set control bit in PERMREG_CONTROL_BYTE
#define PERMREG_SET_ERROR_CODE(A)         PERMREG_ERROR_CODE = A                                                                            //!< set error code

/**
  * @}
  */

/** @defgroup WeSU_Config_Private_Typedef    WeSU Config Private Typedef
  * @{
  */

/**
  * @brief Register management function, triggered by read/write on a specified register
  */
typedef uint8_t (*pCheckFunc_t)(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload);

/**
  * @brief Register management structure typedef
  */
typedef struct sRegCorrectStruct
{
  uint8_t nRegAddressToCheck;                                                   //!< register address to check
  uint8_t nCheckLenght;                                                         //!< number of registers to check
  pCheckFunc_t pCheckFunc;                                                      //!< associated function
}RegCorrectStruct_t;

/**
  * @}
  */

/** @defgroup WeSU_Config_Private_Constants    WeSU Config Private Constants
  * @{
  */

/**
  * @brief Descriptor array to check if the content of the specified persistent register is correct
  */
const RegCorrectStruct_t PermRegCorrectStructVect[] =
{
//nRegAddress, nCheckLenght, pCheckFunc_t
  { 0, 0, NULL }
};

/**
  * @brief Descriptor array to check if the content of the specified persistent register can be modified
  */
const RegCorrectStruct_t PermRegActionStructVect[] =
{ //nRegAddress, nCheckLenght, pCheckFunc_t
  { FW_VERSION_0_REG,                   1, ReadOnly },                  //0x00
  { FW_VERSION_1_REG,                   1, ReadOnly },                  //0x01
  { LED_CONFIG_REG,                     1, LedControlMaskChanged },     //0x02
  { WeSU_DEBUG_MASK_REG,                1, CopyToSession },             //0x1F
  { TX_PWR_LVL_CONTROL_REG,             1, TxPwrLvlChange },            //0x20
  { TIMER_FREQUENCY_REG,                1, TimerFreqChanged },          //0x21
  { PWR_MODE_CONTROL_REG,               1, PwrModeControlChanged },     //0x22
  { HW_CAPABILITIES_REG,                1, ReadOnly },                  //0x23
  { FW_CAPABILITIES_REG,                1, ReadOnly },                  //0x34
  { PEDOMETER_CONTROL_MASK_REG,         1, PedometerCtrlChanged },      //0x35
  { MOTION_FX_CONTROL_MASK_REG,         1, MotionFxCtrlChanged },       //0x3D
  { BLE_DEBUG_CONFIG_REG,               3, CopyToSession },             //0x45
  { HW_OPERATING_CAPABILITIES_REG,      1, HwConfigChanged },           //0x49
  { FW_OPERATING_CAPABILITIES_REG,      1, FwConfigChanged },           //0x4a
  { BLE_CON_INTV_REG,                   1, BleConIntvChanged },         //0x4b
  { 0x4C/*BlueNRG versions*/,           2, ReadOnly },                  //0x4C
  { RED_LED_CONFIG_REG,                 1, RedLedControlMaskChanged },  //0x50
  { MAG_CALIB_CONTROL_REG,              1, FXMagCalibChanged },         //0x60
  { ACC_FullScale_REG,                  1, AccFsChanged },              //0x74
  { ACC_OutputDataRate_REG,             1, AccOdrChanged },             //0x75
  { GYRO_FullScale_REG,                 1, GyroFsChanged },             //0x76
  { GYRO_OutputDataRate_REG,            1, GyroOdrChanged },            //0x77
  { MAG_FullScale_REG,                  1, MagFsChanged },              //0x78
  { MAG_OutputDataRate_REG,             1, MagOdrChanged },             //0x79
  { ACCEVENT_CONF_REG,                  1, AccEventConfChanged },       //0x7A
  { PRESS_TEMP_OutputDataRate_REG,      1, PressOdrChanged },           //0x7b
  { RTC_CONF_0_REG,                     4, RtcConfChanged },            //0x90
  { 0, 0, NULL }
};

/**
  * @brief Descriptor array to check if the content of the specified session register is correct
  */
const RegCorrectStruct_t SessionRegCorrectStructVect[] =
{ //nRegAddress, nCheckLenght, pCheckFunc_t
  { 0, 0, NULL }
};

/**
  * @brief Descriptor array to check if the content of the specified session register can be modified
  */
const RegCorrectStruct_t SessionRegActionStructVect[] =
{ //nRegAddress, nCheckLenght, pCheckFunc_t
  { FW_VERSION_0_REG,                   1, ReadOnly },                  //0x00
  { FW_VERSION_1_REG,                   1, ReadOnly },                  //0x01
  { LED_CONFIG_REG,                     1, LedControlMaskChanged },     //0x02
  { TIMER_FREQUENCY_REG,                1, TimerFreqChanged },          //0x21
  { PWR_MODE_CONTROL_REG,               1, PwrModeControlChanged },     //0x22
  { PEDOMETER_CONTROL_MASK_REG,         1, PedometerCtrlChanged },      //0x35
  { MOTION_FX_CONTROL_MASK_REG,         1, MotionFxCtrlChanged },       //0x3D
  { HW_OPERATING_CAPABILITIES_REG,      1, HwConfigChanged },           //0x49
  { FW_OPERATING_CAPABILITIES_REG,      1, FwConfigChanged },           //0x4A
  { BLE_CON_INTV_REG,                   1, BleConIntvChanged },         //0x4B
  { RED_LED_CONFIG_REG,                 1, RedLedControlMaskChanged },  //0x50
  { MAG_CALIB_CONTROL_REG,              1, FXMagCalibChanged },         //0x60
  { ACCEVENT_CONF_REG,                  1, AccEventConfChanged },       //0x7A
  { RTC_CONF_0_REG,                     4, RtcConfChanged },            //0x90
  { PERMREG_START_DUMMY_REG,            1, DummyRegChange },            //0xF0
  { PERMREG_DUMMY_ALT_POWER_MODE_REG,   1, PwrModeAltRegChange },       //0xF2
  { 0, 0, NULL }
};

/**
  * @}
  */

/** @defgroup WeSU_Config_Imported_Variables     WeSU Config Imported Variables
  * @{
  */

extern volatile int bIsConnected;
extern uint8_t xCurSmFunc;

extern unsigned char isCal;
extern unsigned char isStartFxCalib;

extern void *ACCELERO_handle;
extern void *GYRO_handle;
extern void *MAGNETO_handle;
extern void *PRESSURE_handle;
extern void *TEMPERATURE_handle;


/**
  * @}
  */

/** @defgroup WeSU_Config_Imported_Functions     WeSU Config Imported Functions
  * @{
  */
extern void SystemClock_Config_MSI_2MHz(void);
extern void SystemClock_Config_MSI_4MHz(void);
extern void SystemClock_Config_HSI_12MHz(void);
extern void SystemClock_Config_HSE_18MHz(void);
extern void SystemClock_Config_HSE_24MHz(void);
extern void SystemClock_Config_HSI_32MHz(void);
extern void SystemClock_Config_RTC_HSE32MHz(void);

extern void (*SystemClock_Config_APP)(void);
extern void (*SystemClock_Config_APP_STARTUP)(void);

/**
  * @}
  */


/** @defgroup WeSU_Config_Exported_Constants     WeSU Config Exported Constants
  * @{
  */

const uint16_t nGlobalConfStruct[] ROM_VARS     //!< Persistent Registers Default Values
= {
  PERSISTENT_DEFAULT_REG_0x00,
  PERSISTENT_DEFAULT_REG_0x01,
  PERSISTENT_DEFAULT_REG_0x02,
  PERSISTENT_DEFAULT_REG_0x03,
  PERSISTENT_DEFAULT_REG_0x04,
  PERSISTENT_DEFAULT_REG_0x05,
  PERSISTENT_DEFAULT_REG_0x06,
  PERSISTENT_DEFAULT_REG_0x07,
  PERSISTENT_DEFAULT_REG_0x08,
  PERSISTENT_DEFAULT_REG_0x09,
  PERSISTENT_DEFAULT_REG_0x0A,
  PERSISTENT_DEFAULT_REG_0x0B,
  PERSISTENT_DEFAULT_REG_0x0C,
  PERSISTENT_DEFAULT_REG_0x0D,
  PERSISTENT_DEFAULT_REG_0x0E,
  PERSISTENT_DEFAULT_REG_0x0F,
  PERSISTENT_DEFAULT_REG_0x10,
  PERSISTENT_DEFAULT_REG_0x11,
  PERSISTENT_DEFAULT_REG_0x12,
  PERSISTENT_DEFAULT_REG_0x13,
  PERSISTENT_DEFAULT_REG_0x14,
  PERSISTENT_DEFAULT_REG_0x15,
  PERSISTENT_DEFAULT_REG_0x16,
  PERSISTENT_DEFAULT_REG_0x17,
  PERSISTENT_DEFAULT_REG_0x18,
  PERSISTENT_DEFAULT_REG_0x19,
  PERSISTENT_DEFAULT_REG_0x1A,
  PERSISTENT_DEFAULT_REG_0x1B,
  PERSISTENT_DEFAULT_REG_0x1C,
  PERSISTENT_DEFAULT_REG_0x1D,
  PERSISTENT_DEFAULT_REG_0x1E,
  PERSISTENT_DEFAULT_REG_0x1F,
  PERSISTENT_DEFAULT_REG_0x20,
  PERSISTENT_DEFAULT_REG_0x21,
  PERSISTENT_DEFAULT_REG_0x22,
  PERSISTENT_DEFAULT_REG_0x23,
  PERSISTENT_DEFAULT_REG_0x24,
  PERSISTENT_DEFAULT_REG_0x25,
  PERSISTENT_DEFAULT_REG_0x26,
  PERSISTENT_DEFAULT_REG_0x27,
  PERSISTENT_DEFAULT_REG_0x28,
  PERSISTENT_DEFAULT_REG_0x29,
  PERSISTENT_DEFAULT_REG_0x2A,
  PERSISTENT_DEFAULT_REG_0x2B,
  PERSISTENT_DEFAULT_REG_0x2C,
  PERSISTENT_DEFAULT_REG_0x2D,
  PERSISTENT_DEFAULT_REG_0x2E,
  PERSISTENT_DEFAULT_REG_0x2F,
  PERSISTENT_DEFAULT_REG_0x30,
  PERSISTENT_DEFAULT_REG_0x31,
  PERSISTENT_DEFAULT_REG_0x32,
  PERSISTENT_DEFAULT_REG_0x33,
  PERSISTENT_DEFAULT_REG_0x34,
  PERSISTENT_DEFAULT_REG_0x35,
  PERSISTENT_DEFAULT_REG_0x36,
  PERSISTENT_DEFAULT_REG_0x37,
  PERSISTENT_DEFAULT_REG_0x38,
  PERSISTENT_DEFAULT_REG_0x39,
  PERSISTENT_DEFAULT_REG_0x3A,
  PERSISTENT_DEFAULT_REG_0x3B,
  PERSISTENT_DEFAULT_REG_0x3C,
  PERSISTENT_DEFAULT_REG_0x3D,
  PERSISTENT_DEFAULT_REG_0x3E,
  PERSISTENT_DEFAULT_REG_0x3F,
  PERSISTENT_DEFAULT_REG_0x40,
  PERSISTENT_DEFAULT_REG_0x41,
  PERSISTENT_DEFAULT_REG_0x42,
  PERSISTENT_DEFAULT_REG_0x43,
  PERSISTENT_DEFAULT_REG_0x44,
  PERSISTENT_DEFAULT_REG_0x45,
  PERSISTENT_DEFAULT_REG_0x46,
  PERSISTENT_DEFAULT_REG_0x47,
  PERSISTENT_DEFAULT_REG_0x48,
  PERSISTENT_DEFAULT_REG_0x49,
  PERSISTENT_DEFAULT_REG_0x4A,
  PERSISTENT_DEFAULT_REG_0x4B,
  PERSISTENT_DEFAULT_REG_0x4C,
  PERSISTENT_DEFAULT_REG_0x4D,
  PERSISTENT_DEFAULT_REG_0x4E,
  PERSISTENT_DEFAULT_REG_0x4F,
  PERSISTENT_DEFAULT_REG_0x50,
  PERSISTENT_DEFAULT_REG_0x51,
  PERSISTENT_DEFAULT_REG_0x52,
  PERSISTENT_DEFAULT_REG_0x53,
  PERSISTENT_DEFAULT_REG_0x54,
  PERSISTENT_DEFAULT_REG_0x55,
  PERSISTENT_DEFAULT_REG_0x56,
  PERSISTENT_DEFAULT_REG_0x57,
  PERSISTENT_DEFAULT_REG_0x58,
  PERSISTENT_DEFAULT_REG_0x59,
  PERSISTENT_DEFAULT_REG_0x5A,
  PERSISTENT_DEFAULT_REG_0x5B,
  PERSISTENT_DEFAULT_REG_0x5C,
  PERSISTENT_DEFAULT_REG_0x5D,
  PERSISTENT_DEFAULT_REG_0x5E,
  PERSISTENT_DEFAULT_REG_0x5F,
  PERSISTENT_DEFAULT_REG_0x60,
  PERSISTENT_DEFAULT_REG_0x61,
  PERSISTENT_DEFAULT_REG_0x62,
  PERSISTENT_DEFAULT_REG_0x63,
  PERSISTENT_DEFAULT_REG_0x64,
  PERSISTENT_DEFAULT_REG_0x65,
  PERSISTENT_DEFAULT_REG_0x66,
  PERSISTENT_DEFAULT_REG_0x67,
  PERSISTENT_DEFAULT_REG_0x68,
  PERSISTENT_DEFAULT_REG_0x69,
  PERSISTENT_DEFAULT_REG_0x6A,
  PERSISTENT_DEFAULT_REG_0x6B,
  PERSISTENT_DEFAULT_REG_0x6C,
  PERSISTENT_DEFAULT_REG_0x6D,
  PERSISTENT_DEFAULT_REG_0x6E,
  PERSISTENT_DEFAULT_REG_0x6F,
  PERSISTENT_DEFAULT_REG_0x70,
  PERSISTENT_DEFAULT_REG_0x71,
  PERSISTENT_DEFAULT_REG_0x72,
  PERSISTENT_DEFAULT_REG_0x73,
  PERSISTENT_DEFAULT_REG_0x74,
  PERSISTENT_DEFAULT_REG_0x75,
  PERSISTENT_DEFAULT_REG_0x76,
  PERSISTENT_DEFAULT_REG_0x77,
  PERSISTENT_DEFAULT_REG_0x78,
  PERSISTENT_DEFAULT_REG_0x79,
  PERSISTENT_DEFAULT_REG_0x7A,
  PERSISTENT_DEFAULT_REG_0x7B,
  PERSISTENT_DEFAULT_REG_0x7C,
  PERSISTENT_DEFAULT_REG_0x7D,
  PERSISTENT_DEFAULT_REG_0x7E,
  PERSISTENT_DEFAULT_REG_0x7F,
  PERSISTENT_DEFAULT_REG_0x80,
  PERSISTENT_DEFAULT_REG_0x81,
  PERSISTENT_DEFAULT_REG_0x82,
  PERSISTENT_DEFAULT_REG_0x83,
  PERSISTENT_DEFAULT_REG_0x84,
  PERSISTENT_DEFAULT_REG_0x85,
  PERSISTENT_DEFAULT_REG_0x86,
  PERSISTENT_DEFAULT_REG_0x87,
  PERSISTENT_DEFAULT_REG_0x88,
  PERSISTENT_DEFAULT_REG_0x89,
  PERSISTENT_DEFAULT_REG_0x8A,
  PERSISTENT_DEFAULT_REG_0x8B,
  PERSISTENT_DEFAULT_REG_0x8C,
  PERSISTENT_DEFAULT_REG_0x8D,
  PERSISTENT_DEFAULT_REG_0x8E,
  PERSISTENT_DEFAULT_REG_0x8F,
  PERSISTENT_DEFAULT_REG_0x90,
  PERSISTENT_DEFAULT_REG_0x91,
  PERSISTENT_DEFAULT_REG_0x92,
  PERSISTENT_DEFAULT_REG_0x93,
  PERSISTENT_DEFAULT_REG_0x94,
  PERSISTENT_DEFAULT_REG_0x95,
  PERSISTENT_DEFAULT_REG_0x96,
  PERSISTENT_DEFAULT_REG_0x97,
  PERSISTENT_DEFAULT_REG_0x98,
  PERSISTENT_DEFAULT_REG_0x99,
  PERSISTENT_DEFAULT_REG_0x9A,
  PERSISTENT_DEFAULT_REG_0x9B,
  PERSISTENT_DEFAULT_REG_0x9C,
  PERSISTENT_DEFAULT_REG_0x9D,
  PERSISTENT_DEFAULT_REG_0x9E,
  PERSISTENT_DEFAULT_REG_0x9F,
  PERSISTENT_DEFAULT_REG_0xA0,
  PERSISTENT_DEFAULT_REG_0xA1,
  PERSISTENT_DEFAULT_REG_0xA2,
  PERSISTENT_DEFAULT_REG_0xA3,
  PERSISTENT_DEFAULT_REG_0xA4,
  PERSISTENT_DEFAULT_REG_0xA5,
  PERSISTENT_DEFAULT_REG_0xA6,
  PERSISTENT_DEFAULT_REG_0xA7,
  PERSISTENT_DEFAULT_REG_0xA8,
  PERSISTENT_DEFAULT_REG_0xA9,
  PERSISTENT_DEFAULT_REG_0xAA,
  PERSISTENT_DEFAULT_REG_0xAB,
  PERSISTENT_DEFAULT_REG_0xAC,
  PERSISTENT_DEFAULT_REG_0xAD,
  PERSISTENT_DEFAULT_REG_0xAE,
  PERSISTENT_DEFAULT_REG_0xAF,
  PERSISTENT_DEFAULT_REG_0xB0,
  PERSISTENT_DEFAULT_REG_0xB1,
  PERSISTENT_DEFAULT_REG_0xB2,
  PERSISTENT_DEFAULT_REG_0xB3,
  PERSISTENT_DEFAULT_REG_0xB4,
  PERSISTENT_DEFAULT_REG_0xB5,
  PERSISTENT_DEFAULT_REG_0xB6,
  PERSISTENT_DEFAULT_REG_0xB7,
  PERSISTENT_DEFAULT_REG_0xB8,
  PERSISTENT_DEFAULT_REG_0xB9,
  PERSISTENT_DEFAULT_REG_0xBA,
  PERSISTENT_DEFAULT_REG_0xBB,
  PERSISTENT_DEFAULT_REG_0xBC,
  PERSISTENT_DEFAULT_REG_0xBD,
  PERSISTENT_DEFAULT_REG_0xBE,
  PERSISTENT_DEFAULT_REG_0xBF,
  PERSISTENT_DEFAULT_REG_0xC0,
  PERSISTENT_DEFAULT_REG_0xC1,
  PERSISTENT_DEFAULT_REG_0xC2,
  PERSISTENT_DEFAULT_REG_0xC3,
  PERSISTENT_DEFAULT_REG_0xC4,
  PERSISTENT_DEFAULT_REG_0xC5,
  PERSISTENT_DEFAULT_REG_0xC6,
  PERSISTENT_DEFAULT_REG_0xC7,
  PERSISTENT_DEFAULT_REG_0xC8,
  PERSISTENT_DEFAULT_REG_0xC9,
  PERSISTENT_DEFAULT_REG_0xCA,
  PERSISTENT_DEFAULT_REG_0xCB,
  PERSISTENT_DEFAULT_REG_0xCC,
  PERSISTENT_DEFAULT_REG_0xCD,
  PERSISTENT_DEFAULT_REG_0xCE,
  PERSISTENT_DEFAULT_REG_0xCF,
  PERSISTENT_DEFAULT_REG_0xD0,
  PERSISTENT_DEFAULT_REG_0xD1,
  PERSISTENT_DEFAULT_REG_0xD2,
  PERSISTENT_DEFAULT_REG_0xD3,
  PERSISTENT_DEFAULT_REG_0xD4,
  PERSISTENT_DEFAULT_REG_0xD5,
  PERSISTENT_DEFAULT_REG_0xD6,
  PERSISTENT_DEFAULT_REG_0xD7,
  PERSISTENT_DEFAULT_REG_0xD8,
  PERSISTENT_DEFAULT_REG_0xD9,
  PERSISTENT_DEFAULT_REG_0xDA,
  PERSISTENT_DEFAULT_REG_0xDB,
  PERSISTENT_DEFAULT_REG_0xDC,
  PERSISTENT_DEFAULT_REG_0xDD,
  PERSISTENT_DEFAULT_REG_0xDE,
  PERSISTENT_DEFAULT_REG_0xDF,
  PERSISTENT_DEFAULT_REG_0xE0,
  PERSISTENT_DEFAULT_REG_0xE1,
  PERSISTENT_DEFAULT_REG_0xE2,
  PERSISTENT_DEFAULT_REG_0xE3,
  PERSISTENT_DEFAULT_REG_0xE4,
  PERSISTENT_DEFAULT_REG_0xE5,
  PERSISTENT_DEFAULT_REG_0xE6,
  PERSISTENT_DEFAULT_REG_0xE7,
  PERSISTENT_DEFAULT_REG_0xE8,
  PERSISTENT_DEFAULT_REG_0xE9,
  PERSISTENT_DEFAULT_REG_0xEA,
  PERSISTENT_DEFAULT_REG_0xEB,
  PERSISTENT_DEFAULT_REG_0xEC,
  PERSISTENT_DEFAULT_REG_0xED,
  PERSISTENT_DEFAULT_REG_0xEE,
  PERSISTENT_DEFAULT_REG_0xEF,
  PERSISTENT_DEFAULT_REG_0xF0,
  PERSISTENT_DEFAULT_REG_0xF1,
  PERSISTENT_DEFAULT_REG_0xF2,
  PERSISTENT_DEFAULT_REG_0xF3,
  PERSISTENT_DEFAULT_REG_0xF4,
  PERSISTENT_DEFAULT_REG_0xF5,
  PERSISTENT_DEFAULT_REG_0xF6,
  PERSISTENT_DEFAULT_REG_0xF7,
  PERSISTENT_DEFAULT_REG_0xF8,
  PERSISTENT_DEFAULT_REG_0xF9,
  PERSISTENT_DEFAULT_REG_0xFA,
  PERSISTENT_DEFAULT_REG_0xFB,
  PERSISTENT_DEFAULT_REG_0xFC,
  PERSISTENT_DEFAULT_REG_0xFD,
  PERSISTENT_DEFAULT_REG_0xFE,
  PERSISTENT_DEFAULT_REG_0xFF,
};

/**
 * @}
 */

/** @defgroup WeSU_Config_Private_Variables     WeSU Config Private Variables
  * @{
  */

  uint8_t pDestbuffer[512+1];                                                   //!< Multi-frame facility: destination buffer (not implemented)
  uint32_t nMultiFragOffset = 0;                                                //!< Multi-frame facility: offset (not implemented)
  uint8_t nNextFrameNumber = 0;                                                 //!< Multi-frame facility: next frame number (not implemented)

  static char WesuName[8] = {0};                                                //!< WeSU board name
  static uint8_t WesuAddr[6] = {0};                                             //!< WeSU board public address
  static uint8_t WesuAddrDefault[6] = { BLE_PUB_ADDR_DEFAULT_0,
                                      BLE_PUB_ADDR_DEFAULT_1,
                                      BLE_PUB_ADDR_DEFAULT_2,
                                      BLE_PUB_ADDR_DEFAULT_3,
                                      BLE_PUB_ADDR_DEFAULT_4,
                                      BLE_PUB_ADDR_DEFAULT_5 };                 //!< default public address

/**
 * @}
 */

/** @defgroup WeSU_Config_Exported_Variables    WeSU Config Exported Variables
  * @{
  */
DrvStatusTypeDef sensorStatusAccelero = COMPONENT_ERROR;                        //!< Accelerometer status
DrvStatusTypeDef sensorStatusGyro = COMPONENT_ERROR;                            //!< Gyroscope status
DrvStatusTypeDef sensorStatusMag = COMPONENT_ERROR;                             //!< Magnetometer status
DrvStatusTypeDef sensorStatusPress = COMPONENT_ERROR;                           //!< Pressure status
DrvStatusTypeDef sensorStatusTemp = COMPONENT_ERROR;                            //!< Temperature status

/**
 * @}
 */

/** @addtogroup WeSU_Config_Exported_Functions
  * @{
  */


/**
 * @brief  Puts the device in connectable mode.
 *         If you want to specify a UUID list in the advertising data, those data can
 *         be specified as a parameter in aci_gap_set_discoverable().
 *         For manufacture data, aci_gap_update_adv_data must be called
 * @param       nMin, nMax
 * @retval None
 */
/* Ex.:
 *
 *  tBleStatus ret;
 *  const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G'};
 *  const uint8_t serviceUUIDList[] = {AD_TYPE_16_BIT_SERV_UUID,0x34,0x12};
 *  const uint8_t manuf_data[] = {4, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0x05, 0x02, 0x01};
 *
 *  ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
 *                                 8, local_name, 3, serviceUUIDList, 0, 0);
 *  ret = aci_gap_update_adv_data(5, manuf_data);
 *
 */
void setConnectable(uint16_t nMin, uint16_t nMax, uint8_t boardSleeping)
{
  PRINTF_INFO("General Discoverable Mode.\n");

  char cLocalNameAdv[MAX_BLUENRG_NAME_LEN_ADV+10] = { 0 };

  uint8_t nNameLen = 0;
  uint8_t xPubAddressType = PUBLIC_ADDR;

  WESU_GET_NAME(cLocalNameAdv,MAX_BLUENRG_NAME_LEN_ADV);

  nNameLen = strlen(cLocalNameAdv);

  /* disable scan response */
  hci_le_set_scan_resp_data(0,NULL);

    /* Set public address on BlueNRG */
    /*int ret = */aci_gap_set_non_discoverable();

    /*ret = */aci_gap_set_discoverable(ADV_IND, nMin, nMax, xPubAddressType, NO_WHITE_LIST_USE,
                                   nNameLen, cLocalNameAdv, 0, NULL, 0, 0);

    uint8_t advbuffer[]=
    {
      0x0D, //lenght of MANUF_SPECIFIC advertising item
      AD_TYPE_MANUFACTURER_SPECIFIC_DATA, //Type
      0x01, //Protocol version
      HW_ID, //Dev ID
      0x00, 0x00, //Group A Features (big endian)
      0x00, 0x00, //Group B Features (big endian)
      0x00, 0x00, 0x00, //Public device address (48 bits) Company assigned
      0x00, 0x00, 0x00, //Public device address (48 bits) Company id
    };

    advbuffer[4] = HIGH_BYTE(*HW_CAPABILITIES_REGADDR);
    advbuffer[5] = LOW_BYTE(*HW_CAPABILITIES_REGADDR);
    advbuffer[6] = HIGH_BYTE(*FW_CAPABILITIES_REGADDR);
    advbuffer[7] = LOW_BYTE(*FW_CAPABILITIES_REGADDR);

    uint16_t nAddedFeatures = 0x0000;

#if USE_CUSTOM_ALGORITHM1
    nAddedFeatures |= ALGORITHM1_BLUEST_MASK;
#endif /* USE_CUSTOM_ALGORITHM1 */

#if USE_CUSTOM_ALGORITHM2
    nAddedFeatures |= ALGORITHM2_BLUEST_MASK;
#endif /* USE_CUSTOM_ALGORITHM2 */

#if USE_CUSTOM_ALGORITHM3
    nAddedFeatures |= ALGORITHM3_BLUEST_MASK;
#endif /* USE_CUSTOM_ALGORITHM3 */

#if USE_CUSTOM_ALGORITHM4
    nAddedFeatures |= ALGORITHM4_BLUEST_MASK;
#endif /* USE_CUSTOM_ALGORITHM4 */
    

#if USE_CUSTOM_GP_ALGORITHM
    advbuffer[3] |= 0x20;
#endif //USE_CUSTOM_GP_ALGORITHM

#if USE_CUSTOM_ACCEVENT
    nAddedFeatures |= ACCEVENT_BLUEST_MASK;
#endif /* USE_CUSTOM_ACCEVENT */    

#if USE_SLEEP_INFO
    if(boardSleeping)
    {
      advbuffer[3] |= 0x40;
    }
#endif /* USE_SLEEP_INFO */
    
    advbuffer[6] |= HIGH_BYTE(nAddedFeatures);
    advbuffer[7] |= LOW_BYTE(nAddedFeatures);

    memcpy(&advbuffer[8], WesuAddr, 6);
    if(xPubAddressType == RANDOM_ADDR)
    {
      advbuffer[0] = 0x07;
    }

    aci_gap_update_adv_data(14, advbuffer);
}

void PrintSystemParameters()
{
  if(!(SESSION_REG(nRsvdDummyRegSystemFlag) & SYS_PARAMS_BACKUP_SESSION_DONE))
  {
    DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nWriting session backup data");
    WesuBackupSystemParameters(SYS_PARAMS_BACKUP_SESSION_DONE);
    SESSION_REG(nRsvdDummyRegSystemFlag) |= SYS_PARAMS_BACKUP_SESSION_DONE;
  }
  if(!(SESSION_REG(nRsvdDummyRegSystemFlag) & SYS_PARAMS_BACKUP_POWERON_DONE))
  {
    DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nWriting poweron backup data");
    WesuBackupSystemParameters(SYS_PARAMS_BACKUP_POWERON_DONE);
    SESSION_REG(nRsvdDummyRegSystemFlag) |= SYS_PARAMS_BACKUP_POWERON_DONE;
  }
  WesuSaveSystemParameters();
  //Print system parameters: ticks, rtc time, errors...
  DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nTicks: %d ", SESSION_REG(nTicks));
  DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nProcCount: %d ", SESSION_REG(nUserProcessCount));
  if(SESSION_REG(nWakeSource) & SYSTEM_STARTUP_FLAG_IWDG_RESET)
  {
    DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nThe system has been resumed from IWDG reset");
  }
#if USE_RTC
  DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nDate: %s %.2d-%s-%d",BSP_RTC_GetWeekDayName(SESSION_REG(tm_wday)),SESSION_REG(tm_day),BSP_RTC_GetMonthName(SESSION_REG(tm_mon)),SESSION_REG(tm_year));
  DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "\r\nTime: %.2d:%.2d:%.2d\r\n",SESSION_REG(tm_hour),SESSION_REG(tm_min),SESSION_REG(tm_sec));
#endif //USE_RTC
}

/**
 * @brief  Manage led status for blinking
 * @param       nLedMode
 * @retval None
 */
void LedManageStart(uint8_t nLedMode)
{
  //ledmode
  //0 on inizio ciclo, off fine ciclo
  //1 toggle inizio ciclo
  //2 always on
  //3 always off
  switch(nLedMode)
  {
  case LED_MODE_MEASURE_TIMER_EXP_EVENT_PROC:
    APP_BSP_LED_On(LED);
    break;
  case LED_MODE_TOGGLE_TM_EXP_EVENT:
    APP_BSP_LED_Toggle(LED);
    break;
  case LED_MODE_SMART_BLINKING:
    {
      static uint32_t nTicksSmartLedToggle = 0;
      static uint8_t bOldIsConnected = 0;
      if(bIsConnected!=bOldIsConnected)
      {
        nTicksSmartLedToggle = 0;
      }
      bOldIsConnected = bIsConnected;

      if(bIsConnected)
      {
        if(HAL_GetTick() > LED_INTERVAL_CONNECTED + nTicksSmartLedToggle)
        {
          APP_BSP_LED_On(LED);
          if((0 == APP_BSP_LED_GetState(LED)) && (HAL_GetTick() > LED_INTERVAL_CONNECTED_ON + LED_INTERVAL_CONNECTED + nTicksSmartLedToggle))
          {
            nTicksSmartLedToggle = HAL_GetTick();
          }
        }
        else
        {
          APP_BSP_LED_Off(LED);
        }
      }
      else
      {
        if(HAL_GetTick() >= LED_INTERVAL_DISCONNECTED + nTicksSmartLedToggle)
        {
          if((0 == APP_BSP_LED_GetState(LED)) && (HAL_GetTick() >= LED_INTERVAL_DISCONNECTED_ON + LED_INTERVAL_DISCONNECTED + nTicksSmartLedToggle))
          {
            nTicksSmartLedToggle = HAL_GetTick();
          }
          else
          {
            APP_BSP_LED_On(LED);
          }
        }
        else
        {
          APP_BSP_LED_Off(LED);
        }
      }
    }
    break;
  case LED_MODE_OFF:
    APP_BSP_LED_Off(LED);
    break;
  case LED_MODE_OFF2:
    APP_BSP_LED_Off(LED);
    break;
  case LED_MODE_MEASURE_SENSORS_READ:
    APP_BSP_LED_On(LED);
    break;
  case LED_MODE_MEASURE_BLUENRG_UPDATE:
    APP_BSP_LED_On(LED);
    break;
  case LED_MODE_SENSORS_NOT_CALIBRATED:
    {
      static uint32_t nTicksLedToggle = 0;

      if(HAL_GetTick()/750 > nTicksLedToggle + 1)
      {
        if(BSP_LED_GetState(LED))
        {
          APP_LedSmoothRampDown(25);
          APP_BSP_LED_Off(LED);
        }
        else
        {
          APP_LedSmoothRampUp(25);
          APP_BSP_LED_On(LED);
        }
        nTicksLedToggle = HAL_GetTick()/750;
      }
      else
      {
      }
    }
    break;
  default:
    APP_BSP_LED_On(LED);
    break;
  }
}

/**
 * @brief  Manage led status for blinking
 * @param       nLedMode
 * @retval None
 */
void LedManageEnd(uint8_t nLedMode)
{
  switch(nLedMode)
  {
  case LED_MODE_MEASURE_TIMER_EXP_EVENT_PROC:
    APP_BSP_LED_Off(LED);
    break;
  case LED_MODE_TOGGLE_TM_EXP_EVENT:
    break;
  case LED_MODE_SMART_BLINKING:
    break;
  case LED_MODE_OFF:
    break;
  case LED_MODE_OFF2:
    break;
  case LED_MODE_MEASURE_SENSORS_READ:
    APP_BSP_LED_Off(LED);
    break;
  case LED_MODE_MEASURE_BLUENRG_UPDATE:
    APP_BSP_LED_Off(LED);
    break;
  case LED_MODE_SENSORS_NOT_CALIBRATED:
    break;
  default:
//    LED_Off();
    break;
  }
}


/** @brief Initialize all the devices in the PWR SubSystem
 * @retval None
 */
void Init_PWR_MGMT(void)
{
  /* Initialize power management pins */
  BSP_PWMGT_Init();

  /* Initialize the Charger */
  BSP_CHARGER_Init();

#if USE_STC3115
  /* Initialize Gas Gauge */
  BSP_GG_IO_Init();
  BSP_GG_Init();
#endif //USE_STC3115
}


/**
 * @brief Initialize All the BlueNRG management
 * @retval None
 */
void BluenrgInitialization()
{
  /* Reset BlueNRG versions */
  SESSION_REG(xBNRG_HW_ver) = 0;
  SESSION_REG(xBNRG_FW_ver) = 0;

  /* Copy current setting and disable printf output on BlueNRG */
  uint16_t nTempBleEnabledDebug = ENABLED_DEBUG_LEVEL_BLE;
  ENABLED_DEBUG_LEVEL_BLE = 0;

  /* Initialize the BlueNRG */
  PRINTF("\r\nInit_BlueNRG_Hw\r\n");
  Init_BlueNRG_Hw();
  
  DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "BlueNRG HwVer 0x%.2X, FwVer 0x%.4X\r\n", SESSION_REG(xBNRG_HW_ver), SESSION_REG(xBNRG_FW_ver));
  
  PRINTF("Init_BlueNRG_Stack\r\n");
  Init_BlueNRG_Stack();

  /* Initialize the BlueNRG Custom services */
  PRINTF("Init_BlueNRG_Custom_Services\r\n");
  Init_BlueNRG_Custom_Services();

  /* Set output power level */
  uint8_t h = ((*((uint16_t*)(uint32_t)TX_PWR_LVL_CONTROL_REGADDR)) & 0x0100) >> 8;
  uint8_t p = ((*((uint16_t*)(uint32_t)TX_PWR_LVL_CONTROL_REGADDR)) & 0x07);

  if(TxPwrLvlChange(0,0,(uint8_t*)TX_PWR_LVL_CONTROL_REGADDR))
  {
    //default values
    p=DEFAULT_BLE_TX_POWER_LEVEL;
    h=DEFAULT_BLE_TX_HIGH_POWER;
  }

  aci_hal_set_tx_power_level(h,p);

  BluenrgResetParameters();
  ENABLED_DEBUG_LEVEL_BLE = nTempBleEnabledDebug;

}

/**
 * @brief Get BlueNRG hw and fw version
 * @retval None
 */
uint8_t getBlueNRGVersion(uint8_t *hwVersion, uint16_t *fwVersion)
{
  uint8_t status;
  uint8_t hci_version, lmp_pal_version;
  uint16_t hci_revision, manufacturer_name, lmp_pal_subversion;

  status = hci_le_read_local_version(&hci_version, &hci_revision, &lmp_pal_version,
				     &manufacturer_name, &lmp_pal_subversion);

  if (status == BLE_STATUS_SUCCESS) {
    *hwVersion = hci_revision >> 8;
    *fwVersion = (hci_revision & 0xFF) << 8;              // Major Version Number
    *fwVersion |= ((lmp_pal_subversion >> 4) & 0xF) << 4; // Minor Version Number
    *fwVersion |= lmp_pal_subversion & 0xF;               // Patch Version Number
  }

  HCI_Process();

  return status;
}

/**
 * @brief Check BlueNRG hw and fw version
 * @retval None
 */
void Check_BlueNRG_Versions()
{
  uint8_t hwVersion = 0;
  uint16_t fwVersion = 0;

  /* WARNING: Get BlueNRG stack version.  */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  /* WARNING: The only HW BlueNRG cut version supported
  is greater or equal than 3.0 */
  if ((hwVersion < 0x30/*HW_CUT_30*/) && (hwVersion != 0))
  {
    /* Go to infinite loop if BlueNRG hw version is not 3.1 */
    APP_BSP_LED_On(LED);
    while(1);
  }

  SESSION_REG(xBNRG_HW_ver) = hwVersion;
  SESSION_REG(xBNRG_FW_ver) = fwVersion;
#if BLUENRG_MS
  if(SESSION_REG(xBNRG_HW_ver) != 0x31)
  {
    /* Go to infinite loop if BlueNRG hw version is not 3.1 */
    APP_BSP_LED_On(LED);
    PRINTF("\r\nWrong BlueNRG hw version\r\n");
    while(1);
  }
#else //BLUENRG_MS
  if(SESSION_REG(xBNRG_HW_ver) != 0x30)
  {
    /* Go to infinite loop if BlueNRG hw version is not 3.0 */
    APP_BSP_LED_On(LED);
    PRINTF("\r\nWrong BlueNRG hw version\r\n");
    while(1);
  }
#endif //BLUENRG_MS
}

/** @brief Initialize all the devices in the BLE SubSystem
 * @retval None
 */
void Init_BlueNRG_Hw(void)
{
  tBleStatus ret;

  /* Initialize the BlueNRG SPI driver */
  BNRG_SPI_Init();

  BlueNRG_RST_AndKeep();
  bIsConnected = 0;

  /* Initialize the BlueNRG HCI */
  HCI_Init();

  /* Reset BlueNRG hardware */
  BlueNRG_RST();

  /* Check BlueNRG HW/FW version */
  Check_BlueNRG_Versions();

  /* Reset BlueNRG hardware */
  BlueNRG_RST();

  /* Get BlueNRG mac address configuration */
  WESU_GET_MAC_ADDRESS(WesuAddr,6);

  WESU_GET_NAME(WesuName, 7);

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  WesuAddr/*bdaddr*/);
  if(ret){
     PRINTF("\r\nSetting BD_ADDR failed\r\n");
     goto fail;
  }

  ret = aci_gatt_init();
  if(ret){
     PRINTF("\r\nGATT_Init failed\r\n");
     goto fail;
  }
fail:
  return;
}

/** @brief Initialize the BLE STACK
 * @retval None
 */
void Init_BlueNRG_Stack(void)
{
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;

#if BLUENRG_MS
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, strlen(WesuName)-1, &service_handle,
                     &dev_name_char_handle, &appearance_char_handle);
#else
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, &service_handle,
                     &dev_name_char_handle, &appearance_char_handle);
#endif

  if(ret != BLE_STATUS_SUCCESS){
     PRINTF("\r\nGAP_Init failed\r\n");
     goto fail;
  }

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   strlen(WesuName)-1, (uint8_t *)&WesuName[1]);

  if(ret){
     PRINTF("\r\naci_gatt_update_char_value failed\r\n");
    while(1);
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL, 7, 16,
                                     USE_FIXED_PIN_FOR_PAIRING, 123456,
                                     BONDING);
  if (ret != BLE_STATUS_SUCCESS) {
     PRINTF("\r\nGAP setting Authentication failed\r\n");
     goto fail;
  }

  PRINTF("SERVER: BLE Stack Initialized \r\n\t\tWesuName= %s\r\n\t\tBoardMAC = %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\r\n\n",
         WesuName,
		 WesuAddr[5],WesuAddr[4],WesuAddr[3],WesuAddr[2],WesuAddr[1],WesuAddr[0]);
  return;

fail:
  return;
}


/** @brief Initialize all the Custom BlueNRG services
 * @retval None
 */
void Init_BlueNRG_Custom_Services(void)
{
  int ret;

  ret = Add_Feature_Service();
  if(ret == BLE_STATUS_SUCCESS)
     PRINTF("Feature Service  added successfully\r\n");
  else
     PRINTF("\r\nError while adding Feature Service\r\n");

  ret = Add_Console_Service();
  if(ret == BLE_STATUS_SUCCESS)
     PRINTF("Console Service added successfully\r\n");
  else
     PRINTF("\r\nError while adding Console Service\r\n");

  ret = Add_Config_Service();
  if(ret == BLE_STATUS_SUCCESS)
     PRINTF("Config  Service added successfully\r\n");
  else
     PRINTF("\r\nError while adding Config Service\r\n");
}


/**
 * @brief  Configure all the Registers Session and Permanent
 * @retval void
 */
void DebugConfiguration()
{
  static RegsStruct_t *SessionRegs;
  static RegsStruct_t *PersistentRegs;
  uint8_t a;

  PersistentRegs = (RegsStruct_t *)PERMREG_STRUCT_START_ADDRESS;
  SessionRegs = (RegsStruct_t *)SESSIONREG_STRUCT_START_ADDRESS;

  //The following section can be removed
  a = PersistentRegs->nRegister00 & 0xFF;
  a = SessionRegs->nRegister00 & 0xFF;
  a = nFwVer[0];
  a = 0;

  if(a){while(a){}}

  bDebuggerConnected = 0;

  /* Uncomment the following line to enable the "printf" output on IAR Terminal I/O */
  /* Warning: be sure you are using the board connected to the debugger, otherwise an HardFault is issued */
  /* bDebuggerConnected = 1;*/

  BSP_DbgDisable();
}


/** @brief SystemReconfigHw
 * @retval Bitmask for startup flag identification
 */
void SystemReconfigHw()
{
  if(xCurSmFunc!=WESU_SYS_POWER_UNKNOWN)
  {
    /* Set an invalid power mode in order to trigger the system reconfiguration */
    xCurSmFunc = WESU_SYS_POWER_RECONFIG_HW;
  }
}

/** @brief WesuGetSystemStartup
 * @retval Bitmask for startup flag identification
 */
uint32_t WesuGetSystemStartup()
{
  uint32_t nStartup = 0;
  if(__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
  {
    nStartup |= SYSTEM_STARTUP_FLAG_POWERON;
  }
  
  if(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
  {
    nStartup |= SYSTEM_STARTUP_FLAG_SOFT_RESET;
  }
  else if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    nStartup |= SYSTEM_STARTUP_FLAG_IWDG_RESET;
  }
  else if(__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
  {
    nStartup |= SYSTEM_STARTUP_FLAG_RESET_PIN;
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
  return nStartup;
}


/** @brief UpdateSubSampling: API to update Subsampling in different condition
 *  @param 
 *  @retval None
 */
void UpdateSubSampling()
{
  if(SESSION_REG(nTimerFrequency)<= 25)
  {
    SESSION_REG(nPwrSubSampl)           = 1;
    SESSION_REG(nTemperatureSubSampl)   = 1;
    SESSION_REG(nPressSubSampl)         = 1;
    SESSION_REG(nRawMotionSubSampl)     = 1;
    SESSION_REG(nPedometerSubSampl)     = 1;
    SESSION_REG(nMotionFxSubSampl)      = 1;
    SESSION_REG(nMotionArSubSampl)      = 1;
    SESSION_REG(nMotionCpSubSampl)      = 1;
    SESSION_REG(nAccEventSubSampl)      = 1;
  }
  else
  {
    SESSION_REG(nPwrSubSampl)           = ALGORITHM_SUBSAMPL_MASK(SESSION_DEFAULT_REG_0x25);
    SESSION_REG(nTemperatureSubSampl)   = ALGORITHM_SUBSAMPL_MASK(SESSION_DEFAULT_REG_0x26);
    SESSION_REG(nPressSubSampl)         = ALGORITHM_SUBSAMPL_MASK(SESSION_DEFAULT_REG_0x28);
    SESSION_REG(nRawMotionSubSampl)     = ALGORITHM_SUBSAMPL_MASK(SESSION_DEFAULT_REG_0x2B);
    SESSION_REG(nPedometerSubSampl)     = ALGORITHM_SUBSAMPL_MASK(SESSION_DEFAULT_REG_0x35);
    SESSION_REG(nMotionFxSubSampl)      = ALGORITHM_SUBSAMPL_MASK(SESSION_DEFAULT_REG_0x3D);
    SESSION_REG(nMotionArSubSampl)      = ALGORITHM_SUBSAMPL_MASK(SESSION_DEFAULT_REG_0x39);
    SESSION_REG(nMotionCpSubSampl)      = ALGORITHM_SUBSAMPL_MASK(SESSION_DEFAULT_REG_0x38);
    SESSION_REG(nAccEventSubSampl)      = ALGORITHM_SUBSAMPL_MASK(SESSION_DEFAULT_REG_0x40);
  }
}


/** @brief SystemGotoShutdown
 * @retval None
 */
void SystemGotoShutdown()
{
  APP_BSP_LED_On(LED);
  HAL_Delay(500);
#if USE_STC3115
  BSP_GG_Stop();
#endif //USE_STC3115
  BSP_PWMGT_BoardShutdown();
  while(1)
  {
    APP_BSP_LedSmoothBlink(50);
  }
}


/** @brief SensorsShutdown
 * @retval None
 */
void SensorsShutdown()
{
  DISABLE_SENSORS_READ_ON_INTERRUPT();
  
  BSP_PRESSURE_Sensor_Disable( PRESSURE_handle );
  BSP_TEMPERATURE_Sensor_Disable( TEMPERATURE_handle );
  
  BSP_ACCELERO_Sensor_Disable( ACCELERO_handle );
  
  BSP_GYRO_Sensor_Disable( GYRO_handle );

  BSP_MAGNETO_Sensor_Disable( MAGNETO_handle );
}

void SensorsPrintInitStatus()
{
  if(sensorStatusAccelero == COMPONENT_OK)
  {
    PRINTF("BSP_ACCELERO_Init OK\r\n");
  }
  else
  {
    PRINTF("BSP_ACCELERO_Init FAIL\r\n");
  }
  
  if(sensorStatusGyro == COMPONENT_OK)
  {
    PRINTF("BSP_GYRO_Init OK\r\n");
  }
  else
  {
    PRINTF("BSP_GYRO_Init FAIL\r\n");
  }
  
  if(sensorStatusMag == COMPONENT_OK)
  {
    PRINTF("BSP_MAGNETO_Init OK\r\n");
  }
  else
  {
    PRINTF("BSP_MAGNETO_Init FAIL\r\n");
  }
  
  if(sensorStatusPress == COMPONENT_OK)
  {
    PRINTF("BSP_PRESSURE_Init OK\r\n");
  }
  else
  {
    PRINTF("BSP_PRESSURE_Init FAIL\r\n");
  }
  
  if(sensorStatusTemp == COMPONENT_OK)
  {
    PRINTF("BSP_TEMPERATURE_Init OK\r\n");
  }
  else
  {
    PRINTF("BSP_TEMPERATURE_Init FAIL\r\n");
  }
}

/** @brief SensorsConfigLowPowerModeWith Algos and Event from Accelerometer
 * @retval None
 */
void SensorsConfigLowPowerRunWithAlgos()
{
  DISABLE_SENSORS_READ_ON_INTERRUPT();

  SensorsConfigInit();
  
  /*******************/
    Algorithms_Init(0);
  /*******************/

  /* Configure the MEMS sensors to run in low power */

  if(READ_SENSORS_ACC_ENABLED())
  {
    // Low Performance disabled
    BSP_ACCELERO_Set_LP_Mode(ACCELERO_handle,LSM6DS3_ACC_GYRO_LP_XL_ENABLED);

    uint16_t nData;
    nData = ACC_FullScale_LOWPOWER;
    AccFsChanged(0,0,(uint8_t*)&nData);
    if(nData != ACC_FullScale_LOWPOWER)
    {
      BSP_PERSREGS_WRITE(ACC_FullScale_REG, (uint8_t*)&nData, 1);
    }

    nData = ACC_OutputDataRate_ALGOS;
    AccOdrChanged(0,0,(uint8_t*)&nData);
    if(nData != ACC_OutputDataRate_Macro)
    {
      BSP_PERSREGS_WRITE(ACC_OutputDataRate_REG, (uint8_t*)&nData, 1);
    }
  }

  if(READ_SENSORS_GYRO_ENABLED())
  {
    BSP_GYRO_Set_LP_Mode(GYRO_handle, LSM6DS3_ACC_GYRO_LP_EN_ENABLED);
    
    uint16_t nData;
    nData = GYRO_FullScale_LOWPOWER;
    GyroFsChanged(0,0,(uint8_t*)&nData);
    if(nData != GYRO_FullScale_LOWPOWER)
    {
      BSP_PERSREGS_WRITE(GYRO_FullScale_REG, (uint8_t*)&nData, 1);
    }

    nData = GYRO_OutputDataRate_ALGOS;
    GyroOdrChanged(0,0,(uint8_t*)&nData);
    if(nData != GYRO_OutputDataRate_Macro)
    {
      BSP_PERSREGS_WRITE(GYRO_OutputDataRate_REG, (uint8_t*)&nData, 1);
    }
  }

  if(READ_SENSORS_MAG_ENABLED())
  {
    BSP_MAGNETO_Set_LP_Mode( MAGNETO_handle, LIS3MDL_MAG_LP_ENABLE);
  }
  
  if(READ_SENSORS_TEMP_ENABLED())
  {
    BSP_PRESSURE_Set_ODR_Value(PRESSURE_handle,PRESS_TEMP_OutputDataRate_LOWPOWER);     // ODR= 1Hz
    BSP_PRESSURE_Set_AvgT(PRESSURE_handle,LPS25HB_AVGT_8);                              // AVG= 8 samples
  }

  if(READ_SENSORS_PRESS_ENABLED())
  {
    BSP_PRESSURE_Set_ODR_Value(PRESSURE_handle,PRESS_TEMP_OutputDataRate_LOWPOWER);     // ODR= 1Hz
    BSP_PRESSURE_Set_AvgP(PRESSURE_handle,LPS25HB_AVGP_8);                              // AVG= 8 samples
  }
}

/** @brief SensorsConfigLowPowerRun
 * @retval None
 */
void SensorsConfigLowPowerRun()
{
  DISABLE_SENSORS_READ_ON_INTERRUPT();  

  SensorsConfigInit();

  /* Configure the MEMS sensors to run in low power */

  if(READ_SENSORS_ACC_ENABLED())
  {     // Low Performance enabled
    BSP_ACCELERO_Set_LP_Mode(ACCELERO_handle,LSM6DS3_ACC_GYRO_LP_XL_ENABLED);
  }

  if(READ_SENSORS_GYRO_ENABLED())
  {     // Low Performance enabled
    BSP_GYRO_Set_LP_Mode(GYRO_handle, LSM6DS3_ACC_GYRO_LP_EN_ENABLED);
  }

  if(READ_SENSORS_MAG_ENABLED())
  {
    BSP_MAGNETO_Set_LP_Mode( MAGNETO_handle, LIS3MDL_MAG_LP_ENABLE);
  }

  if(READ_SENSORS_TEMP_ENABLED())
  {
    BSP_PRESSURE_Set_ODR_Value(PRESSURE_handle,PRESS_TEMP_OutputDataRate_LOWPOWER);     // ODR= 1Hz
    BSP_PRESSURE_Set_AvgT(PRESSURE_handle,LPS25HB_AVGT_8);                              // AVG= 8 samples
  }

  if(READ_SENSORS_PRESS_ENABLED())
  {
    BSP_PRESSURE_Set_ODR_Value(PRESSURE_handle,PRESS_TEMP_OutputDataRate_LOWPOWER);     // ODR= 1Hz
    BSP_PRESSURE_Set_AvgP(PRESSURE_handle,LPS25HB_AVGP_8);                              // AVG= 8 samples
  }
}

/** @brief SensorsConfigJustAcc
 * @retval None
 */
void SensorsConfigJustAcc()
{
  DISABLE_SENSORS_READ_ON_INTERRUPT();  //To force the Read_Sensor in the Main
  
  SensorsConfigInit();
 
  /* Configure the MEMS sensors to run in low power */
  if(READ_SENSORS_ACC_ENABLED())
  {     // Low Performance enabled
    BSP_ACCELERO_Set_LP_Mode(ACCELERO_handle,LSM6DS3_ACC_GYRO_LP_XL_ENABLED);
  }
}


/** @brief SensorsConfigInit
 * Initialize and Enable/Disable the Sensors in according with the Macro
 * @retval None
 */
void SensorsConfigInit()
{
  SUSPEND_SENSORS_READ_ON_INTERRUPT();
  if(READ_SENSORS_ACC_ENABLED())
    {
      PRINTF("ACCELEROMETER ENABLING\r\n");
      sensorStatusAccelero = BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &ACCELERO_handle );
      BSP_ACCELERO_Sensor_Enable( ACCELERO_handle );
    }
  else
    {
      PRINTF("ACCELEROMETER DISABLING\r\n");
      sensorStatusAccelero = BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &ACCELERO_handle );
      BSP_ACCELERO_Sensor_Disable( ACCELERO_handle );
    }  
  
  if(READ_SENSORS_GYRO_ENABLED())
    {
      PRINTF("GYROSCOPE ENABLING\r\n");
      sensorStatusGyro = BSP_GYRO_Init( GYRO_SENSORS_AUTO, &GYRO_handle );
      BSP_GYRO_Sensor_Enable( GYRO_handle );
    }
  else
    {
      PRINTF("GYROSCOPE DISABING\r\n");
      sensorStatusGyro = BSP_GYRO_Init( GYRO_SENSORS_AUTO, &GYRO_handle );
      BSP_GYRO_Sensor_Disable( GYRO_handle );
    }

  if(READ_SENSORS_MAG_ENABLED())
    {
      PRINTF("MAGNETOMETER ENABLING\r\n");
      sensorStatusMag = BSP_MAGNETO_Init( MAGNETO_SENSORS_AUTO, &MAGNETO_handle );
      BSP_MAGNETO_Sensor_Enable( MAGNETO_handle );
    }
  else
    {
      PRINTF("MAGNETOMETER DISABLING\r\n");
      sensorStatusMag = BSP_MAGNETO_Init( MAGNETO_SENSORS_AUTO, &MAGNETO_handle );
      BSP_MAGNETO_Sensor_Disable( MAGNETO_handle );
    }

  if(READ_SENSORS_PRESS_ENABLED())
    {
      PRINTF("PRESSURE ENABLING\r\n");
      sensorStatusPress = BSP_PRESSURE_Init( PRESSURE_SENSORS_AUTO, &PRESSURE_handle );
      BSP_PRESSURE_Sensor_Enable( PRESSURE_handle );
    }
  else
    {
      PRINTF("PRESSURE DISABLING\r\n");
      sensorStatusPress = BSP_PRESSURE_Init( PRESSURE_SENSORS_AUTO, &PRESSURE_handle );
      BSP_PRESSURE_Sensor_Disable( PRESSURE_handle );
    }

  if(READ_SENSORS_TEMP_ENABLED())
    {
      PRINTF("TEMPERATURE ENABLING\r\n");
      sensorStatusTemp = BSP_TEMPERATURE_Init( TEMPERATURE_SENSORS_AUTO, &TEMPERATURE_handle );
      BSP_TEMPERATURE_Sensor_Enable( TEMPERATURE_handle );
    }
  else
    {
      PRINTF("TEMPERATURE DISABLING\r\n");
      sensorStatusTemp = BSP_TEMPERATURE_Init( TEMPERATURE_SENSORS_AUTO, &TEMPERATURE_handle );
      BSP_TEMPERATURE_Sensor_Disable( TEMPERATURE_handle );
    }
  
    LSM6DS3_ACC_GYRO_W_DRDY_XL_on_INT2(GYRO_handle, LSM6DS3_ACC_GYRO_INT2_DRDY_XL_DISABLED);
    LSM6DS3_ACC_GYRO_W_DRDY_TEMP_on_INT2(GYRO_handle, LSM6DS3_ACC_GYRO_INT2_DRDY_TEMP_DISABLED);
    LSM6DS3_ACC_GYRO_W_FIFO_TSHLD_on_INT2(GYRO_handle, LSM6DS3_ACC_GYRO_INT2_FTH_DISABLED);
    LSM6DS3_ACC_GYRO_W_OVERRUN_on_INT2(GYRO_handle, LSM6DS3_ACC_GYRO_INT2_OVR_DISABLED);
    LSM6DS3_ACC_GYRO_W_FSS5_on_INT2(GYRO_handle, LSM6DS3_ACC_GYRO_INT2_FSS5_DISABLED);
    LSM6DS3_ACC_GYRO_W_SIGN_MOT_on_INT2(GYRO_handle, LSM6DS3_ACC_GYRO_INT2_SIGN_MOT_DISABLED);
    LSM6DS3_ACC_GYRO_W_PEDO_STEP_on_INT2(GYRO_handle, LSM6DS3_ACC_GYRO_INT2_PEDO_DISABLED);
    LSM6DS3_ACC_GYRO_W_DRDY_G_on_INT2(GYRO_handle, LSM6DS3_ACC_GYRO_INT2_DRDY_G_DISABLED);
    
  if(READ_SENSORS_GYRO_ENABLED())
  {
    PRINTF("GYROSCOPE LSM6DS3_ACC_GYRO_W_DRDY_G_on_INT2\r\n");
    LSM6DS3_ACC_GYRO_W_DRDY_G_on_INT2(GYRO_handle, LSM6DS3_ACC_GYRO_INT2_DRDY_G_ENABLED);
  }
  else
  {
    LSM6DS3_ACC_GYRO_W_DRDY_G_on_INT2(GYRO_handle, LSM6DS3_ACC_GYRO_INT2_DRDY_G_DISABLED);
  }
}



/** @brief SensorsConfigFullRun
 * @retval None
 */
void SensorsConfigFullRun()
{
  SUSPEND_SENSORS_READ_ON_INTERRUPT();
  
  SensorsConfigInit();
  
  /*******************/ 
  Algorithms_Init(0);
  /*******************/

  if(READ_SENSORS_ACC_ENABLED())
  { // Low Performance disabled
    BSP_ACCELERO_Set_LP_Mode(ACCELERO_handle,LSM6DS3_ACC_GYRO_LP_XL_DISABLED);

    uint16_t nData;
    nData = ACC_FullScale_Macro;
    AccFsChanged(0,0,(uint8_t*)&nData);
    if(nData != ACC_FullScale_Macro)
    {
      BSP_PERSREGS_WRITE(ACC_FullScale_REG, (uint8_t*)&nData, 1);
    }

    nData = ACC_OutputDataRate_Macro;
    AccOdrChanged(0,0,(uint8_t*)&nData);
    if(nData != ACC_OutputDataRate_Macro)
    {
      BSP_PERSREGS_WRITE(ACC_OutputDataRate_REG, (uint8_t*)&nData, 1);
    }
  }
  if(READ_SENSORS_GYRO_ENABLED())
  { // Low Performance disabled
    BSP_GYRO_Set_LP_Mode(GYRO_handle, LSM6DS3_ACC_GYRO_LP_EN_DISABLED);

    uint16_t nData;
    nData = GYRO_FullScale_Macro;
    GyroFsChanged(0,0,(uint8_t*)&nData);
    if(nData != GYRO_FullScale_Macro)
    {
      BSP_PERSREGS_WRITE(GYRO_FullScale_REG, (uint8_t*)&nData, 1);
    }

    nData = GYRO_OutputDataRate_Macro;
    GyroOdrChanged(0,0,(uint8_t*)&nData);
    if(nData != GYRO_OutputDataRate_Macro)
    {
      BSP_PERSREGS_WRITE(GYRO_OutputDataRate_REG, (uint8_t*)&nData, 1);
    }
  }
  if(READ_SENSORS_MAG_ENABLED())
  {
    BSP_MAGNETO_Set_LP_Mode( MAGNETO_handle, LIS3MDL_MAG_LP_DISABLE);

    uint16_t nData;
    nData = MAG_FullScale_Macro;
    MagFsChanged(0,0,(uint8_t*)&nData);
    if(nData != MAG_FullScale_Macro)
    {
      BSP_PERSREGS_WRITE(MAG_FullScale_REG, (uint8_t*)&nData, 1);
    }

    nData = MAG_OutputDataRate_Macro;
    MagOdrChanged(0,0,(uint8_t*)&nData);
    if(nData != MAG_OutputDataRate_Macro)
    {
      BSP_PERSREGS_WRITE(MAG_OutputDataRate_REG, (uint8_t*)&nData, 1);
    }
  }

  if(READ_SENSORS_PRESS_ENABLED() || READ_SENSORS_TEMP_ENABLED() )
  {
    uint16_t nData;
    nData = PRESS_TEMP_OutputDataRate_Macro;
    PressOdrChanged(0,0,(uint8_t*)&nData);
    if(nData != PRESS_TEMP_OutputDataRate_Macro)
    {
      BSP_PERSREGS_WRITE(PRESS_TEMP_OutputDataRate_REG, (uint8_t*)&nData, 1);
    }
  }
  if( PROCESS_MOTION_FX_ENABLED() && READ_SENSORS_GYRO_ENABLED() )
  {
    ENABLE_SENSORS_READ_ON_INTERRUPT();
    RESUME_SENSORS_READ_ON_INTERRUPT();
  }
}

/** @brief SystemGotoFullRunMode
 * @retval None
 */
void SystemGotoFullRunMode()
{
  SESSION_REG(nTimerFrequency) = SESSION_DEFAULT_REG_0x21;
  SESSION_REG(nReadSensorMask) = SESSION_DEFAULT_REG_0x49;
  SESSION_REG(nProcessFwMask) = SESSION_DEFAULT_REG_0x4A;
  SESSION_REG(nLedBlinkMode) = LED_MODE_SMART_BLINKING;
  
  UpdateSubSampling();
  
  bConnParamUpdate = bIsConnected;
  SESSION_REG(nBleConIntv) = SESSION_DEFAULT_REG_0x4B;
  
  SystemClock_Config_APP = SystemClock_Config_RTC_HSE32MHz;
  SystemClock_Config_APP();
  
  SensorsConfigFullRun();
  SESSION_REG(nRsvdDummyRegDebug) = 0;
}

/** @brief SystemGotoHardwareReconfig
 * @retval None
 */
void SystemGotoHardwareReconfig()
{
  DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "System hardware reconfiguration\r\n");
  UpdateSubSampling();
  
  SensorsConfigFullRun();
}


/** @brief SystemGotoFullRunWfiMode
 * @retval None
 */
void SystemGotoFullRunWfiMode()
{
  SESSION_REG(nTimerFrequency) = SESSION_DEFAULT_REG_0x21;
  SESSION_REG(nReadSensorMask) = SESSION_DEFAULT_REG_0x49;
  SESSION_REG(nProcessFwMask) = SESSION_DEFAULT_REG_0x4A;
  SESSION_REG(nLedBlinkMode) = LED_MODE_SMART_BLINKING;
  
  UpdateSubSampling();
    
  bConnParamUpdate = bIsConnected;
  SESSION_REG(nBleConIntv) = SESSION_DEFAULT_REG_0x4B;

  SystemClock_Config_APP = SystemClock_Config_RTC_HSE32MHz;
  SystemClock_Config_APP();

  SensorsConfigFullRun();
  SESSION_REG(nRsvdDummyRegDebug) = 1;
}


/** @brief SystemGotoLowPowerRunMode: TimerFreq @50 
 *                                    All the sensors in FULLRUN
 *                                    All Algos with SYSCLK @24MHz 
 * @retval None
 */
void SystemGotoLowPowerRunMode()
{
  SESSION_REG(nTimerFrequency) = 50;
  SESSION_REG(nReadSensorMask) = 0;          
  SESSION_REG(nReadSensorMask) = ACC_ENABLED_MASK_  | \
                                 GYRO_ENABLED_MASK_ | \
                                 MAG_ENABLED_MASK_  | \
                                 PRESS_ENABLED_MASK_| \
                                 TEMP_ENABLED_MASK_ | \
                                 FUEL_GAUGE_ENABLED_MASK_;
  
  UpdateSubSampling();

  SESSION_REG(nProcessFwMask) = SESSION_DEFAULT_REG_0x4A; 
  
  SESSION_REG(nLedBlinkMode) = LED_MODE_SMART_BLINKING;

  SensorsConfigFullRun();
  SESSION_REG(nRsvdDummyRegDebug) = 1;

  SystemClock_Config_APP = SystemClock_Config_HSE_24MHz;
  SystemClock_Config_APP();
}


/** @brief SystemGotoLowPowerRunWithAlgos: TimerFreq @50 
 *                                         No GYRO
 *                                         No MotionFX, just AR-CP-AE
 * @retval None
 */
void SystemGotoLowPowerRunWithAlgos()
{
  SESSION_REG(nTimerFrequency) = 50;
  SESSION_REG(nReadSensorMask) = 0;           // To Disable Gyro & Mag Not Used
  SESSION_REG(nReadSensorMask) = ACC_ENABLED_MASK_  | \
                                 PRESS_ENABLED_MASK_| \
                                 TEMP_ENABLED_MASK_ | \
                                 FUEL_GAUGE_ENABLED_MASK_;
  
  UpdateSubSampling();

  SESSION_REG(nProcessFwMask) = 0;            // To Disable the Algos not Used
  SESSION_REG(nProcessFwMask) = ALGORITHM1_BLUEST_MASK | \
                                ALGORITHM2_BLUEST_MASK | \
                                ACCEVENT_BLUEST_MASK;
  
  SESSION_REG(nLedBlinkMode) = LED_MODE_SMART_BLINKING;

  SensorsConfigLowPowerRunWithAlgos();
  SESSION_REG(nRsvdDummyRegDebug) = 1;

  SystemClock_Config_APP = SystemClock_Config_MSI_4MHz;
  SystemClock_Config_APP();
}

/** @brief SystemGotoRunModeJustAcc:TimerFreq @50 
 *                                  Just Acc & Batt
 *                                  No MotionFX, just AR-CP-AE
 * @retval None
 */
void SystemGotoLowPowerRunModeJustAcc()
{
    SESSION_REG(nTimerFrequency) =  25;
    SESSION_REG(nReadSensorMask) =  ACC_ENABLED_MASK_ | \
                                    FUEL_GAUGE_ENABLED_MASK_;
    UpdateSubSampling();


    SESSION_REG(nProcessFwMask) = 0; // To Disable the Algos not Used
    SESSION_REG(nProcessFwMask) = ACCEVENT_BLUEST_MASK;
    SESSION_REG(nLedBlinkMode) = LED_MODE_SMART_BLINKING;

  SensorsConfigJustAcc();
  SESSION_REG(nRsvdDummyRegDebug) = 1;

  SystemClock_Config_APP = SystemClock_Config_MSI_4MHz;
  SystemClock_Config_APP();
}


/** @brief SystemGotoLowPowerRunMode: TimerFreq @25 
 *                                    All the sensors in LOWPOWER
 *                                    No Algos 
 * @retval None
 */
void SystemGotoLowPowerMode()
{
  SESSION_REG(nTimerFrequency) =  25;
  
  SESSION_REG(nReadSensorMask) =  ACC_ENABLED_MASK_  | \
                                  GYRO_ENABLED_MASK_ | \
                                  MAG_ENABLED_MASK_  | \
                                  PRESS_ENABLED_MASK_| \
                                  TEMP_ENABLED_MASK_ | \
                                  FUEL_GAUGE_ENABLED_MASK_;
  UpdateSubSampling();
  
  SESSION_REG(nProcessFwMask) = 0;    // No Algos
  SESSION_REG(nProcessFwMask) = ACCEVENT_BLUEST_MASK;
  SESSION_REG(nLedBlinkMode)  = LED_MODE_SMART_BLINKING;
  
  SensorsConfigLowPowerRun();
  SESSION_REG(nRsvdDummyRegDebug) = 1;

  SystemClock_Config_APP = SystemClock_Config_MSI_4MHz;
  SystemClock_Config_APP();
}

/** @brief SystemGotoRunSleepMode: TimerFreq @50 
 *                                 Just Acc & Batt
 *                                 No MotionFX, just AR-CP-AE 
 * @retval None
 */
void SystemGotoRunSleepMode()
{
    SESSION_REG(nTimerFrequency) =  1;
    SESSION_REG(nPwrSubSampl) = 1;

    SESSION_REG(nReadSensorMask) =  ACC_ENABLED_MASK_ | \
                                    FUEL_GAUGE_ENABLED_MASK_;
    UpdateSubSampling();


    SESSION_REG(nProcessFwMask) = 0; // No FW Algos
    SESSION_REG(nProcessFwMask) = ACCEVENT_BLUEST_MASK;
    
    SESSION_REG(nLedBlinkMode) = LED_MODE_MEASURE_TIMER_EXP_EVENT_PROC;
    
    SESSION_REG(nLPMask) = LPCFG_MCU_ENTER_SLEEP | \
                           LPCFG_CHANGE_MSI_CLK;

    SensorsConfigJustAcc();
    SESSION_REG(nRsvdDummyRegDebug) = 1;

    SystemClock_Config_APP =  SystemClock_Config_MSI_4MHz;
    SystemClock_Config_APP();
}


/** @brief SystemGotoLowPowerRunMode using all the sensors in a low power configuration
 * @retval None
 */
void SystemGotoRunLowPowerSleepMode()
{
    SESSION_REG(nTimerFrequency) =  1;
    SESSION_REG(nReadSensorMask) =   ACC_ENABLED_MASK_ | \
                                     FUEL_GAUGE_ENABLED_MASK_;
    UpdateSubSampling();
 
    SESSION_REG(nProcessFwMask) = 0; // No MotionFX
    SESSION_REG(nProcessFwMask) = ACCEVENT_BLUEST_MASK;
    
    SESSION_REG(nLedBlinkMode) = LED_MODE_MEASURE_TIMER_EXP_EVENT_PROC;

    SESSION_REG(nLPMask) = LPCFG_MCU_ENTER_LP_SLEEP | \
                           LPCFG_CHANGE_MSI_CLK;
    SensorsConfigJustAcc();
    SESSION_REG(nRsvdDummyRegDebug) = 1;

    SystemClock_Config_APP = SystemClock_Config_MSI_4MHz;
    SystemClock_Config_APP();
}




/** @brief SystemGotoPermanentStopBlePmMode
 *  
 *  @retval None
 */
void SystemGotoPermanentStopBlePmMode()
{
#if USE_STC3115
  if(*GG_PWRDOWN_MODE_REGADDR)
  {
    BSP_GG_PowerSavingMode();
  }
  else
  {
    BSP_GG_Stop();
  }
#endif //USE_STC3115

  SESSION_REG(nTimerFrequency) =  1;
  SESSION_REG(nReadSensorMask) =  ACC_ENABLED_MASK_;
  
  UpdateSubSampling();

  SESSION_REG(nProcessFwMask) = 0; // No Algos
  SESSION_REG(nProcessFwMask) = ACCEVENT_BLUEST_MASK;

  SESSION_REG(nLedBlinkMode) = LED_MODE_OFF;

  SensorsConfigJustAcc();
  BluenrgInitialization();
  uint16_t ai = *BLE_ADV_INTV_SLEEP_REGADDR;
  if(AdvIntvChange(0,0,(uint8_t*)BLE_ADV_INTV_SLEEP_REGADDR))
  {
    //default values
    ai=0x0800;
  }
  
  PRINTF("setConnectable()\r\n");
  setConnectable(ai, ai, 1);
  HCI_Process();

  if(BSP_PB_GetState(BUTTON_USER))
  {
    SystemGotoShutdown();
  }
  else
  {
    SESSION_REG(bLP) = 1;
    SESSION_REG(nLPMask) = LPCFG_MCU_ENTER_STOP_W_BLE | \
                           LPCFG_TIMERS_SHUTDOWN | \
                           LPCFG_GPIO_AN_IN | \
                           LPCFG_SWITCH_TO_HSE_CLK;
  }
}

/** @brief SystemGotoPermanentStopBleMode
 * @retval None
 */
void SystemGotoPermanentStopBleMode()
{
#if USE_STC3115
  if(*GG_PWRDOWN_MODE_REGADDR)
  {
    BSP_GG_PowerSavingMode();
  }
  else
  {
    BSP_GG_Stop();
  }
#endif //USE_STC3115
  
  SensorsShutdown();
  
  BluenrgInitialization();
  uint16_t ai = *BLE_ADV_INTV_SLEEP_REGADDR;
  if(AdvIntvChange(0,0,(uint8_t*)BLE_ADV_INTV_SLEEP_REGADDR))
  {
    //default values
    ai=0x0800;
  }
  
  PRINTF("setConnectable()\r\n");
  setConnectable(ai, ai, 1);
  HCI_Process();
  
  if(BSP_PB_GetState(BUTTON_USER))
  {
    SystemGotoShutdown();
  }
  else
  {
    SESSION_REG(bLP) = 1;
    SESSION_REG(nLPMask) = LPCFG_MCU_ENTER_STOP_W_BLE | \
                           LPCFG_TIMERS_SHUTDOWN | \
                           LPCFG_GPIO_AN_IN | \
                           LPCFG_SWITCH_TO_HSE_CLK;
  }
}


/** @brief SystemGotoPermanentStopMode
 * @retval None
 */
void SystemGotoPermanentStopMode()
{
#if USE_STC3115
  if(*GG_PWRDOWN_MODE_REGADDR)
  {
    BSP_GG_PowerSavingMode();
  }
  else
  {
    BSP_GG_Stop();
  }
#endif //USE_STC3115
  
  SensorsShutdown();
  
  Init_BlueNRG_Hw();
  
  if(BSP_PB_GetState(BUTTON_USER))
  {
    SystemGotoShutdown();
  }
  else
  {
    SESSION_REG(bLP) = 1;
    SESSION_REG(nLPMask) = LPCFG_MCU_ENTER_STOP_RESET | \
                           LPCFG_TIMERS_SHUTDOWN | \
                           LPCFG_GPIO_AN_IN | \
                           LPCFG_SWITCH_TO_HSE_CLK;
  }
}

/** @brief SystemGotoRebootMode
 * @retval None
 */
void SystemGotoRebootMode()
{
#if USE_STC3115
  if(*GG_PWRDOWN_MODE_REGADDR)
  {
    BSP_GG_PowerSavingMode();
  }
  else
  {
    BSP_GG_Stop();
  }
#endif //USE_STC3115
  
  SensorsShutdown();
  
  WESU_SYSTEM_RESET();
}


/** @brief SystemGotoRebootDefSettingsMode
 * @retval None
 */
void SystemGotoRebootDefSettingsMode()
{
#if USE_STC3115
  if(*GG_PWRDOWN_MODE_REGADDR)
  {
    BSP_GG_PowerSavingMode();
  }
  else
  {
    BSP_GG_Stop();
  }
#endif //USE_STC3115
  
  SensorsShutdown();
  
  WESU_SYSTEM_RESET();
}


/** 
 * @brief WesuSaveSystemParameters Save Persistent registers with system parameters
 * @retval None
 */
void WesuSaveSystemParameters()
{
  rtc_time_t t;
  memcpy(&t, (rtc_time_t *)&(SESSION_REG(tm_hour)), 8);
  switch(t.tm_rtcConf)
  {
  case RTC_AppConfigDefault:
    t.tm_rtcConf = RTC_AppConfigDefaultRunning;
    break;
  case RTC_AppConfigDefaultRunning:
    break;
  case RTC_AppConfigRestored:
    t.tm_rtcConf = RTC_AppConfigRestoredRunning;
    break;
  case RTC_AppConfigRestoredRunning:
    break;
  case RTC_AppConfigUser:
    t.tm_rtcConf = RTC_AppConfigUserRunning;
    break;
  case RTC_AppConfigUserRunning:
    break;
  case RTC_AppConfigForced:
    t.tm_rtcConf = RTC_AppConfigUserRunning;
    break;
  default:
    t.tm_rtcConf = RTC_AppConfigInvalid;
    break;
  }

  if(t.tm_rtcConf != RTC_AppConfigInvalid)
  {
    BSP_PERSREGS_WRITE(RTC_CONF_0_REG, (void*)&t, 4);
  }
  
  BSP_PERSREGS_WRITE(SYSTEM_TICKS_BACKUP_SESSION_REG, (void*)&SESSION_REG(nTicks), 2);
  BSP_PERSREGS_WRITE(STEPS_BACKUP_SESSION_REG, (void*)&SESSION_REG(nPedoStepCounter), 2);
  BSP_PERSREGS_WRITE(BATTERY_VOLTAGE_BACKUP_SESSION_REG, (void*)&SESSION_REG(fBatteryVolt), 2);
}

/** 
 * @brief WesuBackupSystemParameters Backup Persistent registers with system parameters
 * @param nMode one of the following: SYS_PARAMS_BACKUP_SESSION_DONE, SYS_PARAMS_BACKUP_POWERON_DONE
 * @retval None
 */
void WesuBackupSystemParameters(uint32_t nMode)
{
  switch(nMode)
  {
  case SYS_PARAMS_BACKUP_SESSION_DONE:
    BSP_PERSREGS_WRITE_BACKUP(RTC_CONF_0_REG, (void*)RTC_CONF_0_REGADDR, 4);
    
    BSP_PERSREGS_WRITE_BACKUP(SYSTEM_TICKS_BACKUP_SESSION_REG, (void*)SYSTEM_TICKS_BACKUP_SESSION_REGADDR, 2);
    BSP_PERSREGS_WRITE_BACKUP(STEPS_BACKUP_SESSION_REG, (void*)STEPS_BACKUP_SESSION_REGADDR, 2);
    BSP_PERSREGS_WRITE_BACKUP(BATTERY_VOLTAGE_BACKUP_SESSION_REG, (void*)BATTERY_VOLTAGE_BACKUP_SESSION_REGADDR, 2);
    break;
  case SYS_PARAMS_BACKUP_POWERON_DONE:
    BSP_PERSREGS_WRITE_BACKUP(RTC_BACKUP_CONF_0_REG, (void*)RTC_CONF_0_REGADDR, 4);
    
    BSP_PERSREGS_WRITE_BACKUP(SYSTEM_TICKS_BACKUP_POWERON_REG, (void*)SYSTEM_TICKS_BACKUP_SESSION_REGADDR, 2);
    BSP_PERSREGS_WRITE_BACKUP(STEPS_BACKUP_POWERON_REG, (void*)STEPS_BACKUP_SESSION_REGADDR, 2);
    BSP_PERSREGS_WRITE_BACKUP(BATTERY_VOLTAGE_BACKUP_POWERON_REG, (void*)BATTERY_VOLTAGE_BACKUP_SESSION_REGADDR, 2);
    break;
  }
}

/** 
 * @brief WesuGetBackupData Get Backup data: Persistent registers SYSTEM_TICKS_BACKUP_SESSION_REG, BATTERY_VOLTAGE_BACKUP_SESSION_REG, STEPS_BACKUP_SESSION_REG, 
 * @param nMode one of the following: SYS_PARAMS_BACKUP_SESSION_DONE:, SYS_PARAMS_BACKUP_POWERON_DONE
 * @param d pointer to output data
 * @retval None
 */
void WesuGetBackupData(uint32_t nMode, WesuBackupData_t *d)
{
  switch(nMode)
  {
  case SYS_PARAMS_BACKUP_SESSION_DONE:
    BSP_PERSREGS_READ_BACKUP(RTC_CONF_0_REG, (void*)&d->tm_hour, 4);
    
    BSP_PERSREGS_READ_BACKUP(SYSTEM_TICKS_BACKUP_SESSION_REG, (void*)&d->nTicks, 2);
    BSP_PERSREGS_READ_BACKUP(STEPS_BACKUP_SESSION_REG, (void*)&d->nSteps, 2);
    BSP_PERSREGS_READ_BACKUP(BATTERY_VOLTAGE_BACKUP_SESSION_REG, (void*)&d->fBatteryVolt, 2);
    break;
  case SYS_PARAMS_BACKUP_POWERON_DONE:
    BSP_PERSREGS_READ_BACKUP(RTC_BACKUP_CONF_0_REG, (void*)&d->tm_hour, 4);
    
    BSP_PERSREGS_READ_BACKUP(SYSTEM_TICKS_BACKUP_POWERON_REG, (void*)&d->nTicks, 2);
    BSP_PERSREGS_READ_BACKUP(STEPS_BACKUP_POWERON_REG, (void*)&d->nSteps, 2);
    BSP_PERSREGS_READ_BACKUP(BATTERY_VOLTAGE_BACKUP_POWERON_REG, (void*)&d->fBatteryVolt, 2);
    break;
  }
}

/** @brief WesuSysChangeRunMode
 * @param nMode one of the following: WESU_SYS_MODE_GO_IN_RUN or WESU_SYS_MODE_GO_IN_STOP
 * @retval None
 */
void WesuSysChangeRunMode(uint8_t nMode)
{
  if(xCurSmFunc == WESU_SYS_POWER_RECONFIG_HW)
  {
    DISABLE_SENSORS_READ_ON_INTERRUPT();
    SystemGotoHardwareReconfig();
    xCurSmFunc = SESSION_REG(nLPFunction);
  }
  if(xCurSmFunc != SESSION_REG(nLPFunction))
  {
    DISABLE_SENSORS_READ_ON_INTERRUPT();
    SystemClock_Config_APP_STARTUP();
    APP_BSP_LedSmoothBlink(10);

    if(nMode & WESU_SYS_MODE_GO_IN_RUN)
    {
      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_FULLRUN)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_FULLRUN\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
        SystemGotoFullRunMode();
      }
      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_FULLRUN_WFI)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_FULLRUN_WFI\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
        SystemGotoFullRunWfiMode();
      }
      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_CONN_ONLY)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_CONN_ONLY\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
        SystemGotoFullRunWfiMode();
      }

      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_LOWPOWER_RUN)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_LOWPOWER_RUN\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
        SystemGotoLowPowerRunMode();
      } 
      
      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_LOWPOWER_RUN_WITH_ALGO)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_LOWPOWER_RUN_WITH_ALGO\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
        SystemGotoLowPowerRunWithAlgos();
      }

      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_LOWPOWER_RUN_JUST_ACC)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_LOWPOWER_RUN_JUST_ACC\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
        SystemGotoLowPowerRunModeJustAcc();
      }

      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_LOWPOWER)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_LOWPOWER_RUN_JUST_ACC\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
        SystemGotoLowPowerMode();
      }
            
      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_SLEEP)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_SLEEP\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
        SystemGotoRunSleepMode();               //use *** Sleep mode ***
      }

      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_LOWPOWER_SLEEP)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_LOWPOWER_SLEEP\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
        SystemGotoRunLowPowerSleepMode();       //use *** Low power sleep mode ***
      }
      

      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_STOP)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_STOP\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
      }
    }
    if(nMode & WESU_SYS_MODE_GO_IN_STOP)
    {
      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_PERM_PM_BLE_STOP)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_PERM_PM_BLE_STOP\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
#if USE_IWDG == 0
        APP_BSP_LedSmoothBlink(20);
        APP_BSP_LedSmoothBlink(20);
        APP_LedSmoothRampDown(30);
#endif //USE_IWDG
        SystemGotoPermanentStopBlePmMode();
      }
      
      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_PERM_BLE_STOP)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_PERM_BLE_STOP\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
#if USE_IWDG == 0
        APP_BSP_LedSmoothBlink(20);
        APP_BSP_LedSmoothBlink(20);
        APP_LedSmoothRampDown(30);
#endif //USE_IWDG
        SystemGotoPermanentStopBleMode();
      }
     
      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_PERM_STOP)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_PERM_STOP\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
#if USE_IWDG == 0
        APP_BSP_LedSmoothBlink(20);
        APP_BSP_LedSmoothBlink(20);
        APP_LedSmoothRampDown(30);
#endif //USE_IWDG
        SystemGotoPermanentStopMode();
      }
      
      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_REBOOT)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_REBOOT\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
        SystemGotoRebootMode();
      }
      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_REBOOT_DEFSETTINGS)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_REBOOT_DEFSETTINGS\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
        SystemGotoRebootDefSettingsMode();
      }
      if(SESSION_REG(nLPFunction) == WESU_SYS_POWER_SHUTDOWN)
      {
        DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Power mode changed to:\r\n WESU_SYS_POWER_SHUTDOWN\r\n");
        xCurSmFunc = SESSION_REG(nLPFunction);
        SystemGotoShutdown();
      }
    }
  }
  else
    if((SESSION_REG(nLPFunction) == WESU_SYS_POWER_SLEEP) | (SESSION_REG(nLPFunction) == WESU_SYS_POWER_LOWPOWER_SLEEP))
      if(nMode & WESU_SYS_MODE_GO_IN_STOP)
      {
        SESSION_REG(bLP) = 1;
      }
}


/**
 * @brief  Initialize register management
 * @retval None
 */
void RegistersInitialization()
{
  if( PERSISTENT_REGS_VALIDATION() == FALSE )
  {
    DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "PERSISTENT_REGS_VALIDATION() fail. Loading default persistent settings\r\n");
    LOAD_DEFAULT_EEPROM_REGS();
  }

  if( SESSION_REG(nLPFunction) == WESU_SYS_POWER_REBOOT_DEFSETTINGS )
  {
    DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "Loading default persistent settings upon request\r\n");
    LOAD_DEFAULT_EEPROM_REGS();
  }

  if( RAM_REGS_VALIDATION() == FALSE )
  {
    DBG_PRINTF_WESU_CONFIG(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "RAM_REGS_VALIDATION() fail. Loading default session settings\r\n");
    memset(GlobalSessionStruct, 0, sizeof(GlobalVarsStruct_t));
    SESSION_REG(nVersion) = FIRMWARE_VERSION;
    SESSION_REG(nVersion1) = FIRMWARE_VERSION;
  }

  if( (SESSION_REG(nWakeSource) & SYSTEM_STARTUP_FLAG_CLOSE_SESSION) == 0)
  {
    SESSION_REG(nWakeSource)=0;
  }
  
  SESSION_REG(nWakeSource) &= ~SYSTEM_STARTUP_FLAG_CLOSE_SESSION;
  
  SESSION_REG(nWakeSource) |= WesuGetSystemStartup();
  if(SESSION_REG(nWakeSource) & SYSTEM_STARTUP_FLAG_POWERON)
  {
    /* Here user can take action based on the system startup flag */
    SESSION_REG(nWakeSource) = SYSTEM_STARTUP_FLAG_POWERON;
    LowPowerFlag = WESU_GET_BUTTON_PRESS_MODE;
  }
  if(SESSION_REG(nWakeSource) & SYSTEM_STARTUP_FLAG_RESET_PIN)
  {
    /* Here user can take action based on the system startup flag */
    SESSION_REG(nWakeSource) = SYSTEM_STARTUP_FLAG_RESET_PIN;
  }
  if(SESSION_REG(nWakeSource) & SYSTEM_STARTUP_FLAG_GO_LP)
  {
    /* Here user can take action based on the system startup flag */
      LowPowerFlag = SESSION_REG(nLPFunction);
  }
  if(SESSION_REG(nWakeSource) & SYSTEM_STARTUP_FLAG_IWDG_RESET)
  {
    /* Here user can take action based on the system startup flag */
  }
  
  LOAD_RUNNING_nLPFunction();

  if(memcmp(BLE_PUB_ADDR_REGADDR,WesuAddrDefault,6)==0)
  {
    uint8_t nTempAddress[6];
    GET_DEVICE_SERIAL_NUMBER(nTempAddress,6);
    BSP_PERSREGS_WRITE(BLE_PUB_ADDR_REG, (void*)&nTempAddress, 3);
  }

  SESSION_REG(bLP) = SESSION_DEFAULT_REG_0x5C;
  SESSION_REG(nLPMask) = SESSION_DEFAULT_REG_0x5A;
  SESSION_REG(nUserProcessCount) = SESSION_DEFAULT_REG_0x98;
  SESSION_REG(LowBatteryStatus) = 0xFF;
  SESSION_REG(UsbCableCondition) = 0xFF;

  SESSION_REG(nTimerDrdyCtrlMask)         = 0;

  SESSION_REG(nReadSensorMask)            = SESSION_DEFAULT_REG_0x49;
  SESSION_REG(nProcessFwMask)             = SESSION_DEFAULT_REG_0x4A;

  SESSION_REG(nTimerFrequency)            = SESSION_DEFAULT_REG_0x21;
  SESSION_REG(nPwrCtrlMask)               = ALGORITHM_CHANNEL_MASK(SESSION_DEFAULT_REG_0x25);
  SESSION_REG(nTemperatureCtrlMask)       = ALGORITHM_CHANNEL_MASK(SESSION_DEFAULT_REG_0x26);
  SESSION_REG(nRawMotionCtrlMask)         = ALGORITHM_CHANNEL_MASK(SESSION_DEFAULT_REG_0x2B);
  SESSION_REG(nPressCtrlMask)             = ALGORITHM_CHANNEL_MASK(SESSION_DEFAULT_REG_0x28);
  SESSION_REG(nPedometerCtrlMask)         = ALGORITHM_CHANNEL_MASK(SESSION_DEFAULT_REG_0x35);
  SESSION_REG(nMotionCpCtrlMask)          = ALGORITHM_CHANNEL_MASK(SESSION_DEFAULT_REG_0x38);
  SESSION_REG(nMotionArCtrlMask)          = ALGORITHM_CHANNEL_MASK(SESSION_DEFAULT_REG_0x39);
  
  UpdateSubSampling();
  SESSION_REG(nMotionFxCtrlMask)          = ALGORITHM_CHANNEL_MASK(SESSION_DEFAULT_REG_0x3D);

  SESSION_REG(nAccEventCtrlMask)          = ALGORITHM_CHANNEL_MASK(SESSION_DEFAULT_REG_0x3F);
  SESSION_REG(nAccEventConf)              = SESSION_DEFAULT_REG_0x7A;


  SESSION_REG(nLedControlMask)            = SESSION_DEFAULT_REG_0x02;
  
  SESSION_REG(nBleConIntv)                = SESSION_DEFAULT_REG_0x5E;
  
  SESSION_REG(WeSU_DEBUG_SEVERITY_ble)          = SESSION_DEFAULT_REG_0x45;
  SESSION_REG(WeSU_DEBUG_SEVERITY_usart)        = SESSION_DEFAULT_REG_0x46;
  SESSION_REG(WeSU_APP_DEFAULT_SEVERITY)        = SESSION_DEFAULT_REG_0x47;
  SESSION_REG(WeSU_DEBUG_MASK)                  = SESSION_DEFAULT_REG_0x1F;
  SESSION_REG(WeSU_IWDG_RELOAD_VALUE)           = SESSION_DEFAULT_REG_0x1E;
  
  if(SESSION_REG(WeSU_IWDG_RELOAD_VALUE) < 100)
  {
    //check watchdog reload
    if(SESSION_REG(nTimerFrequency) < 50)
    {
      uint16_t nMinReloadValue = 1 + (1000 / SESSION_REG(nTimerFrequency));
      if(SESSION_REG(WeSU_IWDG_RELOAD_VALUE) < nMinReloadValue)
      {
        SESSION_REG(WeSU_IWDG_RELOAD_VALUE) = nMinReloadValue;
      }
    }
    else
    {
      if(SESSION_REG(WeSU_IWDG_RELOAD_VALUE) == 0)
      {
        SESSION_REG(WeSU_IWDG_RELOAD_VALUE) = WeSU_IWDG_DEFAULT_RELOAD_VALUE;
      }
    }
  }
  
#if USE_CUSTOM_ALGORITHM1
  SESSION_REG(xMAR_Status) = 2;
#endif //USE_CUSTOM_ALGORITHM1

#if USE_CUSTOM_ALGORITHM2
  SESSION_REG(xMCP_Status) = 2;
#endif //USE_CUSTOM_ALGORITHM1
  
  SESSION_REG(nRsvdDummyRegSystemFlag) &= ~SYS_PARAMS_BACKUP_SESSION_DONE;
}

/**
 * @brief Handler function for readonly registers: Write not allowed
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t ReadOnly(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  return 1;
}

/**
 * @brief Write register handler function for led control mask
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t LedControlMaskChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  //perform changes on led control
  SESSION_REG(nLedControlMask) = *(xRegType_t*)pPayload;
  if(LED_CONFIG_USER_ENABLED())
  {
    //led controlled by smartphone
    if(LED_CONFIG_USER_CMD_ON())
    {
      APP_BSP_LED_OnPrivate(LED);
    }
    if(LED_CONFIG_USER_CMD_OFF())
    {
      APP_BSP_LED_OffPrivate(LED);
    }
  }
  else
  {
    SESSION_REG(nLedBlinkMode) = LED_MODE_SMART_BLINKING;
  }
  return 0;//OK
}

/**
 * @brief Write register handler function for led control mask
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t RedLedControlMaskChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  //perform changes on led control
  SESSION_REG(nRedLedControlMask) = *(xRegType_t*)pPayload;
  if(RED_LED_CONFIG_USER_ENABLED())
  {
    //led controlled by smartphone
    if(RED_LED_CONFIG_USER_CMD_ON())
    {
      APP_BSP_LED_OnPrivate(RED_LED);
    }
    if(RED_LED_CONFIG_USER_CMD_OFF())
    {
      APP_BSP_LED_OffPrivate(RED_LED);
    }
  }
  else
  {
  }
  return 0;//OK
}


/**
 * @brief Write register handler function to copy the value to the associated session reg
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t CopyToSession(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = * APP_DEBUG_CONFIG_REGADDR;
  a = SESSION_REG(WeSU_APP_DEFAULT_SEVERITY);
  a = *(xRegType_t*)pPayload;
  
  //perform changes on power mode control
  RegsStructArray_t* SessionRegsArray = (void *)SESSIONREG_STRUCT_START_ADDRESS;
  SessionRegsArray->nRegisters[nRegAddr] = a;
  
  return 0;//OK
}

/**
 * @brief Write register handler function for power mode control
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t PwrModeAltRegChange(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  if(POWER_MODE_STOPPING(*(xRegType_t*)pPayload))
  {
    //perform changes on power mode control
    SESSION_REG(nLPFunction) = *(xRegType_t*)pPayload;
    
    return 1;
  }
  return 1;//only stopping power mode can be written here
}

/**
 * @brief Write register handler function for power mode control
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t PwrModeControlChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = * PWR_MODE_CONTROL_REGADDR;
  a=SESSION_REG(nLPFunction);
  if(a!=*(xRegType_t*)pPayload)
  {
    //perform changes on power mode control
    SESSION_REG(nLPFunction) = *(xRegType_t*)pPayload;

    if(POWER_MODE_STOPPING(*(xRegType_t*)pPayload))
    {
      return 1;
    }
  }
  return 0;//OK
}


/**
 * @brief Write register handler function to change Motion Fx control register
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t MotionFxCtrlChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = *MOTION_FX_CONTROL_MASK_REGADDR;

  SET_LOW_BYTE(a, SESSION_REG(nMotionFxCtrlMask));
  SET_HIGH_BYTE(a, SESSION_REG(nMotionFxSubSampl));

  if(a!=*(xRegType_t*)pPayload)
  {
    //perform changes on FX control
    SESSION_REG(nMotionFxCtrlMask) = LOW_BYTE(*(xRegType_t*)pPayload);
    SESSION_REG(nMotionFxSubSampl) = HIGH_BYTE(*(xRegType_t*)pPayload);
  }
  return 0;
}
/**
 * @brief Write register handler function to change AHRS control register
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t TimerFreqChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = *TIMER_FREQUENCY_REGADDR;
  a = SESSION_REG(nTimerFrequency);

  if(a!=*(xRegType_t*)pPayload)
  {
    //perform changes on timer frequency
    SESSION_REG(nTimerFrequency) = *(xRegType_t*)pPayload;
  }
  return 0;
}

/**
 * @brief Write register handler function to change Motion FX calibration variable
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t FXMagCalibChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a=*(xRegType_t*)pPayload;

  if(a==FX_CALIBRATION)
  {
    //perform changes on timer frequency
   isStartFxCalib=1;
  }
  return 0;
}

/**
 * @brief Write register handler function to change Accelerometer Full scale
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t AccFsChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = *ACC_FullScale_REGADDR;
  a = *(xRegType_t*)pPayload;
  //perform changes on accelerometer
  if(READ_SENSORS_ACC_ENABLED())
  {
    SUSPEND_SENSORS_READ_ON_INTERRUPT();
    BSP_ACCELERO_Set_FS_Value(ACCELERO_handle, a);
    float f = a;
    BSP_ACCELERO_Get_FS(ACCELERO_handle, &f);
    RESUME_SENSORS_READ_ON_INTERRUPT();
    if(((uint32_t)pPayload & 0x20000000) == 0x20000000)
    {
      *(uint16_t*)pPayload = (uint16_t)f;
    }
    else
    {
    }
  }
  return 0;
}

/**
 * @brief Write register handler function to change Accelerometer output data rate
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t AccOdrChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = *ACC_OutputDataRate_REGADDR;
  a = *(xRegType_t*)pPayload;
  //perform changes on accelerometer
  if(READ_SENSORS_ACC_ENABLED())
  {
    SUSPEND_SENSORS_READ_ON_INTERRUPT();
    BSP_ACCELERO_Set_ODR_Value(ACCELERO_handle, a);
    float f = a;
    BSP_ACCELERO_Get_ODR(ACCELERO_handle, &f);
    RESUME_SENSORS_READ_ON_INTERRUPT();
    if(((uint32_t)pPayload & 0x20000000) == 0x20000000)
    {
      *(uint16_t*)pPayload = (uint16_t)f;
    }
    else
    {
    }
  }
  return 0;
}

/**
 * @brief Write register handler function to change gyroscope Full scale
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t GyroFsChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = *GYRO_FullScale_REGADDR;
  a = *(xRegType_t*)pPayload;
  //perform changes on gyroscope
  if(READ_SENSORS_GYRO_ENABLED())
  {
    SUSPEND_SENSORS_READ_ON_INTERRUPT();
    BSP_GYRO_Set_FS_Value(GYRO_handle, a);
    float f = a;
    BSP_GYRO_Get_FS(GYRO_handle, &f);
    RESUME_SENSORS_READ_ON_INTERRUPT();
    if(((uint32_t)pPayload & 0x20000000) == 0x20000000)
    {
      *(uint16_t*)pPayload = (uint16_t)f;
    }
    else
    {
    }
  }
  return 0;
}

/**
 * @brief Write register handler function to change gyroscope output data rate
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t GyroOdrChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = *GYRO_OutputDataRate_REGADDR;
  a = *(xRegType_t*)pPayload;
  //perform changes on gyroscope
  if(READ_SENSORS_GYRO_ENABLED())
  {
    SUSPEND_SENSORS_READ_ON_INTERRUPT();
    BSP_GYRO_Set_ODR_Value(GYRO_handle, a);
    float f = a;
    BSP_GYRO_Get_ODR(GYRO_handle, &f);
    RESUME_SENSORS_READ_ON_INTERRUPT();
    if(((uint32_t)pPayload & 0x20000000) == 0x20000000)
    {
      *(uint16_t*)pPayload = (uint16_t)f;
    }
    else
    {
    }
  }
  return 0;
}

/**
 * @brief Write register handler function to change magnetometer Full scale
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t MagFsChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = *MAG_FullScale_REGADDR;
  a = *(xRegType_t*)pPayload;
  //perform changes on magnetometer
  if(READ_SENSORS_MAG_ENABLED())
  {
    SUSPEND_SENSORS_READ_ON_INTERRUPT();
    BSP_MAGNETO_Set_FS_Value(MAGNETO_handle, a);
    float f = a;
    BSP_MAGNETO_Get_FS(MAGNETO_handle, &f);
    RESUME_SENSORS_READ_ON_INTERRUPT();
    if(((uint32_t)pPayload & 0x20000000) == 0x20000000)
    {
      *(uint16_t*)pPayload = (uint16_t)f;
    }
    else
    {
    }
  }
  return 0;
}

/**
 * @brief Write register handler function to change magnetometer output data rate
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t MagOdrChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = *MAG_OutputDataRate_REGADDR;
  a = *(xRegType_t*)pPayload;
  //perform changes on magnetometer
  if(READ_SENSORS_MAG_ENABLED())
  {
    SUSPEND_SENSORS_READ_ON_INTERRUPT();
    BSP_MAGNETO_Set_ODR_Value(MAGNETO_handle, a);
    float f = a;
    BSP_MAGNETO_Get_ODR(MAGNETO_handle, &f);
    RESUME_SENSORS_READ_ON_INTERRUPT();
    if(((uint32_t)pPayload & 0x20000000) == 0x20000000)
    {
      *(uint16_t*)pPayload = (uint16_t)f;
    }
    else
    {
    }
  }
  return 0;
}

/**
 * @brief Write register handler function to change magnetometer output data rate
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t AccEventConfChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = *ACCEVENT_CONF_REGADDR;
  a = *(xRegType_t*)pPayload;
  //perform changes on magnetometer
  if(ACCEVENT_FEATURE_ENABLED())
  {
    //perform changes on accevent configuration
    SESSION_REG(nAccEventConf) = a;
    SystemReconfigHw();
  }
  return 0;
}

/**
 * @brief Write register handler function to change pressure sensor output data rate
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t PressOdrChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = *PRESS_TEMP_OutputDataRate_REGADDR;
  a = *(xRegType_t*)pPayload;
  //perform changes on pressure sensor
  if(READ_SENSORS_PRESS_ENABLED())
  {
    SUSPEND_SENSORS_READ_ON_INTERRUPT();
    BSP_PRESSURE_Set_ODR_Value(PRESSURE_handle, a);
    float f = a;
    BSP_PRESSURE_Get_ODR(PRESSURE_handle, &f);
    RESUME_SENSORS_READ_ON_INTERRUPT();
    if(((uint32_t)pPayload & 0x20000000) == 0x20000000)
    {
      *(uint16_t*)pPayload = (uint16_t)f;
    }
    else
    {
    }
  }
  return 0;
}

/**
 * @brief Write register handler function to change pedometer control register
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t PedometerCtrlChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = *PEDOMETER_CONTROL_MASK_REGADDR;

  SET_LOW_BYTE(a, SESSION_REG(nPedometerCtrlMask));
  SET_HIGH_BYTE(a, SESSION_REG(nPedometerSubSampl));

  if(a!=*(xRegType_t*)pPayload)
  {
    //perform changes on pedometer control
    SESSION_REG(nPedometerCtrlMask) = LOW_BYTE(*(xRegType_t*)pPayload);
    SESSION_REG(nPedometerSubSampl) = HIGH_BYTE(*(xRegType_t*)pPayload);
  }
  return 0;
}

/**
 * @brief Write register handler function to change BlueNRG connection interval
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t BleConIntvChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = *BLE_CON_INTV_REGADDR;

  a = SESSION_REG(nBleConIntv);

  if(a!=*(xRegType_t*)pPayload)
  {
    //perform changes on BlueNRG connection interval control register
    extern volatile uint8_t bConnParamUpdate;
    bConnParamUpdate = 1;
    SESSION_REG(nBleConIntv) = *(xRegType_t*)pPayload;
  }
  return 0;
}

/**
 * @brief Write register handler function to change hardware configuration
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t HwConfigChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = *HW_OPERATING_CAPABILITIES_REGADDR;

  a = SESSION_REG(nReadSensorMask);

  if(a!=*(xRegType_t*)pPayload)
  {
    //perform changes on hardware configuration control register
    SESSION_REG(nReadSensorMask) = *(xRegType_t*)pPayload;
    SystemReconfigHw();
  }
  return 0;
}

/**
 * @brief Write register handler function to change firmware configuration
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t FwConfigChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t a;
  a = *FW_OPERATING_CAPABILITIES_REGADDR;

  a = SESSION_REG(nProcessFwMask);
  
  if(a!=*(xRegType_t*)pPayload)
  {
    //perform changes on algorithms configuration control register
    SESSION_REG(nProcessFwMask) = *(xRegType_t*)pPayload;
    SystemReconfigHw();
  }
  return 0;
}

/**
 * @brief Write register handler function to set RTC time and date
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t RtcConfChanged(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  #if USE_RTC
  static uint8_t nRtcUpdStatus = 0;
  static rtc_time_t nRtcUpdBuffer = {0};
  uint16_t a;
  a = *(xRegType_t*)pPayload;
  
  if(nRegAddr == RTC_CONF_0_REG)
  {
    if(IS_RTC_HOUR24(LOW_BYTE(a)) &&
       IS_RTC_MINUTES(HIGH_BYTE(a)))
    {
      nRtcUpdStatus |=1;
      nRtcUpdBuffer.tm_hour = LOW_BYTE(a);
      nRtcUpdBuffer.tm_min = HIGH_BYTE(a);
      return 0;
    }
  }
  else if(nRegAddr == RTC_CONF_1_REG)
  {
    if(IS_RTC_SECONDS(LOW_BYTE(a)) &&
      IS_RTC_DATE(HIGH_BYTE(a)))
    {
      if(nRtcUpdStatus == 1)
      {
        nRtcUpdStatus |=2;
        nRtcUpdBuffer.tm_sec = LOW_BYTE(a);
        nRtcUpdBuffer.tm_day = HIGH_BYTE(a);
        return 0;
      }
      else
      {
        nRtcUpdStatus =0;
      }
    }
  }
  else if(nRegAddr == RTC_CONF_2_REG)
  {
    if(IS_RTC_MONTH(LOW_BYTE(a)) &&
      IS_RTC_YEAR(HIGH_BYTE(a)))
    {
      if(nRtcUpdStatus == 3)
      {
        nRtcUpdStatus |=4;
        nRtcUpdBuffer.tm_mon = LOW_BYTE(a);
        nRtcUpdBuffer.tm_year = HIGH_BYTE(a);
        return 0;
      }
      else
      {
        nRtcUpdStatus =0;
      }
    }
  }
  else if(nRegAddr == RTC_CONF_3_REG)
  {
    if(IS_RTC_WEEKDAY(LOW_BYTE(a)) &&
      IS_RTC_APP_CONFIG(HIGH_BYTE(a)))
    {
      if(nRtcUpdStatus == 7)
      {
        nRtcUpdStatus =0;
        nRtcUpdBuffer.tm_wday = LOW_BYTE(a);
        nRtcUpdBuffer.tm_rtcConf = HIGH_BYTE(a);
        //ready to set rtc, check saved time
        rtc_time_t nStatusRtc;
        WeSU_GetDateTime(&nStatusRtc);
        if(nRtcUpdBuffer.tm_rtcConf != RTC_AppConfigInvalid)
        {
          if( (nRtcUpdBuffer.tm_rtcConf == RTC_AppConfigForced) ||
             (nRtcUpdBuffer.tm_rtcConf == RTC_AppConfigUser && nStatusRtc.tm_rtcConf != RTC_AppConfigUserRunning) )
          {
            nRtcUpdBuffer.tm_rtcConf = RTC_AppConfigUserRunning;
            if(0==WeSU_SetDateTime(&nRtcUpdBuffer))
            {
              /* rtc config changed, trigger backups */ 
              CLEAR_FLAG_32(SESSION_REG(nRsvdDummyRegSystemFlag), SYS_PARAMS_BACKUP_SESSION_DONE);
              CLEAR_FLAG_32(SESSION_REG(nRsvdDummyRegSystemFlag), SYS_PARAMS_BACKUP_POWERON_DONE);
            }
            else
            {
              return 1;
            }
          }
        }
        return 1;
      }
      else
      {
        nRtcUpdStatus =0;
      }
    }
  }
  #endif //USE_RTC
  return 1;//error
}

/**
 * @brief Write register handler function for BlueNRG tx power level
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t TxPwrLvlChange(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint8_t p = ((*((uint16_t*)pPayload)) & 0xFF);
  if( (p>0x0F) || (p<0x08) )//pa_level not allowed > ( 0x07 | 0x08) // 0x08 is the mask for valid value
      return 1;
  uint8_t h = ((*((uint16_t*)pPayload)) & 0xFF00) >> 8;
  if( (h>0x09) || (h<0x08) )//en_high_power not allowed >1
      return 1;
  uint8_t h1 = ((h<<8) & 0x0100) >> 8;
  uint8_t p1 = ((p) & 0x07);
  
  /*tBleStatus r =*/ aci_hal_set_tx_power_level(h1,p1);
  
  return 0;//OK
}

/**
 * @brief Write register handler function for BlueNRG advertising interval
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t AdvIntvChange(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t p = *((uint16_t*)pPayload);
  if( (p<0x0020) || (p>0x4000) )//AdvIntervMin 0x0020 -> 0x4000
      return 1;
  return 0;//OK
}

/**
 * @brief Write application address to be executed at next startup by reset manager
 * @param imageBase image base address (e.g. 0x08020000)
 * @retval None
 */
void Switch_To_OTA_Service_Manager_Application(uint32_t imageBase)
{
  BSP_EEPROM_WriteBuffer(NEW_APP_MEM_INFO, (uint32_t)&imageBase, 4);
//  FLASH_Status flashStatus;
//  
//  DATA_EEPROM_Unlock();
//  flashStatus = DATA_EEPROM_ProgramWord(NEW_APP_MEM_INFO, imageBase);               
//  DATA_EEPROM_Lock();
//  if (flashStatus == FLASH_COMPLETE)
//  {
//    /* We are jumping towards new app */
//    /* Reset manager will take care of running the new application*/
//  }
}

/**
 * @brief Write USB DFU flag
 * @param nDfuFlag 1 start in USB DFU mode
 * @retval None
 */
void Switch_To_USB_DFU(uint32_t nDfuFlag)
{
  BSP_EEPROM_WriteBuffer(USB_DFU_MEM_INFO, (uint32_t)&nDfuFlag, 4);
//  FLASH_Status flashStatus;
//  
//  DATA_EEPROM_Unlock();
//  flashStatus = DATA_EEPROM_ProgramWord(USB_DFU_MEM_INFO, (nDfuFlag!=FALSE));
//  DATA_EEPROM_Lock();
//  if (flashStatus == FLASH_COMPLETE)
//  {
//    if(nDfuFlag)
//    {
//      /* We are jumping towards new app */
//      /* Reset manager will take care of running the new application*/
//    }
//  }
}

/**
 * @brief Write register handler function for dummy register change
 * @param nRegAddr
 * @param nChkLen
 * @param pPayload
 * @retval 0: operation OK, 1 not allowed
 */
uint8_t DummyRegChange(uint8_t nRegAddr, uint8_t nChkLen, uint8_t *pPayload)
{
  uint16_t p = *((uint16_t*)pPayload);
  if( p == RESTART_BLUENRG_BRIDGE_MODE )//Try to write RESTART_BLUENRG_BRIDGE_MODE to restart in USB-BlueNRG bridge mode
  {
    // Check if BLUENRG_BRIDGE_FW_ADDRESS is a RAM address, i.e. BLUENRG_BRIDGE_FW is present in the correct address
    if (((*(__IO uint32_t*)BLUENRG_BRIDGE_FW_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
    {
      //OK, prepare address for restart and reset
      Switch_To_OTA_Service_Manager_Application(BLUENRG_BRIDGE_FW_ADDRESS);
      HAL_Delay(WESU_DELAY_BEFORE_RESET);
      WESU_SYSTEM_RESET();
      return 1;
    }
  }
  else if( p == RESTART_DFU_OTA_MODE )//Try to write RESTART_DFU_OTA_MODE to restart in OTA dfu mode
  {
    // Check if OTA_FW_ADDRESS is a RAM address, i.e. OTA_FW_ADDRESS is present in the correct address
    if (((*(__IO uint32_t*)OTA_FW_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
    {
      //OK, prepare address for restart and reset
      Switch_To_OTA_Service_Manager_Application(OTA_FW_ADDRESS);
      HAL_Delay(WESU_DELAY_BEFORE_RESET);
      WESU_SYSTEM_RESET();
      return 1;
    }
  }
  else if( p == RESTART_DFU_USB_MODE )//Try to write RESTART_DFU_USB_MODE to restart in USB dfu mode
  {
    //OK, prepare address for restart and reset
    Switch_To_USB_DFU(TRUE);
    HAL_Delay(WESU_DELAY_BEFORE_RESET);
    WESU_SYSTEM_RESET();
    return 1;
  }
  else if( p == RESTART_NORMAL_MODE )//Try to write RESTART_NORMAL_MODE to restart in application mode
  {
    //OK, prepare address for restart and reset
    Switch_To_USB_DFU(FALSE);
    Switch_To_OTA_Service_Manager_Application(APP_FW_ADDRESS);
//    WESU_SYSTEM_RESET();
    return 1;
  }
  else
  {
  }
  return 1;
}


/**
 * @brief Check if a specified register has an associated function
 * @param pFStructVect pointer to descriptor array
 * @param pFStructSize size of descriptor array
 * @param nReg register to check
 * @retval one of the following: pointer to associated function
 */
static inline pCheckFunc_t GetRegFunc(RegCorrectStruct_t const * pFStructVect, uint8_t pFStructSize, uint8_t nReg)
{
  int i;
  for(i=0;i<pFStructSize;i++)
  {
    if( (pFStructVect[i].nRegAddressToCheck <= nReg) && (nReg < pFStructVect[i].nRegAddressToCheck+pFStructVect[i].nCheckLenght))
    {
      return pFStructVect[i].pCheckFunc;
    }
  }
  return 0;
}

/**
 * @brief Check if action on the specified register must trigger an associated function
 * @param pFStructVect pointer to descriptor array
 * @param pFStructSize size of descriptor array
 * @param nReg register to check
 * @param pPayload frame payload 
 * @retval 0: if no function found, operation is allowed; otherwise the function result
 */
static inline int CheckRegVect(RegCorrectStruct_t const * pFStructVect, uint8_t pFStructSize, uint8_t nReg, uint8_t* pPayload)
{
  pCheckFunc_t f;
  f = GetRegFunc(pFStructVect, pFStructSize, nReg);
  if(f)
    return f(nReg, 1, pPayload);
  return 0;
}

/**
 * @brief Check if writing on the specified persistent register is allowed
 * @param nReg register to check
 * @param nLen number of registers to check
 * @param pPayload frame payload 
 * @retval 0: operation allowed; 1: operation not allowed
 */
static inline uint8_t PermRegActionForWrite(uint8_t nReg, uint8_t nLen, uint8_t* pPayload)
{
  int j;
  for(j=0;j<nLen; j++)
  {
    if(CheckRegVect(PermRegActionStructVect, countof(PermRegActionStructVect), nReg+j, pPayload + (j*2)))
    {
      return 1;
    }
  }
  return 0;//OK
}

/**
 * @brief Check if writing on the specified session register is allowed
 * @param nReg register to check
 * @param nLen number of registers to check
 * @param pPayload frame payload 
 * @retval 0: operation allowed; 1: operation not allowed
 */
static inline uint8_t SessionRegActionForWrite(uint8_t nReg, uint8_t nLen, uint8_t* pPayload)
{
  int j;
  for(j=0;j<nLen; j++)
  {
    if(CheckRegVect(SessionRegActionStructVect, countof(SessionRegActionStructVect), nReg+j, pPayload + (j*2)))
    {
      return 1;
    }
  }
  return 0;//OK
}

/**
 * @brief Check if the content of the specified persistent register is correct
 * @param nReg register to check
 * @param nLen number of registers to check
 * @param pPayload frame payload 
 * @retval 0: operation allowed; 1: operation not allowed
 */
static inline uint8_t PermRegCheckCorrect(uint8_t nReg, uint8_t nLen, uint8_t* pPayload)
{
  int j;
  for(j=0;j<nLen; j++)
  {
    if(CheckRegVect(PermRegCorrectStructVect, countof(PermRegCorrectStructVect), nReg+j, pPayload + (j*2)))
    {
      return 1;
    }
  }
  return 0;//OK
}

/**
 * @brief Check if the content of the specified session register is correct
 * @param nReg register to check
 * @param nLen number of registers to check
 * @param pPayload frame payload 
 * @retval 0: operation allowed; 1: operation not allowed
 */
static inline uint8_t SessionRegCheckCorrect(uint8_t nReg, uint8_t nLen, uint8_t* pPayload)
{
  int j;
  for(j=0;j<nLen; j++)
  {
    if(CheckRegVect(SessionRegCorrectStructVect, countof(SessionRegCorrectStructVect), nReg+j, pPayload + (j*2)))
    {
      return 1;
    }
  }
  return 0;//OK
}

/**
 * @brief Write register handler function for dummy register change
 * @param pCommand
 * @param nFrameLenght
 * @retval one of the following: PERMREG_NEW_OPERATION_COMPLETE_WITH_ACK, PERMREG_NEW_OPERATION_COMPLETE or PERMREG_NO_OPERATION
 */
uint8_t ProcessRegCommand(RegFrameStruct_t *pCommand, uint8_t nFrameLenght)
{
  uint8_t nRet = PERMREG_NO_OPERATION;
  RegFrameStruct_t *pPointerToFrame = pCommand;
  if(nFrameLenght >= 4)
  {
    if(PERMREG_CONTROL_BYTE & PERMREG_PENDING_OPERATION)
    {
      if(PERMREG_CONTROL_BYTE & PERMREG_ACK_REQUIRED)
      {
        nRet = PERMREG_NEW_OPERATION_COMPLETE_WITH_ACK;
      }
      else
      {
        nRet = PERMREG_NEW_OPERATION_COMPLETE;
      }
      
      if(PERMREG_ERROR_CODE == 0)
      {
        SUSPEND_SENSORS_READ_ON_INTERRUPT();
        pPointerToFrame = pCommand;
        PERMREG_SET_ERROR_CODE(PERMREG_NO_ERROR_CODE);
        PERMREG_SET_CONTROL_BIT(PERMREG_ERROR_CONDITION,0);
        //check lenght, need other checks?
        if( /*(PERMREG_LENGHT >= PERMREG_MIN_PAYLOAD_LENGHT) && */ (PERMREG_LENGHT <= PERMREG_MAX_PAYLOAD_LENGHT) )
        {
          {
            if(PERMREG_CONTROL_BYTE & PERMREG_EEPROM_OPERATION)
            {//EEPROM
              if(PERMREG_CONTROL_BYTE & PERMREG_WRITE_OPERATION)
              {
                if(PermRegCheckCorrect(PERMREG_REG_ADDR, PERMREG_LENGHT, PERMREG_PAYLOAD_POINTER))
                {
                  PERMREG_SET_ERROR_CODE(PERMREG_ERROR_WRONG_FORMAT);
                  PERMREG_SET_CONTROL_BIT(PERMREG_ERROR_CONDITION,1);
                }
                else if(PermRegActionForWrite(PERMREG_REG_ADDR, PERMREG_LENGHT, PERMREG_PAYLOAD_POINTER))
                {
                  PERMREG_SET_ERROR_CODE(PERMREG_ERROR_ACTION_NOT_ALLOWED/*PERMREG_ERROR_ACTION*/);
                  PERMREG_SET_CONTROL_BIT(PERMREG_ERROR_CONDITION,1);
                }
                else
                {
                  BSP_PERSREGS_WRITE(PERMREG_REG_ADDR, PERMREG_PAYLOAD_POINTER, PERMREG_LENGHT);
                }
                PERMREG_LENGHT = 4;
              }
              else
              {
                BSP_PERSREGS_READ(PERMREG_REG_ADDR, PERMREG_PAYLOAD_POINTER, PERMREG_LENGHT);
                if(PermRegCheckCorrect(PERMREG_REG_ADDR, PERMREG_LENGHT, PERMREG_PAYLOAD_POINTER))
                {
                  PERMREG_SET_ERROR_CODE(PERMREG_ERROR_WRONG_FORMAT);
                  PERMREG_SET_CONTROL_BIT(PERMREG_ERROR_CONDITION,1);
                }
              }
            }
            else
            {//RAM
              if(PERMREG_CONTROL_BYTE & PERMREG_WRITE_OPERATION)
              {
                if(SessionRegCheckCorrect(PERMREG_REG_ADDR, PERMREG_LENGHT, PERMREG_PAYLOAD_POINTER))
                {
                  PERMREG_SET_ERROR_CODE(PERMREG_ERROR_WRONG_FORMAT);
                  PERMREG_SET_CONTROL_BIT(PERMREG_ERROR_CONDITION,1);
                }
                else if(SessionRegActionForWrite(PERMREG_REG_ADDR, PERMREG_LENGHT, PERMREG_PAYLOAD_POINTER))
                {
                  PERMREG_SET_ERROR_CODE(PERMREG_ERROR_ACTION_NOT_ALLOWED/*PERMREG_ERROR_ACTION*/);
                  PERMREG_SET_CONTROL_BIT(PERMREG_ERROR_CONDITION,1);
                }
                else
                {
                  BSP_SESSIONREGS_WRITE(PERMREG_REG_ADDR, PERMREG_PAYLOAD_POINTER, PERMREG_LENGHT);
                }
                PERMREG_LENGHT = 4;
              }
              else
              {
                BSP_SESSIONREGS_READ(PERMREG_REG_ADDR, PERMREG_PAYLOAD_POINTER, PERMREG_LENGHT);
                if(SessionRegCheckCorrect(PERMREG_REG_ADDR, PERMREG_LENGHT, PERMREG_PAYLOAD_POINTER))
                {
                  PERMREG_SET_ERROR_CODE(PERMREG_ERROR_WRONG_FORMAT);
                  PERMREG_SET_CONTROL_BIT(PERMREG_ERROR_CONDITION,1);
                }
              }
            }
          }
        }
        else
        {
          PERMREG_SET_ERROR_CODE(PERMREG_ERROR_LENGHT);
          PERMREG_SET_CONTROL_BIT(PERMREG_ERROR_CONDITION,1);
        }
        PERMREG_SET_CONTROL_BIT(PERMREG_PENDING_OPERATION,0);
        if(PERMREG_CONTROL_BYTE & PERMREG_ACK_REQUIRED)
        {
          nRet = PERMREG_NEW_OPERATION_COMPLETE_WITH_ACK;
        }
        else
        {
          nRet = PERMREG_NEW_OPERATION_COMPLETE;
        }
        RESUME_SENSORS_READ_ON_INTERRUPT();
      }
      else
      {
        PERMREG_SET_ERROR_CODE(PERMREG_NO_ERROR_CODE);
      }
    }
    else
    {
      nRet = PERMREG_NO_OPERATION;
    }
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


/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
