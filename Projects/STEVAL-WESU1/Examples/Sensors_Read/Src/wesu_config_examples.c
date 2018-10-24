/**
  ******************************************************************************
  * @file    Sensors_Read/Src/wesu_config_examples.c
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

#include "BlueST_Protocol_examples.h"
#include "wesu_config_examples.h"
#include "wesu_charger.h"
#include "algorithms_examples.h"


/** @addtogroup WeSU_Examples                   WeSU Examples       
  * @{
  */

/** @addtogroup WeSU_User_Examples              WeSU User Examples
  * @{
  */

/** @addtogroup WeSU_Config_Examples            WeSU Config Examples
  * @{
  * @brief WeSU Configuration customized for the DEMO
  */

    
extern volatile uint8_t set_connectable;
extern volatile int bIsConnected;

extern void *ACCELERO_handle;
extern void *GYRO_handle;
extern void *MAGNETO_handle;
extern void *PRESSURE_handle;
extern void *TEMPERATURE_handle;

extern void SystemClock_Config_RTC_HSE32MHz(void);
extern void SystemClock_Config_MSI_2MHz(void);
extern void SystemClock_Config_HSI_32MHz(void);
extern void SystemClock_Config_HSE_18MHz(void);
extern void SystemClock_Config_HSI_12MHz(void);

extern void (*SystemClock_Config_APP)(void);
extern void (*SystemClock_Config_APP_STARTUP)(void);


  uint16_t xBNRG_HW_ver;                                                        //!< BlueNRG HW version
  uint16_t xBNRG_FW_ver;                                                        //!< BlueNRG FW version
  static char WesuName[8] = {0};                                                //!< WeSU board name
  static uint8_t WesuAddr[6] = {0};                                             //!< WeSU board public address
  
DrvStatusTypeDef sensorStatusAccelero;                                          //!< Accelerometer status
DrvStatusTypeDef sensorStatusGyro;                                              //!< Gyroscope status
DrvStatusTypeDef sensorStatusMag;                                               //!< Magnetometer status
DrvStatusTypeDef sensorStatusPress;                                             //!< Pressure status
DrvStatusTypeDef sensorStatusTemp;                                              //!< Temperature status  
  


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
void setConnectable(uint16_t nMin, uint16_t nMax)
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
    int ret = aci_gap_set_non_discoverable();
    
    ret = aci_gap_set_discoverable(ADV_IND, nMin, nMax, xPubAddressType, NO_WHITE_LIST_USE,
                                   nNameLen, cLocalNameAdv, 0, NULL, 0, 0);
  
    uint8_t advbuffer[]=
    {
      0x0D, //lenght of MANUF_SPECIFIC advertising item
      AD_TYPE_MANUFACTURER_SPECIFIC_DATA, //Type
      0x01, //Protocol version
      0x01, //Dev ID
      0x00, 0x00, //Group A Features (big endian)
      0x00, 0x00, //Group B Features (big endian)
      0x00, 0x00, 0x00, //Public device address (48 bits) Company assigned
      0x00, 0x00, 0x00, //Public device address (48 bits) Company id
    };
    
    advbuffer[4] = 0;
    advbuffer[5] = 0;
    advbuffer[6] = 0;
    advbuffer[7] = 0;
    
    uint16_t nAddedFeaturesA = 0x0000;
    uint16_t nAddedFeaturesB = 0x0000;
    
#if ACCELEROMETER_EXAMPLE
    nAddedFeaturesA |= HW_CAP_ACCEL;
#endif //ACCELEROMETER_EXAMPLE
#if GYROSCOPE_EXAMPLE
    nAddedFeaturesA |= HW_CAP_GYRO;
#endif //GYROSCOPE_EXAMPLE
#if MAGNETOMETER_EXAMPLE
    nAddedFeaturesA |= HW_CAP_MAGN;
#endif //MAGNETOMETER_EXAMPLE
#if PRESS_EXAMPLE
    nAddedFeaturesA |= HW_CAP_PRESS;
#endif //PRESS_EXAMPLE
#if TEMPERATURE_EXAMPLE
    nAddedFeaturesA |= HW_CAP_TEMPERATURE;
#endif //TEMPERATURE_EXAMPLE
#if GAS_GAUGE
    nAddedFeaturesA |= HW_CAP_GG;
#endif //GAS_GAUGE
      
    advbuffer[4] = HIGH_BYTE(nAddedFeaturesA);
    advbuffer[5] = LOW_BYTE(nAddedFeaturesA);
#if USE_CUSTOM_ALGORITHM1
    nAddedFeaturesB |= ALGORITHM1_BLUEST_MASK;
#endif /* USE_CUSTOM_ALGORITHM1 */
    
    advbuffer[6] = HIGH_BYTE(nAddedFeaturesB);
    advbuffer[7] = LOW_BYTE(nAddedFeaturesB);
    
    memcpy(&advbuffer[8], WesuAddr, 6);
    if(xPubAddressType == RANDOM_ADDR)
    {
      advbuffer[0] = 0x07;
    }
    
    aci_gap_update_adv_data(14, advbuffer);
}


/**
 * @brief  Manage led status for blinking
 * @param       nLedMode
 * @retval None
 */
void LedManageStart(uint8_t nLedMode)
{
  //ledmode 
  if(bIsConnected)
  {
    APP_BSP_LED_Toggle(LED);
  }
  else
  {
    APP_BSP_LED_Off(LED);
  }
}

/**
 * @brief  Manage led status for blinking
 * @param       nLedMode
 * @retval None
 */
void LedManageEnd(uint8_t nLedMode)
{
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
  
 /* Give time STC3115 to startup */
//  HAL_Delay(100);
  
  /* Initialize Gas Gauge */
  BSP_GG_IO_Init();
  BSP_GG_Init();
}


/**
 * @brief Initialize All the BueNRG management
 * @retval None
 */
void BluenrgInitialization()
{
  /* Reset BlueNRG versions */
  SESSION_REG(xBNRG_HW_ver) = 0;
  SESSION_REG(xBNRG_FW_ver) = 0;
  
  /* Initialize the BlueNRG */
  PRINTF("\r\nInit_BlueNRG_Hw\r\n");
  Init_BlueNRG_Hw();
  
  DBG_PRINTF(WeSU_DEBUG_SEVERITY_STARTUP_MESSAGES, "HwVer 0x%.2X, FwVer 0x%.4X\r\n", SESSION_REG(xBNRG_HW_ver), SESSION_REG(xBNRG_FW_ver));
  
  PRINTF("Init_BlueNRG_Stack\r\n");
  Init_BlueNRG_Stack();
    
  /* Initialize the BlueNRG Custom services */
  PRINTF("Init_BlueNRG_Custom_Services\r\n");
  Init_BlueNRG_Custom_Services();
  
  /* Set output power level */ 
  uint8_t h;
  uint8_t p;
  
  {
    //default values
    p=DEFAULT_BLE_TX_POWER_LEVEL;
    h=DEFAULT_BLE_TX_HIGH_POWER;
  }
  
  aci_hal_set_tx_power_level(h,p);
//  aci_hal_set_tx_power_level(1,6); 
  
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
    while(1);
  }
#else //BLUENRG_MS
  if(SESSION_REG(xBNRG_HW_ver) != 0x30)
  {
    /* Go to infinite loop if BlueNRG hw version is not 3.0 */
    APP_BSP_LED_On(LED);
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
  
  /* The STEVAL-WESU1 board must be configured as SERVER */
//  Osal_MemCpy(bdaddr, WesuAddr, sizeof(WesuAddr));
  
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
//  uint8_t bdaddr[6];
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
}

/** @brief SensorsConfigFullRun
 * @retval None
 */
void SensorsConfig()
{
  /* Initialize the MEMS 6X sensors */
  if(ACCELEROMETER_EXAMPLE)
  {
    PRINTF("BSP_ACCELERO_Init\r\n");
    sensorStatusAccelero = BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &ACCELERO_handle );
  }
  if(GYROSCOPE_EXAMPLE)
  {
    PRINTF("BSP_GYRO_Init\r\n");
    sensorStatusGyro = BSP_GYRO_Init( GYRO_SENSORS_AUTO, &GYRO_handle );
  }
  if(MAGNETOMETER_EXAMPLE)
  {
    /* Initialize the MEMS MAGNETOMETER */
    PRINTF("BSP_MAGNETO_Init\r\n");
    sensorStatusMag = BSP_MAGNETO_Init( MAGNETO_SENSORS_AUTO, &MAGNETO_handle );
  }

  /* Initialize the PRESSURE sensor */
  if(PRESS_EXAMPLE)
  {
    PRINTF("BSP_PRESSURE_Init\r\n");
    sensorStatusPress = BSP_PRESSURE_Init( PRESSURE_SENSORS_AUTO, &PRESSURE_handle );
  }
    
  /* Initialize the TEMPERATURE sensor */
  if(TEMPERATURE_EXAMPLE)
  {
    PRINTF("BSP_TEMPERATURE_Init\r\n");
    sensorStatusTemp = BSP_TEMPERATURE_Init( TEMPERATURE_SENSORS_AUTO, &TEMPERATURE_handle );
  }
    
  if(ACCELEROMETER_EXAMPLE)
  {
    BSP_ACCELERO_Sensor_Enable( ACCELERO_handle );
    BSP_ACCELERO_Set_FS_Value(ACCELERO_handle, ACC_FullScale_DEFAULT);
    BSP_ACCELERO_Set_ODR_Value(ACCELERO_handle, ACC_OutputDataRate_DEFAULT);
  }
  if(GYROSCOPE_EXAMPLE)
  {
    BSP_GYRO_Sensor_Enable( GYRO_handle );
    BSP_GYRO_Set_FS_Value(GYRO_handle, GYRO_FullScale_DEFAULT);
    BSP_GYRO_Set_ODR_Value(GYRO_handle, GYRO_OutputDataRate_DEFAULT);
  }
  if(MAGNETOMETER_EXAMPLE)
  {
    BSP_MAGNETO_Sensor_Enable( MAGNETO_handle );
    BSP_MAGNETO_Set_FS_Value( MAGNETO_handle , MAG_FullScale_DEFAULT);
    BSP_MAGNETO_Set_ODR_Value( MAGNETO_handle , MAG_OutputDataRate_DEFAULT);
  }
  if(TEMPERATURE_EXAMPLE)
  {
    BSP_TEMPERATURE_Sensor_Enable( TEMPERATURE_handle );
  }
  if(PRESS_EXAMPLE)
  {
    BSP_PRESSURE_Sensor_Enable( PRESSURE_handle );
    BSP_PRESSURE_Set_ODR_Value( PRESSURE_handle , PRESS_TEMP_OutputDataRate_DEFAULT);
  }
  Algorithms_Init(0);
//  BSP_MAGNETO_ITConfig();
//  BSP_MAGNETO_DisableIT();
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
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */



/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
