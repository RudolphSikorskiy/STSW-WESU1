/**
  ******************************************************************************
  * @file    WESU_DEMO/Src/BlueST_Protocol.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   Add/manage Bluetooth Low Energy services using BlueST protocol.
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

#include "main.h"
#include "console.h"
#include <stdio.h>
#include "BlueST_Protocol.h"
#include "wesu_config.h"


/** @addtogroup WeSU_Demo       WeSU Demo      
  * @{
  */

/** @addtogroup WeSU_User               WeSU User
  * @{
  */

/** @addtogroup BlueST_PROTOCOL          BlueST PROTOCOL
  * @{
  * @brief API to provide the BlueST Protocol specs on WeSU
  */


/** @defgroup BlueST_PROTOCOL_Exported_Variables BlueST PROTOCOL Exported Variables
 * @{
 */

int bIsConnected = FALSE;                                                       //!< a connection is active
volatile uint8_t bConnParamUpdate = 0;                                          //!< trigger a parameter update request to master
uint8_t set_connectable = TRUE;                                                 //!< request to go in advertising

/**
 * @}
 */


/** @defgroup BlueST_PROTOCOL_Private_Variables BlueST PROTOCOL Private Variables
 * @{
 */

volatile uint16_t connection_handle = 0;                                        //!< handle to active connection
uint16_t HWServHandle;                                                          //!< handle to sensors service
uint16_t pwrCharHandle;                                                         //!< handle to power characteristic
uint16_t TemperatureCharHandle;                                                 //!< handle to temperature characteristic
uint16_t PressureCharHandle;                                                    //!< handle to pressure characteristic
uint16_t AccGyroMagCharHandle;                                                  //!< handle to accelerometer, gyroscope and magnetometer characteristic
#if USE_QUATERNION_FLOAT
uint16_t QuaterFloatCharHandle;                                                 //!< handle to quaternion float characteristic
#endif /* USE_QUATERNION_FLOAT */
uint16_t QuaternionsCharHandle;                                                 //!< handle to MotionFX characteristic


#if USE_CUSTOM_ALGORITHM1
  uint16_t Algorithm1Handle;                                                    //!< handle to algorithm1 characteristic
  uint8_t ConnectionAlgo1=0;                                                    //!< Connection active on algorithm1 data
#endif /* USE_CUSTOM_ALGORITHM1 */

#if USE_CUSTOM_ALGORITHM2
  uint16_t Algorithm2Handle;                                                    //!< handle to algorithm2 characteristic
  uint8_t ConnectionAlgo2=0;                                                    //!< Connection active on algorithm1 data
#endif /* USE_CUSTOM_ALGORITHM2 */
  
#if USE_CUSTOM_ALGORITHM3
  uint16_t Algorithm3Handle;                                                    //!< handle to algorithm3 characteristic (Pedometer SW)
  uint8_t ConnectionAlgo3=0;                                                    //!< Connection active on algorithm1 data
#endif /* USE_CUSTOM_ALGORITHM3 */
  
#if USE_CUSTOM_ALGORITHM4
  uint16_t Algorithm4Handle;                                                    //!< handle to algorithm4 characteristic (Free fall)
  uint8_t ConnectionAlgo4=0;                                                    //!< Connection active on algorithm1 data
#endif /* USE_CUSTOM_ALGORITHM4 */
  
#if USE_CUSTOM_ACCEVENT                                                         //!< handle to Acc Event characteristic
  uint16_t AccEventCharHandle;
  uint16_t AccEventNotification=0;
#endif /* USE_CUSTOM_ACCEVENT */

#if USE_CUSTOM_GP_ALGORITHM
  uint16_t GP_ServHandle;                                                          //!< handle to sensors service
  uint16_t GP_ALGORITHMHandle;                                                    //!< handle to GP_ALGORITHM characteristic (Free fall)
  uint8_t ConnectionAlgoGP=0;                                                    //!< Connection active on algorithm1 data
#endif /* USE_CUSTOM_GP_ALGORITHM */
  
uint16_t ConfigBlueSTServHandle;                                                //!< handle to configuration service
uint16_t ConfigRegsCharHandle;                                                  //!< handle to registers management characteristic

uint16_t ConsoleBlueSTHandle;                                                   //!< handle to console service
uint16_t TermCharHandle;                                                        //!< handle to terminal characteristic
uint16_t StdErrCharHandle;                                                      //!< handle to stderr characteristic


uint16_t TimeStamp=0;                                                           //!< BlueST 16-bit timestamp

uint32_t ConnectionTemp=0;                                                      //!< Connection active on temperature data
uint32_t ConnectionPres=0;                                                      //!< Connection active on pressure data
uint32_t ConnectionAccGyroMag=0;                                                //!< Connection active on acc/gyro/mag data
uint32_t ConnectionPower=0;                                                //!< Connection active on acc/gyro/mag data

uint32_t QuaterFloatNotification=0;                                             //!< Connection active on AHRS data
uint32_t QuaternionNotification  =0;                                            //!< Connection active on MotionFX data

uint32_t StdErrNotification =0;                                                 //!< Connection active on stderr data
uint32_t TermNotification   =0;                                                 //!< Connection active on terminal data

uint8_t BufferToWrite[256];                                                     //!< temp buffer 
uint8_t BytesToWrite;                                                           //!< temp buffer lenght


static uint8_t LastStderrBuffer[CONSOLE_MAX_CHAR_LEN];                          //!< last stderr buffer
static uint8_t LastStderrLen;                                                   //!< last stderr buffer lenght
static uint8_t LastTermBuffer[CONSOLE_MAX_CHAR_LEN];                            //!< last terminal buffer
static uint8_t LastTermLen;                                                     //!< last terminal buffer lenght
      
/**
 * @}
 */

/** @addtogroup BlueST_PROTOCOL_Private_Macros   BlueST PROTOCOL Private Macros
  * @{
  */

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
{\
  uuid_struct[0 ] = uuid_0 ; uuid_struct[1 ] = uuid_1 ; uuid_struct[2 ] = uuid_2 ; uuid_struct[3 ] = uuid_3 ; \
  uuid_struct[4 ] = uuid_4 ; uuid_struct[5 ] = uuid_5 ; uuid_struct[6 ] = uuid_6 ; uuid_struct[7 ] = uuid_7 ; \
  uuid_struct[8 ] = uuid_8 ; uuid_struct[9 ] = uuid_9 ; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
  uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}                                                                               //!< Store 128bit UUID into buffer

#define STORE_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )          //!< Store 16-bit Value into a buffer in Little Endian Format

#define STORE_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[3] =  (uint8_t) (val>>24) ) )         //!< Store 32-bit Value into a buffer in Little Endian Format

#define STORE_BE_32(buf, val)    ( ((buf)[3] =  (uint8_t) (val)     ) , \
                                   ((buf)[2] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[1] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[0] =  (uint8_t) (val>>24) ) )         //!< Store 32-bit Value into a buffer in Big Endian Format
/**
  * @}
  */

/** @addtogroup BlueST_PROTOCOL_Private_Defines  BlueST PROTOCOL Private Defines
  * @{
  */

/* Hardware Characteristics Service */
#define COPY_HW_SENS_BLUEST_SERVICE_UUID(uuid_struct)           COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST Sensors service
#define COPY_TEMPERATURE_BLUEST_CHAR_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x04,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST temperature characteristic
#define COPY_PRESSURE_BLUEST_CHAR_UUID(uuid_struct)             COPY_UUID_128(uuid_struct,0x00,0x10,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST pressure characteristic
#define COPY_MAGNETOMETER_BLUEST_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x00,0x20,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST magnetometer characteristic
#define COPY_GYROSCOPE_BLUEST_CHAR_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0x00,0x40,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST gyroscope characteristic
#define COPY_ACCELEROMETER_BLUEST_CHAR_UUID(uuid_struct)        COPY_UUID_128(uuid_struct,0x00,0x80,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST accelerometer characteristic
#define COPY_ACC_GYRO_MAG_BLUEST_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x00,0xE0,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST accelerometer, gyroscope and magnetometer characteristic
#define COPY_PWR_BLUEST_CHAR_UUID(uuid_struct)                  COPY_UUID_128(uuid_struct,0x00,0x02,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST power/gas-gauge characteristic

/* Software Characteristics Service */
#define COPY_QUATERNIONS_BLUEST_CHAR_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST FX characteristic
#define COPY_QUATER_FLOAT_BLUEST_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x80,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST AHRS characteristic
/* Console Service */
#define COPY_CONSOLE_SERVICE_UUID(uuid_struct)                  COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0E,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST Console service
#define COPY_TERM_CHAR_UUID(uuid_struct)                        COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST Terminal characteristic
#define COPY_STDERR_CHAR_UUID(uuid_struct)                      COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST Stderr characteristic

/* Configuration Service */
#define COPY_CONFIG_SERVICE_UUID(uuid_struct)                   COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0F,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST Configuration service
#define COPY_CONFIG_BLUEST_REGISTERS_CHAR_UUID(uuid_struct)     COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x00,0x0F,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST Registers management characteristic

/**
  * @}
  */

/** @defgroup BlueST_PROTOCOL_Private_Functions    BlueST PROTOCOL Private Functions
  * @{
  */    

static void       GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);  //!< Connection complete callback
static void       GAP_DisconnectionComplete_CB(void);                           //!< Disconnection complete callback
static tBleStatus Stderr_Update_AfterRead(void);
static tBleStatus Term_Update_AfterRead(void);


/**
  * @}
  */

/** @addtogroup BlueST_PROTOCOL_Private_Functions
  * @{
  */

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifdef ACC_BLUENRG_CONGESTION
  #define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
  static int32_t breath = 0;



/* @brief  Update the value of a characteristic avoiding (for a short time) to
 *         send the next updates if an error in the previous sending has
 *         occurred.
 * @param  The handle of the service
 * @param  The handle of the characteristic
 * @param  The offset of the characteristic
 * @param  The length of the characteristic
 * @param  The pointer to the characteristic
 * @retval tBleStatus Status
 */
tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle, 
				      uint16_t charHandle,
				      uint8_t charValOffset,
				      uint8_t charValueLen,   
				      const uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
  
  if (breath > 0) {
    breath--;
  } else {
    ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,charValue);
    
    if (ret != BLE_STATUS_SUCCESS){
      breath = ACC_BLUENRG_CONGESTION_SKIP;
    }
  }
  
  return (ret);
}

#else /* ACC_BLUENRG_CONGESTION */
  #define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif /* ACC_BLUENRG_CONGESTION */

#define ACI_GATT_ADD_SERV(A,B,C,D,Handle) aci_gatt_add_serv(A,B,C,D,Handle); //PRINTF("BS:"#Handle""); PRINTF(":%d\r\n",*Handle);
#define ACI_GATT_ADD_CHAR(A,B,C,D,E,F,G,H,I,Handle) aci_gatt_add_char(A,B,C,D,E,F,G,H,I,Handle); //PRINTF("BC:"#Handle""); PRINTF(":%d\r\n",*Handle);
#define ACI_GATT_ADD_CHAR_DESC  aci_gatt_add_char_desc

#endif /* DOXYGEN_SHOULD_SKIP_THIS */

/**
 * @brief  Update Stderr characteristic value after a read request
 * @retval Status
 */
static tBleStatus Stderr_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = ACI_GATT_UPDATE_CHAR_VALUE(ConsoleBlueSTHandle, StdErrCharHandle, 0, LastStderrLen , LastStderrBuffer);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value after a read request
 * @retval tBleStatus      Status
 */
static tBleStatus Term_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = ACI_GATT_UPDATE_CHAR_VALUE(ConsoleBlueSTHandle, TermCharHandle, 0, LastTermLen , LastTermBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error Updating Stdout Char\r\n");
    if(StdErrNotification){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Stdout Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    }
    
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  This function is called when there is a BLE Connection Complete event.
 * @param  addr Address of peer device
 * @param  handle Connection handle
 * @retval None
 */
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
  bIsConnected = TRUE;
  bConnParamUpdate = bIsConnected;
  connection_handle = handle;

  DBG_PRINTF_BLUENRG(WeSU_TERMINAL_DEBUG_SEVERITY, ">>>>>>CONNECTED %02x:%02x:%02x:%02x:%02x:%02x\r\n",addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]);

  TimeStamp=0;
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @retval None
 */
void GAP_DisconnectionComplete_CB(void)
{
  bIsConnected = FALSE;
  bConnParamUpdate = bIsConnected;

  DBG_PRINTF_BLUENRG(WeSU_TERMINAL_DEBUG_SEVERITY, "<<<<<<DISCONNECTED\r\n");

  /* Make the device connectable again. */
  set_connectable = TRUE;
  
#if USE_CUSTOM_ALGORITHM1
  ConnectionAlgo1=0;
#endif //USE_CUSTOM_ALGORITHM1
#if USE_CUSTOM_ALGORITHM2
  ConnectionAlgo2=0;
#endif //USE_CUSTOM_ALGORITHM2
#if USE_CUSTOM_ALGORITHM3
  ConnectionAlgo3=0;
#endif //USE_CUSTOM_ALGORITHM3
#if USE_CUSTOM_ALGORITHM4
  ConnectionAlgo4=0;
#endif //USE_CUSTOM_ALGORITHM4
#if USE_CUSTOM_GP_ALGORITHM
  ConnectionAlgoGP=0;
#endif //USE_CUSTOM_GP_ALGORITHM
#if USE_CUSTOM_ACCEVENT
  AccEventNotification=0;
#endif //USE_CUSTOM_ACCEVENT
  
  ConnectionTemp=0;
  ConnectionPres=0;
  ConnectionPower=0;
  ConnectionAccGyroMag=0;

  QuaterFloatNotification=0;

  StdErrNotification=0;
  TermNotification  =0;

  TimeStamp=0;
}

/**
  * @}
  */


/** @addtogroup BlueST_PROTOCOL_Exported_Functions
  * @{
  */

/**
 * @brief  This function is called to reset bluenrg parameters
 * @retval None
 */
void BluenrgResetParameters(void)
{
  GAP_DisconnectionComplete_CB();
}

/**
 * @brief  Update Stderr characteristic value
 * @param  data string to write
 * @param  length lenght of string to write
 * @retval Status 
 */
tBleStatus Stderr_Update(uint8_t *data, uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split thhe code in packages */
  for(Offset =0; Offset<length; Offset +=CONSOLE_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>CONSOLE_MAX_CHAR_LEN) ?  CONSOLE_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastStderrBuffer,data+Offset,DataToSend);
    LastStderrLen = DataToSend;

    ret = ACI_GATT_UPDATE_CHAR_VALUE(ConsoleBlueSTHandle, StdErrCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS)
    {
      PRINTF("Error Updating Stderr Char\r\n");
      return BLE_STATUS_ERROR;
    }
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value
 * @param  data string to write
 * @param  length lenght of string to write
 * @retval Status
 */
tBleStatus Term_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split thhe code in packages */
  for(Offset =0; Offset<length; Offset +=CONSOLE_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>CONSOLE_MAX_CHAR_LEN) ?  CONSOLE_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastTermBuffer,data+Offset,DataToSend);
    LastTermLen = DataToSend;

    if(TermNotification)
    {
      static uint8_t termUpdateDelay = 0;
      HAL_Delay(termUpdateDelay);
      
      ret = ACI_GATT_UPDATE_CHAR_VALUE(ConsoleBlueSTHandle, TermCharHandle, 0, DataToSend , data+Offset);
      if (ret != BLE_STATUS_SUCCESS) {
        PRINTF("Error Updating Stdout Char\r\n");
        termUpdateDelay++;
        if(StdErrNotification){
          BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Stdout Char\r\n");
          Stderr_Update(BufferToWrite,BytesToWrite);
        }
        
        return BLE_STATUS_ERROR;
      }
    }
  }

  return BLE_STATUS_SUCCESS;
}


/**
 * @brief  Add the Config service using a vendor specific profile
 * @retval tBleStatus Status
 */
tBleStatus Add_Config_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_CONFIG_SERVICE_UUID(uuid);
  ret = ACI_GATT_ADD_SERV(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 4,&ConfigBlueSTServHandle);

  if (ret != BLE_STATUS_SUCCESS)
    goto fail;

  COPY_CONFIG_BLUEST_REGISTERS_CHAR_UUID(uuid);
  
  ret =  ACI_GATT_ADD_CHAR(ConfigBlueSTServHandle, UUID_TYPE_128, uuid, 20 /* Max Dimension */,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ|CHAR_PROP_WRITE,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &ConfigRegsCharHandle);

  if (ret != BLE_STATUS_SUCCESS)
    goto fail;

  return BLE_STATUS_SUCCESS;

  fail:
  PRINTF("Error while adding Configuration service.\n");
  return BLE_STATUS_ERROR ;
}


/**
 * @brief  Add the Console service using a vendor specific profile
 * @retval tBleStatus Status
 */
tBleStatus Add_Console_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_CONSOLE_SERVICE_UUID(uuid);
  ret = ACI_GATT_ADD_SERV(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 8,&ConsoleBlueSTHandle);

  if (ret != BLE_STATUS_SUCCESS)
    goto fail;

  COPY_TERM_CHAR_UUID(uuid);
  ret =  ACI_GATT_ADD_CHAR(ConsoleBlueSTHandle, UUID_TYPE_128, uuid, CONSOLE_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY | CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE | CHAR_PROP_READ ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP /*| GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP*/,
                           16, 1, &TermCharHandle);

  if (ret != BLE_STATUS_SUCCESS)
    goto fail;

  COPY_STDERR_CHAR_UUID(uuid);
  ret =  ACI_GATT_ADD_CHAR(ConsoleBlueSTHandle, UUID_TYPE_128, uuid, CONSOLE_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &StdErrCharHandle);

  if (ret != BLE_STATUS_SUCCESS)
     goto fail;

  /* Enable printf output on BlueNRG */
  return BLE_STATUS_SUCCESS;

fail:
  PRINTF("Error while adding Console service.\n");
  return BLE_STATUS_ERROR ;
}

#if USE_QUATERNION_FLOAT 
/**
 * @brief  Update the AHRS Feature 
 * @retval tBleStatus Status
 */
tBleStatus Quat_Float_Update(uint32_t *data)
{
  tBleStatus ret;
  uint8_t buff[20];
  
  STORE_LE_16(buff,TimeStamp);
  STORE_LE_32(buff+6,data[3]);
  STORE_LE_32(buff+10,data[2]);
  STORE_LE_32(buff+14,data[1]);
  STORE_LE_32(buff+2,data[0]);
  
  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, QuaterFloatCharHandle, 0, 18, buff);
  
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error Updating Quaternion Float Char\r\n");
    if(StdErrNotification){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Quaternion Float Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    }
    
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
#endif /* USE_QUATERNION_FLOAT */ 

/**
 * @brief  Update quaternions characteristic value
 * @param  Axes_TypeDef *data Structure containing the quaterions
 * @retval tBleStatus      Status
 */
tBleStatus Quat_Update(SensorAxes_t *data)
{
  tBleStatus ret;    

  uint8_t buff[2+ 6*SEND_N_QUATERNIONS];


  STORE_LE_16(buff  ,TimeStamp);

#if SEND_N_QUATERNIONS == 1
  STORE_LE_16(buff+2,data[0].AXIS_X);
  STORE_LE_16(buff+4,data[0].AXIS_Y);
  STORE_LE_16(buff+6,data[0].AXIS_Z);
  
#elif SEND_N_QUATERNIONS == 2
  STORE_LE_16(buff+2,data[0].AXIS_X);
  STORE_LE_16(buff+4,data[0].AXIS_Y);
  STORE_LE_16(buff+6,data[0].AXIS_Z);

  STORE_LE_16(buff+8 ,data[1].AXIS_X);
  STORE_LE_16(buff+10,data[1].AXIS_Y);
  STORE_LE_16(buff+12,data[1].AXIS_Z);
#elif SEND_N_QUATERNIONS == 3
  STORE_LE_16(buff+2,data[0].AXIS_X);
  STORE_LE_16(buff+4,data[0].AXIS_Y);
  STORE_LE_16(buff+6,data[0].AXIS_Z);

  STORE_LE_16(buff+8 ,data[1].AXIS_X);
  STORE_LE_16(buff+10,data[1].AXIS_Y);
  STORE_LE_16(buff+12,data[1].AXIS_Z);

  STORE_LE_16(buff+14,data[2].AXIS_X);
  STORE_LE_16(buff+16,data[2].AXIS_Y);
  STORE_LE_16(buff+18,data[2].AXIS_Z);
#else
#error SEND_N_QUATERNIONS could be only 1,2,3
#endif
  
  if(QuaternionNotification)
  {
    ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, QuaternionsCharHandle, 0, 2+6*SEND_N_QUATERNIONS, buff);
    
    if (ret != BLE_STATUS_SUCCESS){
      PRINTF("Error Updating Quat Char\r\n");
      if(StdErrNotification){
        BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Quat Char\r\n");
        Stderr_Update(BufferToWrite,BytesToWrite);
      }
      return BLE_STATUS_ERROR;
    }
  }
  return BLE_STATUS_SUCCESS;
}

#if USE_CUSTOM_ALGORITHM1
/**
 * @brief  Update the Algorithm1 Feature
 * @param  xMAR_Value
 * @retval tBleStatus Status
 */
tBleStatus Algorithm1_Update(uint16_t xMAR_Value)
{
  tBleStatus ret;
  uint8_t buff[20];
  
  STORE_LE_16(buff,TimeStamp);
  STORE_LE_16(buff+2,xMAR_Value);
  
  if(ConnectionAlgo1)
  {
    ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, Algorithm1Handle, 0, ALGORITHM1_CHAR_SIZE, buff);
    
    if (ret != BLE_STATUS_SUCCESS)
    {
      PRINTF("Error while updating Algorithm1 characteristic.\n") ;
      if(StdErrNotification){
        BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Algorithm1 Char\r\n");
        Stderr_Update(BufferToWrite,BytesToWrite);
      }
      return BLE_STATUS_ERROR;
    }
  }
  return BLE_STATUS_SUCCESS;
}
#endif //USE_CUSTOM_ALGORITHM1

#if USE_CUSTOM_ALGORITHM2
/**
 * @brief  Update the Algorithm2 Feature
 * @param  xMCP_Value
 * @retval tBleStatus Status
 */
tBleStatus Algorithm2_Update(uint16_t xMCP_Value)
{
  tBleStatus ret;
  uint8_t buff[20];
  
  STORE_LE_16(buff,TimeStamp);
  STORE_LE_16(buff+2,xMCP_Value);
  
  if(ConnectionAlgo2)
  {
    ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, Algorithm2Handle, 0, ALGORITHM2_CHAR_SIZE, buff);
    
    if (ret != BLE_STATUS_SUCCESS)
    {
      PRINTF("Error while updating Algorithm2 characteristic.\n") ;
      if(StdErrNotification){
        BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Algorithm2 Char\r\n");
        Stderr_Update(BufferToWrite,BytesToWrite);
      }
      return BLE_STATUS_ERROR;
    }
  }
  return BLE_STATUS_SUCCESS;
}
#endif //USE_CUSTOM_ALGORITHM2

#if USE_CUSTOM_ALGORITHM3       //USE ALGORITHM3_SW

/**
* @brief  Update the Algorithm3 Feature
 * @param  nSteps
 * @param  nFreq
 * @retval tBleStatus Status
 */
tBleStatus Algorithm3_Update(uint32_t nSteps, uint16_t nFreq)
{
  tBleStatus ret;
  uint8_t buff[20];
  
  STORE_LE_16(buff,TimeStamp);
  STORE_LE_32(buff+2,nSteps);
  STORE_LE_16(buff+6,nFreq);
  
  if(ConnectionAlgo3)
  {
    ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, Algorithm3Handle, 0, ALGORITHM3_CHAR_SIZE, buff);
    
    if (ret != BLE_STATUS_SUCCESS)
    {
      PRINTF("Error while updating Algorithm3 characteristic.\n") ;
      if(StdErrNotification){
        BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Algorithm3 Char\r\n");
        Stderr_Update(BufferToWrite,BytesToWrite);
      }
      return BLE_STATUS_ERROR;
    }
  }
  return BLE_STATUS_SUCCESS;
}

#endif //USE_CUSTOM_ALGORITHM3 


#if USE_CUSTOM_ALGORITHM4
/**
 * @brief  Update the Algorithm4 Feature
 * @param  nFreeFall
 * @retval tBleStatus Status
 */
tBleStatus Algorithm4_Update(uint8_t nFreeFall)
{
  tBleStatus ret;
  uint8_t buff[20];
  
  uint8_t i;
  for(i = 0; i < 20; i ++)
  {
    uint16_t a = 0;
    a = i;
    buff[i] = a;
    buff[i] = 0;
  }
  STORE_LE_16(buff,TimeStamp);
  STORE_LE_16(buff+2,nFreeFall);
  
  if(ConnectionAlgo4)
  {
    ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, Algorithm4Handle, 0, ALGORITHM4_CHAR_SIZE, buff);
    
    if (ret != BLE_STATUS_SUCCESS)
    {
      PRINTF("Error while updating Algorithm4 characteristic.\n") ;
      if(StdErrNotification){
        BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Algorithm4 Char\r\n");
        Stderr_Update(BufferToWrite,BytesToWrite);
      }
      return BLE_STATUS_ERROR;
    }
  }
  return BLE_STATUS_SUCCESS;
}
#endif //USE_CUSTOM_ALGORITHM4

#if USE_CUSTOM_ACCEVENT //new pedometer in accevent

/**
 * @brief  Send a notification When the DS3 detects one Acceleration event
 * @param  Command to Send
 * @retval tBleStatus Status
 */
tBleStatus AccEvent_Update(uint8_t nAccEvStatus, uint16_t StepCounter )
{
  tBleStatus ret;
  uint8_t buff[2+3];

  STORE_LE_16(buff  ,TimeStamp);
  STORE_LE_16(buff+2,nAccEvStatus);
  STORE_LE_16(buff+3,StepCounter);

  if(AccEventNotification)
  {
    ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, AccEventCharHandle, 0, 2+3,buff);
    if (ret != BLE_STATUS_SUCCESS){
      PRINTF("Error Updating AccEvent Char\r\n");
      if(StdErrNotification){
        BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating AccEvent_Update Char\r\n");
        Stderr_Update(BufferToWrite,BytesToWrite);
      }
      
      return BLE_STATUS_ERROR;
    }
  }
  return BLE_STATUS_SUCCESS;
}


#endif //USE_CUSTOM_ACCEVENT

#if USE_CUSTOM_GP_ALGORITHM
/**
 * @brief  Update the GP_ALGORITHM Feature
 * @param  nFreeFall
 * @retval tBleStatus Status
 */
tBleStatus GP_ALGORITHM_Update(uint8_t nStatus)
{
  tBleStatus ret;
  uint8_t buff[20];
  
  uint8_t i;
  for(i = 0; i < 20; i ++)
  {
    uint16_t a = 0;
    a = i;
    buff[i] = a;
    buff[i] = 0;
  }
  STORE_LE_16(buff,TimeStamp);
  STORE_LE_16(buff+2,nStatus);
  
  if(ConnectionAlgoGP)
  {
    ret = ACI_GATT_UPDATE_CHAR_VALUE(GP_ServHandle, GP_ALGORITHMHandle, 0, GP_ALGORITHM_CHAR_SIZE, buff);
    
    if (ret != BLE_STATUS_SUCCESS)
    {
      PRINTF("Error while updating GP_ALGORITHM characteristic.\n") ;
      if(StdErrNotification){
        BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating GP_ALGORITHM Char\r\n");
        Stderr_Update(BufferToWrite,BytesToWrite);
      }
      return BLE_STATUS_ERROR;
    }
  }
  return BLE_STATUS_SUCCESS;
}
#endif //USE_CUSTOM_GP_ALGORITHM

/**
 * @brief  Update the Config characteristic
 * @retval tBleStatus Status
 */
tBleStatus ctrl1_Update(uint8_t* data, uint8_t length)
{
  tBleStatus ret;    
  uint8_t buff[20];
  
  memcpy(buff,data,length);
  
  ret = ACI_GATT_UPDATE_CHAR_VALUE(ConfigBlueSTServHandle, ConfigRegsCharHandle, 0, length, buff);
  if (ret != BLE_STATUS_SUCCESS){
    PRINTF("Error Updating Config Char\r\n");
    if(StdErrNotification){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Config Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    }
    
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Add the HW Feature service using a vendor specific profile
 * @retval tBleStatus Status
 */
tBleStatus Add_Feature_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];
  
  COPY_HW_SENS_BLUEST_SERVICE_UUID(uuid);
  
  uint8_t max_attr_records = 7 /* Sensor Fusion Short&Long precision */;
#if USE_CUSTOM_ALGORITHM1
  max_attr_records++;
#endif /* USE_CUSTOM_ALGORITHM1 */
  
#if USE_CUSTOM_ALGORITHM2
  max_attr_records++;
#endif /* USE_CUSTOM_ALGORITHM2 */  

#if USE_CUSTOM_ALGORITHM3
  max_attr_records++;
#endif /* USE_CUSTOM_ALGORITHM3 */  

#if USE_CUSTOM_ALGORITHM4
  max_attr_records++;
#endif /* USE_CUSTOM_ALGORITHM4 */  
  
#if USE_CUSTOM_ACCEVENT
  max_attr_records++;   //for AccEventCharHandle
#endif /* USE_CUSTOM_ACCEVENT */  

  ret = ACI_GATT_ADD_SERV(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3*max_attr_records,&HWServHandle);
  if (ret != BLE_STATUS_SUCCESS)
    goto fail;
  
#if USE_CUSTOM_GP_ALGORITHM
  max_attr_records=0;
  max_attr_records++;
  COPY_GP_ALGORITHM_BLUEST_SERVICE_UUID(uuid);
  
  ret = ACI_GATT_ADD_SERV(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3*max_attr_records,&GP_ServHandle);
  if (ret != BLE_STATUS_SUCCESS)
    goto fail;
#endif /* USE_CUSTOM_GP_ALGORITHM */  
  

  COPY_ACC_GYRO_MAG_BLUEST_CHAR_UUID(uuid);
  ret =  ACI_GATT_ADD_CHAR(HWServHandle, UUID_TYPE_128, uuid, 2+3*3*2,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AccGyroMagCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
    goto fail;
  
  COPY_PRESSURE_BLUEST_CHAR_UUID(uuid);
  ret =  ACI_GATT_ADD_CHAR(HWServHandle, UUID_TYPE_128, uuid, 4+2,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &PressureCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
    goto fail;
  
  COPY_TEMPERATURE_BLUEST_CHAR_UUID(uuid);
  ret =  ACI_GATT_ADD_CHAR(HWServHandle, UUID_TYPE_128, uuid, 2+2,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &TemperatureCharHandle); 
  if (ret != BLE_STATUS_SUCCESS)
    goto fail;  
  
  COPY_PWR_BLUEST_CHAR_UUID(uuid);  
  ret =  ACI_GATT_ADD_CHAR(HWServHandle, UUID_TYPE_128, uuid, 9, 
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ, 
                           ATTR_PERMISSION_NONE, 
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &pwrCharHandle);
  if (ret != BLE_STATUS_SUCCESS) 
    goto fail;

#if USE_CUSTOM_ACCEVENT

  COPY_ACCEVENT_BLUEST_CHAR_UUID(uuid);
  ret =  ACI_GATT_ADD_CHAR(HWServHandle, UUID_TYPE_128, uuid, ACCEVENT_CHAR_SIZE,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &AccEventCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#endif /* USE_CUSTOM_ACCEVENT */
  
  
#if USE_CUSTOM_ALGORITHMFX 
  COPY_QUATERNIONS_BLUEST_CHAR_UUID(uuid);
  ret =  ACI_GATT_ADD_CHAR(HWServHandle, UUID_TYPE_128, uuid, 2+6*SEND_N_QUATERNIONS,
                           CHAR_PROP_NOTIFY  | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &QuaternionsCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#endif /* USE_CUSTOM_ALGORITHMFX */  
  
#if USE_QUATERNION_FLOAT  
  COPY_QUATER_FLOAT_BLUEST_CHAR_UUID(uuid);
  ret =  ACI_GATT_ADD_CHAR(HWServHandle, UUID_TYPE_128, uuid, 20,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &QuaterFloatCharHandle);

  if (ret != BLE_STATUS_SUCCESS)
    goto fail;
#endif /* USE_QUATERNION_FLOAT */
  
#if USE_CUSTOM_ALGORITHM1
  COPY_ALGORITHM1_BLUEST_CHAR_UUID(uuid);
  ret =  ACI_GATT_ADD_CHAR(HWServHandle, UUID_TYPE_128, uuid, ALGORITHM1_CHAR_SIZE,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &Algorithm1Handle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#endif /* USE_CUSTOM_ALGORITHM1 */

#if USE_CUSTOM_ALGORITHM2
  COPY_ALGORITHM2_BLUEST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServHandle, UUID_TYPE_128, uuid, ALGORITHM2_CHAR_SIZE,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &Algorithm2Handle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#endif /* USE_CUSTOM_ALGORITHM2 */
  
#if USE_CUSTOM_ALGORITHM3
  COPY_ALGORITHM3_BLUEST_CHAR_UUID(uuid);
  ret =  ACI_GATT_ADD_CHAR(HWServHandle, UUID_TYPE_128, uuid, ALGORITHM3_CHAR_SIZE,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &Algorithm3Handle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#endif /* USE_CUSTOM_ALGORITHM3 */
  
#if USE_CUSTOM_ALGORITHM4
  COPY_ALGORITHM4_BLUEST_CHAR_UUID(uuid);
  ret =  ACI_GATT_ADD_CHAR(HWServHandle, UUID_TYPE_128, uuid, ALGORITHM4_CHAR_SIZE,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &Algorithm4Handle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#endif /* USE_CUSTOM_ALGORITHM4 */
  
#if USE_CUSTOM_GP_ALGORITHM
  COPY_GP_ALGORITHM_BLUEST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(GP_ServHandle, UUID_TYPE_128, uuid, GP_ALGORITHM_CHAR_SIZE,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &GP_ALGORITHMHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#endif /* USE_CUSTOM_GP_ALGORITHM */
  
  return BLE_STATUS_SUCCESS;

fail:
  PRINTF("Error while adding HW's Characteristcs service.\n");
  return BLE_STATUS_ERROR ;
}


/**
 * @brief  Update Battery charge characteristic value
 * @retval tBleStatus      Status
 */
tBleStatus Pwr_Update(int16_t chg, int16_t voltage, int16_t current, uint8_t nPwrStat)
{
  tBleStatus ret; 
  uint8_t buff[2+8];
  
  STORE_LE_16(buff ,TimeStamp);
  STORE_LE_16(buff+2,chg);
  STORE_LE_16(buff+4,voltage);
  STORE_LE_16(buff+6,current);
  STORE_LE_16(buff+8,nPwrStat);
  buff[8] = nPwrStat;
  
  if(ConnectionPower)
  {
    ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, pwrCharHandle, 0, 9, buff);
    
    if (ret != BLE_STATUS_SUCCESS){
      PRINTF("Error while updating Power characteristic.\n") ;
      if(StdErrNotification){
        BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Power Char\r\n");
        Stderr_Update(BufferToWrite,BytesToWrite);
      }
      return BLE_STATUS_ERROR ;
    }
  }
  return BLE_STATUS_SUCCESS;
}


/**
 * @brief  Update accelerometer, gyroscope and magnetometer characteristic value
 * @param  Acc Structure containing acceleration value in mg
 * @param  Gyro Structure containing Gyroscope value
 * @param  Mag Structure containing magneto value
 * @retval tBleStatus      Status
 */
tBleStatus AccGyroMag_Update(AxesRaw_TypeDef* Acc, AxesRaw_TypeDef* Gyro, AxesRaw_TypeDef* Mag)
{  
  tBleStatus ret;

  uint8_t buff[2+3*3*2];

  STORE_LE_16(buff   ,TimeStamp);
  
  STORE_LE_16(buff+2 ,Acc->AXIS_X);
  STORE_LE_16(buff+4 ,Acc->AXIS_Y);
  STORE_LE_16(buff+6 ,Acc->AXIS_Z);
  
  STORE_LE_16(buff+8 ,Gyro->AXIS_X);
  STORE_LE_16(buff+10,Gyro->AXIS_Y);
  STORE_LE_16(buff+12,Gyro->AXIS_Z);
  
  STORE_LE_16(buff+14,Mag->AXIS_X);
  STORE_LE_16(buff+16,Mag->AXIS_Y);
  STORE_LE_16(buff+18,Mag->AXIS_Z);
  
  if(ConnectionAccGyroMag)
  {
    ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, AccGyroMagCharHandle, 0, 2+3*3*2, buff);
    
    if (ret != BLE_STATUS_SUCCESS){
      PRINTF("Error Updating Acc/Gyro/Mag Char\r\n");
      if(StdErrNotification){
        BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Acc/Gyro/Mag Char\r\n");
        Stderr_Update(BufferToWrite,BytesToWrite);
      }
      
      return BLE_STATUS_ERROR ;
    }
  }
  return BLE_STATUS_SUCCESS;	
}


/**
 * @brief  Update temperature characteristic value
 * @param  temp Temperature in tenths of degree 
 * @retval Status
 */
tBleStatus Temp_Update(int16_t temp)
{  
  tBleStatus ret;
  
  uint8_t buff[2+2];

  STORE_LE_16(buff  ,TimeStamp);
  STORE_LE_16(buff+2,temp);
  
  if(ConnectionTemp)
  {
    ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, TemperatureCharHandle, 0, 2+2,buff);
    
    if (ret != BLE_STATUS_SUCCESS){
      PRINTF("Error Updating Temp Char\r\n");
      if(StdErrNotification){
        BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Temp Char\r\n");
        Stderr_Update(BufferToWrite,BytesToWrite);
      }
      
      return BLE_STATUS_ERROR ;
    }
  }
  return BLE_STATUS_SUCCESS;
  
}

/**
 * @brief  Update pressure characteristic value
 * @param  press Pressure in mbar 
 * @retval Status
 */
tBleStatus Press_Update(int32_t press)
{  
  tBleStatus ret;
  uint8_t buff[4+2];

  STORE_LE_16(buff  ,TimeStamp);
  STORE_LE_32(buff+2,press);
  
  if(ConnectionPres)
  {
    ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, PressureCharHandle, 0, 4+2,buff);
    
    if (ret != BLE_STATUS_SUCCESS){
      PRINTF("Error Updating Press Char\r\n");
      if(StdErrNotification){
        BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Press Char\r\n");
        Stderr_Update(BufferToWrite,BytesToWrite);
      }
      
      return BLE_STATUS_ERROR ;
    }
  }
  return BLE_STATUS_SUCCESS;
}



/**
 * @brief  Send a request to the Master to update the connection parameters (e.g. Connection interval)
 * @retval None
 */
void UpdateConnectionParameters()
{
  if(bConnParamUpdate)
  {
    bConnParamUpdate = 0;
#if USE_UPDATE_CONNECTION_PARAMETER
    tBleStatus r = 100;
    if(SESSION_REG(nBleConIntv) >=6 && SESSION_REG(nBleConIntv) <=3200 )
    {
      
      extern tBleStatus aci_l2cap_connection_parameter_update_request(uint16_t conn_handle, uint16_t interval_min,
							 uint16_t interval_max, uint16_t slave_latency,
							 uint16_t timeout_multiplier);

      r = aci_l2cap_connection_parameter_update_request(connection_handle, 9/*interval_min*/, SESSION_REG(nBleConIntv)/*interval_max*/, 
                                                        0/*slave_latency*/, 600/*timeout_mult*/);
      if(r==0)
      {
        DBG_PRINTF_BLUENRG(WeSU_TERMINAL_DEBUG_SEVERITY, "Conn Intv set successful to %d\r\n", SESSION_REG(nBleConIntv));
      }
      else
      {
        DBG_PRINTF_BLUENRG(WeSU_TERMINAL_DEBUG_SEVERITY, "Conn Intv set to %d completed with error %d\r\n", SESSION_REG(nBleConIntv), r);
      }
    }
#else
  SESSION_REG(nBleConIntv) = 0;
#endif //USE_UPDATE_CONNECTION_PARAMETER
  }
}


/**
 * @brief  This function is called when there is a Bluetooth Read request
 * @param  handle Handle of the attribute
 * @retval None
 */
void Read_Request_CB(uint16_t handle)
{
  if(handle == AccGyroMagCharHandle + 1)
  {
    AccGyroMag_Update(&ACC_Value,&GYR_Value,&MAG_Value); 
  }
  else if(handle == TemperatureCharHandle + 1)
  {
    Temp_Update(TEMP_Value); 
  }
  else if(handle == pwrCharHandle + 1)
  {
    Pwr_Update(BATT_SOC_Value, BATT_V_Value,BATT_I_Value,BATT_STATUS_Value);
  }
  else if(handle == PressureCharHandle + 1)
  {
    Press_Update(PRESS_Value);
  }
  else if (handle == StdErrCharHandle + 1)
  {
    /* Send again the last packet for StdError */
    Stderr_Update_AfterRead();
  }
  else if (handle == TermCharHandle + 1)
  {
    /* Send again the last packet for Terminal */
    if(Term_Update_AfterRead()){  SET_BNRG_ERROR_FLAG(); }
  }
  
  if(connection_handle != 0)
  {
    aci_gatt_allow_read(connection_handle);
  }
}

/**
 * @brief  This function is called when there is a change on the gatt attribute
 * With this function it's possible to understand if one application 
 * is subscribed or not to the one service
 * @param attr_handle Handle of the attribute
 * @param att_data attribute data
 * @param data_length length of the data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length) {
#if USE_QUATERNION_FLOAT
   if(attr_handle == QuaterFloatCharHandle + 2){
    if (att_data[0] == 01) {
      QuaterFloatNotification=1;
    } else if (att_data[0] == 0){
      QuaterFloatNotification=0;
    }
#ifdef DEBUG_CONNECTION
    if(TermNotification) {
      BytesToWrite = sprintf((char *)BufferToWrite,"--->QuatF=%s\r\n", QuaterFloatNotification ? "ON" : "OFF");
     if(Term_Update(BufferToWrite,BytesToWrite)){  SET_BNRG_ERROR_FLAG(); }
    }
#endif /* DEBUG_CONNECTION */
    PRINTF("--->QuatF=%s\r\n", QuaterFloatNotification ? "ON" : "OFF");
   }else 
#endif /* USE_QUATERNION_FLOAT */ 
    if(attr_handle == QuaternionsCharHandle + 2){
    if (att_data[0] == 01) {
      QuaternionNotification=1;
    } else if (att_data[0] == 0){
      QuaternionNotification=0;
    }
#ifdef DEBUG_CONNECTION
    if(TermNotification) {
      BytesToWrite = sprintf((char *)BufferToWrite,"--->Quater=%s\r\n", QuaternionNotification ? "ON" : "OFF");
     Term_Update(BufferToWrite,BytesToWrite);
    }
#endif /* DEBUG_CONNECTION */   
    PRINTF("--->Quater=%s\r\n", QuaternionNotification ? "ON" : "OFF");
   } else if(attr_handle == ConfigRegsCharHandle + 2){
    //anything to do?
  } else if(attr_handle == AccGyroMagCharHandle + 2) {
    if (att_data[0] == 01) {
      ConnectionAccGyroMag = 1;
    } else if (att_data[0] == 0) {
      ConnectionAccGyroMag = 0;
    }
#ifdef DEBUG_CONNECTION
    if(TermNotification) {
      BytesToWrite = sprintf((char *)BufferToWrite,"--->Acc/Gyro/Mag=%s\r\n", ConnectionAccGyroMag ? "ON" : "OFF");
     if(Term_Update(BufferToWrite,BytesToWrite)){  SET_BNRG_ERROR_FLAG(); }
    }
#endif /* DEBUG_CONNECTION */
    PRINTF("--->Acc/Gyro/Mag=%s\r\n", ConnectionAccGyroMag ? "ON" : "OFF");
  } else if(attr_handle == TemperatureCharHandle + 2){
    if (att_data[0] == 01) {
      ConnectionTemp=1;
    } else if (att_data[0] == 0){
      ConnectionTemp=0;
    }
#ifdef DEBUG_CONNECTION
    if(TermNotification) {
      BytesToWrite = sprintf((char *)BufferToWrite,"--->Temp=%s\r\n", ConnectionTemp ? "ON" : "OFF");
     if(Term_Update(BufferToWrite,BytesToWrite)){  SET_BNRG_ERROR_FLAG(); }
    }
#endif /* DEBUG_CONNECTION */
    PRINTF("--->Temp=%s\r\n", ConnectionTemp ? "ON" : "OFF");
  } else if(attr_handle == PressureCharHandle + 2){
    if (att_data[0] == 01) {
      ConnectionPres=1;
    } else if (att_data[0] == 0){
      ConnectionPres=0;
    }
#ifdef DEBUG_CONNECTION
    if(TermNotification) {
      BytesToWrite = sprintf((char *)BufferToWrite,"--->Pres=%s\r\n", ConnectionPres ? "ON" : "OFF");
     if(Term_Update(BufferToWrite,BytesToWrite)){  SET_BNRG_ERROR_FLAG(); }
    }
#endif /* DEBUG_CONNECTION */
    PRINTF("--->Pres=%s\r\n", ConnectionPres ? "ON" : "OFF");
  } else if(attr_handle == StdErrCharHandle + 2){
    if (att_data[0] == 01) {
      StdErrNotification=1;
    } else if (att_data[0] == 0){
      StdErrNotification=0;
    }
#ifdef DEBUG_CONNECTION
    if(TermNotification) {
      BytesToWrite = sprintf((char *)BufferToWrite,"--->StdE=%s\r\n", StdErrNotification ? "ON" : "OFF");
     if(Term_Update(BufferToWrite,BytesToWrite)){  SET_BNRG_ERROR_FLAG(); }
    }
#endif /* DEBUG_CONNECTION */
    PRINTF("--->StdE=%s\r\n", StdErrNotification ? "ON" : "OFF");
  } else if(attr_handle == TermCharHandle + 2){
    if (att_data[0] == 01) {
      TermNotification=1;
    } else if (att_data[0] == 0){
      TermNotification=0;
    }
#ifdef DEBUG_CONNECTION
    PRINTF("--->Term=%s\r\n", TermNotification ? "ON" : "OFF");
#endif /* DEBUG_CONNECTION */
  } else if (attr_handle == TermCharHandle + 1){
    /* Received one write from Client on Terminal characteristc */
    ProcessTerminalStringBle(att_data, data_length);
  } else if (attr_handle == pwrCharHandle + 2) {
    /* Change notification status */
    if (att_data[0] == 01) {
      ConnectionPower=1;
    } else if (att_data[0] == 0){
      ConnectionPower=0;
    }
    PRINTF("--->Power=%s\r\n", ConnectionPower ? "ON" : "OFF");
#if USE_CUSTOM_ALGORITHM1
  } else if (attr_handle == Algorithm1Handle + 2) {
    /* Change notification status */
    if (att_data[0] == 01) {
      ConnectionAlgo1=1;
    } else if (att_data[0] == 0){
      ConnectionAlgo1=0;
    }
    PRINTF("--->Algo1=%s\r\n", ConnectionAlgo1 ? "ON" : "OFF");
#endif /* USE_CUSTOM_ALGORITHM1 */

#if USE_CUSTOM_ALGORITHM2
  } else if (attr_handle == Algorithm2Handle + 2) {
    /* Change notification status */
    if (att_data[0] == 01) {
      ConnectionAlgo2=1;
    } else if (att_data[0] == 0){
      ConnectionAlgo2=0;
    }
    PRINTF("--->Algo2=%s\r\n", ConnectionAlgo2 ? "ON" : "OFF");
#endif /* USE_CUSTOM_ALGORITHM2 */
  
#if USE_CUSTOM_ALGORITHM3
  } else if (attr_handle == Algorithm3Handle + 2) {
    /* Change notification status */
    if (att_data[0] == 01) {
      ConnectionAlgo3=1;
    } else if (att_data[0] == 0){
      ConnectionAlgo3=0;
    }
    PRINTF("--->Algo3=%s\r\n", ConnectionAlgo3 ? "ON" : "OFF");
#endif /* USE_CUSTOM_ALGORITHM3 */
  
#if USE_CUSTOM_ALGORITHM4
  } else if (attr_handle == Algorithm4Handle + 2) {
    /* Change notification status */
    if (att_data[0] == 01) {
      ConnectionAlgo4=1;
    } else if (att_data[0] == 0){
      ConnectionAlgo4=0;
    }
    PRINTF("--->Algo4=%s\r\n", ConnectionAlgo4 ? "ON" : "OFF");
#endif /* USE_CUSTOM_ALGORITHM4 */

#if USE_CUSTOM_ACCEVENT
  } else if(attr_handle == AccEventCharHandle + 2) {
    if (att_data[0] == 01) {
      AccEventNotification=1;
    } else if (att_data[0] == 0) {
      AccEventNotification=0;
    }
    PRINTF("--->AccEvent=%s\r\n", AccEventNotification ? "ON" : "OFF");
#endif /* USE_CUSTOM_ACCEVENT */
 
#if USE_CUSTOM_GP_ALGORITHM
  } else if (attr_handle == GP_ALGORITHMHandle + 2) {
    /* Change notification status */
    if (att_data[0] == 01) {
      ConnectionAlgoGP=1;
    } else if (att_data[0] == 0){
      ConnectionAlgoGP=0;
    }
    PRINTF("--->AlgoGP=%s\r\n", ConnectionAlgoGP ? "ON" : "OFF");
#endif /* USE_CUSTOM_GP_ALGORITHM */
  } else if (attr_handle == ConfigRegsCharHandle + 1) {
    uint8_t pDataCopy[64];
    memcpy(pDataCopy, att_data, 64);
    if(ProcessRegCommand((RegFrameStruct_t*)pDataCopy, data_length) == PERMREG_NEW_OPERATION_COMPLETE_WITH_ACK)
    {
      if(N_PAYLOAD_LENGHT((RegFrameStruct_t*)pDataCopy)>=8)
      {
        N_PAYLOAD_LENGHT((RegFrameStruct_t*)pDataCopy) = 8;
      }
      ctrl1_Update(pDataCopy,(2*N_PAYLOAD_LENGHT((RegFrameStruct_t*)pDataCopy))+4);
    }
  } else {
    if(StdErrNotification){
      BytesToWrite = sprintf((char *)BufferToWrite, "Notification UNKNOWN handle%d\r\n", attr_handle);
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      PRINTF("Notification UNKNOWN handle %d\r\n", attr_handle);
    }
  }
}

/**
 * @brief  This function is called whenever there is an ACI event to be processed.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  pckt Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  if(hci_pckt->type != HCI_EVENT_PKT)
    return;
  
  
  switch(event_pckt->evt){
    
  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;
    
  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      
      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;
    
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){
      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *pr = (void*)blue_evt->data; 
          Read_Request_CB(pr->attr_handle);                    
        }
        break;
      case EVT_BLUE_GATT_WRITE_PERMIT_REQ:
        {
          tBleStatus ret;
          evt_gatt_write_permit_req *evt = (evt_gatt_write_permit_req *)blue_evt->data;
          
          ret = aci_gatt_write_response(connection_handle, evt->attr_handle,
                                        TRUE,
                                        0,
                                        evt->data_length,
                                        evt->data);
          if(ret){
            PRINTF("aci_gatt_write_response failed: 0x%02X\n", ret);
          }
          else{
            Attribute_Modified_CB(evt->attr_handle, evt->data, evt->data_length);
          }
          
        }
        break;
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
          evt_gatt_attr_modified *evt = (evt_gatt_attr_modified*)blue_evt->data;          
          Attribute_Modified_CB(evt->attr_handle, evt->att_data, evt->data_length);
        }
        break;
      }
    }
    break;
  }    
}


/**
 * @brief  Read a license from EEPROM
 * @param  nLic Index of the license to read
 * @param  pPayload Pointer to the output buffer
 * @retval None
 */
void BSP_LICENSE_Read(uint8_t nLic, uint8_t* pPayload)
{
  BSP_EEPROM_ReadBuffer(LICENSE_GET_ADDR(nLic), (uint32_t)pPayload, LICENSE_SIZE);
}

/**
 * @brief  Write a license to EEPROM
 * @param  nLic Index of the license to read
 * @param  pPayload Pointer to the input buffer 
 * @retval None
 */
void BSP_LICENSE_Write(uint8_t nLic, uint8_t *pPayload)
{
  BSP_EEPROM_WriteBuffer(LICENSE_GET_ADDR(nLic), (uint32_t)pPayload, LICENSE_SIZE);
}

/**
 * @brief  Read persistent registers
 * @param  nReg The register to read
 * @param  pPayload Pointer to the output buffer
 * @param  nLen Number of registers to read
 * @retval None
 */
void BSP_PERSREGS_Read(uint8_t nReg, uint8_t *pPayload, uint32_t nLen)
{
  BSP_EEPROM_ReadBuffer(PERMREG_STRUCT_GET_ADDR(nReg), (uint32_t)pPayload, nLen * PERMREG_REG_SIZE);
}

/**
 * @brief  Write persistent registers
 * @param  nReg The register to write
 * @param  pPayload Pointer to the input buffer
 * @param  nLen Number of registers to write
 * @retval None
 */
void BSP_PERSREGS_Write(uint8_t nReg, uint8_t *pPayload, uint32_t nLen)
{
  BSP_EEPROM_WriteBuffer(PERMREG_STRUCT_GET_ADDR(nReg), (uint32_t)pPayload, nLen * PERMREG_REG_SIZE);
}

/**
 * @brief  Read persistent registers
 * @param  nReg The register to read
 * @param  pPayload Pointer to the output buffer
 * @param  nLen Number of registers to read
 * @retval None
 */
void BSP_PERSREGS_ReadBackup(uint8_t nReg, uint8_t *pPayload, uint32_t nLen)
{
  BSP_EEPROM_ReadBuffer(PERMREG_STRUCT_BCK_GET_ADDR(nReg), (uint32_t)pPayload, nLen * PERMREG_REG_SIZE);
}

/**
 * @brief  Write backup persistent registers
 * @param  nReg The register to write
 * @param  pPayload Pointer to the input buffer
 * @param  nLen Number of registers to write
 * @retval None
 */
void BSP_PERSREGS_WriteBackup(uint8_t nReg, uint8_t *pPayload, uint32_t nLen)
{
  BSP_EEPROM_WriteBuffer(PERMREG_STRUCT_BCK_GET_ADDR(nReg), (uint32_t)pPayload, nLen * PERMREG_REG_SIZE);
}

/**
 * @brief  Read session registers
 * @param  nReg The register to read
 * @param  pPayload Pointer to the output buffer
 * @param  nLen Number of registers to read
 * @retval None
 */
void BSP_SESSIONREGS_Read(uint16_t nReg, uint8_t* pPayload, uint32_t nLen)
{
  uint32_t nRamAddr = ((uint32_t)SESSIONREG_STRUCT_START_ADDRESS + (nReg * PERMREG_REG_SIZE));
  memcpy(pPayload, (void*)nRamAddr, nLen * PERMREG_REG_SIZE);
}

/**
 * @brief  Write session registers
 * @param  nReg The register to write
 * @param  pPayload Pointer to the input buffer
 * @param  nLen Number of registers to write
 * @retval None
 */
void BSP_SESSIONREGS_Write(uint16_t nReg, uint8_t* pPayload, uint32_t nLen)
{
  uint32_t nRamAddr = ((uint32_t)SESSIONREG_STRUCT_START_ADDRESS + (nReg * PERMREG_REG_SIZE));
  memcpy((void*)nRamAddr, pPayload, nLen * PERMREG_REG_SIZE);
}

/**
 * @brief  Read persistent registers
 * @param  optype Operation type
 * @param  nReg The register to manage
 * @param  pPayload Pointer to the buffer
 * @param  nLen Number of registers to manage
 * @retval None
 */
void BSP_REGS_Manage(uint8_t optype, uint8_t nReg, uint8_t *pPayload, uint32_t nLen)
{
  switch(optype)
  {
  case (REGS_MANAGE_PERSISTENT|REGS_MANAGE_READ):
    BSP_PERSREGS_Read(nReg, pPayload, nLen);
    break;
  case (REGS_MANAGE_PERSISTENT|REGS_MANAGE_WRITE):
    BSP_PERSREGS_Write(nReg, pPayload, nLen);
    break;
  case (REGS_MANAGE_SESSION|REGS_MANAGE_READ):
    BSP_SESSIONREGS_Read(nReg, pPayload, nLen);
    break;
  case (REGS_MANAGE_SESSION|REGS_MANAGE_WRITE):
    BSP_SESSIONREGS_Write(nReg, pPayload, nLen);
    break;
  }
}

/**
 * @brief  Manage persistent registers, triggering associated actions
 * @param  optype Operation type (read/write)
 * @param  nReg The register to manage
 * @param  pPayload Pointer to the buffer (input/output)
 * @param  nLen Number of registers to manage
 * @retval None
 */
void BSP_REGS_ManageWAction(uint8_t optype, uint8_t nReg, uint8_t *pPayload, uint32_t nLen)
{
  uint8_t nArr[PERMREG_MAX_LENGHT];//max 64 regs per frame
  RegFrameStruct_t *p = (RegFrameStruct_t *)(&nArr[0]);
  p->nRegAddress = nReg;
  p->nErrorCode = 0;
  p->nPayloadLenght = nLen;
  switch(optype)
  {
  case (REGS_MANAGE_PERSISTENT|REGS_MANAGE_READ):
    p->nControlByte = PERMREG_PENDING_OPERATION | PERMREG_ACK_REQUIRED | PERMREG_EEPROM_OPERATION | PERMREG_READ_OPERATION;
    break;
  case (REGS_MANAGE_PERSISTENT|REGS_MANAGE_WRITE):
    p->nControlByte = PERMREG_PENDING_OPERATION | PERMREG_ACK_REQUIRED | PERMREG_EEPROM_OPERATION | PERMREG_WRITE_OPERATION;
    break;
  case (REGS_MANAGE_SESSION|REGS_MANAGE_READ):
    p->nControlByte = PERMREG_PENDING_OPERATION | PERMREG_ACK_REQUIRED | PERMREG_RAM_OPERATION | PERMREG_READ_OPERATION;
    break;
  case (REGS_MANAGE_SESSION|REGS_MANAGE_WRITE):
    p->nControlByte = PERMREG_PENDING_OPERATION | PERMREG_ACK_REQUIRED | PERMREG_RAM_OPERATION | PERMREG_WRITE_OPERATION;
    break;
  }
  if(nLen < (PERMREG_MAX_PAYLOAD_LENGHT/2))
  {
    memcpy(p->pPayload, pPayload, 2*nLen);
    ProcessRegCommand(p, 4+nLen*2);
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
