/**
  ******************************************************************************
  * @file    Sensors_Read/Src/BlueST_Protocol_examples.c
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

#include "main_examples.h"
#include "console_examples.h"
#include <stdio.h>
#include "BlueST_Protocol_examples.h"
#include "wesu_config_examples.h"


/** @addtogroup WeSU_Examples        WeSU Examples       
  * @{
  */

/** @addtogroup WeSU_User_Examples               WeSU User Examples
  * @{
  */

/** @addtogroup BlueST_PROTOCOL_Examples          BlueST PROTOCOL Examples
  * @{
  * @brief API to provide the BlueST Protocol specs on WeSU
  */



int bIsConnected = FALSE;                                                       //!< a connection is active
uint8_t set_connectable = TRUE;                                                 //!< request to go in advertising

volatile uint16_t connection_handle = 0;                                        //!< handle to active connection
uint16_t HWServHandle;                                                          //!< handle to sensors service
uint16_t pwrCharHandle;                                                         //!< handle to power characteristic
uint16_t TemperatureCharHandle;                                                 //!< handle to temperature characteristic
uint16_t PressureCharHandle;                                                    //!< handle to pressure characteristic
uint16_t AccGyroMagCharHandle;                                                  //!< handle to accelerometer, gyroscope and magnetometer characteristic
uint16_t QuaterFloatCharHandle;                                                 //!< handle to AHRS characteristic

#if USE_CUSTOM_ALGORITHM1
  uint16_t Algorithm1Handle;                                                    //!< handle to algorithm1 characteristic
#endif /* USE_CUSTOM_ALGORITHM1 */

uint16_t ConsoleBlueSTHandle;                                                   //!< handle to console service
uint16_t TermCharHandle;                                                        //!< handle to terminal characteristic
uint16_t StdErrCharHandle;                                                      //!< handle to stderr characteristic

uint8_t bBlueNRGErrorFlag;            //!< see @ref PERS_SESS_DESC_REG_0x97

uint32_t StdErrNotification =0;                                                 //!< Connection active on stderr data
uint32_t TermNotification   =0;                                                 //!< Connection active on terminal data

uint8_t BufferToWrite[256];                                                     //!< temp buffer 
uint8_t BytesToWrite;                                                           //!< temp buffer lenght


static uint8_t LastStderrBuffer[CONSOLE_MAX_CHAR_LEN];                          //!< last stderr buffer
static uint8_t LastStderrLen;                                                   //!< last stderr buffer lenght
static uint8_t LastTermBuffer[CONSOLE_MAX_CHAR_LEN];                            //!< last terminal buffer
static uint8_t LastTermLen;                                                     //!< last terminal buffer lenght

uint16_t TimeStamp=0;                                                           //!< BlueST 16-bit timestamp

uint32_t ConnectionTemp=0;                                                      //!< Connection active on temperature data
uint32_t ConnectionPres=0;                                                      //!< Connection active on pressure data
uint32_t ConnectionAccGyroMag=0;                                                //!< Connection active on acc/gyro/mag data

      

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

/* Hardware Characteristics Service */
#define COPY_HW_SENS_BLUEST_SERVICE_UUID(uuid_struct)           COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST Sensors service
#define COPY_TEMPERATURE_BLUEST_CHAR_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x04,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST temperature characteristic
#define COPY_PRESSURE_BLUEST_CHAR_UUID(uuid_struct)             COPY_UUID_128(uuid_struct,0x00,0x10,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST pressure characteristic
#define COPY_MAGNETOMETER_BLUEST_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x00,0x20,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST magnetometer characteristic
#define COPY_GYROSCOPE_BLUEST_CHAR_UUID(uuid_struct)            COPY_UUID_128(uuid_struct,0x00,0x40,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST gyroscope characteristic
#define COPY_ACCELEROMETER_BLUEST_CHAR_UUID(uuid_struct)        COPY_UUID_128(uuid_struct,0x00,0x80,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST accelerometer characteristic
#define COPY_ACC_GYRO_MAG_BLUEST_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x00,0xE0,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST accelerometer, gyroscope and magnetometer characteristic
#define COPY_PWR_BLUEST_CHAR_UUID(uuid_struct)                  COPY_UUID_128(uuid_struct,0x00,0x02,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST power/gas-gauge characteristic

/* Console Service */
#define COPY_CONSOLE_SERVICE_UUID(uuid_struct)                  COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0E,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST Console service
#define COPY_TERM_CHAR_UUID(uuid_struct)                        COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST Terminal characteristic
#define COPY_STDERR_CHAR_UUID(uuid_struct)                      COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)                      //!< Prepare UUID for BLUEST Stderr characteristic


static void       GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);  //!< Connection complete callback
static void       GAP_DisconnectionComplete_CB(void);                           //!< Disconnection complete callback
static tBleStatus Stderr_Update_AfterRead(void);
static tBleStatus Term_Update_AfterRead(void);



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
    if(StdErrNotification){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Stdout Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else
      PRINTF("Error Updating Stdout Char\r\n");
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
  
  connection_handle = handle;

//#ifdef DEBUG_CONNECTION
  PRINTF(">>>>>>CONNECTED %x:%x:%x:%x:%x:%x\r\n",addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]);
//#endif /* DEBUG_CONNECTION */

  TimeStamp=0;
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @retval None
 */
void GAP_DisconnectionComplete_CB(void)
{
  bIsConnected = FALSE;
  

//#ifdef DEBUG_CONNECTION  
  PRINTF("<<<<<<DISCONNECTED\r\n");
//#endif /* DEBUG_CONNECTION */  

  /* Make the device connectable again. */
  set_connectable = TRUE;

  ConnectionTemp=0;
  ConnectionPres=0;
  ConnectionAccGyroMag=0;
  
  TimeStamp=0;
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
      return BLE_STATUS_ERROR;
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
        termUpdateDelay++;
        if(StdErrNotification){
          BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Stdout Char\r\n");
          Stderr_Update(BufferToWrite,BytesToWrite);
        } else
          PRINTF("Error Updating Stdout Char\r\n");
        return BLE_STATUS_ERROR;
      }
    }
  }

  return BLE_STATUS_SUCCESS;
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
//  ENABLED_DEBUG_LEVEL_BLE = WeSU_DEBUG_SEVERITY_VERBOSE;
  return BLE_STATUS_SUCCESS;

fail:
  //PRINTF("Error while adding Console service.\n");
  return BLE_STATUS_ERROR ;
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
  
  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, Algorithm1Handle, 0, ALGORITHM1_CHAR_SIZE, buff);
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    //PRINTF("Error while updating Algorithm1 characteristic.\n") ;
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
#endif //USE_CUSTOM_ALGORITHM1

/**
 * @brief  Add the HW Feature service using a vendor specific profile
 * @retval tBleStatus Status
 */
tBleStatus Add_Feature_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];
  
  COPY_HW_SENS_BLUEST_SERVICE_UUID(uuid);
  
  uint8_t max_attr_records = 6 /* Sensor Fusion Short&Long precision */;
#if USE_CUSTOM_ALGORITHM1
  max_attr_records++;
#endif /* USE_CUSTOM_ALGORITHM1 */
  
  ret = ACI_GATT_ADD_SERV(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3*max_attr_records,&HWServHandle);
  if (ret != BLE_STATUS_SUCCESS)
    goto fail;
  

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

#if USE_CUSTOM_ALGORITHM1
  COPY_ALGORITHM1_BLUEST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServHandle, UUID_TYPE_128, uuid, ALGORITHM1_CHAR_SIZE,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &Algorithm1Handle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#endif /* USE_CUSTOM_ALGORITHM1 */
  
  return BLE_STATUS_SUCCESS;

fail:
  //PRINTF("Error while adding HW's Characteristcs service.\n");
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
    
    ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, pwrCharHandle, 0, 9, buff);
    
  if (ret != BLE_STATUS_SUCCESS){
    //PRINTF("Error while updating ACC characteristic.\n") ;
    return BLE_STATUS_ERROR ;
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
  
  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, AccGyroMagCharHandle, 0, 2+3*3*2, buff);
	
  if (ret != BLE_STATUS_SUCCESS){
    if(StdErrNotification){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Acc/Gyro/Mag Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else
      PRINTF("Error Updating Acc/Gyro/Mag Char\r\n");
    return BLE_STATUS_ERROR ;
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
  
  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, TemperatureCharHandle, 0, 2+2,buff);
  
  if (ret != BLE_STATUS_SUCCESS){
    if(StdErrNotification){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Temp Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else
      PRINTF("Error Updating Temp Char\r\n");
    return BLE_STATUS_ERROR ;
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
  
  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServHandle, PressureCharHandle, 0, 4+2,buff);
  
  if (ret != BLE_STATUS_SUCCESS){
  if(StdErrNotification){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Press Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else
      PRINTF("Error Updating Press Char\r\n");
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;
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
  
  //EXIT:
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
   if(attr_handle == QuaterFloatCharHandle + 2){
#ifdef DEBUG_CONNECTION
    if(TermNotification) {
      BytesToWrite = sprintf((char *)BufferToWrite,"--->QuatF=%s\r\n", QuaterFloatNotification ? "ON" : "OFF");
     if(Term_Update(BufferToWrite,BytesToWrite)){  SET_BNRG_ERROR_FLAG(); }
    } else
      PRINTF("--->QuatF=%s\r\n", QuaterFloatNotification ? "ON" : "OFF");
#endif /* DEBUG_CONNECTION */
#if USE_CUSTOM_ALGORITHM1
   }else if(attr_handle == Algorithm1Handle + 2){
    if (att_data[0] == 01) {
//      QuaterFloatNotification=1;
    } else if (att_data[0] == 0){
//      QuaterFloatNotification=0;
    }
#ifdef DEBUG_CONNECTION
    if(TermNotification) {
      BytesToWrite = sprintf((char *)BufferToWrite,"--->Algorithm1Handle=%s\r\n", Algorithm1Handle ? "ON" : "OFF");
     if(Term_Update(BufferToWrite,BytesToWrite)){  SET_BNRG_ERROR_FLAG(); }
    } else
      PRINTF("--->Algorithm1Handle=%s\r\n", Algorithm1Handle ? "ON" : "OFF");
#endif /* DEBUG_CONNECTION */
#endif //USE_CUSTOM_ALGORITHM1
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
    } else
      PRINTF("--->Acc/Gyro/Mag=%s\r\n", ConnectionAccGyroMag ? "ON" : "OFF");
#endif /* DEBUG_CONNECTION */
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
    } else
      PRINTF("--->Temp=%s\r\n", ConnectionTemp ? "ON" : "OFF");
#endif /* DEBUG_CONNECTION */
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
    } else
      PRINTF("--->Pres=%s\r\n", ConnectionPres ? "ON" : "OFF");
#endif /* DEBUG_CONNECTION */
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
    } else
      PRINTF("--->StdE=%s\r\n", StdErrNotification ? "ON" : "OFF");
#endif /* DEBUG_CONNECTION */
  } else if(attr_handle == TermCharHandle + 2){
    if (att_data[0] == 01) {
      TermNotification=1;
//      TerminalShowWelcome();
    } else if (att_data[0] == 0){
      TermNotification=0;
    }
#ifdef DEBUG_CONNECTION
    PRINTF("--->Term=%s\r\n", TermNotification ? "ON" : "OFF");
#endif /* DEBUG_CONNECTION */
  } else if (attr_handle == TermCharHandle + 1){
    /* Received one write from Client on Terminal characteristc */
    /* Send it back for testing */
    
    ProcessTerminalStringBle(att_data, data_length);
    
  } else if (attr_handle == pwrCharHandle + 2) {
    /* Change notification status */
#if USE_CUSTOM_ALGORITHM1
  } else if (attr_handle == Algorithm1Handle + 2) {
    /* Change notification status */
#endif /* USE_CUSTOM_ALGORITHM1 */
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
            //PRINTF("write ok\n");
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
 * @}
 */

/**
 * @}
 */

/**
* @}
 */


/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
