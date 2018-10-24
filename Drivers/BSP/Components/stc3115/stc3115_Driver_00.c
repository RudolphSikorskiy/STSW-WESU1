/**
  ******************************************************************************
  * @file    stc3115_Driver_00.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   This file includes the driver for ST3115 Gas Gauge
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

/* Includes ------------------------------------------------------------------*/
#include "stc3115_Driver.h" 
#include "stc3115_Battery.h"

/** @addtogroup BSP             BSP
 * @{
 */

/** @addtogroup COMPONENTS      COMPONENTS
 * @{
 */
 

/** @addtogroup STC3115         STC3115
  * @{
  */

/** @defgroup STC3115_Exported_Types            STC3115_Exported_Types
  * @{
  */
   
  RAMData_TypeDef RAMData;      //!< RAM registers structure

/**
  * @}
  */ 

/** @defgroup STC3115_Imported_Functions         STC3115_Imported_Functions
  * @{
  */
extern void BSP_GG_IO_Init(void);
extern int BSP_GG_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
extern int BSP_GG_IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead);

/**
  * @}
  */


/** @defgroup STC3115_Driver_I2C_ReadWrite_Interface   STC3115_Driver_I2C_ReadWrite_Interface
  * @{
  */

/** 
 * @brief Utility function to read the value stored in one register
 * @param  RegAddress STC3115 register to read
 * @param  value pointer to integer where to store data
 * @retval Error code: 0 ok, -1 error
 */
static int STC3115_ReadByte(int RegAddress, int* value)
{
  int ret;
  uint8_t tmp;
  
  ret = BSP_GG_IO_Read(&tmp, RegAddress, 1);
  *value = tmp & 0xff;
  return ret;
}


/**
 * @brief utility function to write a 8-bit value into a register
 * @param RegAddress STC3115 register
 * @param value value to write
 * @retval Error code: 0 ok, -1 error
 */
static int STC3115_WriteByte(int RegAddress, int value)
{
	uint8_t tmp = value & 0xff;
	return BSP_GG_IO_Write(&tmp, RegAddress, 1);
}


/**
 * @brief Utility function to read the value stored in one register pair
 * @param RegAddress STC3115 register
 * @param value pointer to integer value where to store data
 * @retval Error code: 0 ok, -1 error
 */
static int STC3115_ReadWord(int RegAddress, int* value)
{
  int ret;
  uint16_t tmp;
  
  ret =  BSP_GG_IO_Read((uint8_t*)&tmp, RegAddress, 2);
  *value = tmp & 0xffff;
  return ret;
}


/**
 * @brief Utility function to write a 16-bit value into a register pair
 * @param RegAddress STC3115 register
 * @param value value to write
 * @retval Error code: 0 ok, -1 error
 */
static int STC3115_WriteWord(int RegAddress, int value)
{
  uint16_t tmp = value & 0xffff;
  
  return BSP_GG_IO_Write((uint8_t*)&tmp, RegAddress, 2);
}


/**
  * @}
  */ 



/** @defgroup STC3115_Driver_private_interface  STC3115_Driver_private_interface
  * @{
  */


/**
 * @brief Read the STC3115 status
 * @retval status word (REG_MODE / REG_CTRL), -1 if error
 */
static int STC3115_Status(void)
{
  int value;

  /* First, check the presence of the STC3115 by reading first byte of dev. ID */
  STC3115_ReadByte(STC3115_REG_ID, &value);
  if (value != STC3115_ID)
          return -1;

  /* read REG_MODE and REG_CTRL */
  STC3115_ReadWord(STC3115_REG_MODE, &value);
  value &= 0x7fff;   

  return value;
}


/**
 * @brief Initialize the STC3115 parameters
 * @param ConfigData Pointer to STC3115 Configuration Data structure
 * @retval None
 */
static void STC3115_SetParam(STC3115_ConfigData_TypeDef *ConfigData)
{
	int value;

	/* set GG_RUN=0 before changing algo parameters */
	STC3115_WriteByte(STC3115_REG_MODE,STC3115_VMODE);

	/* init OCV curve */
	BSP_GG_IO_Write((unsigned char *) ConfigData->OCVOffset, STC3115_REG_OCVTAB, OCVTAB_SIZE);

	/* set alm level if different from default */
	if (ConfigData->Alm_SOC !=0 )   
		STC3115_WriteByte(STC3115_REG_ALARM_SOC, ConfigData->Alm_SOC*2); 
	if (ConfigData->Alm_Vbat !=0 ){
		value= ((long)(ConfigData->Alm_Vbat << 9) / VoltageFactor); /* LSB=8*2.2mV */
		STC3115_WriteByte(STC3115_REG_ALARM_VOLTAGE, value);
	}

	/* relaxation timer */
	if (ConfigData->Rsense !=0 ){
		value= ((long)(ConfigData->RelaxCurrent << 9) / (CurrentFactor / ConfigData->Rsense));   /* LSB=8*5.88uV */
		STC3115_WriteByte(STC3115_REG_CURRENT_THRES,value); 
	}

	/* set parameters if different from default, only if a restart is done (battery change) */
	if (RAMData.reg.CC_cnf !=0 ) STC3115_WriteWord(STC3115_REG_CC_CNF,RAMData.reg.CC_cnf); 
	if (RAMData.reg.VM_cnf !=0 ) STC3115_WriteWord(STC3115_REG_VM_CNF,RAMData.reg.VM_cnf); 

	STC3115_WriteByte(STC3115_REG_CTRL,0x03);  /*   clear PORDET, BATFAIL, free ALM pin, reset conv counter */
	STC3115_WriteByte(STC3115_REG_MODE, STC3115_GG_RUN | (STC3115_VMODE * ConfigData->Vmode) | (STC3115_ALM_ENA * ALM_EN));  /*   set GG_RUN=1, set mode, set alm enable */

	return;
}  


/**
 * @brief Initialize and start the STC3115 at application startup
 * @param :ConfigData Pointer to STC3115 Configuration Data structure 
 * @retval 0 if ok, -1 if error
 */
static int STC3115_Startup(STC3115_ConfigData_TypeDef *ConfigData)
{
	int res;
	int ocv;

	/* check STC310x status */
	res = STC3115_Status();
	if (res<0) return(res);

	/* read OCV */
	STC3115_ReadWord(STC3115_REG_OCV, &ocv);

	STC3115_SetParam(ConfigData);  /* set STC3115 parameters  */

	/* rewrite ocv to start SOC with updated OCV curve */
	STC3115_WriteWord(STC3115_REG_OCV, ocv);

	return 0;
}


/**
 * @brief Restore STC3115 state
* @param :ConfigData Pointer to STC3115 Configuration Data structure
 * @retval Always 0
 */
static int STC3115_Restore(STC3115_ConfigData_TypeDef *ConfigData)
{
	int res;

	/* check STC310x status */
	res = STC3115_Status();
	if (res<0) return(res);

	STC3115_SetParam(ConfigData);  /* set STC3115 parameters  */

	/* restore SOC from RAM data */
	STC3115_WriteWord(STC3115_REG_SOC, RAMData.reg.HRSOC);

	return(0);
}


/**
 * @brief Stop the STC3115 at application power down
 * @retval Ok if success, error code otherwise
 */
static int STC3115_Powerdown(void)
{
	int res;

	/* write 0x01 into the REG_CTRL to release IO0 pin open, */
	STC3115_WriteByte(STC3115_REG_CTRL, 0x01);

	/* write 0 into the REG_MODE register to put the STC3115 in standby mode */
	res = STC3115_WriteByte(STC3115_REG_MODE, 0);
	if (res!= OK) return (res);

	return (OK);
}


/**
 * @brief Convert a raw 16-bit value from STC3115 registers into user units (mA, mAh, mV, °C)
 * @param value 
 * @param factor 
 * @retval Converted value (result = value * factor / 4096)
 */
static int STC3115_conv(short value, unsigned short factor)
{
	int v;

	v = ( (long) value * factor ) >> 11;
	v = (v+1)/2;

	return (v);
}


/**
 * @brief Utility function to read the battery data from STC3115 to be called every 5s or so
* @param :BatteryData Pointer to STC3115_BatteryData_TypeDef structure
 * @retval error status (OK, !OK)
 */
static int STC3115_ReadBatteryData(STC3115_BatteryData_TypeDef *BatteryData)
{
	unsigned char data[16];
	int res;
	int value;


	/* read STC3115 registers 0 to 14 */
	res = BSP_GG_IO_Read(data, 0, 15);

	if (res<0) return(res);  /* read failed */

	/* fill the battery status data */
	/* SOC */
	value=data[3]; value = (value<<8) + data[2];
	BatteryData->HRSOC = value;     /* result in 1/512% */
	BatteryData->SOC = (value * 10 + 256) / 512; /* result in 0.1% */

	/* conversion counter */
	value=data[5]; value = (value<<8) + data[4];
	BatteryData->ConvCounter = value;

	/* current */
	value=data[7]; value = (value<<8) + data[6];
	value &= 0x3fff;   /* mask unused bits */
	if (value>=0x2000) value = value - 0x4000;  /* convert to signed value */
	BatteryData->Current = STC3115_conv(value, CurrentFactor/RSENSE);  /* result in mA */

	/* voltage */
	value=data[9]; value = (value<<8) + data[8];
	value &= 0x0fff; /* mask unused bits */
	if (value>=0x0800) value -= 0x1000;  /* convert to signed value */
	value = STC3115_conv(value,VoltageFactor);  /* result in mV */
	BatteryData->Voltage = value;  /* result in mV */

	/* temperature */
	value=data[10]; 
	if (value>=0x80) value -= 0x100;  /* convert to signed value */
	BatteryData->Temperature = value*10;  /* result in 0.1°C */

	/* OCV */
	value=data[14]; value = (value<<8) + data[13];
	value &= 0x3fff; /* mask unused bits */
	if (value>=0x02000) value -= 0x4000;  /* convert to signed value */
	value = STC3115_conv(value,VoltageFactor);  
	value = (value+2) / 4;  /* divide by 4 with rounding */
	BatteryData->OCV = value;  /* result in mV */

	return(OK);
}

/**
  * @}
  */


/** @defgroup STC3115_Driver_RAM_memory_functions       STC3115_Driver_RAM_memory_functions
  * @{
  */


/**
 * @brief utility function to read the RAM data from STC3115
 * @param RamData Pointer to store STC3115 RAM data array to
 * @retval I2C error code
 */
static int STC3115_ReadRamData(unsigned char *RamData)
{
	return BSP_GG_IO_Read(RamData, STC3115_REG_RAM, RAM_SIZE);
}


/**
 * @brief Utility function to write the RAM data into STC3115
 * @param RamData Pointer to read STC3115 RAM data array from
 * @retval I2C error code
 */
static int STC3115_WriteRamData(unsigned char *RamData)
{
	return BSP_GG_IO_Write(RamData, STC3115_REG_RAM, RAM_SIZE);
}


/**
 * @brief Calculate the CRC8
 * @param *data : pointer to byte array, 
 * @param n     : number of bytes
 * @retval CRC value
 */
static int STC3115_CalcRamCRC8(unsigned char *data, int n)
{
	int crc=0;   /* initial value */
	int i, j;

	for (i=0;i<n;i++){
		crc ^= data[i];
		for (j=0;j<8;j++){
			crc <<= 1;
			if (crc & 0x100)  crc ^= 7;
		}
	}
	return(crc & 255);
}

 
/**
 * @brief calculate the RAM CRC
 * @retval CRC value
 */
static int STC3115_UpdateRamCRC(void)
{
	int res;

	res = STC3115_CalcRamCRC8(RAMData.db,RAM_SIZE-1);
	RAMData.db[RAM_SIZE-1] = res;   /* last byte holds the CRC */
	return(res);
}


/**
 * @brief Initialize the STC3115 RAM registers with valid test word and CRC
 * @param ConfigData Pointer to STC3115_ConfigData_TypeDef structure
 * @retval None
 */
static void STC3115_InitRamData(STC3115_ConfigData_TypeDef *ConfigData)
{
  int index;
  //Set full RAM tab to 0
  for (index=0;index<RAM_SIZE;index++) 
          RAMData.db[index]=0;
  //Fill RAM regs
  RAMData.reg.TstWord=RAM_TSTWORD;  /* Fixed word to check RAM integrity */
  RAMData.reg.CC_cnf = ConfigData->CC_cnf;
  RAMData.reg.VM_cnf = ConfigData->VM_cnf;
  /* update the crc */
  STC3115_UpdateRamCRC();
}

/**
  * @}
  */


/** @defgroup STC3115_Driver_interface_functions        STC3115_Driver_interface_functions
 * @{
 */


/**
 * @brief Start the STC3115 battery tracking algorithm system
 * @param ConfigData Pointer to STC3115_ConfigData_TypeDef structure
 * @param BatteryData Pointer to STC3115_BatteryData_TypeDef structure
 * @retval 0 is ok, -1 if STC310x not found or I2C error
 * @note Global STC310x data and gas gauge variables are affected
 */
int GasGauge_Initialization(STC3115_ConfigData_TypeDef *ConfigData, STC3115_BatteryData_TypeDef *BatteryData)
{
	int res,loop;
	int OCVOffset[16] = OCV_OFFSET_TAB;

	/*** Fill configuration structure parameters ***/

	ConfigData->Vmode = VMODE;

	if(RSENSE!=0)	ConfigData->Rsense = RSENSE;
	else ConfigData->Rsense = 10; // default value to avoid division by 0

	ConfigData->CC_cnf = (CAPACITY * ConfigData->Rsense * 250 + 6194) / 12389;

	if(RINT!=0)	ConfigData->VM_cnf = (CAPACITY * RINT * 50 + 24444) / 48889;
	else ConfigData->VM_cnf = (CAPACITY * 200 * 50 + 24444) / 48889; // default value

	for(loop=0;loop<16;loop++)
	{
          if(OCVOffset[loop] > 127) OCVOffset[loop] = 127;
          if(OCVOffset[loop] < -127) OCVOffset[loop] = -127;
          ConfigData->OCVOffset[loop] = OCVOffset[loop];
	}

	ConfigData->Cnom = CAPACITY; 
	ConfigData->RelaxCurrent = CAPACITY / 20;

	ConfigData->Alm_SOC = ALM_SOC;
	ConfigData->Alm_Vbat = ALM_VBAT;
	/* Initialize I2C. Note that if I2C is already initialized this has not effect*/
	BSP_GG_IO_Init();
	/*** Initialize Gas Gauge system ***/
	/*Battery presence status init*/
	BatteryData->Presence = 1;

        /* Clear TstWord before read */
	RAMData.reg.TstWord = 0;
        
        /* check RAM data validity */
	STC3115_ReadRamData(RAMData.db);
 
	if ( (RAMData.reg.TstWord != RAM_TSTWORD) || (STC3115_CalcRamCRC8(RAMData.db,RAM_SIZE)!=0) )
          {
            /* RAM invalid */
            STC3115_InitRamData(ConfigData);
            res=STC3115_Startup(ConfigData);  /* return -1 if I2C error or STC3115 not present */
          }
	else
          {
            /* check STC3115 status */
            STC3115_ReadByte(STC3115_REG_CTRL, &res);
          if ( (res & (STC3115_BATFAIL | STC3115_PORDET)) != 0 )
            {
              res=STC3115_Startup(ConfigData);  /* return -1 if I2C error or STC3115 not present */
            }
		
          else
            {
              res=STC3115_Restore(ConfigData); /* recover from last SOC */
            }
          }

	//Update RAM status
	RAMData.reg.STC3115_Status = STC3115_INIT;
	STC3115_UpdateRamCRC();
	STC3115_WriteRamData(RAMData.db);

	return(res);    /* return -1 if I2C error or STC3115 not present */
}


/**
 * @brief Reset the Gas Gauge system
 * @retval 0 is ok, -1 if I2C error
 */
int GasGauge_Reset(void) 
{
	int res;
	
	/* reset RAM */
	RAMData.reg.TstWord=0;  
	RAMData.reg.STC3115_Status = 0;
	res = STC3115_WriteRamData(RAMData.db);
	if(res != OK) return res;
	/* reset STC3115*/
	res = STC3115_WriteByte(STC3115_REG_CTRL, STC3115_PORDET);  /*   set soft POR */

	return res;
}


/**
 * @brief Put the Gas Gauge in Power saving mode
 * @retval 0 is ok, -1 if I2C error
 */
int GasGauge_PowerSavingMode(void)
{
  return STC3115_SetPowerSavingMode();
}

/**
 * @brief Stop the Gas Gauge system
 * @retval 0 is ok, -1 if I2C error
 */
int GasGauge_Stop(void)
{
	int res;

	/*Save context in RAM*/
	STC3115_ReadRamData(RAMData.db);
	RAMData.reg.STC3115_Status= STC3115_POWERDN;
	/* update the crc */
	STC3115_UpdateRamCRC();
	STC3115_WriteRamData(RAMData.db);
	 
	/*STC3115 Power down*/
	res=STC3115_Powerdown();
	if (res!=0) return (-1);  /* error */

	return(0);  
}


/**
 * @brief Periodic Gas Gauge task, to be called e.g. every 5 sec.
 * @param ConfigData Pointer to STC3115_ConfigData_TypeDef structure
 * @param BatteryData Pointer to STC3115_BatteryData_TypeDef structure
 * @retval 1 if data available, 0 si no data, -1 if error
 * @note Affects global STC310x data and gas gauge variables
 */
int GasGauge_Task(STC3115_ConfigData_TypeDef *ConfigData,STC3115_BatteryData_TypeDef *BatteryData)
{
	int res;

	/* ----------------------------- System state verification ---------------------------- */
	/*Read STC3115 status registers */
	res=STC3115_Status();

	if (res<0) return(res);         /* return if I2C error or STC3115 not responding */  
	BatteryData->status = res;

	/* check STC3115 RAM status (battery has not been changed) */
	STC3115_ReadRamData(RAMData.db);
	if ( (RAMData.reg.TstWord!= RAM_TSTWORD) || (STC3115_CalcRamCRC8(RAMData.db,RAM_SIZE)!=0) )
        {
          /* if RAM non ok, reset it and set init state */
          STC3115_InitRamData(ConfigData); 
          RAMData.reg.STC3115_Status = STC3115_INIT;
	}

	/* check battery presence status */
	if ((BatteryData->status & (STC3115_BATFAIL<<8)) != 0)
        {
          /*Battery disconnection has been detected			*/
                  
          /*BATD pin level is over 1.61 or Vcc is below 2.7V	*/
          BatteryData->Presence = 0;

          /*HW and SW state machine reset*/
          GasGauge_Reset();

          return (-1);
	}

	/* check STC3115 running mode*/
	if ((BatteryData->status & STC3115_GG_RUN) == 0)
        {
            if((RAMData.reg.STC3115_Status == STC3115_RUNNING) || (RAMData.reg.STC3115_Status == STC3115_POWERDN))
                    STC3115_Restore(ConfigData);  /* if RUNNING state, restore STC3115*/
            else
            {
                    STC3115_Startup(ConfigData);  /* if INIT state, initialize STC3115*/
            }
            
            RAMData.reg.STC3115_Status = STC3115_INIT;
	}

	/* --------------------------------- Read battery data ------------------------------- */

	res = STC3115_ReadBatteryData(BatteryData);  
	if (res!=0) return(-1); /* abort in case of I2C failure */


	/* ------------------------------- battery data report ------------------------------- */
	/* check INIT state */
	if (RAMData.reg.STC3115_Status == STC3115_INIT)
        {
          /* INIT state, wait for current & temperature value available: */
          if (BatteryData->ConvCounter>VCOUNT)
          {
              RAMData.reg.STC3115_Status = STC3115_RUNNING;
              /*Battery is connected*/
              BatteryData->Presence = 1;
          }
	}

	if (RAMData.reg.STC3115_Status != STC3115_RUNNING) { /* not running : data partially availalble*/
		BatteryData->ChargeValue = ConfigData->Cnom * BatteryData->SOC / MAX_SOC;
		BatteryData->Current=0;
		BatteryData->Temperature=250;
		BatteryData->RemTime = -1;
	}
	else /*STC3115 running */
	{
  
		/* ---------- process SW algorithms -------- */
			
		/*early empty compensation*/
		if (BatteryData->Voltage<APP_CUTOFF_VOLTAGE)
			BatteryData->SOC = 0;
		else if (BatteryData->Voltage<(APP_CUTOFF_VOLTAGE+200))
			BatteryData->SOC = BatteryData->SOC * (BatteryData->Voltage - APP_CUTOFF_VOLTAGE) / 200;   

		/* Battery charge value calculation */

		BatteryData->ChargeValue = ConfigData->Cnom * BatteryData->SOC / MAX_SOC;

		if ((BatteryData->status & STC3115_VMODE) == 0) /* mixed mode only*/
		{  
			
			/*Lately fully compensation*/
			if ((BatteryData->status & STC3115_VMODE) == 0) /*running in mixed mode*/
			{ 
				
				if(BatteryData->Current > APP_EOC_CURRENT && BatteryData->SOC > 990)
				{
					BatteryData->SOC = 990;
					STC3115_WriteWord(STC3115_REG_SOC,50688);   /* 99% */
				}
			}
			
			/*Remaining time calculation*/
			if(BatteryData->Current < 0)
			{

				BatteryData->RemTime = (BatteryData->RemTime * 4 + BatteryData->ChargeValue / BatteryData->Current * 60 ) / 5;
				if( BatteryData->RemTime  < 0)
					BatteryData->RemTime = -1; /* means no estimated time available */
			}
			else
				BatteryData->RemTime = -1; /* means no estimated time available */
			
		}
		else /* voltage mode only */
		{
			BatteryData->Current=0;
			BatteryData->RemTime = -1;
		}
		
		//SOC min/max clamping
		if(BatteryData->SOC>1000) BatteryData->SOC = MAX_SOC;
		if(BatteryData->SOC<0) BatteryData->SOC = 0;

	}
	/* save SOC */
	RAMData.reg.HRSOC = BatteryData->HRSOC;
	RAMData.reg.SOC = (BatteryData->SOC+5)/10;
	STC3115_UpdateRamCRC();
	STC3115_WriteRamData(RAMData.db);

	if (RAMData.reg.STC3115_Status==STC3115_RUNNING)
		return(1);
	else
		return(0);  /* only SOC, OCV and voltage are valid */
}


/**
 * @brief Set the power saving mode
 * @retval error status (OK, !OK)
 */
int STC3115_SetPowerSavingMode(void)
{
	int res;

	/* Read the mode register*/
	STC3115_ReadByte(STC3115_REG_MODE, &res);

	/* Set the VMODE bit to 1 */
	STC3115_WriteByte(STC3115_REG_MODE, (res | STC3115_VMODE));

	return (OK);
}


/**
 * @brief Stop the power saving mode
 * @retval Error status (OK, !OK)
 */
int STC3115_StopPowerSavingMode(void)
{
	int res;

	/* Read the mode register*/
	STC3115_ReadByte(STC3115_REG_MODE, &res);
	/*STC3115 is in power saving mode by default, cannot be set dynamically in mixed mode.		*/
	/*Change stc3115_Driver.h VMODE parameter, and connect an external sense resistor to STC3115	*/
	if (VMODE != MIXED_MODE) 
		return (!OK); 

	/* Set the VMODE bit to 0 */
	STC3115_WriteByte(STC3115_REG_MODE, (res & ~STC3115_VMODE));

	return (OK);
}


/**
 * @brief Enable alarm functionality
 * @retval error status (OK, !OK)
 */
int STC3115_AlarmSet(void)
{
	int res;

	/* Read the mode register*/
	STC3115_ReadByte(STC3115_REG_MODE, &res);

	/* Set the ALM_ENA bit to 1 */
	STC3115_WriteByte(STC3115_REG_MODE, (res | STC3115_ALM_ENA));
	
	return (OK);
}


/**
 * @brief Stop alarm functionality
 * @retval error status (OK, !OK)
 */
int STC3115_AlarmStop(void)
{
	int res;

	/* Read the mode register*/
	STC3115_ReadByte(STC3115_REG_MODE, &res);

	/* Set the ALM_ENA bit to 0 */
	res = STC3115_WriteByte(STC3115_REG_MODE, (res & ~STC3115_ALM_ENA));

	return (OK);
}


/**
 * @brief Return the ALM status
 * @retval ALM status 00 : no alarm
 *                    01 : SOC alarm
 *                    10 : Voltage alarm
 *                    11 : SOC and voltage alarm
 */
int STC3115_AlarmGet(void)
{
	int res;

	/* Read the mode register*/
	res = STC3115_ReadByte(STC3115_REG_CTRL, &res);
	res = res >> 5;

	return (res & 0x3);
}


/**
 * @brief Clear the alarm signal
 * @retval error status (OK, !OK)
 */
int STC3115_AlarmClear(void)
{
	int res;

	/* clear ALM bits*/
	res = STC3115_WriteByte(STC3115_REG_CTRL, 0x01);
	if (res!= OK) return (res);

	return (res);
}


/**
 * @brief Set the voltage alarm threshold
 * @param ConfigData Pointer to STC3115_ConfigData_TypeDef structure
 * @param VoltThresh Voltage alarm threshold
 * @retval error status (OK, !OK)
 */
int STC3115_AlarmSetVoltageThreshold(STC3115_ConfigData_TypeDef *ConfigData, int VoltThresh)
{
	int res;
	int value;

	ConfigData->Alm_Vbat = VoltThresh;
		
	value= ((long)(ConfigData->Alm_Vbat << 9) / VoltageFactor); /* LSB=8*2.2mV */
	res = STC3115_WriteByte(STC3115_REG_ALARM_VOLTAGE, value);
	if (res!= OK) return (res);

	return (OK);
}


/**
 * @brief Set the State of Charge alarm threshold
 * @param ConfigData Pointer to STC3115_ConfigData_TypeDef structure
 * @param SOCThresh SoC alarm threshold
 * @retval error status (OK, !OK)
 */
int STC3115_AlarmSetSOCThreshold(STC3115_ConfigData_TypeDef *ConfigData, int SOCThresh)
{
	int res;

	ConfigData->Alm_SOC = SOCThresh;
	res = STC3115_WriteByte(STC3115_REG_ALARM_SOC, ConfigData->Alm_SOC*2);
	if (res!= OK) return (res);

	return (OK);
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

/******************* (C) COPYRIGHT 2014 STMicroelectronics *****END OF FILE****/
