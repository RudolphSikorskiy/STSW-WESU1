
#include "hal.h"
#include "hal_types.h"
#include "ble_status.h"
#include "bluenrg_updater_aci.h"
#include "bluenrg_utils.h"
//#include "SDK_EVAL_Spi_Driver.h"

/************** Do not change this define section ************/

#define BLUENRG_32_MHZ          1
#define BLUENRG_32_MHZ_RO       2
#define BLUENRG_16_MHZ          3
#define BLUENRG_16_MHZ_RO       4
#define BLUENRG_CUSTOM_CONFIG   5

#define MASTER_SCA_500ppm       0 // 251 ppm to 500 ppm
#define MASTER_SCA_250ppm       1 // 151 ppm to 250 ppm
#define MASTER_SCA_150ppm       2 // 101 ppm to 150 ppm
#define MASTER_SCA_100ppm       3 // 76 ppm to 100 ppm
#define MASTER_SCA_75ppm        4 // 51 ppm to 75 ppm
#define MASTER_SCA_50ppm        5 // 31 ppm to 50 ppm
#define MASTER_SCA_30ppm        6 // 21 ppm to 30 ppm
#define MASTER_SCA_20ppm        7 // 0 ppm to 20 ppm

#define SMPS_4MHz               0
#define SMPS_8MHz               1

#ifndef SMPS_FREQUENCY
#define SMPS_FREQUENCY          SMPS_4MHz
#endif

#if !BLUENRG_MS && (SMPS_FREQUENCY == SMPS_8MHz)
#error Unsupported SMPS_FREQUENCY
#endif

/************************************************************/


/************** Definitions that can be changed. ************/

#define STACK_MODE              2
#define SLAVE_SCA_PPM           100
#define MASTER_SCA              MASTER_SCA_100ppm
#define HS_STARTUP_TIME_US      642
#define DAY                     06
#define MONTH                   07
#define YEAR                    15
/************************************************************/

/*
 * IMPORTANT!
 * This IFR configurations are only for BlueNRG Firmware v6.4 and 7.1.
 */

#if BLUENRG_CONFIG == BLUENRG_32_MHZ

const IFR_config_TypeDef IFR_config = {
#if BLUENRG_MS
#if SMPS_FREQUENCY == SMPS_4MHz
  0x02,0x3A,0x44,0x02,
  0x34,0x5B,0x02,0x39,
  0xA2,0x02,0x3C,0x20,
  0x00,0xFF,0xFF,0xFF,
#elif SMPS_FREQUENCY == SMPS_8MHz
  0x02,0x3A,0x44,0x02,
  0x34,0x5B,0x02,0x39,
  0xAE,0x00,0xFF,0xFF,
  0x00,0xFF,0xFF,0xFF,
#else
#error Incorrect SMPS_FREQUENCY
#endif /* SMPS_FREQUENCY */
#else
  0x02,0x3A,0x5C,0x02,
  0x39,0xA2,0x02,0x34,
  0x5B,0x00,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
#endif /* BLUENRG_MS */
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  
  0x02,0x1C,0x43,0x02,
  0x20,0xEC,0x02,0x1F,
  0xAF,0x00,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  
  STACK_MODE,
  0xFF,0xFF,0xFF,
  0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,
  0xFFFFFFFF,
  htobl(0x00190000),
  htobl(0x0028F5C2),
  htobs(SLAVE_SCA_PPM),
  MASTER_SCA,
  0xFF,
  htobs(FROM_US_TO_SYS_TIME(HS_STARTUP_TIME_US)),
  0xFF,0xFF,
  0xFFFFFFFF,
  0xFF,   
  INT_TO_BCD(YEAR),INT_TO_BCD(MONTH),INT_TO_BCD(DAY),
  0xFFFFFFFF,  
  0xFFFFFFFF,
  0xFFFFFFFF,
  0xFFFFFFFF,
  0xFFFFFFFF
};

#elif BLUENRG_CONFIG == BLUENRG_32_MHZ_RO

const IFR_config_TypeDef IFR_config = {
#if BLUENRG_MS
#if SMPS_FREQUENCY == SMPS_4MHz
  0x02,0x3A,0x44,0x02,
  0x34,0x1B,0x02,0x39,
  0xA2,0x02,0x3C,0x20,
  0x00,0xFF,0xFF,0xFF,
#elif SMPS_FREQUENCY == SMPS_8MHz
  0x02,0x3A,0x44,0x02,
  0x34,0x1B,0x02,0x39,
  0xAE,0x00,0xFF,0xFF,
  0x00,0xFF,0xFF,0xFF,
#else
#error Incorrect SMPS_FREQUENCY
#endif /* SMPS_FREQUENCY */  
#else
  0x02,0x3A,0x5C,0x02,
  0x39,0xA2,0x02,0x34,
  0x1B,0x00,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
#endif /* BLUENRG_MS */
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  
  0x02,0x1C,0x43,0x02,
  0x20,0xEC,0x02,0x1F,
  0xAF,0x00,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  
  STACK_MODE,
  0xFF,0xFF,0xFF,
  0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,
  0xFFFFFFFF,
  0xFFFFFFFF,
  0xFFFFFFFF,
  htobs(0x01F4), 
  0x00,
  0xFF,
  htobs(FROM_US_TO_SYS_TIME(HS_STARTUP_TIME_US)),
  0xFF,0xFF,
  0xFFFFFFFF,
  0xFF,   
  INT_TO_BCD(YEAR),INT_TO_BCD(MONTH),INT_TO_BCD(DAY),
  0xFFFFFFFF,  
  0xFFFFFFFF,
  0xFFFFFFFF,
  0xFFFFFFFF,
  0xFFFFFFFF  
};

#elif BLUENRG_CONFIG == BLUENRG_16_MHZ

const IFR_config_TypeDef IFR_config = {
#if BLUENRG_MS
#if SMPS_FREQUENCY == SMPS_4MHz
  0x02,0x3A,0x40,0x02,
  0x34,0x5B,0x02,0x39,
  0xA2,0x02,0x3C,0x20,
  0x00,0xFF,0xFF,0xFF,
#elif SMPS_FREQUENCY == SMPS_8MHz
  0x02,0x3A,0x40,0x02,
  0x34,0x5B,0x02,0x39,
  0xAE,0x00,0xFF,0xFF,
  0x00,0xFF,0xFF,0xFF,
#else
#error Incorrect SMPS_FREQUENCY
#endif /* SMPS_FREQUENCY */    
#else
  0x02,0x3A,0x58,0x02,
  0x39,0xA2,0x02,0x34,
  0x5B,0x00,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
#endif /* BLUENRG_MS */
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  
  0x02,0x1C,0x43,0x02,
  0x20,0xEC,0x02,0x1F,
  0xAF,0x00,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  
  STACK_MODE,
  0xFF,0xFF,0xFF,
  0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,
  0xFFFFFFFF,
  htobl(0x00190000),
  htobl(0x0028F5C2),
  htobs(SLAVE_SCA_PPM), 
  MASTER_SCA,
  0xFF,
  htobs(FROM_US_TO_SYS_TIME(HS_STARTUP_TIME_US)),
  0xFF,0xFF,
  0xFFFFFFFF,
  0xFF,   
  INT_TO_BCD(YEAR),INT_TO_BCD(MONTH),INT_TO_BCD(DAY),
  0xFFFFFFFF,  
  0xFFFFFFFF,
  0xFFFFFFFF,
  0xFFFFFFFF,
  0xFFFFFFFF

};

#elif BLUENRG_CONFIG == BLUENRG_16_MHZ_RO

const IFR_config_TypeDef IFR_config = {
#if BLUENRG_MS
#if SMPS_FREQUENCY == SMPS_4MHz
  0x02,0x3A,0x40,0x02,
  0x34,0x1B,0x02,0x39,
  0xA2,0x02,0x3C,0x20,
  0x00,0xFF,0xFF,0xFF,
#elif SMPS_FREQUENCY == SMPS_8MHz
  0x02,0x3A,0x40,0x02,
  0x34,0x1B,0x02,0x39,
  0xAE,0x00,0xFF,0xFF,
  0x00,0xFF,0xFF,0xFF,
#else
#error Incorrect SMPS_FREQUENCY
#endif /* SMPS_FREQUENCY */    
#else
  0x02,0x3A,0x58,0x02,
  0x39,0xA2,0x02,0x34,
  0x1B,0x00,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
#endif /* BLUENRG_MS */
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  
  0x02,0x1C,0x43,0x02,
  0x20,0xEC,0x02,0x1F,
  0xAF,0x00,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  
  STACK_MODE,
  0xFF,0xFF,0xFF,
  0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,
  0xFFFFFFFF,
  0xFFFFFFFF,
  0xFFFFFFFF,
  htobs(0x01F4), 
  0x00,
  0xFF,
  htobs(FROM_US_TO_SYS_TIME(HS_STARTUP_TIME_US)),
  0xFF,0xFF,
  0xFFFFFFFF,
  0xFF,   
  INT_TO_BCD(YEAR),INT_TO_BCD(MONTH),INT_TO_BCD(DAY),
  0xFFFFFFFF,  
  0xFFFFFFFF,
  0xFFFFFFFF,
  0xFFFFFFFF,
  0xFFFFFFFF  
};

#elif BLUENRG_CONFIG == BLUENRG_CUSTOM_CONFIG
/* Copy and paste here your custom IFR_config structure. It can be generated
 * with BlueNRG GUI.
 */
#else
#warning BLUENRG_CONFIG not valid
#endif

