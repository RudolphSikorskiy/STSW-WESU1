/**
  ******************************************************************************
  * @file    stm32_bluenrg_ble.c
  * @author  System Lab
  * @version V1.1.0
  * @date    25-November-2016
  * @brief   BlueNRG hardware management functions 
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
#include "stm32_bluenrg_ble.h"
#include "gp_timer.h" 

#ifdef WESU_DEMO
  #include "BlueST_Protocol.h"
#else //SENSOR_READ EXAMPLE
  #include "BlueST_Protocol_examples.h"
#endif //WESU_DEMO


#ifdef PRINT_CSV_FORMAT
extern volatile uint32_t ms_counter;
#endif /* PRINT_CSV_FORMAT */

/** @addtogroup BSP             BSP
 *  @{
 */

/** @defgroup STEVAL-WESU1      STEVAL-WESU1
 *  @{
 */

/** @defgroup BNRG              BNRG
 *  @{
 */
 
/** @defgroup STM32_BLUENRG_BLE STM32_BLUENRG_BLE
 *  @{
 */

/** @defgroup STM32_BLUENRG_BLE_Private_Defines         STM32_BLUENRG_BLE_Private_Defines
 * @{
 */ 

#define HEADER_SIZE 5           //!< Amount of data Buffer Sent used by BLUNRG SPI Master or Slave */
#define MAX_BUFFER_SIZE 255     //!< MAX Amount of data Buffer Size used by BLUNRG SPI */
#define TIMEOUT_DURATION 15     //!< Max Timeout for BLUNRG SPI */

/**
 * @}
 */

/** @defgroup STM32_BLUENRG_BLE_Private_Variables       STM32_BLUENRG_BLE_Private_Variables
 * @{
 */

SPI_HandleTypeDef BNRGSpiHandle;        //!< BLUNRG SPI Handler

/**
 * @}
 */

/** @defgroup STM32_BLUENRG_BLE_Private_Function_Prototypes     STM32_BLUENRG_BLE_Private_Function_Prototypes
 *  @{
 */

static void us150Delay(void);                  //!< Utility function for delay

/**
 * @}
 */ 
   
/** @defgroup STM32_BLUENRG_BLE_Private_Function     STM32_BLUENRG_BLE Private Function
 *  @{
 */

/**
 * @brief  Utility function for delay
 * @retval None
 */
static void us150Delay(void)
{
  volatile int i;
#if SYSCLK_FREQ == 4000000
  for(i = 0; i < 35; i++)__NOP();
#elif SYSCLK_FREQ == 32000000
  for(i = 0; i < 420; i++)__NOP();
#elif SYSCLK_FREQ == 84000000
  for(i = 0; i < 1125; i++)__NOP();
#else
#error Implement delay function.
#endif    
}
   
/**
 * @}
 */ 


/** @defgroup STM32_BLUENRG_BLE_Exported_Functions      STM32_BLUENRG_BLE_Exported_Functions
 * @{
 */ 

#ifdef PRINT_CSV_FORMAT
/**
 * @brief  This function is a utility to print the log time
 *          in the format HH:MM:SS:MSS (DK GUI time format)
 * @retval None
 */
void print_csv_time(void){
  uint32_t ms = ms_counter;
  PRINT_CSV("%02d:%02d:%02d.%03d", ms/(60*60*1000)%24, ms/(60*1000)%60, (ms/1000)%60, ms%1000);
}
#endif /* PRINT_CSV_FORMAT */


/**
 * @brief  Writes data to a serial interface.
 * @param  data1   :  1st buffer
 * @param  data2   :  2nd buffer
 * @param  n_bytes1: number of bytes in 1st buffer
 * @param  n_bytes2: number of bytes in 2nd buffer
 * @retval None
 */
void Hal_Write_Serial(const void* data1, const void* data2, uint16_t n_bytes1, 
                      uint16_t n_bytes2)
{
  struct timer t;

  Timer_Set(&t, CLOCK_SECOND/10);

#ifdef PRINT_CSV_FORMAT
  print_csv_time();
  for (int i=0; i<n_bytes1; i++) {
    PRINT_CSV(" %02x", ((uint8_t *)data1)[i]);
	 }
  for (int i=0; i<n_bytes2; i++) {
    PRINT_CSV(" %02x", ((uint8_t *)data2)[i]);
	 }
  PRINT_CSV("\n");
#endif

  while(1){
    if(BlueNRG_SPI_Write((uint8_t *)data1,(uint8_t *)data2, n_bytes1, n_bytes2)==0) break;
    if(Timer_Expired(&t)){
      break;
    }
  }
}

/**
 * @brief  Initializes the SPI communication with the BlueNRG
 *         Expansion Board.
 * @retval None
 */
void BNRG_SPI_Init(void)
{
  BNRGSpiHandle.Instance = BNRG_SPI_INSTANCE;
  BNRGSpiHandle.Init.Mode = BNRG_SPI_MODE;
  BNRGSpiHandle.Init.Direction = BNRG_SPI_DIRECTION;
  BNRGSpiHandle.Init.DataSize = BNRG_SPI_DATASIZE;
  BNRGSpiHandle.Init.CLKPolarity = BNRG_SPI_CLKPOLARITY;
  BNRGSpiHandle.Init.CLKPhase = BNRG_SPI_CLKPHASE;
  BNRGSpiHandle.Init.NSS = BNRG_SPI_NSS;
  BNRGSpiHandle.Init.FirstBit = BNRG_SPI_FIRSTBIT;
  BNRGSpiHandle.Init.TIMode = BNRG_SPI_TIMODE;
  BNRGSpiHandle.Init.CRCPolynomial = BNRG_SPI_CRCPOLYNOMIAL;
  BNRGSpiHandle.Init.BaudRatePrescaler = BNRG_SPI_BAUDRATEPRESCALER;
  BNRGSpiHandle.Init.CRCCalculation = BNRG_SPI_CRCCALCULATION;
  
  HAL_SPI_Init(&BNRGSpiHandle);
}

/**
 * @brief  Resets the BlueNRG.
 */
void BlueNRG_RST(void)
{
  HAL_GPIO_WritePin(BNRG_RESET_PORT, BNRG_RESET_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(BNRG_RESET_PORT, BNRG_RESET_PIN, GPIO_PIN_SET);
  HAL_Delay(5);
  
#if USE_TIMER
  HCI_Isr();
#endif  
}

/**
 * @brief  Keep the BlueNRG under reset.
 * @retval None
 */
void BlueNRG_RST_AndKeep(void)
{
  HAL_GPIO_WritePin(BNRG_RESET_PORT, BNRG_RESET_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
}

/**
 * @brief  Reports if the BlueNRG has data for the host micro.
 * @retval uint8_t: 1 if data are present, 0 otherwise
 */
uint8_t BlueNRG_DataPresent(void)
{
  if (HAL_GPIO_ReadPin(BNRG_EXTI_PORT, BNRG_EXTI_PIN) == GPIO_PIN_SET)
      return 1;
  else  
      return 0;
}

/**
 * @brief  Activate internal bootloader using pin.
 * @retval void
 */
void BlueNRG_HW_Bootloader(void)
{
  set_irq_as_output();
  BlueNRG_RST();
  set_irq_as_input();
}

extern SPI_HandleTypeDef BNRGSpiHandle;

/**
 * @brief  Reads from BlueNRG SPI buffer and store data into local buffer.
 * @param  buffer   : Buffer where data from SPI are stored
 * @param  buff_size: Buffer size
 * @retval int32_t  : Number of read bytes
 */
int32_t BlueNRG_SPI_Read_All(uint8_t *buffer,
                             uint8_t buff_size)
{
  uint16_t byte_count;
  uint8_t len = 0;
  uint8_t char_ff = 0xff;
  volatile uint8_t read_char;

  uint8_t header_master[HEADER_SIZE] = {0x0b, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[HEADER_SIZE];

  /* CS reset */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_RESET);

  /* Read the header */  
  HAL_SPI_TransmitReceive(&BNRGSpiHandle, header_master, header_slave, HEADER_SIZE, TIMEOUT_DURATION);
  
  if (header_slave[0] == 0x02) {
    /* device is ready */
    byte_count = (header_slave[4]<<8)|header_slave[3];
  
    if (byte_count > 0) {
  
      /* avoid to read more data that size of the buffer */
      if (byte_count > buff_size){
        byte_count = buff_size;
      }
  
      for (len = 0; len < byte_count; len++){
        HAL_SPI_TransmitReceive(&BNRGSpiHandle, &char_ff, (uint8_t*)&read_char, 1, TIMEOUT_DURATION);
        buffer[len] = read_char;
      }
    }    
  }
  /* Release CS line */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);
  
  // Add a small delay to give time to the BlueNRG to set the IRQ pin low
  // to avoid a useless SPI read at the end of the transaction
  {
	volatile int i;
    for(i = 0; i < 2; i++)__NOP();
  }
  
#ifdef PRINT_CSV_FORMAT
  if (len > 0) {
    print_csv_time();
    for (int i=0; i<len; i++) {
      PRINT_CSV(" %02x", buffer[i]);
    }
    PRINT_CSV("\n");
  }
#endif
  
  return len;   
}

/**
 * @brief  Writes data from local buffer to SPI.
 * @param  data1    : First data buffer to be written
 * @param  data2    : Second data buffer to be written
 * @param  Nb_bytes1: Size of first data buffer to be written
 * @param  Nb_bytes2: Size of second data buffer to be written
 * @retval Number of read bytes
 */
int32_t BlueNRG_SPI_Write(uint8_t* data1,
                          uint8_t* data2, uint8_t Nb_bytes1, uint8_t Nb_bytes2)
{  
  int32_t result = 0;
  
  int32_t spi_fix_enabled = 0;
  
#ifdef ENABLE_SPI_FIX
  spi_fix_enabled = 1;
#endif //ENABLE_SPI_FIX
  
  unsigned char header_master[HEADER_SIZE] = {0x0a, 0x00, 0x00, 0x00, 0x00};
  unsigned char header_slave[HEADER_SIZE]  = {0xaa, 0x00, 0x00, 0x00, 0x00};
  
  unsigned char read_char_buf[MAX_BUFFER_SIZE];

  Disable_SPI_IRQ(); 

  /*
   If the SPI_FIX is enabled the IRQ is set in Output mode, then it is pulled
   high and, after a delay of at least 112us, the CS line is asserted and the
   header transmit/receive operations are started.
   After these transmit/receive operations the IRQ is reset in input mode.
 */
  if (spi_fix_enabled) {
    set_irq_as_output();

    /* Assert CS line after at least 112us */
    us150Delay();
}

  /* CS reset */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_RESET);

  /* Exchange header */  
  HAL_SPI_TransmitReceive(&BNRGSpiHandle, header_master, header_slave, HEADER_SIZE, TIMEOUT_DURATION);
  
  if (spi_fix_enabled) {
    set_irq_as_input();
}

  if (header_slave[0] == 0x02) {
    /* SPI is ready */
    if (header_slave[1] >= (Nb_bytes1+Nb_bytes2)) {
  
      /*  Buffer is big enough */
      if (Nb_bytes1 > 0) {
        HAL_SPI_TransmitReceive(&BNRGSpiHandle, data1, read_char_buf, Nb_bytes1, TIMEOUT_DURATION);
      }
      if (Nb_bytes2 > 0) {
        HAL_SPI_TransmitReceive(&BNRGSpiHandle, data2, read_char_buf, Nb_bytes2, TIMEOUT_DURATION);
}

    } else {
      /* Buffer is too small */
      result = -2;
      }
  } else {
    /* SPI is not ready */
    result = -1;
      }
    
    /* Release CS line */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);
    
  Enable_SPI_IRQ();
    
  return result;
}
      
/**
 * @brief  Set in Output mode the IRQ.
 * @retval None
 */
void set_irq_as_output(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Pull IRQ high */
  GPIO_InitStructure.Pin = BNRG_IRQ_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = BNRG_IRQ_SPEED;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BNRG_IRQ_PORT, &GPIO_InitStructure);
  HAL_GPIO_WritePin(BNRG_IRQ_PORT, BNRG_IRQ_PIN, GPIO_PIN_SET);
}

/**
 * @brief  Set the IRQ in input mode.
 * @retval None
 */
void set_irq_as_input(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* IRQ input */  
  GPIO_InitStructure.Pin = BNRG_IRQ_PIN;
  GPIO_InitStructure.Mode = BNRG_IRQ_MODE;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = BNRG_IRQ_SPEED;
  GPIO_InitStructure.Alternate = BNRG_IRQ_ALTERNATE;    
  HAL_GPIO_Init(BNRG_IRQ_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pull = BNRG_IRQ_PULL;
  HAL_GPIO_Init(BNRG_IRQ_PORT, &GPIO_InitStructure);
}

/**
 * @brief  Enable SPI IRQ.
 * @retval None
 */
void Enable_SPI_IRQ(void)
{
  #if (USE_TIMER == 0)
    HAL_NVIC_EnableIRQ(BNRG_EXTI_IRQn);
  #endif   
}

/**
 * @brief  Disable SPI IRQ.
 * @retval None
 */
void Disable_SPI_IRQ(void)
{   
  #if (USE_TIMER == 0)
    HAL_NVIC_DisableIRQ(BNRG_EXTI_IRQn);
  #endif 
}

/**
 * @brief  Clear Pending SPI IRQ.
 * @retval None
 */
void Clear_SPI_IRQ(void)
{
  HAL_NVIC_ClearPendingIRQ(BNRG_EXTI_IRQn);
}

/**
* @brief Clear EXTI (External Interrupt) line for BNRG IRQ.
* 
*/
void Clear_SPI_EXTI_Flag(void)
{ 
__HAL_GPIO_EXTI_CLEAR_IT(BNRG_IRQ_PIN); 
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
   
/**
 * @}
 */
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
