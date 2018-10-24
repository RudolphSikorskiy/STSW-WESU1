/**
  ******************************************************************************
  * @file    stc3115_Battery.h
  * @author  AMS
  * @version V1.0.0
  * @date    13-January-2015
  * @brief   Application/Battery description and charge curve characterization
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __Battery_H
#define __Battery_H

/** @addtogroup BSP             BSP
  * @{
  */ 

/** @addtogroup COMPONENTS      COMPONENTS
  * @{
  */ 

/** @addtogroup STC3115         STC3115
  * @{
  */

/* **************************************************************************** */
/*        INTERNAL PARAMETERS                                                   */
/*   TO BE ADJUSTED ACCORDING TO BATTERY AND APPLCICATION CHARACTERISTICS       */
/* ---------------------------------------------------------------------------- */

/*Battery parameters define  -------------------------------------------------- */
#define CAPACITY	100	/*!< battery nominal capacity in mAh                    */
#define RINT            200     /*!< Internal battery impedance in mOhms,0 if unknown	*/

#define OCV_OFFSET_TAB {0, -9, 31, 20, -9, -6, -19, -12, 6, 9, -32, -43, -13, -7, -23, -40} /*!< OCVTAB */

/*Application parameters define  --------------------------------------------------     */
#define VMODE           MIXED_MODE      /*!< running mode constant, VM_MODE or MIXED_MODE*/
#define ALM_EN          1               /*!< Alarm enable constant, set at 1 to enable	*/
#define ALM_SOC         10              /*!< SOC alarm in %                             */
#define ALM_VBAT        3600		/*!< Voltage alarm in mV                        */
#define RSENSE          50              /*!< sense resistor in mOhms                    */

#define APP_EOC_CURRENT         2   		/*!< end charge current in mA           */
#define APP_CUTOFF_VOLTAGE      3000   	        /*!< application cut-off voltage in mV  */

/* ********************************************************************************     */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
