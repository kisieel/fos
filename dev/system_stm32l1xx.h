/**
  ******************************************************************************
  * @file    system_stm32l1xx.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-December-2010
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Header File.
  ******************************************************************************  
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  ******************************************************************************  
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32l1xx_system
  * @{
  */  
  
/**
  * @brief Define to prevent recursive inclusion
  */
#ifndef __SYSTEM_STM32L1XX_H
#define __SYSTEM_STM32L1XX_H

#ifdef __cplusplus
 extern "C" {
#endif 

/** @addtogroup STM32L1xx_System_Includes
  * @{
  */

/**
  * @}
  */


/** @addtogroup STM32L1xx_System_Exported_types
  * @{
  */

extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock) */

/**
  * @}
  */

/** @addtogroup STM32L1xx_System_Exported_Constants
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32L1xx_System_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32L1xx_System_Exported_Functions
  * @{
  */

#define GPIO_MODE_Input      0x00
#define GPIO_MODE_GP         0x01
#define GPIO_MODE_AF         0x02
#define GPIO_MODE_Analog     0x03

#define GPIO_TYPE_Pushpull   0x00
#define GPIO_TYPE_Opendrain  0x01

#define GPIO_SPEED_400k      0x00
#define GPIO_SPEED_2M        0x01
#define GPIO_SPEED_10M       0x02
#define GPIO_SPEED_40M       0x03

#define GPIO_PULL_Floating   0x00
#define GPIO_PULL_Pullup     0x01
#define GPIO_PULL_Pulldown   0x02

#define GPIO_AF_AF0          0x00
#define GPIO_AF_AF1          0x01
#define GPIO_AF_AF2          0x02
#define GPIO_AF_AF3          0x03
#define GPIO_AF_AF4          0x04
#define GPIO_AF_AF5          0x05
#define GPIO_AF_AF6          0x06
#define GPIO_AF_AF7          0x07
#define GPIO_AF_AF8          0x08
#define GPIO_AF_AF9          0x09
#define GPIO_AF_AF10         0x0A
#define GPIO_AF_AF11         0x0B
#define GPIO_AF_AF12         0x0C
#define GPIO_AF_AF13         0x0D
#define GPIO_AF_AF14         0x0E
#define GPIO_AF_AF15         0x0F

extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
extern void GPIO_config(unsigned char BLOCK, 
	                      unsigned char PIN,
                        unsigned char MODE,
												unsigned char PULL,
												unsigned char TYPE,
												unsigned char SPEED,
												unsigned char AF);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_STM32L1XX_H */

/**
  * @}
  */
  
/**
  * @}
  */  
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
