/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file  : b_l475e_iot01a1.h
  * @brief : header file for the BSP Common driver
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
*/
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __B_L475E_IOT01A1_H
#define __B_L475E_IOT01A1_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "b_l475e_iot01a1_conf.h"
#include "b_l475e_iot01a1_errno.h"
#include "stm32l475e_iot01_accelero.h"
#include "main.h"

#if (USE_BSP_COM_FEATURE > 0)
  #if (USE_COM_LOG > 0)
    #if defined(__ICCARM__) || defined(__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) /* For IAR and ARM Compiler 5 and 6*/
      #include <stdio.h>
    #endif
  #endif
#endif
/** @addtogroup BSP
 * @{
 */

/** @defgroup B_L475E_IOT01A1
 * @{
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL
 * @{
 */

/** @defgroup STM32L4XX_NUCLEO_LOW_LEVEL_Exported_Constants LOW LEVEL Exported Constants
  * @{
  */
/**
 * @brief STM32L4XX NUCLEO BSP Driver version number V1.0.0
 */
#define __B_L475E_IOT01A1_BSP_VERSION_MAIN   (uint32_t)(0x01) /*!< [31:24] main version */
#define __B_L475E_IOT01A1_BSP_VERSION_SUB1   (uint32_t)(0x00) /*!< [23:16] sub1 version */
#define __B_L475E_IOT01A1_BSP_VERSION_SUB2   (uint32_t)(0x00) /*!< [15:8]  sub2 version */
#define __B_L475E_IOT01A1_BSP_VERSION_RC     (uint32_t)(0x00) /*!< [7:0]  release candidate */
#define __B_L475E_IOT01A1_BSP_VERSION        ((__B_L475E_IOT01A1_BSP_VERSION_MAIN << 24)\
                                                    |(__B_L475E_IOT01A1_BSP_VERSION_SUB1 << 16)\
                                                    |(__B_L475E_IOT01A1_BSP_VERSION_SUB2 << 8 )\
                                                    |(__B_L475E_IOT01A1_BSP_VERSION_RC))

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Exported_Types B_L475E_IOT01A1 LOW LEVEL Exported Types
 * @{
 */

 /**
  * @brief Define for B_L475E_IOT01A1 board
  */
#if !defined (USE_B_L475E_IOT01A1)
 #define USE_B_L475E_IOT01A1
#endif
#ifndef USE_BSP_COM_FEATURE
   #define USE_BSP_COM_FEATURE                  0U
#endif

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_LED B_L475E_IOT01A1 LOW LEVEL LED
 * @{
 */
/** Define number of LED            **/
#define LEDn                              1U
/**  Definition for BSP USER LED 2   **/
#define LED2_PIN                     	  GPIO_PIN_14
#define LED2_GPIO_PORT                    GPIOB
#define LED2_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOB_CLK_DISABLE()

#define BUS_GPIO_INSTANCE GPIO
#define BUS_BSP_LED_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_BSP_LED_GPIO_PORT GPIOB
#define BUS_BSP_LED_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_BSP_LED_GPIO_PIN GPIO_PIN_14

/**
 * @}
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_BUTTON B_L475E_IOT01A1 LOW LEVEL BUTTON
 * @{
 */
/* Button state */
#define BUTTON_RELEASED                   0U
#define BUTTON_PRESSED                    1U
/** Define number of BUTTON            **/
#define BUTTONn                           1U

/**
 * @brief User push-button
 */
  /**  Definition for BSP USER BUTTON   **/

#define BUS_GPIO_INSTANCE GPIO
#define BUS_BSP_BUTTON_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define BUS_BSP_BUTTON_GPIO_PIN GPIO_PIN_13
#define BUS_BSP_BUTTON_GPIO_CLK_DISABLE() __HAL_RCC_GPIOC_CLK_DISABLE()
#define BUS_BSP_BUTTON_GPIO_PORT GPIOC

#define USER_BUTTON_PIN	                  GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT              GPIOC
#define USER_BUTTON_EXTI_IRQn              EXTI15_10_IRQn
#define USER_BUTTON_EXTI_LINE              EXTI_LINE_13
#define H_EXTI_13			  hpb_exti[BUTTON_USER]
/**
 * @}
 */
/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_COM B_L475E_IOT01A1 LOW LEVEL COM
 * @{
 */
/**
 * @brief Definition for COM portx, connected to USART1
 */

#define BUS_USART1_INSTANCE USART1
#define BUS_USART1_TX_GPIO_PIN GPIO_PIN_6
#define BUS_USART1_TX_GPIO_PORT GPIOB
#define BUS_USART1_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_USART1_TX_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_USART1_TX_GPIO_AF GPIO_AF7_USART1
#define BUS_USART1_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BUS_USART1_RX_GPIO_PORT GPIOB
#define BUS_USART1_RX_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()
#define BUS_USART1_RX_GPIO_PIN GPIO_PIN_7
#define BUS_USART1_RX_GPIO_AF GPIO_AF7_USART1

/**
 * @}
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Exported_Types LOW LEVEL Exported Types
  * @{
  */
#ifndef USE_BSP_COM
  #define USE_BSP_COM                           0U
#endif

#ifndef USE_COM_LOG
  #define USE_COM_LOG                           1U
#endif

#ifndef BSP_BUTTON_USER_IT_PRIORITY
  #define BSP_BUTTON_USER_IT_PRIORITY            15U
#endif

typedef enum
{
  LED2 = 0,
  LED_GREEN = LED2,
}Led_TypeDef;

typedef enum
{
  BUTTON_USER = 0U,
}Button_TypeDef;

/* Keep compatibility with CMSIS Pack already delivered */
#define BUTTON_KEY BUTTON_USER

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

#if (USE_BSP_COM_FEATURE > 0)
typedef enum
{
  COM1 = 0U,
  COMn
}COM_TypeDef;

typedef enum
{
 COM_WORDLENGTH_8B     =   UART_WORDLENGTH_8B,
 COM_WORDLENGTH_9B     =   UART_WORDLENGTH_9B,
}COM_WordLengthTypeDef;

typedef enum
{
 COM_STOPBITS_1     =   UART_STOPBITS_1,
 COM_STOPBITS_2     =   UART_STOPBITS_2,
}COM_StopBitsTypeDef;

typedef enum
{
 COM_PARITY_NONE     =  UART_PARITY_NONE,
 COM_PARITY_EVEN     =  UART_PARITY_EVEN,
 COM_PARITY_ODD      =  UART_PARITY_ODD,
}COM_ParityTypeDef;

typedef enum
{
 COM_HWCONTROL_NONE    =  UART_HWCONTROL_NONE,
 COM_HWCONTROL_RTS     =  UART_HWCONTROL_RTS,
 COM_HWCONTROL_CTS     =  UART_HWCONTROL_CTS,
 COM_HWCONTROL_RTS_CTS =  UART_HWCONTROL_RTS_CTS,
}COM_HwFlowCtlTypeDef;

typedef struct
{
  uint32_t             BaudRate;
  COM_WordLengthTypeDef  WordLength;
  COM_StopBitsTypeDef  StopBits;
  COM_ParityTypeDef    Parity;
  COM_HwFlowCtlTypeDef HwFlowCtl;
}COM_InitTypeDef;
#endif

#define MX_UART_InitTypeDef          COM_InitTypeDef
#define MX_UART_StopBitsTypeDef      COM_StopBitsTypeDef
#define MX_UART_ParityTypeDef        COM_ParityTypeDef
#define MX_UART_HwFlowCtlTypeDef     COM_HwFlowCtlTypeDef
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1U)
typedef struct
{
  void (* pMspInitCb)(UART_HandleTypeDef *);
  void (* pMspDeInitCb)(UART_HandleTypeDef *);
} BSP_COM_Cb_t;
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS == 1U) */

/**
 * @}
 */

#define COMn                             1U
#define COM1_UART                        USART1

#define COM_POLL_TIMEOUT                 1000
extern UART_HandleTypeDef hcom_uart[COMn];
#define  huart1 hcom_uart[COM1]

/**
 * @}
 */

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Exported_Variables LOW LEVEL Exported Constants
  * @{
  */
extern EXTI_HandleTypeDef hpb_exti[BUTTONn];
/**
  * @}
  */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Exported_Functions B_L475E_IOT01A1 LOW LEVEL Exported Functions
 * @{
 */

int32_t  BSP_GetVersion(void);
int32_t  BSP_LED_Init(Led_TypeDef Led);
int32_t  BSP_LED_DeInit(Led_TypeDef Led);
int32_t  BSP_LED_On(Led_TypeDef Led);
int32_t  BSP_LED_Off(Led_TypeDef Led);
int32_t  BSP_LED_Toggle(Led_TypeDef Led);
int32_t  BSP_LED_GetState(Led_TypeDef Led);
int32_t  BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
int32_t  BSP_PB_DeInit(Button_TypeDef Button);
int32_t  BSP_PB_GetState(Button_TypeDef Button);
void     BSP_PB_Callback(Button_TypeDef Button);
void     BSP_PB_IRQHandler(Button_TypeDef Button);
#if (USE_BSP_COM_FEATURE > 0)
int32_t  BSP_COM_Init(COM_TypeDef COM);
int32_t  BSP_COM_DeInit(COM_TypeDef COM);
#endif

#if (USE_COM_LOG > 0)
int32_t  BSP_COM_SelectLogPort(COM_TypeDef COM);
#endif

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1U)
int32_t BSP_COM_RegisterDefaultMspCallbacks(COM_TypeDef COM);
int32_t BSP_COM_RegisterMspCallbacks(COM_TypeDef COM , BSP_COM_Cb_t *Callback);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */


// USER CODE BEGIN note
#define NFC_GPIO_GPO_PIN                         GPIO_PIN_4
#define NFC_GPIO_GPO_PIN_PORT                    GPIOE
#define NFC_GPIO_RFDISABLE_PIN                   GPIO_PIN_2
#define NFC_GPIO_RFDISABLE_PIN_PORT              GPIOE
#define NFC_GPIO_CLK_ENABLE()                    __HAL_RCC_GPIOE_CLK_ENABLE();
#define NFC_GPIO_CLK_DISABLE()                   __HAL_RCC_GPIOE_CLK_DISABLE();

/**
 * @brief Definition for COM port1, connected to USART1
 */
#define DISCOVERY_COM1                          USART1
#define DISCOVERY_COM1_CLK_ENABLE()             __HAL_RCC_USART1_CLK_ENABLE()
#define DISCOVERY_COM1_CLK_DISABLE()            __HAL_RCC_USART1_CLK_DISABLE()

#define DISCOVERY_COM1_TX_PIN                   GPIO_PIN_6
#define DISCOVERY_COM1_TX_GPIO_PORT             GPIOB
#define DISCOVERY_COM1_TX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()
#define DISCOVERY_COM1_TX_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOB_CLK_DISABLE()
#define DISCOVERY_COM1_TX_AF                    GPIO_AF7_USART1

#define DISCOVERY_COM1_RX_PIN                   GPIO_PIN_7
#define DISCOVERY_COM1_RX_GPIO_PORT             GPIOB
#define DISCOVERY_COM1_RX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()
#define DISCOVERY_COM1_RX_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOB_CLK_DISABLE()
#define DISCOVERY_COM1_RX_AF                    GPIO_AF7_USART1

#define DISCOVERY_COM1_IRQn                     USART1_IRQn


#define DISCOVERY_COMx_CLK_ENABLE(__INDEX__)            do { if((__INDEX__) == COM1) {DISCOVERY_COM1_CLK_ENABLE();}} while(0)
#define DISCOVERY_COMx_CLK_DISABLE(__INDEX__)           do { if((__INDEX__) == COM1) {DISCOVERY_COM1_CLK_DISABLE();}} while(0)

#define DISCOVERY_COMx_TX_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == COM1) {DISCOVERY_COM1_TX_GPIO_CLK_ENABLE();}} while(0)
#define DISCOVERY_COMx_TX_GPIO_CLK_DISABLE(__INDEX__)   do { if((__INDEX__) == COM1) {DISCOVERY_COM1_TX_GPIO_CLK_DISABLE();}} while(0)

#define DISCOVERY_COMx_RX_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == COM1) {DISCOVERY_COM1_RX_GPIO_CLK_ENABLE();}} while(0)
#define DISCOVERY_COMx_RX_GPIO_CLK_DISABLE(__INDEX__)   do { if((__INDEX__) == COM1) {DISCOVERY_COM1_RX_GPIO_CLK_DISABLE();}} while(0)


/* User can use this section to tailor I2Cx instance used and associated resources */
/* Definition for I2Cx resources */
#define DISCOVERY_I2Cx                             I2C2
#define DISCOVERY_I2Cx_CLK_ENABLE()                __HAL_RCC_I2C2_CLK_ENABLE()
#define DISCOVERY_I2Cx_CLK_DISABLE()               __HAL_RCC_I2C2_CLK_DISABLE()
#define DISCOVERY_DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOB_CLK_DISABLE()

#define DISCOVERY_I2Cx_FORCE_RESET()               __HAL_RCC_I2C2_FORCE_RESET()
#define DISCOVERY_I2Cx_RELEASE_RESET()             __HAL_RCC_I2C2_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define DISCOVERY_I2Cx_SCL_PIN                     GPIO_PIN_10
#define DISCOVERY_I2Cx_SDA_PIN                     GPIO_PIN_11
#define DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT           GPIOB
#define DISCOVERY_I2Cx_SCL_SDA_AF                  GPIO_AF4_I2C2

/* I2C interrupt requests */
#define DISCOVERY_I2Cx_EV_IRQn                     I2C2_EV_IRQn
#define DISCOVERY_I2Cx_ER_IRQn                     I2C2_ER_IRQn

/* I2C clock speed configuration (in Hz)
  WARNING:
   Make sure that this define is not already declared in other files.
   It can be used in parallel by other modules. */
#ifndef DISCOVERY_I2C_SPEED
 #define DISCOVERY_I2C_SPEED                             100000
#endif /* DISCOVERY_I2C_SPEED */

#ifndef DISCOVERY_I2Cx_TIMING
#define DISCOVERY_I2Cx_TIMING                     ((uint32_t)0x00702681)
#endif /* DISCOVERY_I2Cx_TIMING */


/* I2C Sensors address */
/* LPS22HB (Pressure) I2C Address */
#define LPS22HB_I2C_ADDRESS  (uint8_t)0xBA
/* HTS221 (Humidity) I2C Address */
#define HTS221_I2C_ADDRESS   (uint8_t)0xBE

#ifdef USE_LPS22HB_TEMP
/* LPS22HB Sensor hardware I2C address */
#define TSENSOR_I2C_ADDRESS     LPS22HB_I2C_ADDRESS
#else /* USE_HTS221_TEMP */
/* HTS221 Sensor hardware I2C address */
#define TSENSOR_I2C_ADDRESS     HTS221_I2C_ADDRESS
#endif

/* NFC I2C address and specific config parameters */
#define M24SR_I2C_ADDR             (uint8_t)  0xAC /*!< M24SR address */
#define NFC_I2C_STATUS_SUCCESS     (uint16_t) 0x0000
#define NFC_I2C_ERROR_TIMEOUT      (uint16_t) 0x0011
#define NFC_I2C_TIMEOUT_STD        (uint32_t) 8 /* I2C Time out (ms), used to call Transmit/Receive HAL functions */
#define NFC_I2C_TIMEOUT_MAX        (uint32_t) 200 /* I2C Time out (ms), this is the maximum time needed by M24SR to complete any command */
#define NFC_I2C_TRIALS             (uint32_t) 1 /* In case M24SR will reply ACK failed allow to perform retry before returning error (HAL option not used) */


// USER CODE END


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
#ifdef __cplusplus
}
#endif

#endif /* __B_L475E_IOT01A1__H */

