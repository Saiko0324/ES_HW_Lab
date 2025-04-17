/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file  : b_l475e_iot01a1.c
  * @brief : Source file for the BSP Common driver
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

/* Includes ------------------------------------------------------------------*/
#include "b_l475e_iot01a1.h"
#include "stm32l4xx_hal_exti.h"

// USER CODE BEGIN




I2C_HandleTypeDef hI2cHandler;
static void              I2Cx_Error(I2C_HandleTypeDef *i2c_handler, uint8_t Addr);
uint16_t NFC_IO_IsDeviceReady (uint8_t Addr, uint32_t Trials);
// USER CODE END
/** @defgroup BSP BSP
 * @{
 */

/** @defgroup B_L475E_IOT01A1 B_L475E_IOT01A1
 * @{
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL B_L475E_IOT01A1 LOW LEVEL
 *  @brief This file provides set of firmware functions to manage Leds and push-button
 *         available on STM32L4xx-Nucleo Kit from STMicroelectronics.
 * @{
 */

/**
 * @}
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Private_Defines B_L475E_IOT01A1 LOW LEVEL Private Defines
 * @{
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_FunctionPrototypes B_L475E_IOT01A1 LOW LEVEL Private Function Prototypes
 * @{
 */
typedef void (* BSP_EXTI_LineCallback) (void);
typedef void (* BSP_BUTTON_GPIO_Init) (void);

/**
 * @}
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Private_Variables B_L475E_IOT01A1 LOW LEVEL Private Variables
 * @{
 */
typedef void (* BSP_LED_GPIO_Init) (void);
static GPIO_TypeDef*  LED_PORT[LEDn] = {LED2_GPIO_PORT};
static const uint16_t LED_PIN[LEDn]  = {LED2_PIN};
static void LED_USER_GPIO_Init(void);

static GPIO_TypeDef*   BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT};
static const uint16_t  BUTTON_PIN[BUTTONn]  = {USER_BUTTON_PIN};
static const IRQn_Type BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn};
EXTI_HandleTypeDef hpb_exti[BUTTONn] = {{.Line = EXTI_LINE_13}};
USART_TypeDef* COM_USART[COMn] = {COM1_UART};
UART_HandleTypeDef hcom_uart[COMn];
#if (USE_COM_LOG > 0)
static COM_TypeDef COM_ActiveLogPort;
#endif
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1U)
static uint32_t IsUsart1MspCbValid = 0;
#endif
__weak HAL_StatusTypeDef MX_USART1_UART_Init(UART_HandleTypeDef* huart);
/**
 * @}
 */

/** @defgroup B_L475E_IOT01A1_LOW_LEVEL_Private_Functions B_L475E_IOT01A1 LOW LEVEL Private Functions
 * @{
 */
static void BUTTON_USER_EXTI_Callback(void);
static void BUTTON_USER_GPIO_Init(void);
#if (USE_BSP_COM_FEATURE > 0)
static void USART1_MspInit(UART_HandleTypeDef *huart);
static void USART1_MspDeInit(UART_HandleTypeDef *huart);
#endif
/**
 * @brief  This method returns the STM32L4xx NUCLEO BSP Driver revision
 * @retval version: 0xXYZR (8bits for each decimal, R for RC)
 */
int32_t BSP_GetVersion(void)
{
  return (int32_t)__B_L475E_IOT01A1_BSP_VERSION;
}

/**
 * @brief  Configures LED on GPIO and/or on MFX.
 * @param  Led: LED to be configured.
 *              This parameter can be one of the following values:
 *              @arg  LED2, LED4, ...
 * @retval HAL status
 */
int32_t BSP_LED_Init(Led_TypeDef Led)
{
  static const BSP_LED_GPIO_Init LedGpioInit[LEDn] = {LED_USER_GPIO_Init};
  LedGpioInit[Led]();
  return BSP_ERROR_NONE;
}

/**
 * @brief  DeInit LEDs.
 * @param  Led: LED to be configured.
 *              This parameter can be one of the following values:
 *              @arg  LED2, LED4, ...
 * @note Led DeInit does not disable the GPIO clock nor disable the Mfx
 * @retval HAL status
 */
int32_t BSP_LED_DeInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* Turn off LED */
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET);
  /* DeInit the GPIO_LED pin */
  gpio_init_structure.Pin = LED_PIN[Led];
  HAL_GPIO_DeInit(LED_PORT[Led], gpio_init_structure.Pin);

  return BSP_ERROR_NONE;
}

/**
 * @brief  Turns selected LED On.
 * @param  Led: LED to be set on
 *              This parameter can be one of the following values:
 *              @arg  LED1
 *              @arg  LED2
 *              @arg  LED3
 *              @arg  LED4
 * @retval HAL status
 */
int32_t BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT [Led], LED_PIN [Led], GPIO_PIN_SET);

  return BSP_ERROR_NONE;
}

/**
 * @brief  Turns selected LED Off.
 * @param  Led: LED to be set off
 *              This parameter can be one of the following values:
 *              @arg  LED1
 *              @arg  LED2
 *              @arg  LED3
 *              @arg  LED4
 * @retval HAL status
 */
int32_t BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT [Led], LED_PIN [Led], GPIO_PIN_RESET);

  return BSP_ERROR_NONE;
}

/**
 * @brief  Toggles the selected LED.
 * @param  Led: LED to be toggled
 *              This parameter can be one of the following values:
 *              @arg  LED1
 *              @arg  LED2
 *              @arg  LED3
 *              @arg  LED4
 * @retval HAL status
 */
int32_t BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);

  return BSP_ERROR_NONE;
}

/**
 * @brief  Get the status of the LED.
 * @param  Led: LED for which get the status
 *              This parameter can be one of the following values:
 *              @arg  LED1
 *              @arg  LED2
 *              @arg  LED3
 *              @arg  LED4
 * @retval HAL status (1=high, 0=low)
 */
int32_t BSP_LED_GetState(Led_TypeDef Led)
{
  return (int32_t)(HAL_GPIO_ReadPin (LED_PORT [Led], LED_PIN [Led]) == GPIO_PIN_RESET);
}
/**
  * @brief
  * @retval None
  */
static void LED_USER_GPIO_Init(void) {

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUS_BSP_LED_GPIO_PORT, BUS_BSP_LED_GPIO_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin : PTPIN */
  GPIO_InitStruct.Pin = BUS_BSP_LED_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUS_BSP_LED_GPIO_PORT, &GPIO_InitStruct);

}

/**
  * @brief  Configures button GPIO and EXTI Line.
  * @param  Button: Button to be configured
  *                 This parameter can be one of the following values:
  *                 @arg  BUTTON_USER: User Push Button
  * @param  ButtonMode Button mode
  *                    This parameter can be one of the following values:
  *                    @arg  BUTTON_MODE_GPIO: Button will be used as simple IO
  *                    @arg  BUTTON_MODE_EXTI: Button will be connected to EXTI line
  *                                            with interrupt generation capability
  * @retval BSP status
  */
int32_t BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  int32_t ret = BSP_ERROR_NONE;

  static const BSP_EXTI_LineCallback ButtonCallback[BUTTONn] ={BUTTON_USER_EXTI_Callback};
  static const uint32_t  BSP_BUTTON_PRIO [BUTTONn] ={BSP_BUTTON_USER_IT_PRIORITY};
  static const uint32_t BUTTON_EXTI_LINE[BUTTONn] ={USER_BUTTON_EXTI_LINE};
  static const BSP_BUTTON_GPIO_Init ButtonGpioInit[BUTTONn] = {BUTTON_USER_GPIO_Init};

  ButtonGpioInit[Button]();

  if (ButtonMode == BUTTON_MODE_EXTI)
  {
    if(HAL_EXTI_GetHandle(&hpb_exti[Button], BUTTON_EXTI_LINE[Button]) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else if (HAL_EXTI_RegisterCallback(&hpb_exti[Button],  HAL_EXTI_COMMON_CB_ID, ButtonCallback[Button]) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
	else
    {
      /* Enable and set Button EXTI Interrupt to the lowest priority */
      HAL_NVIC_SetPriority((BUTTON_IRQn[Button]), BSP_BUTTON_PRIO[Button], 0x00);
      HAL_NVIC_EnableIRQ((BUTTON_IRQn[Button]));
    }
  }

  return ret;
}

/**
 * @brief  Push Button DeInit.
 * @param  Button Button to be configured
 *                This parameter can be one of the following values:
 *                @arg  BUTTON_USER: Wakeup Push Button
 * @note PB DeInit does not disable the GPIO clock
 * @retval BSP status
 */
int32_t BSP_PB_DeInit(Button_TypeDef Button)
{
  GPIO_InitTypeDef gpio_init_structure;

  gpio_init_structure.Pin = BUTTON_PIN[Button];
  HAL_NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  HAL_GPIO_DeInit(BUTTON_PORT[Button], gpio_init_structure.Pin);

  return BSP_ERROR_NONE;
}

/**
 * @brief  Returns the selected button state.
 * @param  Button Button to be addressed
 *                This parameter can be one of the following values:
 *                @arg  BUTTON_USER
 * @retval The Button GPIO pin value (GPIO_PIN_RESET = button pressed)
 */
int32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return (int32_t)(HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]) == GPIO_PIN_RESET);
}

/**
 * @brief  User EXTI line detection callbacks.
 * @retval None
 */
void BSP_PB_IRQHandler (Button_TypeDef Button)
{
  HAL_EXTI_IRQHandler( &hpb_exti[Button] );
}

/**
 * @brief  BSP Push Button callback
 * @param  Button Specifies the pin connected EXTI line
 * @retval None.
 */
__weak void BSP_PB_Callback(Button_TypeDef Button)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Button);

  /* This function should be implemented by the user application.
     It is called into this driver when an event on Button is triggered. */
}

/**
  * @brief  User EXTI line detection callbacks.
  * @retval None
  */
static void BUTTON_USER_EXTI_Callback(void)
{
  BSP_PB_Callback(BUTTON_USER);
}

/**
  * @brief
  * @retval None
  */
static void BUTTON_USER_GPIO_Init(void) {

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : PTPIN */
  GPIO_InitStruct.Pin = BUS_BSP_BUTTON_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUS_BSP_BUTTON_GPIO_PORT, &GPIO_InitStruct);

}

#if (USE_BSP_COM_FEATURE > 0)
/**
 * @brief  Configures COM port.
 * @param  COM: COM port to be configured.
 *              This parameter can be COM1
 * @param  UART_Init: Pointer to a UART_HandleTypeDef structure that contains the
 *                    configuration information for the specified USART peripheral.
 * @retval BSP error code
 */
int32_t BSP_COM_Init(COM_TypeDef COM)
{
  int32_t ret = BSP_ERROR_NONE;

  if(COM > COMn)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
     hcom_uart[COM].Instance = COM_USART[COM];
#if (USE_HAL_UART_REGISTER_CALLBACKS == 0U)
    /* Init the UART Msp */
    USART1_MspInit(&hcom_uart[COM]);
#else
    if(IsUsart1MspCbValid == 0U)
    {
      if(BSP_COM_RegisterDefaultMspCallbacks(COM) != BSP_ERROR_NONE)
      {
        return BSP_ERROR_MSP_FAILURE;
      }
    }
#endif
    if (MX_USART1_UART_Init(&hcom_uart[COM]))
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
  }

  return ret;
}

/**
 * @brief  DeInit COM port.
 * @param  COM COM port to be configured.
 *             This parameter can be COM1
 * @retval BSP status
 */
int32_t BSP_COM_DeInit(COM_TypeDef COM)
{
  int32_t ret = BSP_ERROR_NONE;

  if(COM > COMn)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    /* USART configuration */
    hcom_uart[COM].Instance = COM_USART[COM];

    #if (USE_HAL_UART_REGISTER_CALLBACKS == 0U)
      USART1_MspDeInit(&hcom_uart[COM]);
    #endif /* (USE_HAL_UART_REGISTER_CALLBACKS == 0U) */

    if(HAL_UART_DeInit(&hcom_uart[COM]) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
  }

  return ret;
}

/**
 * @brief  Configures COM port.
 * @param  huart USART handle
 *               This parameter can be COM1
 * @param  COM_Init Pointer to a UART_HandleTypeDef structure that contains the
 *                  configuration information for the specified USART peripheral.
 * @retval HAL error code
 */

/* USART1 init function */

__weak HAL_StatusTypeDef MX_USART1_UART_Init(UART_HandleTypeDef* huart)
{
  HAL_StatusTypeDef ret = HAL_OK;

  huart->Instance = USART1;
  huart->Init.BaudRate = 115200;
  huart->Init.WordLength = UART_WORDLENGTH_8B;
  huart->Init.StopBits = UART_STOPBITS_1;
  huart->Init.Parity = UART_PARITY_NONE;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_16;
  huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(huart) != HAL_OK)
  {
    ret = HAL_ERROR;
  }

  return ret;
}

#endif
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1U)
/**
 * @brief Register Default USART1 Bus Msp Callbacks
 * @retval BSP status
 */
int32_t BSP_COM_RegisterDefaultMspCallbacks(COM_TypeDef COM)
{
  int32_t ret = BSP_ERROR_NONE;

  if(COM >= COMn)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {

    __HAL_UART_RESET_HANDLE_STATE(&hcom_uart[COM]);

    /* Register default MspInit/MspDeInit Callback */
    if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPINIT_CB_ID, USART1_MspInit) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPDEINIT_CB_ID, USART1_MspDeInit) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      IsUsart1MspCbValid = 1U;
    }
  }

  /* BSP status */
  return ret;
}

/**
 * @brief Register USART1 Bus Msp Callback registering
 * @param Callbacks pointer to USART1 MspInit/MspDeInit callback functions
 * @retval BSP status
 */
int32_t BSP_COM_RegisterMspCallbacks (COM_TypeDef COM , BSP_COM_Cb_t *Callback)
{
  int32_t ret = BSP_ERROR_NONE;

  if(COM >= COMn)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    __HAL_UART_RESET_HANDLE_STATE(&hcom_uart[COM]);

    /* Register MspInit/MspDeInit Callbacks */
    if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPINIT_CB_ID, Callback->pMspInitCb) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else if(HAL_UART_RegisterCallback(&hcom_uart[COM], HAL_UART_MSPDEINIT_CB_ID, Callback->pMspDeInitCb) != HAL_OK)
    {
      ret = BSP_ERROR_PERIPH_FAILURE;
    }
    else
    {
      IsUsart1MspCbValid = 1U;
    }
  }

  /* BSP status */
  return ret;
}
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

#if (USE_COM_LOG > 0)
/**
 * @brief  Select the active COM port.
 * @param  COM COM port to be activated.
 *             This parameter can be COM1
 * @retval BSP status
 */
int32_t BSP_COM_SelectLogPort(COM_TypeDef COM)
{
  if(COM_ActiveLogPort != COM)
  {
    COM_ActiveLogPort = COM;
  }
  return BSP_ERROR_NONE;
}

#if defined(__CC_ARM) /* For arm compiler 5 */
#if !defined(__MICROLIB) /* If not Microlib */

struct __FILE
{
  int dummyVar; //Just for the sake of redefining __FILE, we won't we using it anyways ;)
};

FILE __stdout;

#endif /* If not Microlib */
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050) /* For arm compiler 6 */
#if !defined(__MICROLIB) /* If not Microlib */

FILE __stdout;

#endif /* If not Microlib */
#endif /* For arm compiler 5 */
#if defined(__ICCARM__) /* For IAR */
size_t __write(int Handle, const unsigned char *Buf, size_t Bufsize)
{
  int i;

  for(i=0; i<Bufsize; i++)
  {
    (void)HAL_UART_Transmit(&hcom_uart[COM_ActiveLogPort], (uint8_t *)&Buf[i], 1, COM_POLL_TIMEOUT);
  }

  return Bufsize;
}
#elif defined(__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) /* For ARM Compiler 5 and 6 */
int fputc (int ch, FILE *f)
{
  (void)HAL_UART_Transmit(&hcom_uart[COM_ActiveLogPort], (uint8_t *)&ch, 1, COM_POLL_TIMEOUT);
  return ch;
}
#else /* For GCC Toolchains */
int __io_putchar (int ch)
{
  (void)HAL_UART_Transmit(&hcom_uart[COM_ActiveLogPort], (uint8_t *)&ch, 1, COM_POLL_TIMEOUT);
  return ch;
}
#endif /* For IAR */
#endif /* USE_COM_LOG */
/**
 * @brief  Initializes USART1 MSP.
 * @param  huart USART1 handle
 * @retval None
 */

static void USART1_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    /* Enable Peripheral clock */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = BUS_USART1_TX_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BUS_USART1_TX_GPIO_AF;
    HAL_GPIO_Init(BUS_USART1_TX_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BUS_USART1_RX_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = BUS_USART1_RX_GPIO_AF;
    HAL_GPIO_Init(BUS_USART1_RX_GPIO_PORT, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
}

static void USART1_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(BUS_USART1_TX_GPIO_PORT, BUS_USART1_TX_GPIO_PIN);

    HAL_GPIO_DeInit(BUS_USART1_RX_GPIO_PORT, BUS_USART1_RX_GPIO_PIN);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
}

/* USER CODE BEGIN note*/
static void I2Cx_MspInit(I2C_HandleTypeDef *i2c_handler)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /*** Configure the GPIOs ***/
  /* Enable GPIO clock */
  DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();

  /* Configure I2C Tx, Rx as alternate function */
  gpio_init_structure.Pin = DISCOVERY_I2Cx_SCL_PIN | DISCOVERY_I2Cx_SDA_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_OD;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init_structure.Alternate = DISCOVERY_I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT, &gpio_init_structure);

  HAL_GPIO_Init(DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT, &gpio_init_structure);

  /*** Configure the I2C peripheral ***/
  /* Enable I2C clock */
  DISCOVERY_I2Cx_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  DISCOVERY_I2Cx_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  DISCOVERY_I2Cx_RELEASE_RESET();

  /* Enable and set I2Cx Interrupt to a lower priority */
  HAL_NVIC_SetPriority(DISCOVERY_I2Cx_EV_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_EV_IRQn);

  /* Enable and set I2Cx Interrupt to a lower priority */
  HAL_NVIC_SetPriority(DISCOVERY_I2Cx_ER_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(DISCOVERY_I2Cx_ER_IRQn);
}

/**
  * @brief  DeInitializes I2C MSP.
  * @param  i2c_handler  I2C handler
  * @retval None
  */
static void I2Cx_MspDeInit(I2C_HandleTypeDef *i2c_handler)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* Configure I2C Tx, Rx as alternate function */
  gpio_init_structure.Pin = DISCOVERY_I2Cx_SCL_PIN | DISCOVERY_I2Cx_SDA_PIN;
  HAL_GPIO_DeInit(DISCOVERY_I2Cx_SCL_SDA_GPIO_PORT, gpio_init_structure.Pin);
  /* Disable GPIO clock */
  DISCOVERY_I2Cx_SCL_SDA_GPIO_CLK_DISABLE();

  /* Disable I2C clock */
  DISCOVERY_I2Cx_CLK_DISABLE();
}

/**
  * @brief  Initializes I2C HAL.
  * @param  i2c_handler  I2C handler
  * @retval None
  */
static void I2Cx_Init(I2C_HandleTypeDef *i2c_handler)
{
  /* I2C configuration */
  i2c_handler->Instance              = DISCOVERY_I2Cx;
  i2c_handler->Init.Timing           = DISCOVERY_I2Cx_TIMING;
  i2c_handler->Init.OwnAddress1      = 0;
  i2c_handler->Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  i2c_handler->Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  i2c_handler->Init.OwnAddress2      = 0;
  i2c_handler->Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  i2c_handler->Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

  /* Init the I2C */
  I2Cx_MspInit(i2c_handler);
  HAL_I2C_Init(i2c_handler);

  /**Configure Analogue filter */
  HAL_I2CEx_ConfigAnalogFilter(i2c_handler, I2C_ANALOGFILTER_ENABLE);
}

/**
  * @brief  DeInitializes I2C HAL.
  * @param  i2c_handler  I2C handler
  * @retval None
  */
static void I2Cx_DeInit(I2C_HandleTypeDef *i2c_handler)
{  /* DeInit the I2C */
  I2Cx_MspDeInit(i2c_handler);
  HAL_I2C_DeInit(i2c_handler);
}

/**
  * @brief  Reads multiple data.
  * @param  i2c_handler  I2C handler
  * @param  Addr  I2C address
  * @param  Reg  Reg address
  * @param  MemAddress  memory address
  * @param  Buffer  Pointer to data buffer
  * @param  Length  Length of the data
  * @retval HAL status
  */
static HAL_StatusTypeDef I2Cx_ReadMultiple(I2C_HandleTypeDef *i2c_handler, uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(i2c_handler, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, 1000);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* I2C error occurred */
    I2Cx_Error(i2c_handler, Addr);
  }
  return status;
}


/**
  * @brief  Writes a value in a register of the device through BUS in using DMA mode.
  * @param  i2c_handler  I2C handler
  * @param  Addr  Device address on BUS Bus.
  * @param  Reg  The target register address to write
  * @param  MemAddress  memory address
  * @param  Buffer  The target register value to be written
  * @param  Length  buffer size to be written
  * @retval HAL status
  */
static HAL_StatusTypeDef I2Cx_WriteMultiple(I2C_HandleTypeDef *i2c_handler, uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(i2c_handler, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, 1000);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the I2C Bus */
    I2Cx_Error(i2c_handler, Addr);
  }
  return status;
}

/**
  * @brief  Checks if target device is ready for communication.
  * @note   This function is used with Memory devices
  * @param  i2c_handler  I2C handler
  * @param  DevAddress  Target device address
  * @param  Trials  Number of trials
  * @retval HAL status
  */
static HAL_StatusTypeDef I2Cx_IsDeviceReady(I2C_HandleTypeDef *i2c_handler, uint16_t DevAddress, uint32_t Trials)
{
  return (HAL_I2C_IsDeviceReady(i2c_handler, DevAddress, Trials, 1000));
}

/**
  * @brief  Manages error callback by re-initializing I2C.
  * @param  i2c_handler  I2C handler
  * @param  Addr  I2C Address
  * @retval None
  */
static void I2Cx_Error(I2C_HandleTypeDef *i2c_handler, uint8_t Addr)
{
  /* De-initialize the I2C communication bus */
  HAL_I2C_DeInit(i2c_handler);

  /* Re-Initialize the I2C communication bus */
  I2Cx_Init(i2c_handler);
}

/**
  * @}
  */

/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/
/******************************** LINK Sensors ********************************/

/**
  * @brief  Initializes Sensors low level.
  * @retval None
  */
void SENSOR_IO_Init(void)
{
  I2Cx_Init(&hI2cHandler);
}

/**
  * @brief  DeInitializes Sensors low level.
  * @retval None
  */
void SENSOR_IO_DeInit(void)
{
  I2Cx_DeInit(&hI2cHandler);
}

/**
  * @brief  Writes a single data.
  * @param  Addr  I2C address
  * @param  Reg  Reg address
  * @param  Value  Data to be written
  * @retval None
  */
void SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteMultiple(&hI2cHandler, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT,(uint8_t*)&Value, 1);
}

/**
  * @brief  Reads a single data.
  * @param  Addr  I2C address
  * @param  Reg  Reg address
  * @retval Data to be read
  */
uint8_t SENSOR_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint8_t read_value = 0;

  I2Cx_ReadMultiple(&hI2cHandler, Addr, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&read_value, 1);

  return read_value;
}

/**
  * @brief  Reads multiple data with I2C communication
  *         channel from TouchScreen.
  * @param  Addr  I2C address
  * @param  Reg  Register address
  * @param  Buffer  Pointer to data buffer
  * @param  Length  Length of the data
  * @retval HAL status
  */
uint16_t SENSOR_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
 return I2Cx_ReadMultiple(&hI2cHandler, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  Writes multiple data with I2C communication
  *         channel from MCU to TouchScreen.
  * @param  Addr  I2C address
  * @param  Reg  Register address
  * @param  Buffer  Pointer to data buffer
  * @param  Length  Length of the data
  * @retval None
  */
void SENSOR_IO_WriteMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
  I2Cx_WriteMultiple(&hI2cHandler, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  Checks if target device is ready for communication.
  * @note   This function is used with Memory devices
  * @param  DevAddress  Target device address
  * @param  Trials  Number of trials
  * @retval HAL status
  */
HAL_StatusTypeDef SENSOR_IO_IsDeviceReady(uint16_t DevAddress, uint32_t Trials)
{
  return (I2Cx_IsDeviceReady(&hI2cHandler, DevAddress, Trials));
}

/**
  * @brief  Delay function used in Sensor low level driver.
  * @param  Delay  Delay in ms
  * @retval None
  */
void SENSOR_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/******************************** LINK NFC ********************************/

/**
  * @brief  Initializes Sensors low level.
  * @param  GpoIrqEnable  0x0 is disable, otherwise enabled
  * @retval None
  */
void NFC_IO_Init(uint8_t GpoIrqEnable)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* I2C init */
  I2Cx_Init(&hI2cHandler);

  /* GPIO Ports Clock Enable */
  NFC_GPIO_CLK_ENABLE();

  /* Configure GPIO pins for GPO (PE4) */
  if(GpoIrqEnable == 0)
  {
    GPIO_InitStruct.Pin = NFC_GPIO_GPO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(NFC_GPIO_GPO_PIN_PORT, &GPIO_InitStruct);
  }
  else
  {
    GPIO_InitStruct.Pin = NFC_GPIO_GPO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(NFC_GPIO_GPO_PIN_PORT, &GPIO_InitStruct);
    /* Enable and set EXTI4_IRQn Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI4_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  }

  /* Configure GPIO pins for DISABLE (PE2)*/
  GPIO_InitStruct.Pin = NFC_GPIO_RFDISABLE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NFC_GPIO_RFDISABLE_PIN_PORT, &GPIO_InitStruct);
}

/**
  * @brief  DeInitializes Sensors low level.
  * @retval None
  */
void NFC_IO_DeInit(void)
{
  I2Cx_DeInit(&hI2cHandler);
}

/**
  * @brief  This functions reads a response of the M24SR device
  * @param  Addr  M24SR I2C address
  * @param  pBuffer  Pointer on the buffer to retrieve M24SR response
  * @param  Length  Length of the data
  * @retval Status  Success or Timeout
  */
uint16_t NFC_IO_ReadMultiple (uint8_t Addr, uint8_t *pBuffer, uint16_t Length )
{
  uint16_t status ;

    /* Before calling this function M24SR must be ready: check to detect potential issues */
  status = NFC_IO_IsDeviceReady(Addr, NFC_I2C_TRIALS);
  if (status != NFC_I2C_STATUS_SUCCESS)
  {
    return NFC_I2C_ERROR_TIMEOUT;
  }

  if( HAL_I2C_Master_Receive(&hI2cHandler, Addr, (uint8_t*)pBuffer, Length, NFC_I2C_TIMEOUT_STD) != HAL_OK)
  {
    return NFC_I2C_ERROR_TIMEOUT;
  }

  return NFC_I2C_STATUS_SUCCESS;
}

/**
  * @brief  This functions sends the command buffer
  * @param  Addr  M24SR I2C address
  * @param  pBuffer  pointer to the buffer to send to the M24SR
  * @param  Length  Length of the data
  * @retval Status  Success or Timeout
  */
uint16_t  NFC_IO_WriteMultiple (uint8_t Addr, uint8_t *pBuffer, uint16_t Length)
{
  uint16_t status ;

  /* Before calling this function M24SR must be ready: check to detect potential issues */
  status = NFC_IO_IsDeviceReady(Addr, NFC_I2C_TRIALS);
  if (status != NFC_I2C_STATUS_SUCCESS)
  {
    return NFC_I2C_ERROR_TIMEOUT;
  }

  if( HAL_I2C_Master_Transmit(&hI2cHandler, Addr, (uint8_t*)pBuffer, Length, NFC_I2C_TIMEOUT_STD) != HAL_OK)
  {
    return NFC_I2C_ERROR_TIMEOUT;
  }

  return NFC_I2C_STATUS_SUCCESS;
}

/**
  * @brief  Checks if target device is ready for communication.
  * @param  Addr  M24SR I2C address
  * @param  Trials  Number of trials (currently not present in M24sr)
  * @retval Status  Success or Timeout
  */
uint16_t   NFC_IO_IsDeviceReady (uint8_t Addr, uint32_t Trials)
{
    HAL_StatusTypeDef status;
    uint32_t tickstart = 0;
    uint32_t currenttick = 0;

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Wait until M24SR is ready or timeout occurs */
    do
    {
        status = HAL_I2C_IsDeviceReady(&hI2cHandler, Addr, Trials, NFC_I2C_TIMEOUT_STD);
        currenttick = HAL_GetTick();
    } while( ( (currenttick - tickstart) < NFC_I2C_TIMEOUT_MAX) && (status != HAL_OK) );

    if (status != HAL_OK)
    {
        return NFC_I2C_ERROR_TIMEOUT;
    }

    return NFC_I2C_STATUS_SUCCESS;
}

/**
  * @brief  This function read the state of the M24SR GPO
  * @retval GPIO_PinState  state of the M24SR GPO
  */
void NFC_IO_ReadState(uint8_t * pPinState)
{
  *pPinState = (uint8_t)HAL_GPIO_ReadPin(NFC_GPIO_GPO_PIN_PORT,NFC_GPIO_GPO_PIN);
}

/**
  * @brief  This function sets the state of the M24SR RF disable pin
  * @param  PinState  put RF disable pin of M24SR in PinState (1 or 0)
  */
void NFC_IO_RfDisable(uint8_t PinState)
{
  HAL_GPIO_WritePin(NFC_GPIO_RFDISABLE_PIN_PORT,NFC_GPIO_RFDISABLE_PIN,(GPIO_PinState)PinState);
}

/**
  * @brief  Delay function used in M24SR low level driver.
  * @param  Delay  Delay in ms
  * @retval None
  */
void NFC_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}
/* USER CODE END */

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

