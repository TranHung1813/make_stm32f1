/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

#include "stm32l0xx_ll_adc.h"
#include "stm32l0xx_ll_dma.h"
#include "stm32l0xx_ll_i2c.h"
#include "stm32l0xx_ll_iwdg.h"
#include "stm32l0xx_ll_lpuart.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_rtc.h"
#include "stm32l0xx_ll_tim.h"
#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SMART_MODULE_PWR_CONTROL_Pin LL_GPIO_PIN_13
#define SMART_MODULE_PWR_CONTROL_GPIO_Port GPIOC
#define ADC_VIN_Pin LL_GPIO_PIN_0
#define ADC_VIN_GPIO_Port GPIOA
#define ADC_3V8_Pin LL_GPIO_PIN_1
#define ADC_3V8_GPIO_Port GPIOA
#define FM_USART_TX_Pin LL_GPIO_PIN_2
#define FM_USART_TX_GPIO_Port GPIOA
#define FM_USART_RX_Pin LL_GPIO_PIN_3
#define FM_USART_RX_GPIO_Port GPIOA
#define ADC_5V_Pin LL_GPIO_PIN_4
#define ADC_5V_GPIO_Port GPIOA
#define EXT_WDT_Pin LL_GPIO_PIN_5
#define EXT_WDT_GPIO_Port GPIOA
#define SMART_MODULE_RST_Pin LL_GPIO_PIN_6
#define SMART_MODULE_RST_GPIO_Port GPIOA
#define TO_SM_POWER_STATE_Pin LL_GPIO_PIN_7
#define TO_SM_POWER_STATE_GPIO_Port GPIOA
#define HEARTBEAT_Pin LL_GPIO_PIN_2
#define HEARTBEAT_GPIO_Port GPIOB
#define HEARTBEAT_EXTI_IRQn EXTI2_3_IRQn
#define SW_IN_Pin LL_GPIO_PIN_10
#define SW_IN_GPIO_Port GPIOB
#define SW_OUT_Pin LL_GPIO_PIN_11
#define SW_OUT_GPIO_Port GPIOB
#define SMART_MODULE_PWR_KEY_Pin LL_GPIO_PIN_8
#define SMART_MODULE_PWR_KEY_GPIO_Port GPIOA
#define MIN_USART_TX_Pin LL_GPIO_PIN_9
#define MIN_USART_TX_GPIO_Port GPIOA
#define MIN_USART_RX_Pin LL_GPIO_PIN_10
#define MIN_USART_RX_GPIO_Port GPIOA
#define DISABLE_HOST_UART_PIN_Pin LL_GPIO_PIN_11
#define DISABLE_HOST_UART_PIN_GPIO_Port GPIOA
#define PA_CONTROL_Pin LL_GPIO_PIN_12
#define PA_CONTROL_GPIO_Port GPIOA
#define INPUT1_Pin LL_GPIO_PIN_15
#define INPUT1_GPIO_Port GPIOA
#define INPUT2_Pin LL_GPIO_PIN_3
#define INPUT2_GPIO_Port GPIOB
#define INPUT3_Pin LL_GPIO_PIN_4
#define INPUT3_GPIO_Port GPIOB
#define INPUT4_Pin LL_GPIO_PIN_5
#define INPUT4_GPIO_Port GPIOB
#define OUTPUT1_Pin LL_GPIO_PIN_6
#define OUTPUT1_GPIO_Port GPIOB
#define OUTPUT2_Pin LL_GPIO_PIN_7
#define OUTPUT2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/**
 *  @brief      On systick handle
 */
void sys_increase_tick(void);

/**
 *  @brief      Get current systick in milisecond
 *  @retval     Systick counter
 */
uint32_t sys_get_ms(void);
   
/**
 *  @brief      Delay in ms
 *  @param[in]  ms Delay ms
 */   
void sys_delay_ms(uint32_t ms);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
