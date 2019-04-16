/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define vlv17_Pin GPIO_PIN_3
#define vlv17_GPIO_Port GPIOE
#define vlv18_Pin GPIO_PIN_4
#define vlv18_GPIO_Port GPIOE
#define vlv19_Pin GPIO_PIN_5
#define vlv19_GPIO_Port GPIOE
#define vlv20_Pin GPIO_PIN_13
#define vlv20_GPIO_Port GPIOC
#define vlv21_Pin GPIO_PIN_14
#define vlv21_GPIO_Port GPIOC
#define vlv31_Pin GPIO_PIN_15
#define vlv31_GPIO_Port GPIOC
#define vlv30_Pin GPIO_PIN_0
#define vlv30_GPIO_Port GPIOC
#define vlv29_Pin GPIO_PIN_1
#define vlv29_GPIO_Port GPIOC
#define vlv28_Pin GPIO_PIN_2
#define vlv28_GPIO_Port GPIOC
#define vlv22_Pin GPIO_PIN_3
#define vlv22_GPIO_Port GPIOC
#define vlv23_Pin GPIO_PIN_0
#define vlv23_GPIO_Port GPIOA
#define vlv24_Pin GPIO_PIN_1
#define vlv24_GPIO_Port GPIOA
#define vlv25_Pin GPIO_PIN_2
#define vlv25_GPIO_Port GPIOA
#define vlv26_Pin GPIO_PIN_3
#define vlv26_GPIO_Port GPIOA
#define vlv27_Pin GPIO_PIN_4
#define vlv27_GPIO_Port GPIOC
#define rtd7_Pin GPIO_PIN_5
#define rtd7_GPIO_Port GPIOC
#define FLASH_CS_Pin GPIO_PIN_0
#define FLASH_CS_GPIO_Port GPIOB
#define EEPROM_CS_Pin GPIO_PIN_1
#define EEPROM_CS_GPIO_Port GPIOB
#define TC_MUX_EN_N_Pin GPIO_PIN_7
#define TC_MUX_EN_N_GPIO_Port GPIOE
#define TC_MUX_A3_Pin GPIO_PIN_8
#define TC_MUX_A3_GPIO_Port GPIOE
#define TC_MUX_A2_Pin GPIO_PIN_9
#define TC_MUX_A2_GPIO_Port GPIOE
#define TC_MUX_A1_Pin GPIO_PIN_10
#define TC_MUX_A1_GPIO_Port GPIOE
#define TC_MUX_A0_Pin GPIO_PIN_11
#define TC_MUX_A0_GPIO_Port GPIOE
#define rtd2_Pin GPIO_PIN_12
#define rtd2_GPIO_Port GPIOE
#define rtd3_Pin GPIO_PIN_14
#define rtd3_GPIO_Port GPIOE
#define rtd4_Pin GPIO_PIN_15
#define rtd4_GPIO_Port GPIOE
#define rtd5_Pin GPIO_PIN_12
#define rtd5_GPIO_Port GPIOB
#define rtd6_Pin GPIO_PIN_13
#define rtd6_GPIO_Port GPIOB
#define led0_Pin GPIO_PIN_8
#define led0_GPIO_Port GPIOD
#define led1_Pin GPIO_PIN_9
#define led1_GPIO_Port GPIOD
#define led2_Pin GPIO_PIN_10
#define led2_GPIO_Port GPIOD
#define led3_Pin GPIO_PIN_11
#define led3_GPIO_Port GPIOD
#define adc0_cs_Pin GPIO_PIN_12
#define adc0_cs_GPIO_Port GPIOD
#define adc1_cs_Pin GPIO_PIN_13
#define adc1_cs_GPIO_Port GPIOD
#define adc2_cs_Pin GPIO_PIN_14
#define adc2_cs_GPIO_Port GPIOD
#define adc3_cs_Pin GPIO_PIN_15
#define adc3_cs_GPIO_Port GPIOD
#define adc4_cs_Pin GPIO_PIN_8
#define adc4_cs_GPIO_Port GPIOC
#define adc5_cs_Pin GPIO_PIN_9
#define adc5_cs_GPIO_Port GPIOC
#define adc6_cs_Pin GPIO_PIN_8
#define adc6_cs_GPIO_Port GPIOA
#define rtd0_Pin GPIO_PIN_11
#define rtd0_GPIO_Port GPIOA
#define rtd1_Pin GPIO_PIN_12
#define rtd1_GPIO_Port GPIOA
#define buzzer_Pin GPIO_PIN_15
#define buzzer_GPIO_Port GPIOA
#define vlv0_Pin GPIO_PIN_10
#define vlv0_GPIO_Port GPIOC
#define vlv1_Pin GPIO_PIN_11
#define vlv1_GPIO_Port GPIOC
#define vlv2_Pin GPIO_PIN_12
#define vlv2_GPIO_Port GPIOC
#define vlv3_Pin GPIO_PIN_0
#define vlv3_GPIO_Port GPIOD
#define vlv4_Pin GPIO_PIN_1
#define vlv4_GPIO_Port GPIOD
#define vlv5_Pin GPIO_PIN_2
#define vlv5_GPIO_Port GPIOD
#define vlv6_Pin GPIO_PIN_3
#define vlv6_GPIO_Port GPIOD
#define vlv7_Pin GPIO_PIN_4
#define vlv7_GPIO_Port GPIOD
#define vlv8_Pin GPIO_PIN_7
#define vlv8_GPIO_Port GPIOD
#define vlv9_Pin GPIO_PIN_4
#define vlv9_GPIO_Port GPIOB
#define vlv10_Pin GPIO_PIN_5
#define vlv10_GPIO_Port GPIOB
#define vlv11_Pin GPIO_PIN_6
#define vlv11_GPIO_Port GPIOB
#define vlv12_Pin GPIO_PIN_7
#define vlv12_GPIO_Port GPIOB
#define vlv13_Pin GPIO_PIN_8
#define vlv13_GPIO_Port GPIOB
#define vlv14_Pin GPIO_PIN_9
#define vlv14_GPIO_Port GPIOB
#define vlv15_Pin GPIO_PIN_0
#define vlv15_GPIO_Port GPIOE
#define vlv16_Pin GPIO_PIN_1
#define vlv16_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
