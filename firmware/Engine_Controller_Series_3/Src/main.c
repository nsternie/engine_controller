
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stdlib.h"
#include "globals.h"
#include "calibrations.h"
#include "telem.h"
#include "pack_telem_defines.h"
#include "command.h"
#include "hardware.h"
#include "flash.h"
#include "config.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define millis	((__HAL_TIM_GET_COUNTER(&htim2))/1000)

#define DMA_RX_BUFFER_SIZE          (PACKET_SIZE + 2)
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
//
#define UART_BUFFER_SIZE            256
uint8_t UART_Buffer[UART_BUFFER_SIZE];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

// Function prototypes  ///////////////////////////////////
void print_adc_raw(UART_HandleTypeDef device);				// Print all the ADC things in a decentish table
uint8_t configure_devices();								// Send Configuration data to the spirit.
void send_telem(UART_HandleTypeDef device, uint8_t format);	//
void assemble_telem();

void scale_readings();
void buffer_init(struct buffer *b, uint8_t* data_buffer, size_t size, uint8_t id);
void buffer_read(struct buffer *b, uint8_t* dst, size_t size);
void buffer_write(struct buffer *b, uint8_t* src, size_t size);
void parse_buffer(struct buffer *b);

void error(char * error_message);

void run_auto(struct autosequence *a);
void start_auto(struct autosequence *a);
void stop_auto(struct autosequence *a);
void kill_auto(struct autosequence *a);



/* USER CODE END PFP */

/* USER CODE BEGIN 0 */



void system_init(){
	// Start the timers
	HAL_TIM_Base_Start_IT(&htim4);		// ADC sampling
	HAL_TIM_Base_Start_IT(&htim6);		// rs422 timing
	HAL_TIM_Base_Start_IT(&htim7);		// xbee timing
	HAL_TIM_Base_Start_IT(&htim10);		// motor control timing
	HAL_TIM_Base_Start_IT(&htim2);		// microsecond tick

	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); 	// mtr0
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);	// mtr1

	prime_bridge();

	for(uint8_t n = 0; n < 255; n++){
		upstream_buffer.data[n] = 0;
	}

	HAL_UART_Receive_IT(&huart1, &rs422_in, 1);
	HAL_UART_Receive_IT(&huart6, &uart6_in, 1);
	//HAL_UART_Receive_DMA(&huart6, &uart6_in, PACKET_SIZE+2);


	for(int n = 0; n < 4; n++){
		  command(led0-n,1);
		  HAL_Delay(10);
		  command(led0-n, 0);
	  }

	p = init_parser(BOARD_ID);
	//load_commands(&p);
	add_command(&p, COMMAND_DIGITAL_WRITE, 	digital_write);
	add_command(&p, COMMAND_LED_WRITE, 		led_write);
	add_command(&p, COMMAND_MOTOR_WRITE, 	motor_write);
	add_command(&p, COMMAND_MOTOR_DISABLE, 	motor_disable);
	add_command(&p, COMMAND_MOTOR_ENABLE, 	motor_enable);
	add_command(&p, COMMAND_SET_KP, 		set_kp);
	add_command(&p, COMMAND_SET_KI, 		set_ki);
	add_command(&p, COMMAND_SET_KD, 		set_kd);
	add_command(&p, COMMAND_TELEMRATE_SET, 	telemrate_set);
	add_command(&p, COMMAND_SAMPLERATE_SET, samplerate_set);
	add_command(&p, COMMAND_ARM, 			arm);
	add_command(&p, COMMAND_DISARM, 		disarm);
	add_command(&p, COMMAND_MAIN_AUTO_START,main_auto_start);
	add_command(&p, COMMAND_PWM_SET, 		pwm_set);
	add_command(&p, COMMAND_QD_SET, 		qd_set);
	add_command(&p, COMMAND_TARE, 			tare);
	add_command(&p, COMMAND_AMBIENTIZE, 	ambientize);
	add_command(&p, COMMAND_LOGRATE_SET, 	lograte_set);
	add_command(&p, COMMAND_PRINT_FILE, 	print_file);
	add_command(&p, COMMAND_NUMFILES, 		numfiles);
	add_command(&p, COMMAD_LOG_START, 		log_start);
	add_command(&p, COMMAD_LOG_END, 		log_end);
	add_command(&p, COMMAD_INIT_FS, 		init_fs);
	add_command(&p, COMMAND_TELEM_PAUSE, 		telem_pause);
	add_command(&p, COMMAND_TELEM_RESUME, 		telem_resume);
	add_command(&p, COMMAND_PRIME_BRIDGE,		prime_bridge_wrapper);

	release_device(adc0);
	release_device(adc1);
	release_device(adc2);
	release_device(flash);


  unlock_all(); // Flash init


}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */


 	system_init();
 	read_flash_id();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

 	uint32_t last_motor_millis = 0;
  while (1)
  {

#if BOARD_ID == TARGET_ADDRESS_FLIGHT
	  if(STATE == FIRING){
		  run_auto(millis);
		  __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
	  }
#endif
 //Check if packet is in from downstream engine controller
    for(uint8_t n = 0; n < 255; n++){
        if(upstream_buffer.data[n] == (uint8_t) '\n'){
        	command(led1, 1);
        	//__disable_irq();
        	HAL_Delay(0);
//        	while(millis - xmit_counter < xmit_delay);
            HAL_UART_Transmit(&huart1, upstream_buffer.data, n+1, 0xffff);
//            xmit_counter = millis;
            command(led1, 0);
//            send_rs422_now = 1;
//            pack_telem(telem_unstuffed);
//			stuff_telem(telem_unstuffed, telem_stuffed);
//			send_telem(rs422_com, gui_byte_packet);
//			TIME(telemetry_cycle_time);
//			__HAL_IWDG_RELOAD_COUNTER(&hiwdg);

            upstream_buffer.filled = 0;
            for(int j = 0; j < 255; j++){
                upstream_buffer.data[j] = 0;
            }
            //__enable_irq();
            break;
        }
    }

    if(read_adc_now){
        read_adc_now = 0;
        read_adc(&hspi1);
        scale_readings();
        pack_telem(telem_unstuffed);
		stuff_telem(telem_unstuffed, telem_stuffed);
        TIME(adc_cycle_time);
        if(LOGGING_ACTIVE){
        	save_telem(logfile);
        }
    }
#if BOARD_ID == TARGET_ADDRESS_FLIGHT
    if((millis - last_motor_millis) > 10){
        motor_control();
        last_motor_millis = millis;
    }
#endif

    if(send_rs422_now && TELEM_ACTIVE){
        send_rs422_now = 0;

		send_telem(rs422_com, gui_byte_packet);
        //xmit_counter = millis;

        TIME(telemetry_cycle_time);
        //__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
    }

if(p.buffer[p.filled - 1] == 0){
run_parser(&p);
}

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI4 init function */
static void MX_SPI4_Init(void)
{

  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, vlv17_Pin|vlv18_Pin|vlv19_Pin|TC_MUX_EN_N_Pin 
                          |TC_MUX_A3_Pin|TC_MUX_A2_Pin|TC_MUX_A1_Pin|TC_MUX_A0_Pin 
                          |rtd2_Pin|rtd3_Pin|rtd4_Pin|vlv15_Pin 
                          |vlv16_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, vlv20_Pin|vlv21_Pin|vlv31_Pin|vlv30_Pin 
                          |vlv29_Pin|vlv28_Pin|vlv22_Pin|vlv27_Pin 
                          |rtd7_Pin|adc4_cs_Pin|adc5_cs_Pin|vlv0_Pin 
                          |vlv1_Pin|vlv2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, vlv23_Pin|vlv24_Pin|vlv25_Pin|vlv26_Pin 
                          |adc6_cs_Pin|rtd0_Pin|rtd1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FLASH_CS_Pin|rtd5_Pin|rtd6_Pin|vlv9_Pin 
                          |vlv10_Pin|vlv11_Pin|vlv12_Pin|vlv13_Pin 
                          |vlv14_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, led0_Pin|led1_Pin|led2_Pin|led3_Pin 
                          |adc0_cs_Pin|adc1_cs_Pin|adc2_cs_Pin|adc3_cs_Pin 
                          |vlv3_Pin|vlv4_Pin|vlv5_Pin|vlv6_Pin 
                          |vlv7_Pin|vlv8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : vlv17_Pin vlv18_Pin vlv19_Pin TC_MUX_EN_N_Pin 
                           TC_MUX_A3_Pin TC_MUX_A2_Pin TC_MUX_A1_Pin TC_MUX_A0_Pin 
                           rtd2_Pin rtd3_Pin rtd4_Pin vlv15_Pin 
                           vlv16_Pin */
  GPIO_InitStruct.Pin = vlv17_Pin|vlv18_Pin|vlv19_Pin|TC_MUX_EN_N_Pin 
                          |TC_MUX_A3_Pin|TC_MUX_A2_Pin|TC_MUX_A1_Pin|TC_MUX_A0_Pin 
                          |rtd2_Pin|rtd3_Pin|rtd4_Pin|vlv15_Pin 
                          |vlv16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : vlv20_Pin vlv21_Pin vlv31_Pin vlv30_Pin 
                           vlv29_Pin vlv28_Pin vlv22_Pin vlv27_Pin 
                           rtd7_Pin adc4_cs_Pin adc5_cs_Pin vlv0_Pin 
                           vlv1_Pin vlv2_Pin */
  GPIO_InitStruct.Pin = vlv20_Pin|vlv21_Pin|vlv31_Pin|vlv30_Pin 
                          |vlv29_Pin|vlv28_Pin|vlv22_Pin|vlv27_Pin 
                          |rtd7_Pin|adc4_cs_Pin|adc5_cs_Pin|vlv0_Pin 
                          |vlv1_Pin|vlv2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : vlv23_Pin vlv24_Pin vlv25_Pin vlv26_Pin 
                           adc6_cs_Pin rtd0_Pin rtd1_Pin */
  GPIO_InitStruct.Pin = vlv23_Pin|vlv24_Pin|vlv25_Pin|vlv26_Pin 
                          |adc6_cs_Pin|rtd0_Pin|rtd1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FLASH_CS_Pin rtd5_Pin rtd6_Pin vlv9_Pin 
                           vlv10_Pin vlv11_Pin vlv12_Pin vlv13_Pin 
                           vlv14_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin|rtd5_Pin|rtd6_Pin|vlv9_Pin 
                          |vlv10_Pin|vlv11_Pin|vlv12_Pin|vlv13_Pin 
                          |vlv14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EEPROM_CS_Pin */
  GPIO_InitStruct.Pin = EEPROM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EEPROM_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led0_Pin led1_Pin led2_Pin led3_Pin 
                           adc0_cs_Pin adc1_cs_Pin adc2_cs_Pin adc3_cs_Pin 
                           vlv3_Pin vlv4_Pin vlv5_Pin vlv6_Pin 
                           vlv7_Pin vlv8_Pin */
  GPIO_InitStruct.Pin = led0_Pin|led1_Pin|led2_Pin|led3_Pin 
                          |adc0_cs_Pin|adc1_cs_Pin|adc2_cs_Pin|adc3_cs_Pin 
                          |vlv3_Pin|vlv4_Pin|vlv5_Pin|vlv6_Pin 
                          |vlv7_Pin|vlv8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
