
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/



#define getName(var)  #var
#define get_state(x) (states[x])
#define TIME(x)			(x[0] = (__HAL_TIM_GET_COUNTER(&htim2) - x[1])+1); \
						(x[1] = __HAL_TIM_GET_COUNTER(&htim2))

#define micros	__HAL_TIM_GET_COUNTER(&htim2)
#define millis	((__HAL_TIM_GET_COUNTER(&htim2))/1000)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM5_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

// Function prototypes  ///////////////////////////////////
void command(uint8_t device, int16_t command_value);	 	// Motors, valves
void select_device(uint8_t device);							// For SPI Devices
void release_device(uint8_t device);						// For SPI Devices
void read_adc_brute();										// Brute force read all the adcs. Blocking. Slow. Lazy.
void print_adc_raw(UART_HandleTypeDef device);				// Print all the ADC things in a decentish table
uint8_t configure_devices();								// Send Configuration data to the spirit.
void send_telem(UART_HandleTypeDef device, uint8_t format);	//
void assemble_telem();
uint32_t serial_command(uint8_t* cbuf_in);
void scale_readings();
void buffer_init(struct buffer *b, uint8_t* data_buffer, size_t size, uint8_t id);
void buffer_read(struct buffer *b, uint8_t* dst, size_t size);
void buffer_write(struct buffer *b, uint8_t* src, size_t size);
void parse_buffer(struct buffer *b);
void setpwm(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse);
void error(char * error_message);
void writeMotor(uint8_t device, int16_t motor_command);
void motor_control();
void read_thermocouples();
void run_auto(struct autosequence *a);
void start_auto(struct autosequence *a);
void stop_auto(struct autosequence *a);
void kill_auto(struct autosequence *a);

// END Function prototypes  ///////////////////////////////

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  STATE = MANUAL;
  // Init device alias array because idk how to do it without strcpy :(
	temp = 0;
	for(uint8_t device = vlv0; device <= vlv15; device ++){
	  uint8_t alias[10] = "vlv";
	  if(device < vlv10){
		  alias[3] = temp++ + 48;
	  }
	  else{
		  alias[3] = 1 + 48;
		  alias[4] = temp++ + 38;
	  }
	  strcpy(device_alias[device], alias);
	}
	strcpy(device_alias[delay], "delay");
	strcpy(device_alias[led0], "led0");
	strcpy(device_alias[mtr0], "mtr0");
	strcpy(device_alias[mtr1], "mtr1");
	strcpy(device_alias[mtr2], "mtr2");
	strcpy(device_alias[mtr3], "mtr3");

	// Set auto numbers
	for(int n = 0; n < NUM_AUTOS; n++){
		autos[n].number = n;
	}

	serial_command("set samplerate 50");
	// Motor control loop rate
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 180;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 10000;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
	{
	Error_Handler();
	}

	// Start the timers
	HAL_TIM_Base_Start_IT(&htim4);		// ADC sampling
	HAL_TIM_Base_Start_IT(&htim6);		// rs422 timing
	HAL_TIM_Base_Start_IT(&htim7);		// xbee timing
	HAL_TIM_Base_Start_IT(&htim10);		// motor control timing
	HAL_TIM_Base_Start_IT(&htim2);		// microsecond tick

	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); 	// mtr0
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);	// mtr1
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);	// mtr2
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);	// mtr3

	// Release all SPI devices
	release_device(adc0);
	release_device(adc1);
	release_device(adc2);
	release_device(adc3);
	release_device(adc4);
	release_device(sram);
	release_device(flash);
	release_device(tc0);
	release_device(tc1);
	release_device(tc2);
	release_device(tc3);
	release_device(spirit);		// Technically not a SPI device

	configure_devices();

	// Start receiving data
	select_device(spirit);

	buffer_init(&spirit_buf, spirit_data_buf, UART_BUFFER_SIZE, spirit);
	buffer_init(&rs422_buf, rs422_data_buf, UART_BUFFER_SIZE, rs422);
	buffer_init(&xbee_buf, xbee_data_buf, UART_BUFFER_SIZE, xbee);

	HAL_UART_Receive_IT(&spirit_com, &spirit_in, 1);
	HAL_UART_Receive_IT(&rs422_com, &rs422_in, 1);
	HAL_UART_Receive_IT(&xbee_com, &xbee_in, 1);
	//

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


//while(1){
//	read_adc_brute();
//	scale_readings();
//	TIME(adc_cycle_time);
//	send_telem(rs422_com, full_pretty);
//	HAL_Delay(100);
//}



// Auto test
//autos[0].device[0] = led0;
//autos[0].command[0] = 1;
//autos[0].device[1] = delay;
//autos[0].command[1] = 250;
//autos[0].device[2] = led0;
//autos[0].command[2] = 0;
//autos[0].device[3] = delay;
//autos[0].command[3] = 1000;
//autos[0].length = 3;
//
//start_auto(0);

//strcpy(autos[0].command[0], "command led0 1\r");
//strcpy(autos[0].command[1], "delay 250\r");
//strcpy(autos[0].command[2], "command led0 0\r");
//strcpy(autos[0].command[3], "delay 500\r");
//autos[0].length = 3;


  while(1){
	  HAL_Delay(100);
	  read_adc_brute();
	  scale_readings();
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

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
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
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  htim2.Init.Prescaler = 90;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 900;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim5);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 900;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 900;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 4;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 32768;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 32768;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim9);

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 180;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 4000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
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
  huart6.Init.BaudRate = 921600;
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

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  HAL_GPIO_WritePin(GPIOE, ADC0_CS_Pin|ADC2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, vlv7_Pin|vlv6_Pin|vlv5_Pin|vlv4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, vlv3_Pin|vlv2_Pin|vlv1_Pin|vlv0_Pin 
                          |ADC1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MEM_CS_Pin|EEPROM_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, led0_Pin|led1_Pin|led2_Pin|led3_Pin 
                          |ina_mtr0_Pin|inb_mtr0_Pin|ina_mtr1_Pin|inb_mtr1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ADC0_CS_Pin ADC2_CS_Pin */
  GPIO_InitStruct.Pin = ADC0_CS_Pin|ADC2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : vlv7_Pin vlv6_Pin vlv5_Pin vlv4_Pin */
  GPIO_InitStruct.Pin = vlv7_Pin|vlv6_Pin|vlv5_Pin|vlv4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : vlv3_Pin vlv2_Pin vlv1_Pin vlv0_Pin 
                           ADC1_CS_Pin */
  GPIO_InitStruct.Pin = vlv3_Pin|vlv2_Pin|vlv1_Pin|vlv0_Pin 
                          |ADC1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MEM_CS_Pin EEPROM_CS_Pin */
  GPIO_InitStruct.Pin = MEM_CS_Pin|EEPROM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : led0_Pin led1_Pin led2_Pin led3_Pin 
                           ina_mtr0_Pin inb_mtr0_Pin ina_mtr1_Pin inb_mtr1_Pin */
  GPIO_InitStruct.Pin = led0_Pin|led1_Pin|led2_Pin|led3_Pin 
                          |ina_mtr0_Pin|inb_mtr0_Pin|ina_mtr1_Pin|inb_mtr1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// Functions

// High Level
void command(uint8_t device, int16_t command_value){

	// VALVE CHANNELS ///////////////////////////////////////////////////////////////////
	if((device < 40) && (device >= 20)){ // It is a valve channel (or led)

		// If else will default to a de-energized state if a bad command is sent.
		GPIO_PinState GPIO_COMMAND;

		uint16_t mask = 1;
		mask = mask << (device - vlv0);

		if(command_value == 1){
			GPIO_COMMAND = GPIO_PIN_SET;
			valve_states |= mask;
		}
		else{
			GPIO_COMMAND = GPIO_PIN_RESET;
			mask ^= 0xFFFF;
			valve_states &= mask;
		}
		switch(device){
			case vlv0:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_COMMAND);
				break;
			case vlv1:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_COMMAND);
				break;
			case vlv2:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_COMMAND);
				break;
			case vlv3:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_COMMAND);
				break;
			case vlv4:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_COMMAND);
				break;
			case vlv5:
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_COMMAND);
				break;
			case vlv6:
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_COMMAND);
				break;
			case vlv7:
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_COMMAND);
				break;
			case vlv8:
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_COMMAND);
				break;
			case vlv9:
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_COMMAND);
				break;
			case vlv10:
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_COMMAND);
				break;
			case vlv11:
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_COMMAND);
				break;
			case vlv12:
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_COMMAND);
				break;
			case vlv13:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_COMMAND);
				break;
			case vlv14:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_COMMAND);
				break;
			case vlv15:
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_COMMAND);
				break;

			case led0:
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_COMMAND);
				break;

		}


	} // END VALVES

	// BEGIN MOTORS
	else if( (device >= mtr0) && (device <= mtr3) ){

		motor_setpoint[device-mtr0] = command_value;
		//writeMotor(device, command_value); 		// DEBUGGING ONLY - DELETE EVENTUALLY

	}	// END MOTORS

}
// General Utilities
void select_device(uint8_t device){
	switch(device){
		case adc0:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
			break;
		case adc1:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
			break;
		case adc2:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
			break;
		case adc3:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
			break;
		case adc4:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
			break;

		case sram:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
			break;
		case flash:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
			break;

		case tc0:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
			break;
		case tc1:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
			break;
		case tc2:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			break;
		case tc3:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
			break;

		case spirit:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			break;

		default:
			break;
	}
}
void release_device(uint8_t device){
	switch(device){
		case adc0:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case adc1:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
			break;
		case adc2:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
			break;
		case adc3:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
			break;
		case adc4:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
			break;

		case sram:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
			break;
		case flash:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
			break;

		case tc0:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
			break;
		case tc1:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
			break;
		case tc2:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			break;
		case tc3:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
			break;

		case spirit:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
			break;

		default:
			break;
	}
}
// In progress
void read_adc_brute(){

	// Temp Buffers
	uint8_t tx[2];
	uint8_t rx[2];

	 __disable_irq();

	for(uint8_t adcn = adc0; adcn <= adc4; adcn++){

		// Select channel 0 to start
		tx[0] = 0;
		tx[1] = 0b0000100;
		select_device(adcn);
		if(HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 1) ==  HAL_TIMEOUT){
			//count3++;
		}
		release_device(adcn);

		for(uint8_t channel = 1; channel <= 16; channel++){

			tx[0] = (channel >> 1) | 0b00001000;
			tx[1] = (channel << 7) | 0b00000100;
			select_device(adcn);
			if(HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 1) == HAL_TIMEOUT){

			}

			release_device(adcn);
			adc_data[adcn-adc0][channel-1] = ((rx[1])|(rx[0] << 8)) & 0x0FFF;

		}
	}
	 __enable_irq();
}
// Debug functions
void print_adc_raw(UART_HandleTypeDef device){

	uint8_t line[255];

	snprintf(line, sizeof(line), "\f\r\nChannel\tADC0\tADC1\tADC2\tADC3\tADC4\r\n", adc_data);
	while(HAL_UART_GetState(&device) == HAL_UART_STATE_BUSY_TX);
	HAL_UART_Transmit(&device, (uint8_t*)line, strlen(line), 1);

	for(uint8_t channel = 0; channel <= 15; channel++){
		HAL_Delay(40);
		snprintf(line, sizeof(line), "%d:\t%d\t%d\t%d\t%d\t%d\t\r\n", channel, adc_data[0][channel], adc_data[1][channel], adc_data[2][channel], adc_data[3][channel], adc_data[4][channel]);
		while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
		HAL_UART_Transmit(&device, (uint8_t*)line, strlen(line), 1);
	}


}
uint8_t configure_devices(){

	uint8_t config_succcess = 0;

	if(0){

		uint8_t line[32];
		snprintf(line, sizeof(line), "+++");
		while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
		HAL_UART_Transmit(&spirit_com, (uint8_t*)line, strlen(line), 1);

		HAL_Delay(50);

		snprintf(line, sizeof(line), "ATS02=115200\r\n");
		while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
		HAL_UART_Transmit(&spirit_com, (uint8_t*)line, strlen(line), 1);

		HAL_Delay(50);

		snprintf(line, sizeof(line), "ATO\r\n");
		while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
		HAL_UART_Transmit(&spirit_com, (uint8_t*)line, strlen(line), 1);

		//trace_printf("Spirit_config done\r\n");

	}

	if(0){

			uint8_t tx[2];
			uint8_t rx[2];

			for(uint8_t adcn = adc0; adcn <= adc4; adcn++){

				// Select channel 0 to start
				tx[0] = 0;
				tx[1] = 0;
				select_device(adcn);
				HAL_SPI_TransmitReceive_IT(&hspi1, tx, rx, 2);
				release_device(adcn);

			}

	}

	return config_succcess;

}

void send_telem(UART_HandleTypeDef device, uint8_t format){

	uint8_t line[2048];

	switch(format){
		case gui_v1:
		{

			uint16_t mask = 1;
			uint8_t valve_state[16];
			for(int n = 0; n < 16; n++){

				// Extract the valve state
				if(mask & valve_states){
					valve_state[n] = 1;
				}
				else{
					valve_state[n] = 0;
				}
				mask <<= 1;
			}


			snprintf(line, sizeof(line), "%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%.2f,%.2f,%u,%u,%u,%u,%.2f,%.2f,%u,%.3f,%.3f,%.3f,%.2f,%.2f,%d,%d,%u,%u,%u,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.0f,%.0f,%.0f,%.0f,%d,%s,\r\n",valve_states,pressure[0],pressure[1],pressure[2],pressure[3],pressure[4],pressure[5],pressure[6],pressure[7],samplerate,motor_setpoint[0],motor_setpoint[1],main_cycle_time[0],motor_cycle_time[0],adc_cycle_time[0],telemetry_cycle_time[0],ebatt,ibus,telemetry_rate[0],motor_control_gain[0],motor_control_gain[1],motor_control_gain[2],motor_position[0],motor_position[1],motor_pwm[0],motor_pwm[1],count1,count2,count3,STATE,load[0],load[1],load[2],load[3],thrust_load,thermocouple[0],thermocouple[1],thermocouple[2],thermocouple[3],auto_states,AUTOSTRING);


			//while(HAL_UART_GetState(&device) == HAL_UART_STATE_BUSY_TX);
			HAL_UART_Transmit(&device, (uint8_t*)line, strlen(line), 1);

			strcpy(AUTOSTRING, "0");	// Only send it once every time it is modified
		}
			break;

		case gui_byte_packet:
			command(led0, 1);
			pack_telem(telem_unstuffed);
			stuff_telem(telem_unstuffed, telem_stuffed);
			HAL_UART_Transmit(&device, (uint8_t*)telem_stuffed, PACKET_SIZE+2, 1);
			command(led0, 0);
			break;

		case small_pretty:
		{
			snprintf(line, sizeof(line), "\fBatt: %.2f volts\r\n", ebatt);
			while(HAL_UART_GetState(&device) == HAL_UART_STATE_BUSY_TX);
			HAL_UART_Transmit(&device, (uint8_t*)line, strlen(line), 1);
			break;
		}
		case full_pretty:
		{
			snprintf(line, sizeof(line), "\fSTATE: %s\r\n"
					"Vlv:\tVolts   Amps  State ||| Pressures\r\n", states[STATE]);
			while(HAL_UART_GetState(&device) == HAL_UART_STATE_BUSY_TX);
			HAL_UART_Transmit(&device, (uint8_t*)line, strlen(line), 1);

			uint16_t mask = 1;
			uint8_t one_state;
			for(int n = 0; n < 16; n++){

				// Extract the valve state
				if(mask & valve_states){
					one_state = 1;
				}
				else{
					one_state = 0;
				}
				mask <<= 1;

				snprintf(line, sizeof(line), "%d:\t%.2fV\t%.2fA\t%d   ||| %d\t%.2f\r\n", n, evlv[n], ivlv[n], one_state, n, pressure[n]);
				while(HAL_UART_GetState(&device) == HAL_UART_STATE_BUSY_TX);
				HAL_UART_Transmit(&device, (uint8_t*)line, strlen(line), 1);
			}
			snprintf(line, sizeof(line), "Motor Data:\tSet\tPos\tPWM\tKp\tKi\tKd\r\n");
			while(HAL_UART_GetState(&device) == HAL_UART_STATE_BUSY_TX);
			HAL_UART_Transmit(&device, (uint8_t*)line, strlen(line), 1);
			for(int n = 0; n <= 3; n++){
				snprintf(line, sizeof(line), "mtr%d:\t\t%.2f\t%.2f\t%d\t%.3f\t%.3f\t%.3f\r\n",
						n, motor_setpoint, motor_position[n], motor_pwm[n], motor_control_gain[Kp],
						motor_control_gain[Ki], motor_control_gain[Kd]);
				while(HAL_UART_GetState(&device) == HAL_UART_STATE_BUSY_TX);
				HAL_UART_Transmit(&device, (uint8_t*)line, strlen(line), 1);
			}

			snprintf(line, sizeof(line), "Board Telemtry\r\n"
					"12V:\t%.2fv\t5v:\t%.2f\t3.3V:\t%.2f\r\n"
					"Tbrd:\t%.1f\tTvlv:\t%.f\tTmtr:\t%.1f\r\n"
					"Count 1:\t%d\r\n"
					"Count 2:\t%d\r\n"
					"Count 3:\t%d\r\n"
					"Command Line:\r\n",
					ebatt, e5v, e3v, tbrd, tbrd, tbrd, count1, count2, count3);
			while(HAL_UART_GetState(&device) == HAL_UART_STATE_BUSY_TX);
			HAL_UART_Transmit(&device, (uint8_t*)line, strlen(line), 1);

			// Print command buffer
			for(uint8_t n = 0; n < COMMAND_HISTORY; n++){
				snprintf(line, sizeof(line), "%s\r\n", command_buffer[n]);
				while(HAL_UART_GetState(&device) == HAL_UART_STATE_BUSY_TX);
				HAL_UART_Transmit(&device, (uint8_t*)line, strlen(line), 1);
			}
		}
			break;
		default:
			break;
	}

}
void assemble_telem(){

	telem_unstuffed[0] = valve_states >> 8;
	telem_unstuffed[1] = valve_states & 0x00ff;
	telem_unstuffed[2] = STATE;

	for(int p = 0; p < 16; p++){
		telem_unstuffed[2+2*p-1] = ((uint16_t) pressure[p]) >> 8;
		telem_unstuffed[2+2*p] = ((uint16_t) pressure[p]) & 0x00ff;
	}

	// Stuff the data here
	// HERE "stuff_data(blah blah);

	unstuffed_packet_length = 32;

}
void buffer_init(struct buffer *b, uint8_t* data_buffer, size_t size, uint8_t id){
	b->start = data_buffer;
	b->end = (data_buffer)+size;
	b->head = b->start;
	b->tail = b->start;
	b->length = b->end - b->start;
	b->filled = 0;
	b->id = id;
	b->new_data = 0;

	for(uint16_t n = 0; n < b->length; n++){
		data_buffer[n] = 0;
	}
}
void buffer_read(struct buffer *b, uint8_t* dst, size_t size){
	assert(size<=b->filled);
	for(uint16_t n = 0; n < size; n++){
		*(dst++) = *(b->tail++);
		if(b->tail > b-> end){
			b->tail = b->start;
		}
		b->filled--;
	}
}
void buffer_write(struct buffer *b, uint8_t* src, size_t size){
	for(uint16_t n = 0; n < size; n++){	// How even many bytes there are to write (Usually just 1)
		*(b->head++) = *(src++);		// Write a byte
		if(b->head > b-> end){			// Wrap around if we went past max address
			b->head = b->start;			// Set the head to the start
		}
		if(b->head==b->tail){			// Head has hit the tail
			b->tail++;					// Inc tail
			if(b->tail > b->end){		// Did the tail hit the max addr
				b->tail = b->start;		// Yep, send it to the start
			}
		}
		else{
			b->filled++;				// No hits, just incr filled
		}
	}
	b->new_data++;					// We now need to run parse_buffer at some point
}
void parse_buffer(struct buffer *b){

	for(uint8_t* data_ptr = (b->head)-(b->new_data); data_ptr < b->head; data_ptr++){

		if(b->id == COMMAND_SOURCE){									// Is this the command source
			for(uint8_t n = 0; n < COMMAND_BUFFER_LENGTH; n ++){		// Copy the buffer into the command buffer then
				command_buffer[command_index][n] = rs422_data_buf[n];	//
			}

			if(*(data_ptr) == '\r'){											// Got a return char, run the command and clear the command buffer
				if(command_index < COMMAND_HISTORY-1){					// Register the command
					command_index++;
					serial_command(command_buffer[command_index-1]);	// Copy into command history buffers
				}
				else{													// Command history full, shift all the buffers
					serial_command(command_buffer[command_index]);
					// Shift all the lines up 1, (new command line)
					for(uint8_t n = 0; n < COMMAND_HISTORY-1; n++){
						strcpy(command_buffer[n], command_buffer[n+1]);
					}
				}
				// Clear the new line of the command buffer
				for(uint8_t n = 0; n < 64; n ++){
					command_buffer[command_index][n] = 0;
				}
				// Clear the serial buffer on the commanding channel
				buffer_init(&rs422_buf, rs422_data_buf, UART_BUFFER_SIZE, rs422);
			}
		}
	}
	b->new_data = 0;		// We parsed all new data
}
void scale_readings(){

	for(uint8_t adcn = 0; adcn < 2; adcn++){
		for(uint8_t n = 0; n < 8; n++){
			evlv[7-n+(8*adcn)] = (adc_data[adcn][(2*n)+1])*evlv_cal;
			ivlv[7-n+(8*adcn)] = (adc_data[adcn][(2*n)])*ivlv_cal;
		}
	}

	ebatt = (adc_data[2][0])*ebatt_cal;
	ibus = (adc_data[2][1])*ibus_cal;
	e5v = (adc_data[2][2])*e5v_cal;
	e3v = (adc_data[2][3])*e3v_cal;
	float e3v_correction_factor = 3.300/e3v;

	for(uint8_t n = 0; n <= 3; n++){
		motor_position[n] = adc_data[2][12+n]*motor_pot_slope[n];
		motor_position[n] *= e3v_correction_factor;
		motor_position[n] -= motor_pot_offset[n];
	}

	tbrd = (adc_data[2][5])/1.24;
	tbrd -= 600;

	for(uint8_t n = 0; n < 16; n ++){
		pressure[n] = adc_data[4][15-n]-press_cal[OFFSET][n];
		pressure[n] *= press_cal[SLOPE][n];
	}



	load[0] = adc_data[3][15];
	load[1] = adc_data[3][14];
	load[2] = adc_data[3][13];
	load[3] = adc_data[3][12];
	for(uint8_t n = 0; n < 6; n++){
		load[n] -= load_cal[OFFSET][n];
		load[n] *= load_cal[SLOPE][n];
	}

	thrust_load = load[0]+load[1]+load[2]+load[3];


}
uint32_t  serial_command(uint8_t* cbuf_in){

	char cbuf[COMMAND_BUFFER_LENGTH];
	strcpy(cbuf, cbuf_in);
	char argv[MAX_COMMAND_ARGS][MAX_COMMAND_LENGTH];
	const char s[2] = " ";
	char *token;
	token = strtok(cbuf, s);
	int argc = 0;
	while( token != NULL ){
		strcpy(argv[argc++], token);
		token = strtok(NULL, s);
	}

	if(strcmp(argv[0], "save_auto") == 0){
		autos[LOG_TO_AUTO].current_index = 0;
		print_auto(LOG_TO_AUTO);		// Automatically print it when you finish to verify
		LOG_TO_AUTO = -1;
	}

	if(LOG_TO_AUTO != -1){
		// We are logging to auto index contained in LOG_TO_AUTO
		if(strlen(cbuf_in) <= AUTO_STRING_LENGTH){
			strcpy(autos[LOG_TO_AUTO].command[autos[LOG_TO_AUTO].current_index], cbuf_in);
		}
		else{
			strcpy(autos[LOG_TO_AUTO].command[autos[LOG_TO_AUTO].current_index], "ERROR, COMMAND TO LONG");
		}
		autos[LOG_TO_AUTO].length++;
		autos[LOG_TO_AUTO].current_index++;
		return 0;
	}

	if((strcmp(argv[0], "delay") == 0)){
		return atoi(argv[1]);
	}

	// "command"
	if((strcmp(argv[0], "command") == 0)){
		uint8_t device;
		// Get the device id
		for(device = 0; device < 100; device++){
			if(strcmp(argv[1], device_alias[device]) == 0) break;
		}
		int command_value = atoi(argv[2]);

		command(device, command_value);

	}
	// END "command"

	// "clear"
	// Clears the command history
	else if(strcmp(argv[0], "clear") == 0){
		for(uint8_t n = 0; n < COMMAND_HISTORY; n++){
			for(uint16_t m = 0; m < COMMAND_BUFFER_LENGTH; m++){
				command_buffer[n][m] = 0;
			}
		}
		command_index = 0;
	} // END clear

	// arm
	if((strcmp(argv[0], "arm") == 0)){
		if(STATE == MANUAL){
			STATE = ARMED;
			//command(mtr0, 0);
			//command(mtr1, 0);
			serial_command("enable mtr0 \r");
			serial_command("enable mtr1 \r");
		}
	} // END arm

	// disarm
	if((strcmp(argv[0], "disarm") == 0)){

		STATE = MANUAL;
		serial_command("disable mtr0 \r");
		serial_command("disable mtr1 \r");

	}	// END disarm

	// hotfire
	if((strcmp(argv[0], "hotfire") == 0)){
		if(STATE == ARMED){
			STATE = PRE_IGNITION;
			state_timer = micros;
			start_auto(&hotfire_auto);
		}
	}	// END hotfire

	// set
	else if(strcmp(argv[0], "set") == 0){
		if(strcmp(argv[1], "telemformat") == 0){

			uint8_t temp_format;

			// Get the format
			if(strcmp(argv[3], "gui_1") == 0){
				temp_format = gui_v1;
			}
			else if(strcmp(argv[3], "full_pretty") == 0){
				temp_format = full_pretty;
			}
			else if(strcmp(argv[3], "none") == 0){
				temp_format = none;
			}

			// Set the format
			if(strcmp(argv[2], "xbee") == 0){
				telemetry_format[xbee] = temp_format;
			}
			else if(strcmp(argv[2], "rs422") == 0){
				telemetry_format[rs422] = temp_format;
			}
			else if(strcmp(argv[2], "spirit") == 0){
				telemetry_format[spirit] = temp_format;
			}
		}	// end telemformat
		if(strcmp(argv[1], "telemrate") == 0){

			uint16_t rate = atoi(argv[3]);

			// Set the format
			if(strcmp(argv[2], "xbee") == 0){
				telemetry_rate[xbee] = rate;
			}
			else if(strcmp(argv[2], "rs422") == 0){

				uint16_t period = 100000/rate;

				HAL_TIM_Base_Stop_IT(&htim6);

				htim6.Instance = TIM6;
				htim6.Init.Prescaler = 900;
				htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
				htim6.Init.Period = period;
				htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
				HAL_TIM_Base_Init(&htim6);

				HAL_TIM_Base_Start_IT(&htim6);

				telemetry_rate[rs422] = (100000/period);	// Actual rate
			}
			else if(strcmp(argv[2], "spirit") == 0){
				telemetry_rate[spirit] = rate;
			}
		}	// end telemformat
		if(strcmp(argv[1], "samplerate") == 0){

			samplerate = atoi(argv[2]);
			uint16_t period = 100000/samplerate;

			HAL_TIM_Base_Stop_IT(&htim4);

			htim4.Instance = TIM4;
			htim4.Init.Prescaler = 900;
			htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
			htim4.Init.Period = period;
			htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
			HAL_TIM_Base_Init(&htim4);

			HAL_TIM_Base_Start_IT(&htim4);

			samplerate = (100000/period);	// Actual rate

		}	// End set samplerate
		else if(strcmp(argv[1], "gain") == 0){
			uint8_t gain = atoi(argv[2]);
			char* end;
			float val = strtof(argv[3], &end);

			motor_control_gain[gain] = val;
		}
		else if(strcmp(argv[1], "offset") == 0){
			press_cal[OFFSET][atoi(argv[2])] = atoi(argv[3]);
		}
		else if(strcmp(argv[1], "slope") == 0){
			char* end;
			press_cal[SLOPE][atoi(argv[2])] = strtof(argv[3], &end);
		}
		else if(strcmp(argv[1], "pot_offset") == 0){
			char* end;
			motor_pot_offset[atoi(argv[2])] = strtof(argv[3], &end);
		}
		else if(strcmp(argv[1], "burn_duration") == 0){
			FIRING_DURATION = atoi(argv[2]);
		}
		else if(strcmp(argv[1], "ignition_duration") == 0){
			IGNITION_DURATION = atoi(argv[2]);
		}
		else if(strcmp(argv[1], "post_ignite_delay") == 0){
			POST_IGNITE_DELAY = atoi(argv[2]);
		}

	}	// End "set"
	else if(strcmp(argv[0], "ambientize") == 0){
		//for(uint8_t n = 0; n < 16; n++){
			press_cal[OFFSET][atoi(argv[1])] = adc_data[4][15-atoi(argv[1])];
		//}
	}
	else if(strcmp(argv[0], "enable") == 0){
		// Get the device id
		for(uint8_t n = mtr0; n <= mtr3; n++){
			if(strcmp(argv[1], device_alias[n]) == 0){
				motor_active[n-mtr0] = 1;
				motor_accumulated_error[n-mtr0] = 0;
			}
		}
	}
	else if(strcmp(argv[0], "disable") == 0){
		// Get the device id
		for(uint8_t n = mtr0; n <= mtr3; n++){
			if(strcmp(argv[1], device_alias[n]) == 0){
				motor_active[n-mtr0] = 0;
			}
		}
	}
	else if(strcmp(argv[0], "log") == 0){
		if((strcmp(argv[1], "start") == 0)){
			LOGGING_ACTIVE = 1;
			// Open a csv file and print the header
			// put code here
		}
		else if(strcmp(argv[0], "stop") == 0){
			LOGGING_ACTIVE = 0;
		}
	}
	else if(strcmp(argv[0], "tare") == 0){
		load_cal[OFFSET][0] = adc_data[3][15];
		load_cal[OFFSET][1] = adc_data[3][14];
		load_cal[OFFSET][2] = adc_data[3][13];
		load_cal[OFFSET][3] = adc_data[3][12];
	}
	else if(strcmp(argv[0], "new_auto") == 0){
		LOG_TO_AUTO = atoi(argv[1]);
	}
	else if(strcmp(argv[0], "start_auto") == 0){
		start_auto(&autos[atoi(argv[1])]);
	}
	else if(strcmp(argv[0], "stop_auto") == 0){
		if(strcmp(argv[1], "hotfire_auto") == 0){
			count1++;
			stop_auto(&hotfire_auto);
		}
		else{
			stop_auto(&autos[atoi(argv[1])]);
		}
	}
	else if(strcmp(argv[0], "kill_auto") == 0){
		if(strcmp(argv[1], "hotfire_auto") == 0){
			kill_auto(&hotfire_auto);
		}
		else{
			kill_auto(&autos[atoi(argv[1])]);
		}
	}
	else if(strcmp(argv[0], "print_auto") == 0){
		print_auto(&autos[atoi(argv[1])]);
	}

	// QD
	else if(strcmp(argv[0], "connect") == 0){
		if(strcmp(argv[1], "ox") == 0){
			writeMotor(mtr2, -16000);
			qd_stop_time = millis + QD_ACTUATION_TIME;
		}
		else if(strcmp(argv[1], "fuel") == 0){
			writeMotor(mtr3, -16000);
			qd_stop_time = millis + QD_ACTUATION_TIME;
		}
	}
	else if(strcmp(argv[0], "release") == 0){
		if(strcmp(argv[1], "ox") == 0){
			writeMotor(mtr2, 16000);
			qd_stop_time = millis + QD_ACTUATION_TIME;
		}
		else if(strcmp(argv[1], "fuel") == 0){
			writeMotor(mtr3, 16000);
			qd_stop_time = millis + QD_ACTUATION_TIME;
		}
	}


	else{
		// Invalid command
		cbuf[strlen(cbuf)] = " - INVALID COMMAND";
	}

	return 0;
}
void setpwm(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse)
{
 HAL_TIM_PWM_Stop(&timer, channel); // stop generation of pwm
 TIM_OC_InitTypeDef sConfigOC;
 timer.Init.Period = period; // set the period duration
 HAL_TIM_PWM_Init(&timer); // reinititialise with new period value

 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = pulse; // set the pulse duration
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);

 HAL_TIM_PWM_Start(&timer, channel); // start pwm generation
}
void error(char * error_message){

}


read_thermocouples(){

	__disable_irq();
	uint8_t tx[4];
	uint8_t rx[4];

	for(uint8_t tcx = tc0; tcx <= tc3; tcx++){

		select_device(tcx);
		HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4, 1);
		release_device(tcx);

		int16_t data;
		data = rx[0];
		data <<= 8;
		data |= rx[1];
		data >>= 4;
		thermocouple[tcx-tc0] = data;

	}
	__enable_irq();
}
void run_auto(struct autosequence *a){
	if(a->running == 1){
		if(millis > a->next_exec){

			if(a->current_index == a->length){
				a->current_index = 0;
			}
			else{
				a->last_exec = millis;
				a->next_exec = millis + serial_command(a->command[a->current_index]);
				a->current_index++;
			}
		}
	}
}
void start_auto(struct autosequence *a){

	a->current_index = 0;
	a->running = 1;
	a->next_exec = millis;	// Execute the first command as soon as run_autos is called

	// State feedback to show the auto is running
	uint16_t mask = 1 << a->number;
	auto_states |= mask;

}
void stop_auto(struct autosequence *a){
	// This will eventually be a more graceful version of kill
	// ... so it dosent leave things in a weird state
	// ... TODO
	kill_auto(a);
}
void kill_auto(struct autosequence *a){
	a->running = 0;
	// State feedbck to show the auto is stopped
	uint16_t mask = 1 << a->number;
	auto_states ^= mask;
}
void print_auto(struct autosequence *a){
	AUTOSTRING[0] = '\0';
	//snprintf(AUTOSTRING, sizeof(AUTOSTRING), "Device\tCommand\n");
	for(uint16_t n = 0; n < a->length; n++){		// Recursivley generate the autostring
		uint8_t stripped_string [AUTO_STRING_LENGTH];
		strcpy(stripped_string, a->command[n]);
		strtok(stripped_string, "\r\n");
		strcat(stripped_string, "|");
		strcat(AUTOSTRING, stripped_string);
	}
}
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
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
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
