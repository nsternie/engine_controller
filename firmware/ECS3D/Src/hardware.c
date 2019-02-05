/*
 * hardware.c
 *
 *  Created on: May 14, 2018
 *      Author: nicks
 */


#include "hardware.h"
#include "stm32f4xx_hal.h"
#include "flash.h"
#include "main_auto.h"


extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim9;

extern TIM_HandleTypeDef htim2;
#define millis	((__HAL_TIM_GET_COUNTER(&htim2))/1000)

//extern IWDG_HandleTypeDef hiwdg;

extern UART_HandleTypeDef huart1;

const GPIO_TypeDef* vlv_ports[32] = {
        vlv0_GPIO_Port,
        vlv1_GPIO_Port,
        vlv2_GPIO_Port,
        vlv3_GPIO_Port,
        vlv4_GPIO_Port,
        vlv5_GPIO_Port,
        vlv6_GPIO_Port,
        vlv7_GPIO_Port,
        vlv8_GPIO_Port,
        vlv9_GPIO_Port,
        vlv10_GPIO_Port,
        vlv11_GPIO_Port,
        vlv12_GPIO_Port,
        vlv13_GPIO_Port,
        vlv14_GPIO_Port,
        vlv15_GPIO_Port,
        vlv16_GPIO_Port,
        vlv17_GPIO_Port,
        vlv18_GPIO_Port,
        vlv19_GPIO_Port,
        vlv20_GPIO_Port,
        vlv21_GPIO_Port,
        vlv22_GPIO_Port,
        vlv23_GPIO_Port,
        vlv24_GPIO_Port,
        vlv25_GPIO_Port,
        vlv26_GPIO_Port,
        vlv27_GPIO_Port,
        vlv28_GPIO_Port,
        vlv29_GPIO_Port,
        vlv30_GPIO_Port,
        vlv31_GPIO_Port
};
const uint16_t vlv_pins[32] = {
        vlv0_Pin,
        vlv1_Pin,
        vlv2_Pin,
        vlv3_Pin,
        vlv4_Pin,
        vlv5_Pin,
        vlv6_Pin,
        vlv7_Pin,
        vlv8_Pin,
        vlv9_Pin,
        vlv10_Pin,
        vlv11_Pin,
        vlv12_Pin,
        vlv13_Pin,
        vlv14_Pin,
        vlv15_Pin,
        vlv16_Pin,
        vlv17_Pin,
        vlv18_Pin,
        vlv19_Pin,
        vlv20_Pin,
        vlv21_Pin,
        vlv22_Pin,
        vlv23_Pin,
        vlv24_Pin,
        vlv25_Pin,
        vlv26_Pin,
        vlv27_Pin,
        vlv28_Pin,
        vlv29_Pin,
        vlv30_Pin,
        vlv31_Pin
};

// Hardware wrappers
void read_adc(SPI_HandleTypeDef* SPI_BUS, uint8_t adc_number){

	// Temp Buffers
	uint8_t tx[2];
	uint8_t rx[2];

	 __disable_irq();

	uint8_t adcn = adc0 + adc_number;

	// Select channel 0 to start
	tx[0] = 0;
	tx[1] = 0b0000100;
	select_device(adcn);
	if(HAL_SPI_TransmitReceive(SPI_BUS, tx, rx, 2, 1) ==  HAL_TIMEOUT){
	}
	release_device(adcn);

	for(uint8_t channel = 1; channel <= 16; channel++){

		tx[0] = (channel >> 1) | 0b00001000;
		tx[1] = (channel << 7) | 0b00000100;
		select_device(adcn);
		if(HAL_SPI_TransmitReceive(SPI_BUS, tx, rx, 2, 1) == HAL_TIMEOUT){

		}

		release_device(adcn);
		adc_data[adcn-adc0][channel-1] = ((rx[1])|(rx[0] << 8)) & 0x0FFF;

	}

	 __enable_irq();
}
void set_device(uint8_t device, GPIO_PinState state){
	switch(device){
		case adc0:
			HAL_GPIO_WritePin(adc0_cs_GPIO_Port, adc0_cs_Pin, state);
			break;
		case adc1:
			HAL_GPIO_WritePin(adc1_cs_GPIO_Port, adc1_cs_Pin, state);
			break;
		case adc2:
			HAL_GPIO_WritePin(adc2_cs_GPIO_Port, adc2_cs_Pin, state);
			break;
		case adc3:
			HAL_GPIO_WritePin(adc3_cs_GPIO_Port, adc3_cs_Pin, state);
			break;
		case adc4:
			HAL_GPIO_WritePin(adc4_cs_GPIO_Port, adc4_cs_Pin, state);
			break;
		case adc5:
			HAL_GPIO_WritePin(adc5_cs_GPIO_Port, adc5_cs_Pin, state);
			break;
		case adc6:
			HAL_GPIO_WritePin(adc6_cs_GPIO_Port, adc6_cs_Pin, state);
			break;

		case rtd0:
			HAL_GPIO_WritePin(rtd0_GPIO_Port, rtd0_Pin, state);
			break;
		case rtd1:
			HAL_GPIO_WritePin(rtd1_GPIO_Port, rtd1_Pin, state);
			break;
		case rtd2:
			HAL_GPIO_WritePin(rtd2_GPIO_Port, rtd2_Pin, state);
			break;
		case rtd3:
			HAL_GPIO_WritePin(rtd3_GPIO_Port, rtd3_Pin, state);
			break;
		case rtd4:
			HAL_GPIO_WritePin(rtd4_GPIO_Port, rtd4_Pin, state);
			break;
		case rtd5:
			HAL_GPIO_WritePin(rtd5_GPIO_Port, rtd5_Pin, state);
			break;
		case rtd6:
			HAL_GPIO_WritePin(rtd6_GPIO_Port, rtd6_Pin, state);
			break;
		case rtd7:
			HAL_GPIO_WritePin(rtd7_GPIO_Port, rtd7_Pin, state);
			break;



		case sram:
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
			break;
		case flash:
			HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
			break;

		default:
			break;
	}
}
void select_device(uint8_t device){
	set_device(device, GPIO_PIN_RESET);
}
void release_device(uint8_t device){
	set_device(device, GPIO_PIN_SET);
}
void scale_readings(){

	for(uint8_t adcn = 0; adcn < 4; adcn++){
		for(uint8_t n = 0; n < 8; n++){
			evlv[7-n+(8*adcn)] = (adc_data[adcn][(2*n)+1])*evlv_cal;
			ivlv[7-n+(8*adcn)] = (adc_data[adcn][(2*n)])*ivlv_cal;
		}
	}

	ebatt = (adc_data[4][0])*ebatt_cal;
	ibus = (adc_data[4][1])*ibus_cal;
	e5v = (adc_data[4][2])*e5v_cal;
	e3v = (adc_data[4][3])*e3v_cal;
	i5v = (adc_data[4][9])*i5v_cal;
	i3v = (adc_data[4][8])*i3v_cal;
	float e3v_correction_factor = 3.300/e3v;

	tbrd = (adc_data[4][5])/1.24;
	tbrd -= 600;
	tbrd /= 10;
	tvlv = (adc_data[4][6])/1.24;
	tvlv -= 600;
	tvlv /= 10;
	tmtr = (adc_data[4][7])/1.24;
	tmtr -= 600;
	tmtr /= 10;

	for(uint8_t n = 0; n < 16; n ++){
		pressure[n] = adc_data[1][15-n]-press_cal[OFFSET][n];
		pressure[n] *= press_cal[SLOPE][n];
	}





	load[0] = adc_data[6][9];
	load[1] = adc_data[6][8];
	load[2] = adc_data[6][7];
	load[3] = adc_data[6][6];
	load[4] = adc_data[6][5];
	for(uint8_t n = 0; n < 5; n++){
		load[n] -= load_cal[OFFSET][n];
		load[n] *= load_cal[SLOPE][n];
	}
	thrust_load = load[0]+load[1]+load[2]+load[3]+load[4];


}
void send_telem(UART_HandleTypeDef device, uint8_t format){
	switch(format){
		case gui_byte_packet:
			command(led3, 1);
			HAL_UART_Transmit(&device, (uint8_t*)telem_stuffed, PACKET_SIZE+2, 100);
			command(led3, 0);
			break;
		default:
			break;
	}

}
void save_telem(file* f){
	log_data(f, telem_stuffed, PACKET_SIZE+2);
}
void setpwm(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse)
{

	//pulse = pulse / 2;

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

			case led0:
				HAL_GPIO_WritePin(led0_GPIO_Port, led0_Pin, GPIO_COMMAND);
				break;
			case led1:
				HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_COMMAND);
				break;
			case led2:
				HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_COMMAND);
				break;
			case led3:
				HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_COMMAND);
				break;

			default:
				if(device >= vlv0 && device <= vlv0+31){
					HAL_GPIO_WritePin(vlv_ports[device - vlv0], vlv_pins[device - vlv0], GPIO_COMMAND);\
					break;
				}


		}


	} // END VALVES

	// BEGIN MOTORS
	else if( (device >= mtr0) && (device <= mtr1) ){

		motor_setpoint[device-mtr0] = command_value;
		//writeMotor(device, command_value); 		// DEBUGGING ONLY - DELETE EVENTUALLY

	}	// END MOTORS

}

// S2 commands
void led_write(int32_t argc, int32_t* argv){
	command(led0 - argv[0], argv[1]);
//	__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
}
void digital_write(int32_t argc, int32_t* argv){
	command(vlv0 + argv[0], argv[1]);
}
void set_kp(int32_t argc, int32_t* argv){
	motor_control_gain[0] = argv[0];
}
void set_ki(int32_t argc, int32_t* argv){
	motor_control_gain[1] = argv[0];
}
void set_kd(int32_t argc, int32_t* argv){
	motor_control_gain[2] = argv[0];
}

void arm(int32_t argc, int32_t* argv){
	STATE = ARMED;


}
void disarm(int32_t argc, int32_t* argv){
	STATE = MANUAL;

}
void main_auto_start(int32_t argc, int32_t* argv){
	if(STATE == ARMED){
//		main_auto_start_time = millis;
		STATE = FIRING;
	}
}
void pwm_set(int32_t argc, int32_t* argv){

	return;	// Disabled

	if(argc != 3) return;

	TIM_HandleTypeDef timer;
	uint32_t channel;

	switch(argv[0]){
	case 5:
		timer = htim5;
		break;
	case 9:
		timer = htim9;
		break;
	default:
		break;
		return;
	}

	switch(argv[1]){
	case 1:
		channel = TIM_CHANNEL_1;
		break;
	case 2:
		channel = TIM_CHANNEL_2;
		break;
	default:
		return;
		break;
	}

	uint32_t pulse = argv[2] * 32768;
	pulse /= 100;
	uint16_t pulse_cast = (uint16_t) pulse;

	setpwm(timer, channel, 32768, pulse_cast);

}
void qd_set(int32_t argc, int32_t* argv){


}
void telemrate_set(int32_t argc, int32_t* argv){
	uint16_t period = 100000/argv[0];

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
void samplerate_set(int32_t argc, int32_t* argv){
	samplerate = argv[0];
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
}
void tare(int32_t argc, int32_t* argv){

}
void ambientize(int32_t argc, int32_t* argv){

}
void lograte_set(int32_t argc, int32_t* argv){

}
void print_file(int32_t argc, int32_t* argv){
	filesystem tempfs;
	read_filesystem(&tempfs);
	int filenum = argv[0];
	uint16_t start = tempfs.files[filenum].start_page;
	uint16_t stop = tempfs.files[filenum].stop_page;
	for(int n = start; n <= stop; n++){
		load_page(n);
		uint8_t buffer[2048];
		read_buffer(0, buffer, 2048);
		HAL_UART_Transmit(&huart1, buffer, 2048, 0xffff);
//		__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
	}
}
void numfiles(int32_t argc, int32_t* argv){
	filesystem tempfs;
	read_filesystem(&tempfs);
	uint8_t message[10];
	snprintf(message, sizeof(message), "%d\r\n", tempfs.num_files);
	HAL_UART_Transmit(&huart1, message, strlen(message), 0xffff);
}
void log_start(int32_t argc, int32_t* argv){
	if(LOGGING_ACTIVE == 0){
	  logfile = new_log();
	}
	LOGGING_ACTIVE = 1;
	command(led3, 1);
}
void log_end(int32_t argc, int32_t* argv){
	if(LOGGING_ACTIVE == 1){
		close_log(logfile);
	}
	LOGGING_ACTIVE = 0;
	command(led3, 0);
}
void init_fs(int32_t argc, int32_t* argv){

	for(int n = 0; n < 1024; n++){
		erase_block(64*n);
	  }

	filesystem fs;
	fs.current_file = -1;
	fs.next_file_page = 64;
	fs.num_files = 0;
	file blank_file;
	blank_file.bytes_free = 0;
	blank_file.current_page = 0;
	blank_file.file_number = 0;
	blank_file.start_page = 0;
	blank_file.stop_page = 0;
	for(int n = 0; n < MAX_FILES; n++){
	  fs.files[n] = blank_file;
	}
	write_filesystem(&fs);



	uint8_t message[100];
	snprintf(message, sizeof(message), "init_fs complete\r\n");
	HAL_UART_Transmit(&huart1, message, strlen(message), 0xffff);
}
void telem_pause(int32_t argc, int32_t* argv){
	TELEM_ACTIVE = 0;
}
void telem_resume(int32_t argc, int32_t* argv){
	TELEM_ACTIVE = 1;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

