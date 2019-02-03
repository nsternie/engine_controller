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

extern IWDG_HandleTypeDef hiwdg;

extern UART_HandleTypeDef huart1;

// Hardware wrappers
void read_adc(SPI_HandleTypeDef* SPI_BUS){

	// Temp Buffers
	uint8_t tx[2];
	uint8_t rx[2];

	 __disable_irq();

	for(uint8_t adcn = adc0; adcn <= adc2; adcn++){

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
	}
	 __enable_irq();
}
void set_device(uint8_t device, GPIO_PinState state){
	switch(device){
		case adc0:
			HAL_GPIO_WritePin(ADC0_CS_GPIO_Port, ADC0_CS_Pin, state);
			break;
		case adc1:
			HAL_GPIO_WritePin(ADC1_CS_GPIO_Port, ADC1_CS_Pin, state);
			break;
		case adc2:
			HAL_GPIO_WritePin(ADC2_CS_GPIO_Port, ADC2_CS_Pin, state);
			break;

		case sram:
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
			break;
		case flash:
			HAL_GPIO_WritePin(MEM_CS_GPIO_Port, MEM_CS_Pin, GPIO_PIN_RESET);
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

	for(uint8_t adcn = 0; adcn < 2; adcn++){
		for(uint8_t n = 0; n < 8; n++){
			evlv[7-n+(8*adcn)] = (adc_data[adcn][(2*n)+1])*evlv_cal;
			ivlv[7-n+(8*adcn)] = (adc_data[adcn][(2*n)])*ivlv_cal;
		}
	}

	imtr[0] = adc_data[2][8] * imtr_cal;
	imtr[1] = adc_data[2][9] * imtr_cal;


	ebatt = (adc_data[2][0])*ebatt_cal;
	ibus = (adc_data[2][1])*ibus_cal;
	e5v = (adc_data[2][2])*e5v_cal;
	e3v = (adc_data[2][3])*e3v_cal;
	float e3v_correction_factor = 3.300/e3v;

#define num_avg 3
	static float last_position[4];
	static int index = 0;
	for(uint8_t n = 0; n <= 3; n++){
		last_position[n] = motor_position[n];
		motor_position[n] = adc_data[2][12+n]*motor_pot_slope[n];
		motor_position[n] *= e3v_correction_factor;
		motor_position[n] -= motor_pot_offset[n];

		if(n == 0){
			if(motor_position[n] > 320 || motor_position < 50){
				motor_position[n] = last_position[n];
			}
		}
//		last_position[n][index] = motor_position[n];
//		float sum = 0;
//		for(int m = 0; m < num_avg; m++){
//			sum += last_position[n][m];
//		}
//		motor_position[n] = (sum)/num_avg;
	}


	tbrd = (adc_data[2][5])/1.24;
	tbrd -= 600;
	tbrd /= 10;
	tvlv = (adc_data[2][6])/1.24;
	tvlv -= 600;
	tvlv /= 10;
	tmtr = (adc_data[2][7])/1.24;
	tmtr -= 600;
	tmtr /= 10;



	for(uint8_t n = 0; n < 16; n ++){
		pressure[n] = adc_data[1][15-n]-press_cal[OFFSET][n];
		pressure[n] *= press_cal[SLOPE][n];
	}



//	load[0] = adc_data[3][15];
//	load[1] = adc_data[3][14];
//	load[2] = adc_data[3][13];
//	load[3] = adc_data[3][12];
//	for(uint8_t n = 0; n < 6; n++){
//		load[n] -= load_cal[OFFSET][n];
//		load[n] *= load_cal[SLOPE][n];
//	}
//	thrust_load = load[0]+load[1]+load[2]+load[3];


}
void send_telem(UART_HandleTypeDef device, uint8_t format){
	switch(format){
		case gui_byte_packet:
			command(led0, 1);
			HAL_UART_Transmit(&device, (uint8_t*)telem_stuffed, PACKET_SIZE+2, 100);
			command(led0, 0);
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
			case vlv0:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_COMMAND);
				break;
			case vlv1:
				HAL_GPIO_WritePin(vlv1_GPIO_Port, vlv1_Pin, GPIO_COMMAND);
				break;
			case vlv2:
				HAL_GPIO_WritePin(vlv2_GPIO_Port, vlv2_Pin, GPIO_COMMAND);
				break;
			case vlv3:
				HAL_GPIO_WritePin(vlv3_GPIO_Port, vlv3_Pin, GPIO_COMMAND);
				break;
			case vlv4:
				HAL_GPIO_WritePin(vlv4_GPIO_Port, vlv4_Pin, GPIO_COMMAND);
				break;
			case vlv5:
				HAL_GPIO_WritePin(vlv5_GPIO_Port, vlv5_Pin, GPIO_COMMAND);
				break;
			case vlv6:
				HAL_GPIO_WritePin(vlv6_GPIO_Port, vlv6_Pin, GPIO_COMMAND);
				break;
			case vlv7:
				HAL_GPIO_WritePin(vlv7_GPIO_Port, vlv7_Pin, GPIO_COMMAND);
				break;

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

		}


	} // END VALVES

	// BEGIN MOTORS
	else if( (device >= mtr0) && (device <= mtr1) ){

		motor_setpoint[device-mtr0] = command_value;
		//writeMotor(device, command_value); 		// DEBUGGING ONLY - DELETE EVENTUALLY

	}	// END MOTORS

}
void writeMotor(uint8_t device, int16_t motor_command){

	TIM_HandleTypeDef timer;

	uint8_t dir = (motor_command >=0) ? 1 : 0;

	switch(device){
	case mtr0:
		timer = htim9;

		if(dir){
			HAL_GPIO_WritePin(ina_mtr0_GPIO_Port, ina_mtr0_Pin, GPIO_PIN_SET);		// ina
			HAL_GPIO_WritePin(inb_mtr0_GPIO_Port, inb_mtr0_Pin, GPIO_PIN_RESET);	// inb
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);		// sel
		}
		else{
			HAL_GPIO_WritePin(ina_mtr0_GPIO_Port, ina_mtr0_Pin, GPIO_PIN_RESET); 	// ^^
			HAL_GPIO_WritePin(inb_mtr0_GPIO_Port, inb_mtr0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
		}

		break;

	case mtr1:

		timer = htim5;

		if(dir){
			HAL_GPIO_WritePin(ina_mtr1_GPIO_Port, ina_mtr1_Pin, GPIO_PIN_SET);		// ina
			HAL_GPIO_WritePin(inb_mtr1_GPIO_Port, inb_mtr1_Pin, GPIO_PIN_RESET);	// inb
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);		// sel
		}
		else{
			HAL_GPIO_WritePin(ina_mtr1_GPIO_Port, ina_mtr1_Pin, GPIO_PIN_RESET);	// ^^
			HAL_GPIO_WritePin(inb_mtr1_GPIO_Port, inb_mtr1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		}


		break;
	default:
		// Oops
		break;
	}

	// Config Direction

	// Set PWM value
	setpwm(timer, TIM_CHANNEL_1, 32768, abs(motor_command));
	motor_pwm[device-mtr0] = motor_command;

}
void motor_control(){

	float command_sum;

	for(uint8_t mtrx = 0; mtrx < 2; mtrx++){
		if(motor_active[mtrx]){

			float motor_error = (motor_position[mtrx] - motor_setpoint[mtrx])*pot_polarity[mtrx];


			if(motor_accumulated_error[mtrx] > I_LIMIT){
				motor_accumulated_error[mtrx] = I_LIMIT;
			}
			if(motor_accumulated_error[mtrx] < -I_LIMIT){
				motor_accumulated_error[mtrx] = -I_LIMIT;
			}
			count2 = motor_accumulated_error[mtrx];
			count1 = motor_error;
			count3 = ((motor_position[mtrx] - motor_last_position[mtrx]))*1000;

			float kp, ki, kd;
			if(mtrx == 0){
				kp = 1500;//2000
				ki = 0;//8
				kd = 200;//500
			}
			else if(mtrx == 1){
				kp = 1500;//1200
				ki = 0;
				kd = 0;
			}

			else{
				kp = motor_control_gain[0];
				ki = motor_control_gain[1];
				kd = motor_control_gain[2];
			}
//			kp = motor_control_gain[0];
//			ki = motor_control_gain[1];
//			kd = motor_control_gain[2];
			command_sum = kp * motor_error;
			if(motor_error < INTEGRATOR_ACTIVE_REGION){
				motor_accumulated_error[mtrx] += motor_error;
				command_sum += (ki * motor_accumulated_error[mtrx]);
			}
			else{
				motor_accumulated_error[mtrx] = 0;
			}
			command_sum += (kd * (motor_position[mtrx] - motor_last_position[mtrx]));

			int16_t command = 0;
			if(command_sum > 32700){
				command = 32700;
			}
			else if(command_sum < -32700){
				command = -32700;
			}
			else{
				command = command_sum;
			}

			// Softstop
			if(adc_data[2][12+mtrx] > 3900 || adc_data[2][12+mtrx] < 200){
				//command = 0;
			}

			motor_last_position[mtrx] = motor_position[mtrx];
			writeMotor(mtrx+mtr0, command);

		}
		else{
			// Motor is disabled
			writeMotor(mtrx+mtr0, 0);
		}
	}
}

// S2 commands
void led_write(int32_t argc, int32_t* argv){
	command(led0 - argv[0], argv[1]);
	__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
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
void motor_write(int32_t argc, int32_t* argv){
	motor_setpoint[argv[0]] = argv[1];
}
void motor_disable(int32_t argc, int32_t* argv){
	motor_active[argv[0]] = 0;
}
void motor_enable(int32_t argc, int32_t* argv){
	motor_active[argv[0]] = 1;
}
void arm(int32_t argc, int32_t* argv){
	STATE = ARMED;
	motor_setpoint[OX_VALVE_MOTOR] = OX_CLOSE;
	motor_setpoint[FUEL_VALVE_MOTOR] = FUEL_CLOSE;
	motor_active[0] = 1;
	motor_active[1] = 1;

}
void disarm(int32_t argc, int32_t* argv){
	STATE = MANUAL;
	motor_active[0] = 0;
	motor_active[1] = 0;
}
void main_auto_start(int32_t argc, int32_t* argv){
	if(STATE == ARMED){
		main_auto_start_time = millis;
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
	prime_bridge();
	writeMotor(mtr0 + argv[0], argv[1] * QD_ACTUATION_FORCE);
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
		__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
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
void prime_bridge_wrapper(int32_t argc, int32_t* argv){
	prime_bridge();
}
void prime_bridge(){
	int16_t temp[2];
	temp[0] = motor_pwm[0];
	temp[1] = motor_pwm[1];
	for(int n = 0; n< 2; n++){
		if(temp[n] > 30000){
			temp[n] = 30000;
		}
		if(temp[n] < -30000){
			temp[n] = -30000;
		}

	}

	writeMotor(mtr0, temp[0]+1000);
	writeMotor(mtr1, temp[1]+1000);
	HAL_Delay(5);
	writeMotor(mtr0, temp[0]+-1000);
	writeMotor(mtr1, temp[1]+-1000);
	HAL_Delay(5);
	writeMotor(mtr0, temp[0]);
	writeMotor(mtr1, temp[1]);
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

