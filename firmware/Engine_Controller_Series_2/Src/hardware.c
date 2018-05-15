/*
 * hardware.c
 *
 *  Created on: May 14, 2018
 *      Author: nicks
 */


#include "hardware.h"

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
				HAL_GPIO_WritePin(vlv0_GPIO_Port, vlv0_Pin, GPIO_COMMAND);
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


// High Level

/////////////////////////////////////////////////////////////////
// COMMAND THINGS FOR NOW ///////////////////////////////////////




void led_write(int32_t argc, int32_t* argv){
	command(led0 - argv[0], argv[1]);
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
			//count3++;
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
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
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
void send_telem(UART_HandleTypeDef device, uint8_t format){

	switch(format){
		case gui_byte_packet:
			command(led0, 1);
			pack_telem(telem_unstuffed);
			stuff_telem(telem_unstuffed, telem_stuffed);
			//HAL_UART_Transmit(&device, (uint8_t*)telem_stuffed, PACKET_SIZE+2, 100);
			HAL_UART_Transmit(&device, (uint8_t*)telem_stuffed, PACKET_SIZE+2, 100);
			command(led0, 0);
			break;
		default:
			break;
	}

}
