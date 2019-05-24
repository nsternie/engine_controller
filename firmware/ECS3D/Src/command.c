/*
 * command.c
 *
 *  Created on: Apr 21, 2018
 *      Author: nicks
 */

#include "command.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "globals.h"

extern UART_HandleTypeDef huart6;

#define FinishBlock(X) (*code_ptr = (X), code_ptr = stuffed++, code = 0x01)

void stuff_data(uint8_t *unstuffed, uint8_t *stuffed, char seperator, uint32_t size){
	const uint8_t *end = unstuffed + size;
	uint8_t *code_ptr = stuffed++;
	uint8_t code = 0x01;

	while (unstuffed < end){
		if (*unstuffed == seperator)
			FinishBlock(code);
		else{
			*stuffed++ = *unstuffed;
			if (++code == 0xFF){
				FinishBlock(code);
			}
		}
		unstuffed++;
	}
	*stuffed = seperator;

	FinishBlock(code);
}
void unstuff_data(uint8_t *ptr, uint32_t length, uint8_t *dst)
{
	const uint8_t /**start = dst,*/ *end = ptr + length;
	uint8_t code = 0xFF, copy = 0;

	for (; ptr < end; copy--) {
		if (copy != 0) {
			*dst++ = *ptr++;
		} else {
			if (code != 0xFF)
				*dst++ = 0;
			copy = code = *ptr++;
			if (code == 0)
				break; /* Source length too long */
		}
	}
}
parser init_parser(uint16_t device_id){
	parser p;

	for(int n = 0; n < BUFFER_LENGTH; n++){
		p.buffer[n] = 255;
	}

	p.filled = 0;

	p.device_id = device_id;
	p.bytes_received = 0;
	p.commands_parsed = 0;
	p.commands_executed = 0;
	p.corrupted = 0;

	return p;
}
void pass_byte(parser* p, uint8_t byte){
	p->buffer[p->filled++] = byte;
}
void add_command(parser* p, COMMAND_TYPE command_id, COMMAND_FUNCTION_POINTER){
	command_struct* command_ptr = malloc(sizeof(command_struct));
	command_ptr->id = command_id;
	for(int n = 0; n < NUMBER_OF_ARGS; n++){
		command_ptr->args[n] = 0;
	}
	command_ptr->last_exec = 0;
	command_ptr->num_execs = 0;

	command_ptr->f = f;

	// #TODO
	// CHECK TO MAKE SURE IS IS NOT ALREADY USED

	p->commands[command_id] = command_ptr;
}
void run_parser(parser* p){

	uint8_t *temp = malloc(sizeof(p->buffer));
	if(temp == NULL){
		// Uh oh
	}

	memcpy(temp, p->buffer, sizeof(p->buffer));
	unstuff_data(temp, p->filled, p->buffer);


	int16_t packet_number = p->buffer[0]  << 8 | p->buffer[1];
	int16_t target_id = p->buffer[2]  << 8 | p->buffer[3];
	int16_t command_id = p->buffer[4]  << 8 | p->buffer[5];
	int16_t num_args = p->buffer[6]  << 8 | p->buffer[7];

	if (command_id == 70){
		command_id = 70;
	}

//  Leave this for the absolutely funny business it causes
//	if(num_args > 3){
//		//Clear Buffer
//		command_id = command_id;
//		p->filled = 0;
//		for(int n = 0; n < BUFFER_LENGTH; n++){
//			p->buffer[n] = 255;
//		}
//		free(temp);
//		return;
//	}


	int length_of_packet = 8+4*num_args+2; // 8 overhead, 4 per arg, 2 for checksum
	int32_t *args = malloc(num_args);
	if(args == NULL){

	}

	for(int n = 0; n < num_args; n++){
		args[n] = p->buffer[8+4*n]<<24 | p->buffer[9+4*n]<<16 | p->buffer[10+4*n]<<8 | p->buffer[11+4*n];
	}

	uint8_t checksum_0 = 0, checksum_1 = 0;
	for(int n = 0; n < (length_of_packet - 2)/2; n++){
		checksum_0 ^= p->buffer[2*n];
		checksum_1 ^= p->buffer[2*n+1];

	}

	uint8_t expected_checksum_0 = p->buffer[8+4*num_args];
	uint8_t expected_checksum_1 = p->buffer[8+4*num_args+1];


	if(	checksum_0 == expected_checksum_0 &&
		checksum_1 == expected_checksum_1){
		// Command is valid
		if(target_id == p->device_id){
			//printf("device %d\n", target_id);
			p->commands_executed++;
			p->commands[command_id]->f(num_args, args);
			p->commands[command_id]->num_execs++;
			last_command_id = p->commands_executed % 100;
			last_packet_number = packet_number;
		}
		else{
			HAL_UART_Transmit_IT(&huart6, temp, length_of_packet+2);
		}
	}
	else{
		// Checksum corrupt
		p->corrupted++;
//		// Clear buffer
		p->filled = 0;
		for(int n = 0; n < BUFFER_LENGTH; n++){
			p->buffer[n] = 255;
		}
		free(temp);
		return;
	}
	
	if(length_of_packet > 256){
		int i = 10;
	}

	for(int n = 0; n < length_of_packet; n++){
		p->buffer[n] = p->buffer[n + length_of_packet];
	}

	// Clear buffer
	p->filled = 0;
	for(int n = 0; n < BUFFER_LENGTH; n++){
		p->buffer[n] = 255;
	}
	free(temp);
	return;
	//memcpy( (void*) p->buffer[length_of_packet], (void*) p->buffer[0], (size_t) length_of_packet);
}

//void load_commands(parser* p){
//	add_command(p, COMMAND_DIGITAL_WRITE, digital_write);
//	add_command(p, COMMAND_LED_WRITE, led_write);
//	add_command(p, COMMAND_MOTOR_WRITE, motor_write);
//	add_command(p, COMMAND_MOTOR_DISABLE, motor_disable);
//	add_command(p, COMMAND_MOTOR_ENABLE, motor_enable);
//	add_command(p, COMMAND_SET_KP, set_kp);
//	add_command(p, COMMAND_SET_KI, set_ki);
//	add_command(p, COMMAND_SET_KD, set_kd);
//}

int search_string(uint8_t* arr, uint8_t character, uint32_t length){
	for(int n = 0; n < length; n++){
		if(*(arr+n) == character){
			return 1;
		}
	}
	return 0;
}
