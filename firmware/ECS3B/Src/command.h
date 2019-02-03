/*
 * command.h
 *
 *  Created on: Apr 21, 2018
 *      Author: nicks
 */

#ifndef COMMAND_H_
#define COMMAND_H_

#include <stdint.h>


#define DESCRITION_LENGTH	16
#define COMMAND_TYPE		uint16_t
#define NUMBER_OF_COMMANDS	500
#define NUMBER_OF_ARGS		4
#define ARG_TYPE			int32_t
#define BUFFER_LENGTH		256
#define COMMAND_SEPERATOR	0
#define COMMAND_FUNCTION_POINTER	void (*f)(int32_t, ARG_TYPE*)


typedef struct command_struct{
	COMMAND_TYPE id;
	ARG_TYPE args[NUMBER_OF_ARGS];
	COMMAND_FUNCTION_POINTER;
	uint32_t last_exec;
	uint32_t num_execs;
}command_struct;

typedef struct command_parser_struct{
	command_struct* commands[NUMBER_OF_COMMANDS];

	uint8_t buffer[BUFFER_LENGTH];

	uint32_t filled;

	uint16_t device_id;
	uint32_t bytes_received;
	uint32_t commands_parsed;
	uint32_t commands_executed;
	uint32_t corrupted;
}parser;



void stuff_data(uint8_t *unstuffed, uint8_t *stuffed, char seperator, uint32_t size);

parser init_parser(uint16_t device_id);
void pass_byte(parser* p, uint8_t byte);
void run_parser(parser* p);
void add_command(parser* p, COMMAND_TYPE command_id, COMMAND_FUNCTION_POINTER);
//void load_commands(parser* p);


// Helper functions
int search_string(uint8_t* arr, uint8_t character, uint32_t length);

#endif /* COMMAND_H_ */
