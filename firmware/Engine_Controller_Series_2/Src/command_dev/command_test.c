#define TESTING

#include <stdio.h>
#include <stdlib.h>
#include "command.h"

int pass = 1;

uint8_t test_data[] = {0,1,0,1,0,0,0,4,0,0,0,0,0,0,0,1,0,0,0,2,0,0,0,3,0,4};

void test_command(uint32_t argc, uint32_t* argv);

int main(){

	parser p = init_parser(1);

	add_command(&p, 0, &test_command);
	uint8_t* stuffed_data = malloc(sizeof(test_data));
	stuff_data(test_data, stuffed_data, '\0', sizeof(test_data));

	for(int n = 0; n < sizeof(test_data); n++){
		pass_byte(&p, test_data[n]);
	}

	run_parser(&p);

	// p.commands[0]->f(1, p.commands[0]->args);

	if(pass == 1){
		printf("All tests passed\n");
	}
	return 0;
}


void test_command(uint32_t argc, uint32_t* argv){
	for(int n = 0; n < argc; n++){
		printf("argv[%d] = %d\n", n, argv[n]);
	}
}