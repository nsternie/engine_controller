#include "telem.h"

#define HEADER_1 0
#define HEADER_2 0
#define HEADER_3 0
#define HEADER_4 0
#define HEADER_5 0
#define HEADER_6 0
#define HEADER_7 0
#define HEADER_8 0


void assemble_telem(){

	// HEADER DATA
	telem_unstuffed[0] = HEADER_1;
	telem_unstuffed[0] = HEADER_2;
	telem_unstuffed[0] = HEADER_3;
	telem_unstuffed[0] = HEADER_4;
	telem_unstuffed[0] = HEADER_5;
	telem_unstuffed[0] = HEADER_6;
	telem_unstuffed[0] = HEADER_7;
	telem_unstuffed[0] = HEADER_8;

}