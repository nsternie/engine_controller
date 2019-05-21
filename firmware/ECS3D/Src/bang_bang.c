
#include "bang_bang.h"

uint8_t bang_bang_set_pressure = 100;
uint8_t bang_bang_close_tolerance = .7;

void bang_bang(uint8_t device_1, uint8_t device_2){

	if((((pressure[7] - 310.3030303) * 0.64453125)-4.9) > (bang_bang_set_pressure * bang_bang_close_tolerance)){
		command(device_1, 0);
	}else if((((pressure[7] - 310.3030303) * 0.64453125)-4.9)< (bang_bang_set_pressure - 5)){
		command(device_1, 1);
	}
}