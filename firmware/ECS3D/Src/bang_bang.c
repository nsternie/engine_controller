/*
 * bang_bang.c
 *
 *  Created on: June 27, 2019
 *      Author: rohankap
 */
#include "bang_bang.h"

void bang_bang(){
	float psi_per_millis = 0;
	for (uint8_t n = 1; n < fuel_bb.num_pressures; n++){
		psi_per_millis += (fuel_bb.press_array[1][n] - fuel_bb.press_array[1][n-1])/(fuel_bb.press_array[0][n] - fuel_bb.press_array[0][n-1]);
	}
	psi_per_millis = psi_per_millis/(fuel_bb.num_pressures - 1);
	if (fuel_bb.press_array[1][fuel_bb.num_pressures - 1] + psi_per_millis*fuel_bb.vlv_close_delay >= fuel_bb.tank_upper_pressure){
		command(fuel_bb.vlv_id, 0);
		int j  = 0;
	}
	else if(fuel_bb.press_array[1][fuel_bb.num_pressures - 1] + psi_per_millis*fuel_bb.vlv_open_delay <= fuel_bb.tank_lower_pressure){
		command(fuel_bb.vlv_id, 1);
	}

}


void fill_bb_press_array(uint32_t time){
    for (uint8_t n = 0; n < fuel_bb.num_pressures; n++) {
    	if (n == (fuel_bb.num_pressures - 1)){
    		fuel_bb.press_array[1][n] = (pressure[fuel_bb.ducer_id] - fuel_bb.ducer_b)*fuel_bb.ducer_slope - fuel_bb.ducer_ambient;
    		fuel_bb.press_array[0][n] = time;
    	}
    	else{
    		fuel_bb.press_array[1][n] = fuel_bb.press_array[1][n+1];
    		fuel_bb.press_array[0][n] = fuel_bb.press_array[0][n+1];
    	}
    }
}

