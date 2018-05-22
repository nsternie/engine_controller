/*
 * main_auto.c
 *
 *  Created on: May 21, 2018
 *      Author: nicks
 */

#include "main_auto.h"
#include "config.h"
#include "hardware.h"
#include "globals.h"

#if BOARD_ID == TARGET_ADDRESS_FLIGHT

uint32_t main_auto_start_time;
uint32_t main_auto_time[10] = {
		0,
		1000,
		1000,
		1500,
		7500,
		9000,
		0,
		0,
		0,
		0
};
uint32_t main_auto_index = 0;

void run_auto(uint32_t global_time){
	if(global_time >= (main_auto_time[main_auto_index] + main_auto_start_time)){
		execute_auto(main_auto_index);
		main_auto_index++;
		if(main_auto_index == 8){
			STATE =FULL_DURATION_SAFE;
			main_auto_index = 0;
		}
	}
}

void execute_auto(uint32_t index){
	switch(index){
		case 0:
		{
			int32_t args[1] = {1000};
			samplerate_set(1, args);
		}
			break;
		case 1:
			//command(WATER_DELUGE_CHANNEL, 1);
			break;
		case 2:
			command(IGNITER_CHANNEL, 1);
			break;
		case 3:
			motor_setpoint[OX_VALVE_MOTOR] = 150;
			motor_setpoint[FUEL_VALVE_MOTOR] = 225;
			break;
		case 4:
			motor_setpoint[OX_VALVE_MOTOR] = 70;
			motor_setpoint[FUEL_VALVE_MOTOR] = 100;
			//command(WATER_DELUGE_CHANNEL, 0);
			break;
		case 5:
		{
			int32_t args[1] = {50};
			samplerate_set(1, args);
			log_end(0, args);
		}
			break;
		case 6:
			break;
		case 7:
			break;
		case 8:
			break;
		case 9:
			break;
		case 10:
			break;
	}
}

#endif
