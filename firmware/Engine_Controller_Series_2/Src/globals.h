#include <stdint.h>
#include "command.h"
#include "flash.h"

#ifndef GLOBALS_H
#define GLOBALS_H

#define HEADER_1 0
#define HEADER_2 0
#define HEADER_3 0
#define HEADER_4 0
#define HEADER_5 0
#define HEADER_6 0
#define HEADER_7 0
#define HEADER_8 0

// END TELEM DEFINITIONS //////////////////////////////////


// Device definitions  ////////////////////////////////////
#define delay 0

#define vlv0 20
#define vlv1 21
#define vlv2 22
#define vlv3 23
#define vlv4 24
#define vlv5 25
#define vlv6 26
#define vlv7 27


#define led0 39
#define led1 38
#define led2 37
#define led3 36


#define mtr0 40
#define mtr1 41

#define adc0 50
#define adc1 51
#define adc2 52


#define sram 60
#define flash 61

#define tc0 70
#define tc1 71
#define tc2 72
#define tc3 73

#define spirit 80

#define rs422_com 	huart1
#define xbee_com 	huart4


#define rs422 	0
#define xbee	1
//#define spirit	2

#define MANUAL 				1
#define ARMED 				2
#define IGNITION			3
#define FIRING	 			4
#define FULL_DURATION	 	5
#define PRE_IGNITION	 	6
#define FULL_DURATION_SAFE 	7


#define MAX_COMMAND_ARGS 	7
#define MAX_COMMAND_LENGTH 	32
#define COMMAND_HISTORY 5

#define UART_BUFFER_SIZE 	1024
#define COMMAND_BUFFER_LENGTH 64
#define COMMAND_SOURCE 0


#define none			0
#define full_packet 	1
#define small_packet 	2
#define full_pretty		3
#define small_pretty	4
#define gui_v1			5
#define gui_byte_packet	6




extern uint8_t STATE;
struct buffer{
	uint8_t *start;
	uint8_t *end;
	uint16_t length;
	uint16_t filled;
	uint8_t *head;
	uint8_t *tail;
	uint8_t id;
	uint8_t new_data;
};
struct simple_buf{
	uint8_t filled;
	uint8_t data[256];
};
struct simple_buf upstream_buffer;
extern char* states[10][15];
extern volatile uint8_t read_adc_now;
extern volatile uint8_t send_rs422_now;
extern volatile uint8_t send_xbee_now;
extern volatile uint8_t update_motors_now;
volatile uint8_t relay_packet;
extern uint16_t adc_data[5][16];		// 5 ADCs, 16 channels each.
extern uint16_t valve_states;
extern uint32_t IGNITION_DURATION;
extern uint32_t FIRING_DURATION;
extern uint32_t POST_IGNITE_DELAY;
extern float motor_position[4];
extern float motor_last_position[4];
extern float motor_accumulated_error[4];
extern int16_t motor_pwm[4];
extern uint8_t motor_active[4];
extern uint8_t LOGGING_ACTIVE;
extern uint8_t spirit_data_buf[UART_BUFFER_SIZE];
extern uint8_t rs422_data_buf[UART_BUFFER_SIZE];
extern uint8_t xbee_data_buf[UART_BUFFER_SIZE];
extern struct buffer spirit_buf;
extern struct buffer rs422_buf;
extern struct buffer xbee_buf;
extern uint8_t spirit_in;							// Temp single byte buffer for rx
extern uint8_t rs422_in;							// Temp single byte buffer for rx
extern uint8_t uart6_in;							// Temp single byte buffer for rx
extern uint8_t xbee_in;							// Temp single byte buffer for rx
extern uint8_t command_buffer[COMMAND_HISTORY][COMMAND_BUFFER_LENGTH];
extern uint8_t command_index;
extern uint8_t telem_unstuffed[254];
extern uint8_t telem_stuffed[256];
extern uint8_t unstuffed_packet_length;
extern float evlv[16];
extern float ivlv[16];
float imtr[2];
extern float ebatt;
extern float ibus;
extern float e5v;
extern float e3v;
extern float tbrd, tvlv, tmtr;
extern float pressure[16];
extern float load[6];
extern float thrust_load;
extern float thermocouple[4];
extern uint8_t temp;
extern int32_t count1;
extern int32_t count2;
extern int32_t count3;
extern int32_t debug[8];
extern uint32_t start_time;
extern uint32_t end_time;
extern uint32_t active_time;
extern uint32_t idle_time;
extern float utilization;
extern uint32_t state_timer;
extern uint32_t 	motor_cycle_time[2],
			main_cycle_time[2],
			adc_cycle_time[2],
			telemetry_cycle_time[2],
			telem_utilization_time[2];
extern uint16_t samplerate; // Hz
extern uint8_t telemetry_format[3];
extern uint16_t telemetry_rate[3];
extern uint8_t device_alias[100][10];
//
#define MAX_AUTO_LENGTH 30
#define NUM_AUTOS		5
#define AUTO_STRING_LENGTH	30

struct autosequence{
	char		name[16];
	uint8_t 	number;
	uint8_t 	command[MAX_AUTO_LENGTH][AUTO_STRING_LENGTH];
	uint16_t	current_index;
	uint32_t	last_exec;
	uint32_t	next_exec;
	int16_t 	length;
	uint8_t 	running;
};

struct autosequence autos[NUM_AUTOS];
struct autosequence hotfire_auto;
int16_t LOG_TO_AUTO;
uint8_t AUTOSTRING[1024];
uint16_t auto_states;

parser p;
int16_t last_packet_number;
int16_t last_command_id;

uint32_t qd_stop_time;
#define QD_ACTUATION_TIME 3000;

uint8_t LOGGING_ACTIVE;
uint8_t TELEM_ACTIVE;

file* logfile;

uint8_t error_code;

#endif
