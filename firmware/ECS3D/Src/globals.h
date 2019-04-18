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

#define vlv8 28
#define vlv9 29
#define vlv10 30
#define vlv11 31
#define vlv12 32
#define vlv13 33
#define vlv14 34
#define vlv15 35

#define vlv16 36
#define vlv17 37
#define vlv18 38
#define vlv19 39
#define vlv20 40
#define vlv21 41
#define vlv22 42
#define vlv23 43

#define vlv24 44
#define vlv25 45
#define vlv26 46
#define vlv27 47
#define vlv28 48
#define vlv29 49
#define vlv30 50
#define vlv31 51


#define led0 159
#define led1 158
#define led2 157
#define led3 156



#define adc0 190
#define adc1 191
#define adc2 192
#define adc3 193
#define adc4 194
#define adc5 195
#define adc6 196

#define sram 60
#define flash 61

#define tc0 70
#define tc1 71
#define tc2 72
#define tc3 73
#define tc4 74
#define tc5 75
#define tc6 76
#define tc7 77
#define tc8 78
#define tc9 79
#define tc10 80
#define tc11 81
#define tc12 82
#define tc13 83
#define tc14 84
#define tc15 85


#define rtd0	100
#define rtd1	101
#define rtd2	102
#define rtd3	103
#define rtd4	104
#define rtd5	105
#define rtd6	106
#define rtd7	107


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

//#define UART_BUFFER_SIZE 	1024
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
volatile uint8_t relay_packet;
extern uint16_t adc_data[7][16];		// 5 ADCs, 16 channels each.
extern uint32_t valve_states;
extern uint32_t IGNITION_DURATION;
extern uint32_t FIRING_DURATION;
extern uint32_t POST_IGNITE_DELAY;
extern uint8_t LOGGING_ACTIVE;
//extern uint8_t spirit_data_buf[UART_BUFFER_SIZE];
//extern uint8_t rs422_data_buf[UART_BUFFER_SIZE];
//extern uint8_t xbee_data_buf[UART_BUFFER_SIZE];
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
extern float evlv[32];
extern float ivlv[32];
extern float ebatt;
extern float ibus;
extern float e28v;
extern float e5v;
extern float e3v;
extern float i5v, i3v;
extern float tbrd, tvlv, tmtr;
extern float pressure[22];
extern float tc[16];
extern float rtd[8];
extern uint16_t load[5];
extern float thrust_load;
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

volatile uint32_t xmit_counter;
uint32_t xmit_delay;
uint8_t buffer_of_shame[12];


uint32_t qd_stop_time;
#define QD_ACTUATION_TIME 3000;

uint8_t LOGGING_ACTIVE;
uint8_t TELEM_ACTIVE;

file* logfile;

uint8_t error_code;


// New auto stuff


// end new auto

#endif
