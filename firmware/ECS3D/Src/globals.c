#include "globals.h"

uint8_t STATE;
char* states[10][15] = {
		"MANUAL",
		"ARMED",
		"IGNITION",
		"FIRING",
		"FULL_DURATION"
};
struct simple_buf upstream_buffer;
volatile uint8_t read_adc_now;
volatile uint8_t send_rs422_now;
volatile uint8_t send_xbee_now;
volatile uint8_t update_motors_now;
volatile uint8_t relay_packet = 0;
uint16_t adc_data[6][16];		// 5 ADCs, 16 channels each.
uint16_t valve_states;
uint32_t IGNITION_DURATION = 750000;
uint32_t FIRING_DURATION = 3000000;
uint32_t POST_IGNITE_DELAY = 500000;
uint8_t LOGGING_ACTIVE = 0;
struct buffer rs422_buf;
uint8_t rs422_in;							// Temp single byte buffer for rx
uint8_t uart6_in;
uint8_t command_buffer[COMMAND_HISTORY][COMMAND_BUFFER_LENGTH];
uint8_t command_index = 0;
uint8_t telem_unstuffed[254];
uint8_t telem_stuffed[256];
uint8_t unstuffed_packet_length = 0;
float evlv[32];
float ivlv[32];
float ebatt;
float ibus;
float e5v;
float e3v;
float tbrd, tvlv, tmtr;
float pressure[22];
float load[5];
float thrust_load;
float thermocouple[16];
uint8_t temp = 0;
int32_t count1;
int32_t count2;
int32_t count3;
int32_t debug[8];
uint32_t start_time;
uint32_t end_time;
uint32_t active_time;
uint32_t idle_time;
float utilization;
uint32_t state_timer;
uint32_t 	motor_cycle_time[2],
			main_cycle_time[2],
			adc_cycle_time[2],
			telemetry_cycle_time[2],
			telem_utilization_time[2];
uint16_t samplerate = 50; // Hz
uint8_t telemetry_format[3];
uint16_t telemetry_rate[3] = {10,10,10};
uint8_t device_alias[100][10];
struct autosequence autos[NUM_AUTOS];
int16_t LOG_TO_AUTO = -1;
uint8_t AUTOSTRING[1024];
uint16_t auto_states;
struct autosequence hotfire_auto;

parser p;
int16_t last_packet_number = 0;
int16_t last_command_id = 0;
uint8_t error_code = 0;
uint32_t qd_stop_time = 0;

volatile uint32_t xmit_counter = 0;
uint32_t xmit_delay = 20;
uint8_t TELEM_ACTIVE = 1;


