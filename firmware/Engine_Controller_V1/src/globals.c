#include "globals.h"

uint8_t STATE;
char* states[10][15] = {
		"MANUAL",
		"ARMED",
		"IGNITION",
		"FIRING",
		"FULL_DURATION"
};
volatile uint8_t read_adc_now;
volatile uint8_t send_rs422_now;
volatile uint8_t send_xbee_now;
volatile uint8_t update_motors_now;
uint16_t adc_data[5][16];		// 5 ADCs, 16 channels each.
uint16_t valve_states;
uint32_t IGNITION_DURATION = 750000;
uint32_t FIRING_DURATION = 3000000;
uint32_t POST_IGNITE_DELAY = 500000;
float motor_position[4];
float motor_last_position[4];
float motor_accumulated_error[4];
int16_t motor_pwm[4];
uint8_t motor_active[4];
uint8_t LOGGING_ACTIVE = 0;
uint8_t spirit_data_buf[UART_BUFFER_SIZE];
uint8_t rs422_data_buf[UART_BUFFER_SIZE];
uint8_t xbee_data_buf[UART_BUFFER_SIZE];
struct buffer spirit_buf;
struct buffer rs422_buf;
struct buffer xbee_buf;
uint8_t spirit_in;							// Temp single byte buffer for rx
uint8_t rs422_in;							// Temp single byte buffer for rx
uint8_t xbee_in;							// Temp single byte buffer for rx
uint8_t command_buffer[COMMAND_HISTORY][COMMAND_BUFFER_LENGTH];
uint8_t command_index = 0;
uint8_t telem_unstuffed[254];
uint8_t telem_stuffed[256];
uint8_t unstuffed_packet_length = 0;
float evlv[16];
float ivlv[16];
float ebatt;
float ibus;
float e5v;
float e3v;
float tbrd, tvlv, tmtr;
float pressure[16];
float load[6];
float thrust_load;
float thermocouple[4];
uint8_t temp = 0;
uint32_t count1;
uint32_t count2;
uint32_t count3;
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
