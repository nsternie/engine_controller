/*
 * hardware.h
 *
 *  Created on: May 14, 2018
 *      Author: nicks
 */

#ifndef HARDWARE_H_
#define HARDWARE_H_

// Defines

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "stdlib.h"
#include "globals.h"
#include "calibrations.h"
#include "telem.h"
#include "pack_telem_defines.h"
#include "command.h"



// COMMAND IDs
#define COMMAND_TARE 			30
#define COMMAND_AMBIENTIZE 		31
#define COMMAND_PRIME_BRIDGE	32

#define COMMAND_PRINT_FILE		40
#define COMMAND_NUMFILES        41
#define COMMAD_LOG_START        42
#define COMMAD_LOG_END          43
#define COMMAD_INIT_FS          44
#define COMMAND_TELEM_PAUSE     45
#define COMMAND_TELEM_RESUME	46

#define COMMAND_DIGITAL_WRITE 	50
#define COMMAND_LED_WRITE 		51
#define COMMAND_MOTOR_WRITE 	52
#define COMMAND_MOTOR_DISABLE 	53
#define COMMAND_MOTOR_ENABLE 	54
#define COMMAND_QD_SET 			55
#define COMMAND_PWM_SET			56

#define COMMAND_SET_KP 			60
#define COMMAND_SET_KI 			61
#define COMMAND_SET_KD 			62
#define COMMAND_TELEMRATE_SET	63
#define COMMAND_SAMPLERATE_SET	64
#define COMMAND_LOGRATE_SET		65


#define COMMAND_ARM				100
#define COMMAND_DISARM			101
#define COMMAND_MAIN_AUTO_START	102

#define TARGET_ADDRESS_GROUND 100
#define TARGET_ADDRESS_FLIGHT 101

const uint16_t vlv_pins[32] = {
        vlv0_pin,
        vlv1_pin,
        vlv2_pin,
        vlv3_pin,
        vlv4_pin,
        vlv5_pin,
        vlv6_pin,
        vlv7_pin,
        vlv8_pin,
        vlv9_pin,
        vlv10_pin,
        vlv11_pin,
        vlv12_pin,
        vlv13_pin,
        vlv14_pin,
        vlv15_pin,
        vlv16_pin,
        vlv17_pin,
        vlv18_pin,
        vlv19_pin,
        vlv20_pin,
        vlv21_pin,
        vlv22_pin,
        vlv23_pin,
        vlv24_pin,
        vlv25_pin,
        vlv26_pin,
        vlv27_pin,
        vlv28_pin,
        vlv29_pin,
        vlv30_pin,
        vlv31_pin
};
};

// Hardware Specific functions
void read_adc(SPI_HandleTypeDef* SPI_BUS);
void set_device(uint8_t device, GPIO_PinState state);
void select_device(uint8_t device);
void release_device(uint8_t device);
void send_telem(UART_HandleTypeDef device, uint8_t format);
void save_telem(file* f);
void setpwm(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse);
void command(uint8_t device, int16_t command_value);
void writeMotor(uint8_t device, int16_t motor_command);
void motor_control();

// S2 Command wrappers
void led_write(int32_t argc, int32_t* argv);
void digital_write(int32_t argc, int32_t* argv);
void set_kp(int32_t argc, int32_t* argv);
void set_ki(int32_t argc, int32_t* argv);
void set_kd(int32_t argc, int32_t* argv);
void motor_write(int32_t argc, int32_t* argv);
void motor_disable(int32_t argc, int32_t* argv);
void motor_enable(int32_t argc, int32_t* argv);
void arm(int32_t argc, int32_t* argv);
void disarm(int32_t argc, int32_t* argv);
void main_auto_start(int32_t argc, int32_t* argv);
void qd_set(int32_t argc, int32_t* argv);
void pwm_set(int32_t argc, int32_t* argv);
void telemrate_set(int32_t argc, int32_t* argv);
void samplerate_set(int32_t argc, int32_t* argv);
void tare(int32_t argc, int32_t* argv);
void ambientize(int32_t argc, int32_t* argv);
void lograte_set(int32_t argc, int32_t* argv);
void print_file(int32_t argc, int32_t* argv);
void numfiles(int32_t argc, int32_t* argv);
void log_start(int32_t argc, int32_t* argv);
void log_end(int32_t argc, int32_t* argv);
void init_fs(int32_t argc, int32_t* argv);
void telem_pause(int32_t argc, int32_t* argv);
void telem_resume(int32_t argc, int32_t* argv);
void prime_bridge_wrapper(int32_t argc, int32_t* argv);
void prime_bridge();

#endif /* HARDWARE_H_ */
