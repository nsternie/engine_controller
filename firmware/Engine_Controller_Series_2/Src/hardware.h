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

#define COMMAND_DIGITAL_WRITE 	50
#define COMMAND_LED_WRITE 		51
#define COMMAND_MOTOR_WRITE 	52
#define COMMAND_MOTOR_DISABLE 	53
#define COMMAND_MOTOR_ENABLE 	54
#define COMMAND_QD_SET 			55

#define COMMAND_SET_KP 			60
#define COMMAND_SET_KI 			61
#define COMMAND_SET_KD 			62
#define COMMAND_TELEMRATE_SET	63
#define COMMAND_SAMPLERATE_SET	64
#define COMAND_LOGRATE_SET		65

#define COMMAND_ARM				100
#define COMMAND_DISARM			101
#define COMMAND_MAIN_AUTO_START	102

#define TARGET_ADDRESS_GROUND 100
#define TARGET_ADDRESS_FLIGHT 101

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
void telemrate_set(int32_t argc, int32_t* argv);
void samplerate_set(int32_t argc, int32_t* argv);
void tare(int32_t argc, int32_t* argv);
void ambientize(int32_t argc, int32_t* argv);
void lograte_set(int32_t argc, int32_t* argv);

void read_adc(SPI_HandleTypeDef* SPI_BUS);
void set_device(uint8_t device, GPIO_PinState state);
void select_device(uint8_t device);
void release_device(uint8_t device);
void send_telem(UART_HandleTypeDef device, uint8_t format);

#endif /* HARDWARE_H_ */
