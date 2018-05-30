/*
 * main_auto.h
 *
 *  Created on: May 21, 2018
 *      Author: nicks
 */

#ifndef MAIN_AUTO_H_
#define MAIN_AUTO_H_

#include "stdint.h"

//#define MAX_AUTO_LENGTH 12
uint32_t main_auto_start_time;
uint32_t main_auto_time[10];
uint32_t main_auto_index;

void run_auto(uint32_t global_time);

void execute_auto(uint32_t index);

#endif /* MAIN_AUTO_H_ */
