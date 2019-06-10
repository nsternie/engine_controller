/*
 * hardware.c
 *
 *  Created on: May 14, 2018
 *      Author: nicks
 */


#include "hardware.h"
#include "stm32f4xx_hal.h"
#include "flash.h"
#include "main_auto.h"


extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim9;

extern TIM_HandleTypeDef htim2;
#define millis	((__HAL_TIM_GET_COUNTER(&htim2))/1000)

//extern IWDG_HandleTypeDef hiwdg;

extern UART_HandleTypeDef huart1;

const GPIO_TypeDef* vlv_ports[32] = {
        vlv0_GPIO_Port,
        vlv1_GPIO_Port,
        vlv2_GPIO_Port,
        vlv3_GPIO_Port,
        vlv4_GPIO_Port,
        vlv5_GPIO_Port,
        vlv6_GPIO_Port,
        vlv7_GPIO_Port,
        vlv8_GPIO_Port,
        vlv9_GPIO_Port,
        vlv10_GPIO_Port,
        vlv11_GPIO_Port,
        vlv12_GPIO_Port,
        vlv13_GPIO_Port,
        vlv14_GPIO_Port,
        vlv15_GPIO_Port,
        vlv16_GPIO_Port,
        vlv17_GPIO_Port,
        vlv18_GPIO_Port,
        vlv19_GPIO_Port,
        vlv20_GPIO_Port,
        vlv21_GPIO_Port,
        vlv22_GPIO_Port,
        vlv23_GPIO_Port,
        vlv24_GPIO_Port,
        vlv25_GPIO_Port,
        vlv26_GPIO_Port,
        vlv27_GPIO_Port,
        vlv28_GPIO_Port,
        vlv29_GPIO_Port,
        vlv30_GPIO_Port,
        vlv31_GPIO_Port
};
const uint16_t vlv_pins[32] = {
        vlv0_Pin,
        vlv1_Pin,
        vlv2_Pin,
        vlv3_Pin,
        vlv4_Pin,
        vlv5_Pin,
        vlv6_Pin,
        vlv7_Pin,
        vlv8_Pin,
        vlv9_Pin,
        vlv10_Pin,
        vlv11_Pin,
        vlv12_Pin,
        vlv13_Pin,
        vlv14_Pin,
        vlv15_Pin,
        vlv16_Pin,
        vlv17_Pin,
        vlv18_Pin,
        vlv19_Pin,
        vlv20_Pin,
        vlv21_Pin,
        vlv22_Pin,
        vlv23_Pin,
        vlv24_Pin,
        vlv25_Pin,
        vlv26_Pin,
        vlv27_Pin,
        vlv28_Pin,
        vlv29_Pin,
        vlv30_Pin,
        vlv31_Pin
};

// Hardware wrappers
void read_adc(SPI_HandleTypeDef* SPI_BUS, uint8_t adc_number){

	// Temp Buffers
	uint8_t tx[2];
	uint8_t rx[2];

	 __disable_irq();

	uint8_t adcn = adc0 + adc_number;

	// Select channel 0 to start
	tx[0] = 0;
	tx[1] = 0b0000100;
	select_device(adcn);
	if(HAL_SPI_TransmitReceive(SPI_BUS, tx, rx, 2, 1) ==  HAL_TIMEOUT){
	}
	release_device(adcn);

	for(uint8_t channel = 1; channel <= 16; channel++){

		tx[0] = (channel >> 1) | 0b00001000;
		tx[1] = (channel << 7) | 0b00000100;
		select_device(adcn);
		if(HAL_SPI_TransmitReceive(SPI_BUS, tx, rx, 2, 1) == HAL_TIMEOUT){

		}

		release_device(adcn);
		adc_data[adcn-adc0][channel-1] = ((rx[1])|(rx[0] << 8)) & 0x0FFF;

	}

	 __enable_irq();
}
void read_tc(SPI_HandleTypeDef* SPI_BUS, uint8_t device){

#define lutIndexOffset		200
#define minVoltage			-5603
#define maxVoltage			17819

const int16_t tempToMicroVolts_LUT[551] =
{
	-5603,-5587,-5571,-5555,-5539,-5523,-5506,-5489,-5473,-5456,-5439,-5421,-5404,-5387,-5369,-5351,-5334,-5316,-5297,-5279,
	-5261,-5242,-5224,-5205,-5186,-5167,-5148,-5128,-5109,-5089,-5070,-5050,-5030,-5010,-4989,-4969,-4949,-4928,-4907,-4886,
	-4865,-4844,-4823,-4802,-4780,-4759,-4737,-4715,-4693,-4671,-4648,-4626,-4604,-4581,-4558,-4535,-4512,-4489,-4466,-4443,
	-4419,-4395,-4372,-4348,-4324,-4300,-4275,-4251,-4226,-4202,-4177,-4152,-4127,-4102,-4077,-4052,-4026,-4000,-3975,-3949,
	-3923,-3897,-3871,-3844,-3818,-3791,-3765,-3738,-3711,-3684,-3657,-3629,-3602,-3574,-3547,-3519,-3491,-3463,-3435,-3407,
	-3379,-3350,-3322,-3293,-3264,-3235,-3206,-3177,-3148,-3118,-3089,-3059,-3030,-3000,-2970,-2940,-2910,-2879,-2849,-2818,
	-2788,-2757,-2726,-2695,-2664,-2633,-2602,-2571,-2539,-2507,-2476,-2444,-2412,-2380,-2348,-2316,-2283,-2251,-2218,-2186,
	-2153,-2120,-2087,-2054,-2021,-1987,-1954,-1920,-1887,-1853,-1819,-1785,-1751,-1717,-1683,-1648,-1614,-1579,-1545,-1510,
	-1475,-1440,-1405,-1370,-1335,-1299,-1264,-1228,-1192,-1157,-1121,-1085,-1049,-1013,-976,-940,-904,-867,-830,-794,
	-757,-720,-683,-646,-608,-571,-534,-496,-459,-421,-383,-345,-307,-269,-231,-193,-154,-116,-77,-39,
	0,39,78,117,156,195,234,273,312,352,391,431,470,510,549,589,629,669,709,749,
	790,830,870,911,951,992,1033,1074,1114,1155,1196,1238,1279,1320,1362,1403,1445,1486,1528,1570,
	1612,1654,1696,1738,1780,1823,1865,1908,1950,1993,2036,2079,2122,2165,2208,2251,2294,2338,2381,2425,
	2468,2512,2556,2600,2643,2687,2732,2776,2820,2864,2909,2953,2998,3043,3087,3132,3177,3222,3267,3312,
	3358,3403,3448,3494,3539,3585,3631,3677,3722,3768,3814,3860,3907,3953,3999,4046,4092,4138,4185,4232,
	4279,4325,4372,4419,4466,4513,4561,4608,4655,4702,4750,4798,4845,4893,4941,4988,5036,5084,5132,5180,
	5228,5277,5325,5373,5422,5470,5519,5567,5616,5665,5714,5763,5812,5861,5910,5959,6008,6057,6107,6156,
	6206,6255,6305,6355,6404,6454,6504,6554,6604,6654,6704,6754,6805,6855,6905,6956,7006,7057,7107,7158,
	7209,7260,7310,7361,7412,7463,7515,7566,7617,7668,7720,7771,7823,7874,7926,7977,8029,8081,8133,8185,
	8237,8289,8341,8393,8445,8497,8550,8602,8654,8707,8759,8812,8865,8917,8970,9023,9076,9129,9182,9235,
	9288,9341,9395,9448,9501,9555,9608,9662,9715,9769,9822,9876,9930,9984,10038,10092,10146,10200,10254,10308,
	10362,10417,10471,10525,10580,10634,10689,10743,10798,10853,10907,10962,11017,11072,11127,11182,11237,11292,11347,11403,
	11458,11513,11569,11624,11680,11735,11791,11846,11902,11958,12013,12069,12125,12181,12237,12293,12349,12405,12461,12518,
	12574,12630,12687,12743,12799,12856,12912,12969,13026,13082,13139,13196,13253,13310,13366,13423,13480,13537,13595,13652,
	13709,13766,13823,13881,13938,13995,14053,14110,14168,14226,14283,14341,14399,14456,14514,14572,14630,14688,14746,14804,
	14862,14920,14978,15036,15095,15153,15211,15270,15328,15386,15445,15503,15562,15621,15679,15738,15797,15856,15914,15973,
	16032,16091,16150,16209,16268,16327,16387,16446,16505,16564,16624,16683,16742,16802,16861,16921,16980,17040,17100,17159,
	17219,17279,17339,17399,17458,17518,17578,17638,17698,17759,17819
};

#define FAULT_CODE							9999				// Fault code to be returned if no thermocouple is attached or there are short circuits
#define OOR_CODE							8888				// Out of range code to be returned if the thermocouple microvolts reading is outside our -200 to 350C LUT range
#define COLD_JUNC_SENSITIVITY_COEFF_T		52.18				// Coeff used by MAX31855 to calculate temperature for type T thermocouple (units in uV/C)

	__disable_irq();
	uint8_t tx[4];
	uint8_t rx[4];

	select_device(device);
	HAL_SPI_TransmitReceive(SPI_BUS, tx, rx, 4, 1);
	release_device(device);

	int32_t spiData = rx[0] << 24 | rx[1] << 16 | rx[2] << 8 | rx[3];

	int chipSelectPin;
	long thermocoupleData;
	long refJuncData;
	int faultFlag;
	float uncorrectedThermocoupleTemp;
	float refJuncTemp;
	float totalOutputMicroVolts;
	float refJuncMicroVolts;
	float thermocoupleMicroVolts;
	float correctedThermocoupleTemp;

	faultFlag = (spiData & 0x00010000) >> 16;
	if ((spiData & 0x80000000) == 0x80000000)
	{
	thermocoupleData = (spiData ^ 0xFFFFFFFF) >> 18;
	thermocoupleData++;
	thermocoupleData = thermocoupleData * -1;
	}
	else
	{
	thermocoupleData = spiData >> 18;
	}
	if ((spiData & 0x00008000) == 0x00008000)
	{
	refJuncData = ((spiData ^ 0xFFFFFFFF) >> 4) & 0x00000FFF;
	refJuncData++;
	refJuncData = refJuncData * -1;
	}
	else
	{
	refJuncData = (spiData >> 4) & 0x00000FFF;
	}
	// Calculate out the uncorrected temperatures from the MAX31855 and find the total output voltage in micro volts (using the MAX31855 equation from datasheet)
	uncorrectedThermocoupleTemp = thermocoupleData * 0.25;
	refJuncTemp = refJuncData * 0.0625;
	totalOutputMicroVolts = (COLD_JUNC_SENSITIVITY_COEFF_T) * (uncorrectedThermocoupleTemp - refJuncTemp);
	// Find the reference junction voltage by using the lookup table and finding the equivalent voltage in microvolts for a specific temperature. Use linear interpolation
	// to find the most accurate microvolts for the given temperature --> y2 = m(x2-x1) + y1 (we must cast back as a signed int since pgm_read returns unsigned)
	int refJuncMicrovoltsHigh;
	int refJuncMicrovoltsLow;
	int refJuncMicrovoltsSlope;
	refJuncMicrovoltsHigh = (int)(tempToMicroVolts_LUT[((int)ceil(refJuncTemp) + lutIndexOffset)]);
	refJuncMicrovoltsLow = (int)(tempToMicroVolts_LUT[((int)floor(refJuncTemp) + lutIndexOffset)]);
	refJuncMicrovoltsSlope = (refJuncMicrovoltsHigh - refJuncMicrovoltsLow);
	refJuncMicroVolts = refJuncMicrovoltsSlope * (refJuncTemp - floor(refJuncTemp)) + refJuncMicrovoltsLow;
	// Calculate the voltage of the desired thermocouple junction itself (thermocouple junction and ref junction polarities are opposing in our application
	// with a type T thermocouple --> V_out = V_tc - V_ref)
	thermocoupleMicroVolts = totalOutputMicroVolts + refJuncMicroVolts;
	// Check to make sure this voltage is within our range of -200 to 350C then proceed to lookup table processing, or else return an out or range error
	if (thermocoupleMicroVolts < minVoltage || thermocoupleMicroVolts > maxVoltage)
	{
	}
	else
	{
	// Perform a reverse lookup table sorting to find the temperature from microvolts (this code implements a naive searching algorithm...based on what the voltage is,
	// the search index start begins at different spots and polls each data point - this can probably be more efficient, but was written quickly). The closest two
	// voltage values (less than and greater than) are stored into variables so we can use them to interpolate for 0.01C resolution
	int searchIndex;
	int correctedMicrovoltsHigh;
	int correctedMicrovoltsLow;
	int correctedMicrovoltsSlope;
	// Set the starting points
	if (thermocoupleMicroVolts < 0)
	{
		searchIndex = 0;
	}
	else
	{
		searchIndex = lutIndexOffset;
	}
	// Find the two closest microvolt points (closest high reading that is greater than the thermocouple microvolt reading, and closest low reading that is less than or equal to)
	while (searchIndex != 551)
	{
		correctedMicrovoltsHigh = (int)(tempToMicroVolts_LUT[searchIndex]);
		if (thermocoupleMicroVolts < correctedMicrovoltsHigh)
		{
			correctedMicrovoltsLow = (int)(tempToMicroVolts_LUT[(searchIndex - 1)]);
			break;
		}
		searchIndex++;
	}
	// Find the final corrected temperature from microvolts using linear interpolation - x2 = (y2-y1)/m + x1
	correctedMicrovoltsSlope = correctedMicrovoltsHigh - correctedMicrovoltsLow;
	correctedThermocoupleTemp = ((thermocoupleMicroVolts - correctedMicrovoltsLow) / correctedMicrovoltsSlope) + ((searchIndex-1) - lutIndexOffset);
	tc[device-tc0] = correctedThermocoupleTemp + 273.15;
	}

//	int16_t data;
//	data = rx[0];
//	data <<= 8;
//	data |= rx[1];
//	data /= 16;
//
//	float voltage = data * 0.00004156;
//
//	float t = 0 + 0.02594*pow(voltage, 1) - 0.0000002131*pow(voltage, 2) + 0.0000000007901*pow(voltage, 3) + 0.0000000000004252*pow(voltage, 4) + 0.0000000000000001330*pow(voltage, 5) + 0.00000000000000000002024*pow(voltage, 6);
//	tc[device-tc0] = data;
//	int32_t v = rx[0] << 24 | rx[1] << 16 | rx[2] << 8 | rx[3];
//	if (v & 0x80000000) {
//	    // Negative value, drop the lower 18 bits and explicitly extend sign bits.
//	v = 0xFFFFC000 | ((v >> 18) & 0x00003FFFF);
//	}
//	else {
//	// Positive value, just drop the lower 18 bits.
//	v >>= 18;
//	}
//	//Serial.println(v, HEX);
//
//	float centigrade = v;
//
//	// LSB = 0.25 degrees C
//	centigrade *= 0.25;
//	tc[device-tc0] = centigrade + 273.15;


	__enable_irq();
}
void read_rtd(SPI_HandleTypeDef* SPI_BUS, uint8_t device){

	uint8_t tx[2] = {1, 0};
	uint8_t rx[2] = {0, 0};

	__disable_irq();
	select_device(device);
	if(HAL_SPI_TransmitReceive(SPI_BUS, tx, rx, 2, 1) ==  HAL_TIMEOUT){}
	release_device(device);

	uint8_t high = rx[1];

	tx[0] = 2;

	select_device(device);
	if(HAL_SPI_TransmitReceive(SPI_BUS, tx, rx, 2, 1) ==  HAL_TIMEOUT){}
	release_device(device);

	__enable_irq();
	uint8_t low = rx[1];

	uint16_t value = (high << 8) | low;

	rtd[device - rtd0] = (value/32)-256+273.15;


}
void set_device(uint8_t device, GPIO_PinState state){
	switch(device){
		case adc0:
			HAL_GPIO_WritePin(adc0_cs_GPIO_Port, adc0_cs_Pin, state);
			break;
		case adc1:
			HAL_GPIO_WritePin(adc1_cs_GPIO_Port, adc1_cs_Pin, state);
			break;
		case adc2:
			HAL_GPIO_WritePin(adc2_cs_GPIO_Port, adc2_cs_Pin, state);
			break;
		case adc3:
			HAL_GPIO_WritePin(adc3_cs_GPIO_Port, adc3_cs_Pin, state);
			break;
		case adc4:
			HAL_GPIO_WritePin(adc4_cs_GPIO_Port, adc4_cs_Pin, state);
			break;
		case adc5:
			HAL_GPIO_WritePin(adc5_cs_GPIO_Port, adc5_cs_Pin, state);
			break;
		case adc6:
			HAL_GPIO_WritePin(adc6_cs_GPIO_Port, adc6_cs_Pin, state);
			break;

		case rtd0:
			HAL_GPIO_WritePin(rtd0_GPIO_Port, rtd0_Pin, state);
			break;
		case rtd1:
			HAL_GPIO_WritePin(rtd1_GPIO_Port, rtd1_Pin, state);
			break;
		case rtd2:
			HAL_GPIO_WritePin(rtd2_GPIO_Port, rtd2_Pin, state);
			break;
		case rtd3:
			HAL_GPIO_WritePin(rtd3_GPIO_Port, rtd3_Pin, state);
			break;
		case rtd4:
			HAL_GPIO_WritePin(rtd4_GPIO_Port, rtd4_Pin, state);
			break;
		case rtd5:
			HAL_GPIO_WritePin(rtd5_GPIO_Port, rtd5_Pin, state);
			break;
		case rtd6:
			HAL_GPIO_WritePin(rtd6_GPIO_Port, rtd6_Pin, state);
			break;
		case rtd7:
			HAL_GPIO_WritePin(rtd7_GPIO_Port, rtd7_Pin, state);
			break;



		case sram:
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
			break;
		case flash:
			HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
			break;

		default:
			if(device >= tc0 && device <= tc15){
				uint8_t tcn = device - tc0;
				HAL_GPIO_WritePin(TC_MUX_A3_GPIO_Port, TC_MUX_A3_Pin, tcn & 0b1000);
				HAL_GPIO_WritePin(TC_MUX_A2_GPIO_Port, TC_MUX_A2_Pin, tcn & 0b0100);
				HAL_GPIO_WritePin(TC_MUX_A1_GPIO_Port, TC_MUX_A1_Pin, tcn & 0b0010);
				HAL_GPIO_WritePin(TC_MUX_A0_GPIO_Port, TC_MUX_A0_Pin, tcn & 0b0001);
				HAL_GPIO_WritePin(TC_MUX_EN_N_GPIO_Port, TC_MUX_EN_N_Pin, state);
			}
			break;
	}
}
void select_device(uint8_t device){
	set_device(device, GPIO_PIN_RESET);
}
void release_device(uint8_t device){
	set_device(device, GPIO_PIN_SET);
}
void scale_readings(){

#define ebatt_cal 	0.00324707
#define ibus_cal	0.01418500
#define evlv_cal  	0.00324707
#define ivlv_cal  	0.00322265
#define imtr_cal	0.00322265
#define e5v_cal		0.00161132
#define e3v_cal		0.00161132
#define i5v_cal     1.00000000
#define i3v_cal 	1.00000000
#define tbrd_offset	600.000000
#define tbrd_slope	0.12400000

	for(uint8_t adcn = 0; adcn < 4; adcn++){
		for(uint8_t n = 0; n < 8; n++){
			evlv[7-n+(8*adcn)] = (adc_data[adcn][(2*n)+1])*evlv_cal;
			ivlv[7-n+(8*adcn)] = (adc_data[adcn][(2*n)])*ivlv_cal;
		}
	}

	ebatt = (adc_data[4][0])*ebatt_cal;
	ibus = (adc_data[4][1])*ibus_cal;
	e5v = (adc_data[4][2])*e5v_cal;
	e3v = (adc_data[4][3])*e3v_cal;
	i5v = (adc_data[4][9])*i5v_cal;
	i3v = (adc_data[4][8])*i3v_cal;

	tbrd = (adc_data[4][5])/1.24;
	tbrd -= 600;
	tbrd /= 10;
	tvlv = (adc_data[4][6])/1.24;
	tvlv -= 600;
	tvlv /= 10;
	tmtr = (adc_data[4][7])/1.24;
	tmtr -= 600;
	tmtr /= 10;

	for(uint8_t n = 0; n < 16; n ++){
		pressure[n] = adc_data[5][15-n];
		real_pressure[n] = ((pressure[n] - pressure_b[n]) * pressure_slope[n]) - pressure_ambients[n];
	}

	for(uint8_t n = 0; n < 6; n++){
		pressure[16+n] = adc_data[6][15-n];
		real_pressure[n] = ((pressure[n] - pressure_b[n]) * pressure_slope[n]) - pressure_ambients[n];
	}

	load[0] = adc_data[6][9];
	load[1] = adc_data[6][8];
	load[2] = adc_data[6][7];
	load[3] = adc_data[6][6];
	load[4] = adc_data[6][5];
	thrust_load = load[0]+load[1]+load[2];


}
void send_telem(UART_HandleTypeDef device, uint8_t format){
	switch(format){
		case gui_byte_packet:
			command(led3, 1);
			HAL_UART_Transmit(&device, (uint8_t*)telem_stuffed, PACKET_SIZE+2, 100);
			command(led3, 0);
			break;
		default:
			break;
	}

}
void save_telem(file* f){
	log_data(f, telem_stuffed, PACKET_SIZE+2);
}
void setpwm(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse)
{

	//pulse = pulse / 2;

 HAL_TIM_PWM_Stop(&timer, channel); // stop generation of pwm
 TIM_OC_InitTypeDef sConfigOC;
 timer.Init.Period = period; // set the period duration
 HAL_TIM_PWM_Init(&timer); // reinititialise with new period value

 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = pulse; // set the pulse duration
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);

 HAL_TIM_PWM_Start(&timer, channel); // start pwm generation
}
void command(uint8_t device, int16_t command_value){

	// VALVE CHANNELS ///////////////////////////////////////////////////////////////////
	if( ((device <= vlv31) && (device >= vlv0)) || ((device <= led0) && (device >= led3)) ){ // It is a valve channel (or led)

		// If else will default to a de-energized state if a bad command is sent.
		GPIO_PinState GPIO_COMMAND;

		uint32_t mask = 1;
		mask = mask << (device - vlv0);

		if(command_value == 1){
			GPIO_COMMAND = GPIO_PIN_SET;
			//valve_states |= mask;
		}
		else{
			GPIO_COMMAND = GPIO_PIN_RESET;
			mask ^= 0xFFFFFFFF;
			//valve_states &= mask;
		}
		switch(device){

			case led0:
				HAL_GPIO_WritePin(led0_GPIO_Port, led0_Pin, GPIO_COMMAND);
				break;
			case led1:
				HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_COMMAND);
				break;
			case led2:
				HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_COMMAND);
				break;
			case led3:
				HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_COMMAND);
				break;

			default:
				if(device >= vlv0 && device <= vlv0+31){
					HAL_GPIO_WritePin(vlv_ports[device - vlv0], vlv_pins[device - vlv0], GPIO_COMMAND);\
					break;
				}


		}


	} // END VALVES


}


// S2 commands
void led_write(int32_t argc, int32_t* argv){
	command(led0 - argv[0], argv[1]);
//	__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
}
void digital_write(int32_t argc, int32_t* argv){
	command(vlv0 + argv[0], argv[1]);
}
void set_kp(int32_t argc, int32_t* argv){
//	motor_control_gain[0] = argv[0];
}
void set_ki(int32_t argc, int32_t* argv){
//	motor_control_gain[1] = argv[0];
}
void set_kd(int32_t argc, int32_t* argv){
//	motor_control_gain[2] = argv[0];
}

void arm(int32_t argc, int32_t* argv){
	STATE = ARMED;


}
void disarm(int32_t argc, int32_t* argv){
	STATE = MANUAL;

}

void prime_tanks(int32_t argc, int32_t* argv){
	STATE = PRIME_TANKS;
}

void main_auto_start(int32_t argc, int32_t* argv){
	if(STATE == ARMED){
//		main_auto_start_time = millis;
		STATE = IGNITION;
	}
}
void pwm_set(int32_t argc, int32_t* argv){

	return;	// Disabled

	if(argc != 3) return;

	TIM_HandleTypeDef timer;
	uint32_t channel;

	switch(argv[0]){
	case 5:
		timer = htim5;
		break;
	case 9:
		timer = htim9;
		break;
	default:
		break;
		return;
	}

	switch(argv[1]){
	case 1:
		channel = TIM_CHANNEL_1;
		break;
	case 2:
		channel = TIM_CHANNEL_2;
		break;
	default:
		return;
		break;
	}

	uint32_t pulse = argv[2] * 32768;
	pulse /= 100;
	uint16_t pulse_cast = (uint16_t) pulse;

	setpwm(timer, channel, 32768, pulse_cast);

}
void qd_set(int32_t argc, int32_t* argv){


}
void telemrate_set(int32_t argc, int32_t* argv){
	uint16_t period = 100000/argv[0];

	HAL_TIM_Base_Stop_IT(&htim6);

	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 900;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = period;
	htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim6);

	HAL_TIM_Base_Start_IT(&htim6);

	telemetry_rate[rs422] = (100000/period);	// Actual rate
}
void samplerate_set(int32_t argc, int32_t* argv){
	samplerate = argv[0];
	uint16_t period = 100000/samplerate;

	HAL_TIM_Base_Stop_IT(&htim4);

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 900;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = period;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim4);

	HAL_TIM_Base_Start_IT(&htim4);

	samplerate = (100000/period);	// Actual rate
}

void load_ducer_calibrations(int32_t argc, int32_t* argv){

	for(int i = 0; i < argc; i += 2){
		float b = argv[i]/ 1000.0;
		float slope = argv[i+1] / 1000.0;
		pressure_b[i/2] = b;
		float w = pressure_b[7];
		printf("w %d\n", (int)(w));
		pressure_slope[i/2] = slope;
	}	
}

void tare(int32_t argc, int32_t* argv){

}
void ambientize(int32_t argc, int32_t* argv){

	for(int i = 0; i < argc; i++){
		pressure_ambients[i] = argv[i]/ 1000.0;
	}

}
void lograte_set(int32_t argc, int32_t* argv){

}
void print_file(int32_t argc, int32_t* argv){
	filesystem tempfs;
	read_filesystem(&tempfs);
	int filenum = argv[0];
	uint16_t start = tempfs.files[filenum].start_page;
	uint16_t stop = tempfs.files[filenum].stop_page;
	for(int n = start; n <= stop; n++){
		load_page(n);
		uint8_t buffer[2048];
		read_buffer(0, buffer, 2048);
		HAL_UART_Transmit(&huart1, buffer, 2048, 0xffff);
//		__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
	}
}
void numfiles(int32_t argc, int32_t* argv){
	filesystem tempfs;
	read_filesystem(&tempfs);
	uint8_t message[10];
	snprintf(message, sizeof(message), "%d\r\n", tempfs.num_files);
	HAL_UART_Transmit(&huart1, message, strlen(message), 0xffff);
}
void log_start(int32_t argc, int32_t* argv){
	if(LOGGING_ACTIVE == 0){
	  logfile = new_log();
	}
	LOGGING_ACTIVE = 1;
	command(led3, 1);
}
void log_end(int32_t argc, int32_t* argv){
	if(LOGGING_ACTIVE == 1){
		close_log(logfile);
	}
	LOGGING_ACTIVE = 0;
	command(led3, 0);
}
void init_fs(int32_t argc, int32_t* argv){

	for(int n = 0; n < 1024; n++){
		erase_block(64*n);
	  }

	filesystem fs;
	fs.current_file = -1;
	fs.next_file_page = 64;
	fs.num_files = 0;
	file blank_file;
	blank_file.bytes_free = 0;
	blank_file.current_page = 0;
	blank_file.file_number = 0;
	blank_file.start_page = 0;
	blank_file.stop_page = 0;
	for(int n = 0; n < MAX_FILES; n++){
	  fs.files[n] = blank_file;
	}
	write_filesystem(&fs);



	uint8_t message[100];
	snprintf(message, sizeof(message), "init_fs complete\r\n");
	HAL_UART_Transmit(&huart1, message, strlen(message), 0xffff);
}
void telem_pause(int32_t argc, int32_t* argv){
	TELEM_ACTIVE = 0;
}
void telem_resume(int32_t argc, int32_t* argv){
	TELEM_ACTIVE = 1;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

