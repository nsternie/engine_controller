
#include "bang_bang.h"

uint8_t bang_bang_set_pressure = 75;
uint8_t bang_bang_close_tolerance = .7;
float last_time = 0;
float last_delay = 0;
float error = 0;
float k = .1;
void scale_readings();

void bang_bang(uint8_t device_1, uint8_t device_2, uint32_t time, SPI_HandleTypeDef hspi1, SPI_HandleTypeDef hspi2){
	if(time > last_time + last_delay){
		for(uint8_t n = 0; n < 7; n++){
			if(n == 4){
				read_adc(&hspi2, n);
			}
			else{
				read_adc(&hspi1, n);
			}
		}
		scale_readings();
		if(real_pressure[7] < (bang_bang_set_pressure - 30)){
			command(device_1, 1);
			error = bang_bang_set_pressure - real_pressure[7];
			last_time = time;
			last_delay = error * k;
		}else if(real_pressure[7] > (bang_bang_set_pressure - 25)){
			command(device_1, 0);
		}
	}
}
