//H File for handling everything that needs to happen for bang bang
#include <stdint.h>
#include "command.h"

extern uint8_t bang_bang_set_pressure;
extern uint8_t bang_bang_close_tolerance;

void bang_bang(uint8_t device_1, uint8_t device_2);
