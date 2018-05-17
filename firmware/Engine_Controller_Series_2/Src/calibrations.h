
#ifndef CALIBRATIONS_H
#define CALIBRATIONS_H

#define Kp 0
#define Ki 1
#define Kd 2

#define I_LIMIT 500

#define ebatt_cal 	0.00324707
#define ibus_cal	0.01418500
#define evlv_cal  	0.00324707
#define ivlv_cal  	0.00322265
#define imtr_cal	0.00322265
#define e5v_cal		0.00161132
#define e3v_cal		0.00161132
#define tbrd_offset	600.000000
#define tbrd_slope	0.12400000

#define SLOPE 0
#define OFFSET 1

extern float motor_setpoint[4];
extern float motor_control_gain[3];
extern const float motor_pot_slope[4];
extern float motor_pot_offset[4];
extern const float motor_limit_high[4];
extern const float motor_limit_low[4];
extern const float pot_polarity[4];
extern float load_cal[2][6];
extern float press_cal[2][16];

#endif
