#include "calibrations.h"
#include "hardware.h"



float motor_setpoint[4];
float motor_control_gain[3] = {
		500,
		0.0,
		0.0
};
const float motor_pot_slope[4] = {
		0.0830078,
		0.0830078,
		1,
		1
};
float motor_pot_offset[4] = {
		0,
		0,
		0,
		0
};
const float motor_limit_high[4] = {
		90,
		90,
		90,
		90
};
const float motor_limit_low[4] = {
		0,
		0,
		0,
		0
};
const float pot_polarity[4] = {
		1,
		-1,
		1,
		1
};






float load_cal[2][6] = {{
		// SLOPES
		0.5,
		0.5,
		0.5,
		0.5,
		1,
		1,
},{
		// OFFSETS
		0,
		0,
		0,
		0,
		0,
		0,
}};

#if (BOARD_ID == TARGET_ADDRESS_FLIGHT)
float press_cal[2][16] = {{
		// SLOPES
		0.644531,	// 0
		0.644531,
		0.644531,
		0.004833,
		0.644531,	// 4
		0.644531,
		0.644531,
		0.644531,
		0.644531,	// 8
		0.644531,
		0.644531,
		0.644531,
		1.0,	// 12
		1.0,
		1.0,
		1.0,	// 15
},{
		// OFFSETS
		310.3,	// 0
		310.3,
		310.3,
		0,
		310.3,	// 4
		310.3,
		310.3,
		310.3,
		310.3,	// 8
		310.3,
		310.3,
		310.3,
		310.3,	// 12
		310.3,
		310.3,
		310.3,	// 15
}};
#endif

#if (BOARD_ID == TARGET_ADDRESS_GROUND)
float press_cal[2][16] = {{
		// SLOPES
		0.644531,	// 0
		0.644531,
		0.644531,
		0.644531,
		0.644531,	// 4
		0.644531,
		0.644531,
		0.644531,
		0.644531,	// 8
		0.644531,
		0.644531,
		0.644531,
		1.0,	// 12
		1.0,
		1.0,
		1.0,	// 15
},{
		// OFFSETS
		310.3,	// 0
		310.3,
		310.3,
		310.3,
		310.3,	// 4
		310.3,
		310.3,
		310.3,
		310.3,	// 8
		310.3,
		310.3,
		310.3,
		310.3,	// 12
		310.3,
		310.3,
		310.3,	// 15
}};
#endif
