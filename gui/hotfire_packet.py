write_csv_header = True
def parse_packet(split_line):
	if(write_csv_header):
		device_list = ['valve_states', 'pressure0', 'pressure1', 'pressure2', 'pressure3', 'pressure4', 'pressure5', 'pressure6', 'pressure7', 'samplerate', 'motor_setpoint[0]', 'motor_setpoint[1]', 'main_cycle_time', 'motor_cycle_time', 'adc_cycle_time', 'telemetry_cycle_time', 'ebatt', 'ibus', 'telemetry_rate', 'motor_control_gain[0]', 'motor_control_gain[1]', 'motor_control_gain[2]', 'motor_position[0]', 'motor_position[1]', 'motor_pwm[0]', 'motor_pwm[1]', 'count1', 'count2', 'count3', 'STATE', 'AUTOSTRING', '']
		for device in range(0, len(device_list)):
			if device_list[device] in alias.keys():
				device_list[device] = alias[device_list[device]]
		csv_header = "Time (s),"
			csv_header.append(device, ",")
		write_csv_header = False
	global valve_states
	global pressure
	global samplerate
	global motor_setpoint
	global main_cycle_time
	global motor_cycle_time
	global adc_cycle_time
	global telemetry_cycle_time
	global ebatt
	global ibus
	global telemetry_rate
	global motor_control_gain
	global motor_position
	global motor_pwm
	global count1
	global count2
	global count3
	global STATE
	global AUTOSTRING
	valve_states = int(split_line[0])
	pressure[0] = float(split_line[1])
	pressure[1] = float(split_line[2])
	pressure[2] = float(split_line[3])
	pressure[3] = float(split_line[4])
	pressure[4] = float(split_line[5])
	pressure[5] = float(split_line[6])
	pressure[6] = float(split_line[7])
	pressure[7] = float(split_line[8])
	samplerate = int(split_line[9])
	motor_setpoint[0] = float(split_line[10])
	motor_setpoint[1] = float(split_line[11])
	main_cycle_time = int(split_line[12])
	motor_cycle_time = int(split_line[13])
	adc_cycle_time = int(split_line[14])
	telemetry_cycle_time = int(split_line[15])
	ebatt = float(split_line[16])
	ibus = float(split_line[17])
	telemetry_rate = int(split_line[18])
	motor_control_gain[0] = float(split_line[19])
	motor_control_gain[1] = float(split_line[20])
	motor_control_gain[2] = float(split_line[21])
	motor_position[0] = float(split_line[22])
	motor_position[1] = float(split_line[23])
	motor_pwm[0] = int(split_line[24])
	motor_pwm[1] = int(split_line[25])
	count1 = int(split_line[26])
	count2 = int(split_line[27])
	count3 = int(split_line[28])
	STATE = int(split_line[29])
	AUTOSTRING = str(split_line[30])
