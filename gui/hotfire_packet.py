import struct
class ECParse:

	def parse_packet(packet):
		## GLOBALS ##
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
		global load
		global thrust_load
		global thermouple
		byte_rep = packet[0:2]
		valve_states = int((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[2:4]
		pressure[0] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[4:6]
		pressure[1] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[6:8]
		pressure[2] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[8:10]
		pressure[3] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[10:12]
		pressure[4] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[12:14]
		pressure[5] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[14:16]
		pressure[6] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[16:18]
		pressure[7] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[18:22]
		samplerate = int((float(struct.unpack("<I", byte_rep)[0]))/1)
		byte_rep = packet[22:26]
		motor_setpoint[0] = float((float(struct.unpack("<i", byte_rep)[0]))/1000)
		byte_rep = packet[26:30]
		motor_setpoint[1] = float((float(struct.unpack("<i", byte_rep)[0]))/1000)
		byte_rep = packet[30:32]
		main_cycle_time = int((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[32:34]
		motor_cycle_time = int((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[34:36]
		adc_cycle_time = int((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[36:38]
		telemetry_cycle_time = int((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[38:40]
		ebatt = float((float(struct.unpack("<h", byte_rep)[0]))/1000)
		byte_rep = packet[40:42]
		ibus = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[42:44]
		telemetry_rate = int((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[44:46]
		motor_control_gain[0] = float((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[46:48]
		motor_control_gain[1] = float((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[48:50]
		motor_control_gain[2] = float((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[50:54]
		motor_position[0] = float((float(struct.unpack("<i", byte_rep)[0]))/1000)
		byte_rep = packet[54:58]
		motor_position[1] = float((float(struct.unpack("<i", byte_rep)[0]))/1000)
		byte_rep = packet[58:60]
		motor_pwm[0] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[60:62]
		motor_pwm[1] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[62:66]
		count1 = int((float(struct.unpack("<I", byte_rep)[0]))/1)
		byte_rep = packet[66:70]
		count2 = int((float(struct.unpack("<I", byte_rep)[0]))/1)
		byte_rep = packet[70:74]
		count3 = int((float(struct.unpack("<I", byte_rep)[0]))/1)
		byte_rep = packet[74:75]
		STATE = int((float(struct.unpack("<B", byte_rep)[0]))/1)
		byte_rep = packet[75:77]
		load[0] = float((float(struct.unpack("<h", byte_rep)[0]))/10)
		byte_rep = packet[77:79]
		load[1] = float((float(struct.unpack("<h", byte_rep)[0]))/10)
		byte_rep = packet[79:81]
		load[2] = float((float(struct.unpack("<h", byte_rep)[0]))/10)
		byte_rep = packet[81:83]
		load[3] = float((float(struct.unpack("<h", byte_rep)[0]))/10)
		byte_rep = packet[83:87]
		thrust_load = float((float(struct.unpack("<i", byte_rep)[0]))/10)
		byte_rep = packet[87:89]
		thermocouple[0] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[89:91]
		thermocouple[1] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[91:93]
		thermocouple[2] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[93:95]
		thermocouple[3] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
