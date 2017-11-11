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
		global ivlv
		global evlv
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
		byte_rep = packet[36:40]
		telemetry_cycle_time = int((float(struct.unpack("<I", byte_rep)[0]))/1)
		byte_rep = packet[40:42]
		ebatt = float((float(struct.unpack("<h", byte_rep)[0]))/1000)
		byte_rep = packet[42:44]
		ibus = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[44:46]
		telemetry_rate = int((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[46:48]
		motor_control_gain[0] = float((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[48:50]
		motor_control_gain[1] = float((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[50:52]
		motor_control_gain[2] = float((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[52:56]
		motor_position[0] = float((float(struct.unpack("<i", byte_rep)[0]))/1000)
		byte_rep = packet[56:60]
		motor_position[1] = float((float(struct.unpack("<i", byte_rep)[0]))/1000)
		byte_rep = packet[60:62]
		motor_pwm[0] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[62:64]
		motor_pwm[1] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[64:68]
		count1 = int((float(struct.unpack("<I", byte_rep)[0]))/1)
		byte_rep = packet[68:72]
		count2 = int((float(struct.unpack("<I", byte_rep)[0]))/1)
		byte_rep = packet[72:76]
		count3 = int((float(struct.unpack("<I", byte_rep)[0]))/1)
		byte_rep = packet[76:77]
		STATE = int((float(struct.unpack("<B", byte_rep)[0]))/1)
		byte_rep = packet[77:79]
		load[0] = float((float(struct.unpack("<h", byte_rep)[0]))/10)
		byte_rep = packet[79:81]
		load[1] = float((float(struct.unpack("<h", byte_rep)[0]))/10)
		byte_rep = packet[81:83]
		load[2] = float((float(struct.unpack("<h", byte_rep)[0]))/10)
		byte_rep = packet[83:85]
		load[3] = float((float(struct.unpack("<h", byte_rep)[0]))/10)
		byte_rep = packet[85:89]
		thrust_load = float((float(struct.unpack("<i", byte_rep)[0]))/10)
		byte_rep = packet[89:91]
		thermocouple[0] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[91:93]
		thermocouple[1] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[93:95]
		thermocouple[2] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[95:97]
		thermocouple[3] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[97:99]
		ivlv[0] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[99:101]
		ivlv[1] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[101:103]
		ivlv[2] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[103:105]
		ivlv[3] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[105:107]
		ivlv[4] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[107:109]
		ivlv[5] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[109:111]
		ivlv[6] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[111:113]
		ivlv[7] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[113:115]
		ivlv[15] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[115:117]
		evlv[0] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[117:119]
		evlv[1] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[119:121]
		evlv[2] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[121:123]
		evlv[3] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[123:125]
		evlv[4] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[125:127]
		evlv[5] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[127:129]
		evlv[6] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[129:131]
		evlv[7] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[131:133]
		evlv[8] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[133:135]
		evlv[9] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[135:137]
		evlv[10] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[137:139]
		evlv[11] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[139:141]
		evlv[12] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[141:143]
		evlv[13] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[143:145]
		evlv[14] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[145:147]
		evlv[15] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[147:149]
		LOG_TO_AUTO = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[149:151]
		auto_states = int((float(struct.unpack("<H", byte_rep)[0]))/1)
