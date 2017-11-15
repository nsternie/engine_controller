import time
import struct

class ECParse:

	def __init__(self):
		self.csv_header = "Time (s),valve_states (),pressure[0] (psi),pressure[1] (psi),pressure[2] (psi),pressure[3] (psi),pressure[4] (psi),pressure[5] (psi),pressure[6] (psi),pressure[7] (psi),samplerate (Hz),motor_setpoint[0] (degrees),motor_setpoint[1] (degrees),main_cycle_time (microseconds),motor_cycle_time (microseconds),adc_cycle_time (microseconds),telemetry_cycle_time (microseconds),ebatt (Volts),ibus (Amps),telemetry_rate (Hz),motor_control_gain[0] (),motor_control_gain[1] (),motor_control_gain[2] (),motor_position[0] (),motor_position[1] (),motor_pwm[0] (),motor_pwm[1] (),count1 (),count2 (),count3 (),STATE (),load[0] (lbs),load[1] (lbs),load[2] (lbs),load[3] (lbs),thrust_load (lbs),thermocouple[0] (degC),thermocouple[1] (degC),thermocouple[2] (degC),thermocouple[3] (degC),ivlv[0] (Amps),ivlv[1] (Amps),ivlv[2] (Amps),ivlv[3] (Amps),ivlv[4] (Amps),ivlv[5] (Amps),ivlv[6] (Amps),ivlv[7] (Amps),ivlv[15] (Amps),evlv[0] (Volts),evlv[1] (Volts),evlv[2] (Volts),evlv[3] (Volts),evlv[4] (Volts),evlv[5] (Volts),evlv[6] (Volts),evlv[7] (Volts),evlv[8] (Volts),evlv[9] (Volts),evlv[10] (Volts),evlv[11] (Volts),evlv[12] (Volts),evlv[13] (Volts),evlv[14] (Volts),evlv[15] (Volts),LOG_TO_AUTO (),auto_states (),\n"
		self.valve_states = 0
		self.pressure = [0]*16
		self.samplerate = 0
		self.motor_setpoint = [0]*4
		self.main_cycle_time = 0
		self.motor_cycle_time = 0
		self.adc_cycle_time = 0
		self.telemetry_cycle_time = 0
		self.ebatt = 0
		self.ibus = 0
		self.telemetry_rate = 0
		self.motor_control_gain = [0]*4
		self.motor_position = [0]*4
		self.motor_pwm = [0]*4
		self.count1 = 0
		self.count2 = 0
		self.count3 = 0
		self.STATE = 0
		self.load = [0]*4
		self.thrust_load = 0
		self.thermocouple = [0]*4
		self.ivlv = [0]*16
		self.evlv = [0]*16
		self.LOG_TO_AUTO = 0
		self.auto_states = 0
		self.log_string = ""

	def parse_packet(self, packet):
		byte_rep = packet[0:2]
		self.valve_states = int((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[2:4]
		self.pressure[0] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[4:6]
		self.pressure[1] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[6:8]
		self.pressure[2] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[8:10]
		self.pressure[3] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[10:12]
		self.pressure[4] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[12:14]
		self.pressure[5] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[14:16]
		self.pressure[6] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[16:18]
		self.pressure[7] = float((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[18:22]
		self.samplerate = int((float(struct.unpack("<I", byte_rep)[0]))/1)
		byte_rep = packet[22:26]
		self.motor_setpoint[0] = float((float(struct.unpack("<i", byte_rep)[0]))/1000)
		byte_rep = packet[26:30]
		self.motor_setpoint[1] = float((float(struct.unpack("<i", byte_rep)[0]))/1000)
		byte_rep = packet[30:32]
		self.main_cycle_time = int((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[32:34]
		self.motor_cycle_time = int((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[34:36]
		self.adc_cycle_time = int((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[36:40]
		self.telemetry_cycle_time = int((float(struct.unpack("<I", byte_rep)[0]))/1)
		byte_rep = packet[40:42]
		self.ebatt = float((float(struct.unpack("<h", byte_rep)[0]))/1000)
		byte_rep = packet[42:44]
		self.ibus = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[44:46]
		self.telemetry_rate = int((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[46:48]
		self.motor_control_gain[0] = float((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[48:50]
		self.motor_control_gain[1] = float((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[50:52]
		self.motor_control_gain[2] = float((float(struct.unpack("<H", byte_rep)[0]))/1)
		byte_rep = packet[52:56]
		self.motor_position[0] = float((float(struct.unpack("<i", byte_rep)[0]))/1000)
		byte_rep = packet[56:60]
		self.motor_position[1] = float((float(struct.unpack("<i", byte_rep)[0]))/1000)
		byte_rep = packet[60:62]
		self.motor_pwm[0] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[62:64]
		self.motor_pwm[1] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[64:68]
		self.count1 = int((float(struct.unpack("<i", byte_rep)[0]))/1)
		byte_rep = packet[68:72]
		self.count2 = int((float(struct.unpack("<i", byte_rep)[0]))/1)
		byte_rep = packet[72:76]
		self.count3 = int((float(struct.unpack("<i", byte_rep)[0]))/1)
		byte_rep = packet[76:77]
		self.STATE = int((float(struct.unpack("<B", byte_rep)[0]))/1)
		byte_rep = packet[77:79]
		self.load[0] = float((float(struct.unpack("<h", byte_rep)[0]))/10)
		byte_rep = packet[79:81]
		self.load[1] = float((float(struct.unpack("<h", byte_rep)[0]))/10)
		byte_rep = packet[81:83]
		self.load[2] = float((float(struct.unpack("<h", byte_rep)[0]))/10)
		byte_rep = packet[83:85]
		self.load[3] = float((float(struct.unpack("<h", byte_rep)[0]))/10)
		byte_rep = packet[85:89]
		self.thrust_load = float((float(struct.unpack("<i", byte_rep)[0]))/10)
		byte_rep = packet[89:91]
		self.thermocouple[0] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[91:93]
		self.thermocouple[1] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[93:95]
		self.thermocouple[2] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[95:97]
		self.thermocouple[3] = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[97:99]
		self.ivlv[0] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[99:101]
		self.ivlv[1] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[101:103]
		self.ivlv[2] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[103:105]
		self.ivlv[3] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[105:107]
		self.ivlv[4] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[107:109]
		self.ivlv[5] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[109:111]
		self.ivlv[6] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[111:113]
		self.ivlv[7] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[113:115]
		self.ivlv[15] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[115:117]
		self.evlv[0] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[117:119]
		self.evlv[1] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[119:121]
		self.evlv[2] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[121:123]
		self.evlv[3] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[123:125]
		self.evlv[4] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[125:127]
		self.evlv[5] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[127:129]
		self.evlv[6] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[129:131]
		self.evlv[7] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[131:133]
		self.evlv[8] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[133:135]
		self.evlv[9] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[135:137]
		self.evlv[10] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[137:139]
		self.evlv[11] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[139:141]
		self.evlv[12] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[141:143]
		self.evlv[13] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[143:145]
		self.evlv[14] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[145:147]
		self.evlv[15] = float((float(struct.unpack("<h", byte_rep)[0]))/100)
		byte_rep = packet[147:149]
		self.LOG_TO_AUTO = int((float(struct.unpack("<h", byte_rep)[0]))/1)
		byte_rep = packet[149:151]
		self.auto_states = int((float(struct.unpack("<H", byte_rep)[0]))/1)
		self.log_string = str(time.clock())+','+str(self.valve_states)+','+str(self.pressure[0])+','+str(self.pressure[1])+','+str(self.pressure[2])+','+str(self.pressure[3])+','+str(self.pressure[4])+','+str(self.pressure[5])+','+str(self.pressure[6])+','+str(self.pressure[7])+','+str(self.samplerate)+','+str(self.motor_setpoint[0])+','+str(self.motor_setpoint[1])+','+str(self.main_cycle_time)+','+str(self.motor_cycle_time)+','+str(self.adc_cycle_time)+','+str(self.telemetry_cycle_time)+','+str(self.ebatt)+','+str(self.ibus)+','+str(self.telemetry_rate)+','+str(self.motor_control_gain[0])+','+str(self.motor_control_gain[1])+','+str(self.motor_control_gain[2])+','+str(self.motor_position[0])+','+str(self.motor_position[1])+','+str(self.motor_pwm[0])+','+str(self.motor_pwm[1])+','+str(self.count1)+','+str(self.count2)+','+str(self.count3)+','+str(self.STATE)+','+str(self.load[0])+','+str(self.load[1])+','+str(self.load[2])+','+str(self.load[3])+','+str(self.thrust_load)+','+str(self.thermocouple[0])+','+str(self.thermocouple[1])+','+str(self.thermocouple[2])+','+str(self.thermocouple[3])+','+str(self.ivlv[0])+','+str(self.ivlv[1])+','+str(self.ivlv[2])+','+str(self.ivlv[3])+','+str(self.ivlv[4])+','+str(self.ivlv[5])+','+str(self.ivlv[6])+','+str(self.ivlv[7])+','+str(self.ivlv[15])+','+str(self.evlv[0])+','+str(self.evlv[1])+','+str(self.evlv[2])+','+str(self.evlv[3])+','+str(self.evlv[4])+','+str(self.evlv[5])+','+str(self.evlv[6])+','+str(self.evlv[7])+','+str(self.evlv[8])+','+str(self.evlv[9])+','+str(self.evlv[10])+','+str(self.evlv[11])+','+str(self.evlv[12])+','+str(self.evlv[13])+','+str(self.evlv[14])+','+str(self.evlv[15])+','+str(self.LOG_TO_AUTO)+','+str(self.auto_states)+','