from PyQt5 import QtGui # (the example applies equally well to PySide)
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import serial
import time
import os
from hotfire_packet import ECParse
import struct


# Gloabals
mtr = ['mtr0', 'mtr1', 'mtr2', 'mtr3']
mtr_enable = []
mtr_disable = []
mtr_setpoint = []
mtr_position = []
mtr_pwm = []
mtr_send = []
mtr_setpointfb = []

device_list = []
valve_states = 0
pressure = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
samplerate = 0
motor_setpoint = [0,0]
motor_position = [0,0]
motor_pwm = [0,0]
main_cycle_time = 1
motor_cycle_time = 1
adc_cycle_time = 1
telemetry_cycle_time = 1
ebatt = 0
ibus = 0
telemetry_rate = 0
motor_control_gain = [0,0,0]
count1 = 0
count2 = 0
count3 = 0
STATE = 0
LOG_TO_AUTO = 0
auto_states = 0

#

run_name = input("Enter run name: ")

serial_log = open(run_name+"_serial_log.csv", "w+")
info_log = open(run_name+"_python_log.csv", "w+")
command_log = open(run_name+"_command_log.csv", "w+")
data_log = open(run_name+"_datalog.csv", "w+")
command_log.write("Time, Command/info\n")


## Always start by initializing Qt (only once per application)
app = QtGui.QApplication([])
## Define a top-level widget to hold everything
w = QtGui.QWidget()
w.setWindowTitle('MASA Hotfire GUI - logging to '+run_name)
## Create a grid layout to manage the widgets size and position
layout = QtGui.QGridLayout()
w.setLayout(layout)
# Zero Indexes for the gui layout (row, column)
zr = 2
zc = 2

# Populate the alias dictionary
alias = {}
alias_file = open("devices.alias")
for line in alias_file:
	s = line.split('\t')
	alias[s[0]] = s[1].rstrip('\r\n')

info_log.write("Alias FIle")
for line in alias_file:
	info_log.write(line)
info_log.write(str(alias))
info_log.write("\n")

try:
	if("STATE_N" in alias.keys()):
		state_dict = {}
		for n in range(0, int(alias["STATE_N"])):
			state_dict[n] = alias["STATE"+str(n)]
	else:
		raise Exception("STATE_N definition not found in devices.alias file")
except Exception:
	print(Exception)

# Try to open the serial port

ser = serial.Serial(port=None, baudrate=4000000, timeout=0.5)
ser.port = alias["COM_PORT"]

try:
	ser.open()
	if(ser.is_open):
		ser.readline()
except:
	print("Could not open Serial Port")

# Parse a line and upate GUI fields
write_csv_header = True
def parse_serial():

	if(ser.is_open):
			# Read a packet
		packet = ser.readline()	
		# Unstuff the packet
		data_log.write("Stuffed: "+str(packet)+'\n')
		
		unstuffed = b''
		index = int(packet[0])
		for n in range(1, len(packet)):
			temp = packet[n:n+1]
			if(n == index):
				index = int(packet[n])+n
				temp = b'\n'
			unstuffed = unstuffed + temp
		packet = unstuffed
		data_log.write("Unstuffed: "+str(packet)+'\n\n')
		#line = str(line, 'ascii')
		#try:
			#split_line = line.split(',')
		parse_packet(packet)
		serial_log.write("%.3f," % time.clock())
		serial_log.write(str(packet)+'\n')
		# except:
		# 	print("Error")
		# 	pass

		state_label.setText("STATE = "+state_dict[STATE])

		log_to_auto_label.setText("Logging to auto: "+str(LOG_TO_AUTO))
		# if(AUTOSTRING == "0"):
		# 	pass 	# No new string sent
		# else:
		# 	temp = ""
		# 	split_auto = AUTOSTRING.split('|')
		# 	for chunk in split_auto:
		# 		temp = temp + chunk + "\n"
		# 	autofeedback.setPlainText(temp)
		# 	print("AUTOSTRING RECIEVED: "+AUTOSTRING)

		mask = 1
		running_autos_string = "Running Autos: "
		# Update auto state feedback
		for n in range(0, 16):
			state = 0
			if(mask & auto_states):
				running_autos_string += (str(n)+", ")
			mask = mask << 1

			running_autos_label.setText(running_autos_string)
		# print("Packet parsed")
		# print("battery: "+str(ebatt)+" \t and %.2f" % time.clock())

		mask = 1
		# Update valve state feedback
		for n in range(0, 16):
			state = 0
			if(mask & valve_states):
				state = 1
			valve_buttons[n][2].setText(str(state))
			valve_buttons[n][3].setText(str(ivlv[n]))
			valve_buttons[n][4].setText(str(evlv[n]))
			mask = mask << 1

			pressure_labels[n][1].setText(str(pressure[n])+"psi")
		# Update loop rates
		samplerate_setpointfb.setText(str(samplerate)+"hz")
		telemrate_setpointfb.setText(str(telemetry_rate)+"hz")
		for mtrx in range(0, 4):
			try:
				mtr_setpointfb[mtrx].setText(str(motor_setpoint[mtrx]))
				mtr_position[mtrx].setText(str(motor_position[mtrx]))
				mtr_pwm[mtrx].setText("PWM: "+str(motor_pwm[mtrx]))
			except:
				apperently_i_need_a_statment_here = "I dont really know why..."

		#main_cycle_rate.setText(str(round(1000000/main_cycle_time, 3)))
		motor_cycle_rate.setText(str(round(1000000/motor_cycle_time, 3)))
		adc_cycle_rate.setText(str(round(1000000/adc_cycle_time, 3)))
		telemetry_cycle_rate.setText(str(round(1000000/telemetry_cycle_time, 3)))

		# Board health
		ebatt_value.setText(str(ebatt))
		ibus_value.setText(str(ibus))

		# motor gain feedback
		kpfb.setText(str(motor_control_gain[0]))
		kifb.setText(str(motor_control_gain[1]))
		kdfb.setText(str(motor_control_gain[2]))

		count1_label.setText("Ignition Duration: "+str(count1))
		count2_label.setText("Burn Duration: "+str(count2))
		count3_label.setText("Post ignite delay: "+str(count3))

		state_label.setText("STATE = "+state_dict[STATE])

		thrust_load_label.setText("Thrust = "+str(thrust_load))
		for n in range(0, 4):
			load_label[n].setText(str(n)+": "+str(load[n]))



		
device_list = []
valve_states = 0
pressure = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
samplerate = 0
motor_setpoint = [0,0]
motor_position = [0,0]
motor_pwm = [0,0]
main_cycle_time = 1
motor_cycle_time = 1
adc_cycle_time = 1
telemetry_cycle_time = 1
ebatt = 0
ibus = 0
telemetry_rate = 0
motor_control_gain = [0,0,0]
count1 = 0
count2 = 0
count3 = 0
STATE = 0
load = [0,0,0,0]
thrust_load = 0
thermocouple = [0, 0, 0, 0]
auto_states = 0
LOG_TO_AUTO = 0
ivlv = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
evlv = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]



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





def command(device, command):
	command_string = "command "+str(device)+" "+str(command)
	send(command_string)

def set(variable, value):
	command_string = "set "+str(variable)+" "+str(value)
	send(command_string)

def motor_enable(motor_num, enable):
	command_string = ""
	if(enable):
		command_string += "enable "
	else:
		command_string += "disable "
	command_string += "mtr"+str(motor_num)+" x"
	send(command_string)

def send(command_string):
	command_string = command_string + " \r"
	print("SENDING: "+command_string.rstrip('\n'))
	command_log.write("%.3f,\tSENDING: " % time.clock()+command_string)
	if(ser.is_open):
		ser.write(command_string.encode('ascii'))

valve_buttons = []
pressure_labels = []
for n in range(0, 16):

	# Valve wdgets init
	temp = []
	vlv_id = 'vlv'+str(n)
	if vlv_id in alias.keys():
		vlv_id = alias[vlv_id]
	temp.append(QtGui.QPushButton(str(vlv_id)+' = OFF'))
	temp.append(QtGui.QPushButton(str(vlv_id)+' = ON'))
	temp.append(QtGui.QLabel())
	temp.append(QtGui.QLabel())
	temp.append(QtGui.QLabel())
	valve_buttons.append(temp)

	# Pressure reading widgets init
	ptemp = []
	ptemp.append(QtGui.QLabel())
	ptemp.append(QtGui.QLabel())
	pressure_labels.append(ptemp)
	press_id = "pressure"+str(n)
	if press_id in alias.keys():
		press_id = alias[press_id]
	pressure_labels[n][0].setText(press_id+":")
	pressure_labels[n][1].setText(str(0)+"psi")


## For some reason this doesnt work with a for loop, sorry
valve_buttons[0][0].clicked.connect(lambda: command("vlv0", 0))
valve_buttons[1][0].clicked.connect(lambda: command("vlv1", 0))
valve_buttons[2][0].clicked.connect(lambda: command("vlv2", 0))
valve_buttons[3][0].clicked.connect(lambda: command("vlv3", 0))
valve_buttons[4][0].clicked.connect(lambda: command("vlv4", 0))
valve_buttons[5][0].clicked.connect(lambda: command("vlv5", 0))
valve_buttons[6][0].clicked.connect(lambda: command("vlv6", 0))
valve_buttons[7][0].clicked.connect(lambda: command("vlv7", 0))
valve_buttons[8][0].clicked.connect(lambda: command("vlv8", 0))
valve_buttons[9][0].clicked.connect(lambda: command("vlv9", 0))
valve_buttons[10][0].clicked.connect(lambda: command("vlv10", 0))
valve_buttons[11][0].clicked.connect(lambda: command("vlv11", 0))
valve_buttons[12][0].clicked.connect(lambda: command("vlv12", 0))
valve_buttons[13][0].clicked.connect(lambda: command("vlv13", 0))
valve_buttons[14][0].clicked.connect(lambda: command("vlv14", 0))
valve_buttons[15][0].clicked.connect(lambda: command("vlv15", 0))
valve_buttons[0][1].clicked.connect(lambda: command("vlv0", 1))
valve_buttons[1][1].clicked.connect(lambda: command("vlv1", 1))
valve_buttons[2][1].clicked.connect(lambda: command("vlv2", 1))
valve_buttons[3][1].clicked.connect(lambda: command("vlv3", 1))
valve_buttons[4][1].clicked.connect(lambda: command("vlv4", 1))
valve_buttons[5][1].clicked.connect(lambda: command("vlv5", 1))
valve_buttons[6][1].clicked.connect(lambda: command("vlv6", 1))
valve_buttons[7][1].clicked.connect(lambda: command("vlv7", 1))
valve_buttons[8][1].clicked.connect(lambda: command("vlv8", 1))
valve_buttons[9][1].clicked.connect(lambda: command("vlv9", 1))
valve_buttons[10][1].clicked.connect(lambda: command("vlv10", 1))
valve_buttons[11][1].clicked.connect(lambda: command("vlv11", 1))
valve_buttons[12][1].clicked.connect(lambda: command("vlv12", 1))
valve_buttons[13][1].clicked.connect(lambda: command("vlv13", 1))
valve_buttons[14][1].clicked.connect(lambda: command("vlv14", 1))
#valve_buttons[15][1].clicked.connect(lambda: command("vlv15", 1)) # This is the igniter channel

# motor control
for mtrx in range(0, 4):
	mtr_enable.append(QtGui.QPushButton(mtr[mtrx]+" ENABLE"))
	mtr_disable.append(QtGui.QPushButton(mtr[mtrx]+" DISABLE"))
	mtr_setpoint.append(QtGui.QLineEdit())
	mtr_position.append(QtGui.QLabel("POSITION FB"))
	mtr_pwm.append(QtGui.QLabel("pwm FB"))
	mtr_send.append(QtGui.QPushButton("Command Setpoint"))
	mtr_setpointfb.append(QtGui.QLabel("SETPOINT FB"))

	if mtr[mtrx] in alias.keys():
		mtr_enable[mtrx].setText(alias[mtr[mtrx]]+" ENABLE")
		mtr_disable[mtrx].setText(alias[mtr[mtrx]]+" DISABLE")

	layout.addWidget(mtr_disable[mtrx], zr+1+(2*mtrx), zc+5)
	layout.addWidget(mtr_enable[mtrx], zr+1+(2*mtrx), zc+6)
	layout.addWidget(mtr_send[mtrx],zr+2+(2*mtrx), zc+5)
	layout.addWidget(mtr_pwm[mtrx], zr+1+(2*mtrx), zc+8)	
	layout.addWidget(mtr_setpoint[mtrx], zr+2+(2*mtrx), zc+6)
	layout.addWidget(mtr_setpointfb[mtrx], zr+2+(2*mtrx), zc+7)
	layout.addWidget(mtr_position[mtrx], zr+2+(2*mtrx), zc+8)

mtr_send[0].clicked.connect(lambda: command('mtr0', mtr_setpoint[0].text()))
mtr_send[1].clicked.connect(lambda: command('mtr1', mtr_setpoint[1].text()))
mtr_send[2].clicked.connect(lambda: command('mtr2', mtr_setpoint[2].text()))
mtr_send[3].clicked.connect(lambda: command('mtr3', mtr_setpoint[3].text()))
mtr_enable[0].clicked.connect(lambda: motor_enable(0, 1))
mtr_enable[1].clicked.connect(lambda: motor_enable(1, 1))
mtr_enable[2].clicked.connect(lambda: motor_enable(2, 1))
mtr_enable[3].clicked.connect(lambda: motor_enable(3, 1))
mtr_disable[0].clicked.connect(lambda: motor_enable(0, 0))
mtr_disable[1].clicked.connect(lambda: motor_enable(1, 0))
mtr_disable[2].clicked.connect(lambda: motor_enable(2, 0))
mtr_disable[3].clicked.connect(lambda: motor_enable(3, 0))


# Samplerate Set
samplerate_setpoint = QtGui.QLineEdit()
samplerate_setpointfb = QtGui.QLabel("SAMPLERATE FB")
samplerate_send = QtGui.QPushButton("Update samplerate (Hz)")
samplerate_send.clicked.connect(lambda: set("samplerate", samplerate_setpoint.text()))
layout.addWidget(samplerate_send, zr+7, zc+10)
layout.addWidget(samplerate_setpoint, zr+7, zc+11)
layout.addWidget(samplerate_setpointfb, zr+7, zc+12)
# Telemrate set
telemrate_setpoint = QtGui.QLineEdit()
telemrate_setpointfb = QtGui.QLabel("TELEMRATE FB")
telemrate_send = QtGui.QPushButton("Update telemrate (Hz)")
telemrate_send.clicked.connect(lambda: set("telemrate", ("rs422 "+telemrate_setpoint.text())))

layout.addWidget(telemrate_send, zr+8, zc+10)
layout.addWidget(telemrate_setpoint, zr+8, zc+11)
layout.addWidget(telemrate_setpointfb, zr+8, zc+12)

# Motor gains set
MOTOR_GAINS_LABEL = QtGui.QLabel("Motor Gains")
kp_set = QtGui.QPushButton("Update Kp")
ki_set = QtGui.QPushButton("Update Ki")
kd_set = QtGui.QPushButton("Update Kd")
kp_set.clicked.connect(lambda: set("gain", "0 "+str(kp_input.text())))
ki_set.clicked.connect(lambda: set("gain", "1 "+str(ki_input.text())))
kd_set.clicked.connect(lambda: set("gain", "2 "+str(kd_input.text())))
kp_input = QtGui.QLineEdit()
ki_input = QtGui.QLineEdit()
kd_input = QtGui.QLineEdit()
kpfb = QtGui.QLabel("kpfb")
kifb = QtGui.QLabel("kifb")
kdfb = QtGui.QLabel("kdfb")

layout.addWidget(kp_set, zr+9, zc+5)
layout.addWidget(ki_set, zr+10, zc+5)
layout.addWidget(kd_set, zr+11, zc+5)
layout.addWidget(kp_input, zr+9, zc+6)
layout.addWidget(ki_input, zr+10, zc+6)
layout.addWidget(kd_input, zr+11, zc+6)
layout.addWidget(kpfb, zr+9, zc+7)
layout.addWidget(kifb, zr+10, zc+7)
layout.addWidget(kdfb, zr+11, zc+7)

# State Feedback
state_label = QtGui.QLabel("STATE = N/A")
arm_button = QtGui.QPushButton("ARM")
disarm_button = QtGui.QPushButton("DISARM")
hotfire_button = QtGui.QPushButton("HOTFIRE")
arm_button.clicked.connect(lambda: send("arm"))
disarm_button.clicked.connect(lambda: send("disarm"))
hotfire_button.clicked.connect(lambda: send("hotfire"))

layout.addWidget(state_label, zr+12, zc+5)
layout.addWidget(arm_button, zr+13, zc+5)
layout.addWidget(disarm_button, zr+14, zc+5)
layout.addWidget(hotfire_button, zr+15, zc+5)

# Loads
thrust_load_label = QtGui.QLabel("NET THRUST")
thrust_load_label.setAlignment(Qt.AlignCenter)
load_label = []
for n in range(0, 4):
	load_label.append(QtGui.QLabel("LOAD "+str(n)))

layout.addWidget(thrust_load_label, zr+12, zc+6, 1, 2)
layout.addWidget(load_label[0], zr+13, zc+6)
layout.addWidget(load_label[1], zr+13, zc+7)
layout.addWidget(load_label[2], zr+14, zc+6)
layout.addWidget(load_label[3], zr+14, zc+7)

# Thermoucouples
tc_label = []
for n in range(0, 4):
	tc_label.append(QtGui.QLabel("TC-"+str(n)))

layout.addWidget(tc_label[0], zr+12, zc+11)
layout.addWidget(tc_label[1], zr+13, zc+11)
layout.addWidget(tc_label[2], zr+14, zc+11)
layout.addWidget(tc_label[3], zr+15, zc+11)



# Raw Command
def raw_command():
	send(raw_command_input.text())
	raw_command_input.setText("")
raw_command_input = QtGui.QLineEdit('command entry')
raw_command_send = QtGui.QPushButton("Send Command")
raw_command_send.clicked.connect(raw_command)
raw_command_input.returnPressed.connect(raw_command)

layout.addWidget(raw_command_input, zr+16, zc+5, 1, 2)
layout.addWidget(raw_command_send, zr+16, zc+7)

log_to_auto_label = QtGui.QLabel("LOG_TO_AUTO")
autofeedback = QtGui.QPlainTextEdit("Autosequence feedback")
running_autos_label = QtGui.QLabel("RUNNING_AUTOS")
layout.addWidget(autofeedback, 1, 10, 4, 3)
layout.addWidget(log_to_auto_label, 5, 10)
layout.addWidget(running_autos_label, 6, 10, 1, 2)

# Board Health
BOARD_HEALTH_LABEL = QtGui.QLabel("Board Health")
ebatt_label =  QtGui.QLabel("BATT")
ibus_label =  QtGui.QLabel("I-BUS")
ebatt_value =  QtGui.QLabel("EBATT")
ibus_value =  QtGui.QLabel("IBUS")

layout.addWidget(BOARD_HEALTH_LABEL, zr+9, zc+8)
layout.addWidget(ebatt_label, zr+10, zc+8)
layout.addWidget(ibus_label, zr+11, zc+8)
layout.addWidget(ebatt_value, zr+10, zc+9)
layout.addWidget(ibus_value, zr+11, zc+9)

# Loop times
LOOP_RATE_LABEL = QtGui.QLabel("Loop rates (hz)")
motor_cycle_rate = QtGui.QLabel("MCR")
main_cycle_rate = QtGui.QLabel("MCR")
adc_cycle_rate = QtGui.QLabel("ACR")
telemetry_cycle_rate = QtGui.QLabel("TCR")
motor_cycle_rate_label = QtGui.QLabel("Motor:")
main_cycle_rate_label = QtGui.QLabel("Main:")
adc_cycle_rate_label = QtGui.QLabel("ADC")
telemetry_cycle_rate_label = QtGui.QLabel("Telem")
# Label place

layout.addWidget(LOOP_RATE_LABEL, zr+12, zc+8)
layout.addWidget(main_cycle_rate_label, zr+13, zc+8)
layout.addWidget(motor_cycle_rate_label, zr+14, zc+8)
layout.addWidget(adc_cycle_rate_label, zr+15, zc+8)
layout.addWidget(telemetry_cycle_rate_label, zr+16, zc+8)
# Readout place

layout.addWidget(main_cycle_rate, zr+13, zc+9)
layout.addWidget(motor_cycle_rate, zr+14, zc+9)
layout.addWidget(adc_cycle_rate, zr+15, zc+9)
layout.addWidget(telemetry_cycle_rate, zr+16, zc+9)

# counts things for debugging
count1_label = QtGui.QLabel("COUNT1")
count2_label = QtGui.QLabel("COUNT2")
count3_label = QtGui.QLabel("COUNT3")

layout.addWidget(count1_label, zr+12, zc+10)
layout.addWidget(count2_label, zr+13, zc+10)
layout.addWidget(count3_label, zr+14, zc+10)


columns = 12
col_label = []
for n in range(0, columns+1):
	col_label.append(QtGui.QLabel())

col_label[0].setText("Valve OFF")
col_label[1].setText("Valve ON")
col_label[2].setText("State")

col_label[3].setText("Current")
col_label[4].setText("Voltage")
col_label[5].setText("Pressure")
col_label[7].setText("Value")
col_label[8].setText("Set Values")
col_label[9].setText("Feedback")
col_label[10].setText("Actual")
col_label[11].setText("Actual")




layout.addWidget(col_label[0], zr+0, 0)
layout.addWidget(col_label[1], zr+0, 1)
layout.addWidget(col_label[2], zr+0, 2)
layout.addWidget(col_label[3], zr+0, 3)
layout.addWidget(col_label[4], zr+0, 4)
layout.addWidget(col_label[5], zr+0, 5)
layout.addWidget(col_label[6], zr+0, 6)
layout.addWidget(col_label[7], zr+0, 7)
layout.addWidget(col_label[8], zr+0, 8)

def death():
	command_log.close()
	info_log.close()
	serial_log.close()
	data_log.close()
	app.quit()

KILL = QtGui.QPushButton("KILL")
KILL.clicked.connect(death)

layout.addWidget(KILL, zr+0, zc+10)

# Valve buttons and labels
for n in range(0, 16):

	layout.addWidget(valve_buttons[n][0], zr+n+1, zc+0-2)
	layout.addWidget(valve_buttons[n][1], zr+n+1, zc+1-2)
	layout.addWidget(valve_buttons[n][2], zr+n+1, zc+2-2)
	layout.addWidget(valve_buttons[n][3], zr+n+1, zc+3-2)
	layout.addWidget(valve_buttons[n][4], zr+n+1, zc+4-2)
	layout.addWidget(pressure_labels[n][0], zr+n+1, zc+3)
	layout.addWidget(pressure_labels[n][1], zr+n+1, zc+4)

if(1):
	# Add image

	#logo = QtGui.QLabel(w)
	#logo.setGeometry(1000, 250, 800, 250)
	#use full ABSOLUTE path to the image, not relative

	#logo.setPixmap(QtGui.QPixmap(os.getcwd() + "/masa2.png"))
	pass

if(0):
	p = w.palette()
	p.setColor(w.backgroundRole(), Qt.black)
	w.setPalette(p)

## Display the widget as a new window
w.show()

timer = pg.QtCore.QTimer()
timer.timeout.connect(parse_serial)
timer.start(10) # 100hz

## Start the Qt event loop
app.exec_()