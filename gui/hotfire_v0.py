from PyQt5 import QtGui # (the example applies equally well to PySide)
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import serial
import time
import os

run_name = input("Enter run name: ")

serial_log = open(run_name+"_Serial_log.csv", "w+")
info_log = open(run_name+"_Command_log.csv", "w+")
info_log.write("Time, Command/info\n")
csv_header = "Time(s),valve_states,pressure[0],pressure[1],pressure[2],pressure[3],pressure[4],pressure[5],pressure[6],pressure[7],samplerate,motor_setpoint[0],motor_setpoint[1],main_cycle_time,motor_cycle_time,adc_cycle_time,telemetry_cycle_time,ebatt,ibus,telemetry_rate[0],motor_control_gain[0],motor_control_gain[1],motor_control_gain[2],motor_position[0],motor_position[1],motor_pwm[0],motor_pwm[1],count1,count2,count3,STATE,"
serial_log.write(csv_header)
serial_log.write("\n")

## Always start by initializing Qt (only once per application)
app = QtGui.QApplication([])
## Define a top-level widget to hold everything
w = QtGui.QWidget()
w.setWindowTitle('MASA Hotfire GUI - logging to '+run_name)
## Create a grid layout to manage the widgets size and position
layout = QtGui.QGridLayout()
w.setLayout(layout)


# Populate the alias dictionary
alias = {}
alias_file = open("devices.alias")
for line in alias_file:
	s = line.split('\t')
	alias[s[0]] = s[1].rstrip('\n')

# info_log.write("Alias FIle")
# for line in alias_file:
# 	info_log.write(line)
# info_log.write(str(alias))
# info_log.write("\n")

state_dict = {
	0:"ERROR",
	1:"MANUAL",
	2:"ARMED",
	3:"IGNITION",
	4:"FIRING",
	5:"FULL_DURATION"
}

# Try to open the serial port
ser = serial.Serial(port=None, baudrate=921600)
ser.port = "COM5"
try:
	ser.open()
	if(ser.is_open):
		ser.readline()
except:
	print("Could not open Serial Port")

# Parse a line and upate GUI fields
def parse_serial():
	if(ser.is_open):
		line = ser.readline()	
		line = str(line, 'ascii')
		serial_log.write("%.3f," % time.clock())
		serial_log.write(line.rstrip('\n'))
		split_line = line.split(',')
		parse_packet(split_line)
		#print("Packet parsed")
		#print("battery: "+str(ebatt)+" \t and %.2f" % time.clock())

		mask = 1
		# Update valve state feedback
		for n in range(0, 16):
			state = 0
			if(mask & valve_states):
				state = 1
			valve_buttons[n][2].setText(str(state))
			mask = mask << 1

			pressure_labels[n][1].setText(str(pressure[n])+"psi")
		# Update loop rates
		samplerate_setpointfb.setText(str(samplerate)+"hz")
		telemrate_setpointfb.setText(str(telemetry_rate)+"hz")
		mtr0_setpointfb.setText(str(motor_setpoint[0]))
		mtr1_setpointfb.setText(str(motor_setpoint[1]))
		mtr0_position.setText(str(motor_position[0]))
		mtr1_position.setText(str(motor_position[1]))
		mtr0_pwm.setText(str(motor_pwm[0]))
		mtr1_pwm.setText(str(motor_pwm[1]))

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

def parse_packet(split_line):
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
	info_log.write("%.3f,\tSENDING: " % time.clock()+command_string)
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

valve_buttons[0][0].setText("FILL OX OFF")
valve_buttons[1][0].setText("VENT OX OFF")
valve_buttons[0][1].setText("FILL OX ON")
valve_buttons[1][1].setText("VENT OX ON")
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

# mtr0 control
mtr0_enable = QtGui.QPushButton("mtr0 ENABLE")
mtr0_disable = QtGui.QPushButton("mtr0 DISABLE")
mtr0_setpoint = QtGui.QLineEdit()
mtr0_position = QtGui.QLabel("POSITION FB")
mtr0_pwm = QtGui.QLabel("pwm FB")
mtr0_send = QtGui.QPushButton("Command Setpoint")
mtr0_setpointfb = QtGui.QLabel("SETPOINT FB")
mtr0_enable.clicked.connect(lambda: motor_enable(0, 1))
mtr0_disable.clicked.connect(lambda: motor_enable(0, 0))
mtr0_send.clicked.connect(lambda: command("mtr0", mtr0_setpoint.text()))
layout.addWidget(mtr0_disable, 1, 5)
layout.addWidget(mtr0_enable, 1, 6)
layout.addWidget(mtr0_send,2, 5)
layout.addWidget(mtr0_pwm, 2, 9)
layout.addWidget(mtr0_setpoint, 2, 6)
layout.addWidget(mtr0_setpointfb, 2, 7)
layout.addWidget(mtr0_position, 2, 8)

# mtr1 control
mtr1_enable = QtGui.QPushButton("mtr1 ENABLE")
mtr1_disable = QtGui.QPushButton("mtr1 DISABLE")
mtr1_setpoint = QtGui.QLineEdit()
mtr1_position = QtGui.QLabel("POSITION FB")
mtr1_pwm = QtGui.QLabel("pwm FB")
mtr1_send = QtGui.QPushButton("Command Setpoint")
mtr1_setpointfb = QtGui.QLabel("SETPOINT FB")
mtr1_enable.clicked.connect(lambda: motor_enable(1, 1))
mtr1_disable.clicked.connect(lambda: motor_enable(1, 0))
mtr1_send.clicked.connect(lambda: command("mtr1", mtr1_setpoint.text()))
layout.addWidget(mtr1_disable, 3, 5)
layout.addWidget(mtr1_enable, 3, 6)
layout.addWidget(mtr1_send,4, 5)
layout.addWidget(mtr1_setpoint, 4, 6)
layout.addWidget(mtr1_setpointfb, 4, 7)
layout.addWidget(mtr1_position, 4, 8)
layout.addWidget(mtr1_pwm, 4, 9)

# Samplerate Set
samplerate_setpoint = QtGui.QLineEdit()
samplerate_setpointfb = QtGui.QLabel("SAMPLERATE FB")
samplerate_send = QtGui.QPushButton("Update samplerate (Hz)")
samplerate_send.clicked.connect(lambda: set("samplerate", samplerate_setpoint.text()))
layout.addWidget(samplerate_send, 5, 5)
layout.addWidget(samplerate_setpoint, 5, 6)
layout.addWidget(samplerate_setpointfb, 5, 7)
# Telemrate set
telemrate_setpoint = QtGui.QLineEdit()
telemrate_setpointfb = QtGui.QLabel("TELEMRATE FB")
telemrate_send = QtGui.QPushButton("Update telemrate (Hz)")
telemrate_send.clicked.connect(lambda: set("telemrate", ("rs422 "+telemrate_setpoint.text())))
layout.addWidget(telemrate_send, 6, 5)
layout.addWidget(telemrate_setpoint, 6, 6)
layout.addWidget(telemrate_setpointfb, 6, 7)

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
layout.addWidget(kp_set, 9, 5)
layout.addWidget(ki_set, 10, 5)
layout.addWidget(kd_set, 11, 5)
layout.addWidget(kp_input, 9, 6)
layout.addWidget(ki_input, 10, 6)
layout.addWidget(kd_input, 11, 6)
layout.addWidget(kpfb, 9, 7)
layout.addWidget(kifb, 10, 7)
layout.addWidget(kdfb, 11, 7)

# State Feedback
state_label = QtGui.QLabel("STATE = N/A")
arm_button = QtGui.QPushButton("ARM")
disarm_button = QtGui.QPushButton("DISARM")
hotfire_button = QtGui.QPushButton("HOTFIRE")
arm_button.clicked.connect(lambda: send("arm"))
disarm_button.clicked.connect(lambda: send("disarm"))
hotfire_button.clicked.connect(lambda: send("hotfire"))
layout.addWidget(state_label, 12, 5)
layout.addWidget(arm_button, 13, 5)
layout.addWidget(disarm_button, 14, 5)
layout.addWidget(hotfire_button, 15, 5)


# Raw Command
raw_command_input = QtGui.QLineEdit('command entry')
raw_command_send = QtGui.QPushButton("Send Command")
raw_command_send.clicked.connect(lambda: send(raw_command_input.text()))
raw_command_input.returnPressed.connect(lambda: send(raw_command_input.text()))
layout.addWidget(raw_command_input, 16, 5, 1, 2)
layout.addWidget(raw_command_send, 16, 7)

# Board Health
BOARD_HEALTH_LABEL = QtGui.QLabel("Board Health")
ebatt_label =  QtGui.QLabel("BATT")
ibus_label =  QtGui.QLabel("I-BUS")
ebatt_value =  QtGui.QLabel("EBATT")
ibus_value =  QtGui.QLabel("IBUS")
layout.addWidget(BOARD_HEALTH_LABEL, 9, 8)
layout.addWidget(ebatt_label, 10, 8)
layout.addWidget(ibus_label, 11, 8)
layout.addWidget(ebatt_value, 10, 9)
layout.addWidget(ibus_value, 11, 9)

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
layout.addWidget(LOOP_RATE_LABEL, 12, 8)
layout.addWidget(main_cycle_rate_label, 13, 8)
layout.addWidget(motor_cycle_rate_label, 14, 8)
layout.addWidget(adc_cycle_rate_label, 15, 8)
layout.addWidget(telemetry_cycle_rate_label, 16, 8)
# Readout place
layout.addWidget(main_cycle_rate, 13, 9)
layout.addWidget(motor_cycle_rate, 14, 9)
layout.addWidget(adc_cycle_rate, 15, 9)
layout.addWidget(telemetry_cycle_rate, 16, 9)

# counts things for debugging
count1_label = QtGui.QLabel("COUNT1")
count2_label = QtGui.QLabel("COUNT2")
count3_label = QtGui.QLabel("COUNT3")
layout.addWidget(count1_label, 12, 10)
layout.addWidget(count2_label, 13, 10)
layout.addWidget(count3_label, 14, 10)

columns = 9
col_label = []
for n in range(0, columns+1):
	col_label.append(QtGui.QLabel())

col_label[0].setText("Valve OFF")
col_label[1].setText("Valve ON")
col_label[2].setText("State")
col_label[3].setText("Pressure")
col_label[4].setText("Value")
col_label[5].setText("Set Values")
col_label[7].setText("Feedback")
col_label[8].setText("Actual")


layout.addWidget(col_label[0], 0, 0)
layout.addWidget(col_label[1], 0, 1)
layout.addWidget(col_label[2], 0, 2)
layout.addWidget(col_label[3], 0, 3)
layout.addWidget(col_label[4], 0, 4)
layout.addWidget(col_label[5], 0, 5)
layout.addWidget(col_label[6], 0, 6)
layout.addWidget(col_label[7], 0, 7)
layout.addWidget(col_label[8], 0, 8)


def death():
	info_log.close()
	serial_log.close()
	app.quit()

KILL = QtGui.QPushButton("KILL")
KILL.clicked.connect(death)
layout.addWidget(KILL, 0, 10)

# Valve buttons and labels
for n in range(0, 16):
	layout.addWidget(valve_buttons[n][0], n+1, 0)
	layout.addWidget(valve_buttons[n][1], n+1, 1)
	layout.addWidget(valve_buttons[n][2], n+1, 2)
	layout.addWidget(pressure_labels[n][0], n+1, 3)
	layout.addWidget(pressure_labels[n][1], n+1, 4)

if(1):
	# Add image
	logo = QtGui.QLabel(w)
	logo.setGeometry(800, 100, 800, 300)
	#use full ABSOLUTE path to the image, not relative
	logo.setPixmap(QtGui.QPixmap(os.getcwd() + "/masa2.png"))

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