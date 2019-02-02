from PyQt5 import QtGui
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import serial
import time
import os
from hotfire_packet import ECParse
import struct

parser = ECParse()

# Gloabals
mtr = ['mtr0', 'mtr1', 'mtr2', 'mtr3']
mtr_enable = []
mtr_disable = []
mtr_setpoint = []
mtr_position = []
mtr_pwm = []
mtr_send = []
mtr_setpointfb = []


run_name = input("Enter run name: ")

serial_log = open('data/'+run_name+"_serial_log.csv", "w+")
info_log = open('data/'+run_name+"_python_log.csv", "w+")
command_log = open('data/'+run_name+"_command_log.csv", "w+")
data_log = open('data/'+run_name+"_datalog.csv", "w+")

command_log.write("Time, Command/info\n")
data_log.write(parser.csv_header)

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
	print("INVALID STATE ALIAS DEFINITIONS")

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

	try:
		if(ser.is_open):
				# Read a packet
			packet = ser.readline()	
			# Unstuff the packet
			unstuffed = b''
			index = int(packet[0])
			for n in range(1, len(packet)):
				temp = packet[n:n+1]
				if(n == index):
					index = int(packet[n])+n
					temp = b'\n'
				unstuffed = unstuffed + temp
			packet = unstuffed
			#line = str(line, 'ascii')
			#try:
				#split_line = line.split(',')
			try:
				parser.parse_packet(packet)
			except:
				print("Parser error")
				info_log.write(time.ctime()+" parser error\n")
			data_log.write(parser.log_string+'\n')
			serial_log.write("%.3f," % time.clock())
			serial_log.write(str(packet)+'\n')
			# except:
			# 	print("Error")
			# 	pass

			state_label.setText("STATE = "+state_dict[parser.STATE])

			log_to_auto_label.setText("Logging to auto: "+str(parser.LOG_TO_AUTO))
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
				if(mask & parser.auto_states):
					running_autos_string += (str(n)+", ")
				mask = mask << 1

				running_autos_label.setText(running_autos_string)
			# print("Packet parsed")
			# print("battery: "+str(ebatt)+" \t and %.2f" % time.clock())

			mask = 1
			# Update valve state feedback
			for n in range(0, 16):
				state = 0
				if(mask & parser.valve_states):
					state = 1
				valve_buttons[n][2].setText(str(state))
				valve_buttons[n][3].setText(str(parser.ivlv[n]))
				valve_buttons[n][4].setText(str(parser.evlv[n]))
				mask = mask << 1

				pressure_labels[n][1].setText(str(parser.pressure[n])+"psi")
			# Update loop rates
			samplerate_setpointfb.setText(str(parser.samplerate)+"hz")
			telemrate_setpointfb.setText(str(parser.telemetry_rate)+"hz")
			for mtrx in range(0, 4):
				try:
					mtr_setpointfb[mtrx].setText(str(parser.motor_setpoint[mtrx]))
					mtr_position[mtrx].setText(str(parser.motor_position[mtrx]))
					mtr_pwm[mtrx].setText("PWM: "+str(parser.motor_pwm[mtrx]))
				except:
					pass

			#main_cycle_rate.setText(str(round(1000000/main_cycle_time, 3)))
			motor_cycle_rate.setText(str(round(1000000/parser.motor_cycle_time, 3)))
			adc_cycle_rate.setText(str(round(1000000/parser.adc_cycle_time, 3)))
			telemetry_cycle_rate.setText(str(round(1000000/parser.telemetry_cycle_time, 3)))

			# Board health
			ebatt_value.setText(str(parser.ebatt))
			ibus_value.setText(str(parser.ibus))

			# motor gain feedback
			kpfb.setText(str(parser.motor_control_gain[0]))
			kifb.setText(str(parser.motor_control_gain[1]))
			kdfb.setText(str(parser.motor_control_gain[2]))

			count1_label.setText("count1: "+str(parser.count1))
			count2_label.setText("count2: "+str(parser.count2))
			count3_label.setText("count3: "+str(parser.count3))

			state_label.setText("STATE = "+state_dict[parser.STATE])

			thrust_load_label.setText("Thrust = "+str(parser.thrust_load))
			for n in range(0, 4):
				load_label[n].setText(str(n)+": "+str(parser.load[n]))
			for n in range(0, 4):
				tc_label[n].setText("TC-"+str(n)+": "+str(parser.thermocouple[n]))
	except Exception:
		print(Exception)


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
layout.addWidget(autofeedback, zr+1, zc+10, 4, 3)
layout.addWidget(log_to_auto_label, zr+5, zc+10)
layout.addWidget(running_autos_label, zr+6, zc+10, 1, 2)

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