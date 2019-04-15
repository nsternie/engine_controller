from PyQt5 import QtGui
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import pandas as pd
import serial
import struct
import sys
import time
import os

from hotfire_packet import ECParse
from PlotDefinition import PlotDefinition

###############################################################################
###   CONFIGURATION   #########################################################
                                                                             ##       
# Plotting                                                                   ##                 
ground_num_plots = 3                                                         ##                           
flight_num_plots = 3                                                         ##                           
plot_area_height = 30                                                        ##                            
BUFFER_SIZE = 1000                                                           ##                         
                                                                             ##       
                                                                             ##       
                                                                             ##       
###   END CONFIGURATION   #####################################################
###############################################################################

parser = ECParse()
packet_number = 0
ground_packet_number = 0
flight_packet_number = 0

# COMMAND IDs
COMMAND_TARE             =    30
COMMAND_AMBIENTIZE         =    31
COMMAND_DIGITAL_WRITE     =    50
COMMAND_LED_WRITE         =    51
COMMAND_MOTOR_WRITE     =    52
COMMAND_MOTOR_DISABLE     =    53
COMMAND_MOTOR_ENABLE     =    54
COMMAND_QD_SET             =    55
COMMAND_SET_KP             =    60
COMMAND_SET_KI             =    61
COMMAND_SET_KD             =    62
COMMAND_TELEMRATE_SET    =    63
COMMAND_SAMPLERATE_SET    =    64
COMAND_LOGRATE_SET        =    65
COMMAND_ARM                =    100
COMMAND_DISARM            =    101
COMMAND_MAIN_AUTO_START    =    102
TARGET_ADDRESS_GROUND     =    100
TARGET_ADDRESS_FLIGHT     =    101

COMMAND_PRINT_FILE      =   40
COMMAND_NUMFILES        =   41
COMMAND_LOG_START       =   42
COMMAND_LOG_END         =   43
COMMAND_INIT_FS         =   44
COMMAND_TELEM_PAUSE     =   45
COMMAND_TELEM_RESUME   =   46




def s2_command(target_id, command_id, argc, argv):
    global packet_number, ground_packet_number, flight_packet_number;
    packet_number += 1
    command_id = command_id;
    packet = [0]*(8+4*argc+2)
    packet[0] = packet_number >> 8
    packet[1] = packet_number & 0xff
    packet[2] = target_id >> 8
    packet[3] = target_id & 0xff
    packet[4] = command_id >> 8
    packet[5] = command_id & 0xff
    packet[6] = argc >> 8
    packet[7] = argc & 0xff
    for n in range(argc):
        packet[8+4*n] = (argv[n] >> 24) & 0xff
        packet[9+4*n] = (argv[n] >> 16) & 0xff
        packet[10+4*n] = (argv[n] >> 8) & 0xff
        packet[11+4*n] = argv[n] & 0xff
    for n in range(4+2*argc):
        packet[8+4*argc] ^= packet[2*n]
        packet[9+4*argc] ^= packet[2*n+1]
    
    packet = stuff_array(packet, 0)
    tosend = bytes(packet)
    ser.write(tosend)

    print(command_id, argc, argv)

    if(target_id == TARGET_ADDRESS_FLIGHT):
        flight_packet_number += 1
        flight_command_log.write(str(packet)+'\n')


def stuff_array(arr, seperator):
    arr.append(0)
    arr.insert(0, 0)
    first_sep = 1;
    for x in arr[1:]:
        if x == seperator:
            break
        first_sep += 1
    index = 1
    while(index < len(arr)-1):
        if(arr[index] == seperator):
            offset = 1
            while(arr[index+offset] != seperator):
                offset += 1
            arr[index] = offset
            index += offset
        else:
            index += 1
    arr[0] = first_sep
    return arr


def get_file(target_id):
    s2_command(target_id, COMMAND_TELEM_PAUSE, 0, []);
    print("Telemetry paused.")
    time.sleep(0.1)

    response = 1               #
    print("There are "+str(response)+" files to read.")
    filelist = []
    for filenum in range(response):
        print("Downloading file "+str(filenum)+"...")
        mydir = os.getcwd()+"\\data\\"+str(time.strftime("%Y_%m_%d_-_%H%M_%S"))
        os.makedirs(mydir)
        filename = mydir+"\\"+str(filenum)+".bin"
        filelist.append(filename)
        binfile = open(filename, "wb")              # Open the binary
        readfile = True
        s2_command(target_id, COMMAND_PRINT_FILE, 1, [0]);
        while(readfile):
           try:
             page = ser.read(2048)                  # Read a page
             if(len(page) != 2048):
                readfile = False
             binfile.write(page)                    # Log to bin
           except SerialTimeoutException:       # Read pages until the fc stops sending them
             readfile = False                       # Then move on to the next file
           except:
             print("Serial error")                  # Oops
             exit(1)
        binfile.close()                             # Close bin file


    s2_command(target_id, COMMAND_TELEM_RESUME, 0, []);
    print("All files downloaded.")
    print("Telemetry Resumed.")
    os.system("python ec_binary_parser.py "+filename)
    os.system("python csvplot.py "+filename)
    print("")

# Gloabals


run_name = sys.argv[1]

flight_serial_log = open('data/'+run_name+"_flight_serial_log.csv", "w+")
flight_info_log = open('data/'+run_name+"_flight_python_log.csv", "w+")
flight_command_log = open('data/'+run_name+"_flight_command_log.csv", "w+")
flight_data_log = open('data/'+run_name+"_flight_datalog.csv", "w+")
flight_command_log.write("Time, Command/info\n")
flight_data_log.write(parser.csv_header)

###############################################################################
# APPLICATION INITILIZATION ###################################################
###############################################################################
app = QtGui.QApplication([])
flight_window = QtGui.QWidget();
flight_window.setWindowTitle("Flight")
flight_layout = QtGui.QGridLayout()
flight_window.setLayout(flight_layout)

zr = 2
zc = 2


################### PLOT DEV ########################

flight_active_plots = [0]*parser.num_items





def flight_plots_update(item_clicked):
    if(not item_clicked.column() == 2):
        return
    if(flight_active_plots[item_clicked.row()]):
        temp_item = QtGui.QTableWidgetItem("off")
        temp_item.setBackground(QtGui.QColor(250,150,150))
        flight_table.setItem(item_clicked.row(), item_clicked.column(), temp_item)
        flight_active_plots[item_clicked.row()] = False
        flight_plot_1.removeItem(flight_curves[item_clicked.row()])
    else:
        temp_item = QtGui.QTableWidgetItem("on")
        temp_item.setBackground(QtGui.QColor(150, 250, 150))
        flight_table.setItem(item_clicked.row(), item_clicked.column(), temp_item)
        flight_active_plots[item_clicked.row()] = True
        flight_curves[item_clicked.row()] = (flight_plot_1.plot(flight_x[n], flight_y[n], pen=(0,0,255)))


flight_clear_plot = QtGui.QPushButton("Clear plot")
flight_layout.addWidget(flight_clear_plot, zr+0, zc-1)
flight_clear_plot.clicked.connect(lambda: flight_reset_plot())

pg.setConfigOption('background', 'w')
flight_plots = []
for n in range(flight_num_plots):
    flight_plots.append(pg.PlotWidget(title='flight Plots '+str(n+1)))
    flight_layout.addWidget(flight_plots[n], zr+(plot_area_height/flight_num_plots)*n, zc+10, int(plot_area_height/flight_num_plots), 7)



def flight_reset_plot():
    global flight_x, flight_y, BUFFER_SIZE, flight_curves
    flight_y = [[]]*parser.num_items
    flight_x = [[]]*parser.num_items

flight_y = [[]]*parser.num_items
flight_x = [[]]*parser.num_items
flight_curves = [None]*parser.num_items



##################### END PLOT DEV ########################

# Full telemetry table
flight_table = QtGui.QTableWidget()
flight_table.setRowCount(parser.num_items)
flight_table.setColumnCount(3)
for n in range(parser.num_items):
    flight_table.setItem(n,0, QtGui.QTableWidgetItem(parser.items[n]))
    temp_item = QtGui.QTableWidgetItem("off")
    temp_item.setBackground(QtGui.QColor(250,150,150))
    flight_table.setItem(n, 2, temp_item)
flight_layout.addWidget(flight_table,  zr+0, zc+2, 25, 8    )
flight_table.clicked.connect(flight_plots_update)


# Populate the alias dictionary
alias = {}
alias_file = open("devices.alias")
for line in alias_file:
    s = line.split('\t')
    alias[s[0]] = s[1].rstrip('\n')

for info_log in [flight_info_log]:
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

cal_file = open('calibrations')
cal_slope = {}
cal_offset = {}
for line in cal_file:
    s = line.split('\t')
    cal_slope[s[0]] = float(s[1])
    cal_offset[s[0]] = float(s[2].rstrip('\n'))

# Try to open the serial port
ser = serial.Serial(port=None, baudrate=int(alias["BAUDRATE"]), timeout=1.5)
ser.port = alias["COM_PORT"]
try:
    ser.open()
    if(ser.is_open):
        ser.readline()
        print("Port active on "+ser.port)
    else:
        print("Serial port is not open")
except:
    print("Could not open Serial Port")

# ambientize init
f_press = [0]*22
f_ambient = [0]*22

def write_ambient():
    with open('ambient.txt', 'w') as f:
        for p in f_press:
            f.write(str(p) + "\n")

def read_ambient():
    with open('ambient.txt') as f:
        f_ambient = f.read().splitlines()

def ambientize():
    write_ambient()
    read_ambient()
      


###############################################################################
#  END APPLICATION INITILIZATION ##############################################
###############################################################################
# Parse a line and upate GUI fields


write_csv_header = True
def parse_serial():
    global ground_y, ground_x, ground_curves
    # try:
    if(True):
        if(ser.is_open):
            # Read a packet
            packet = ser.readline()
            # print(packet)
            print(len(packet))
            # if not(len(packet) == 139):
            #      os.system("python G:\\Code\\python\\alert_bot.py \"Ivalid packet detected at"+time.ctime()+"\"")
            #     # death()
            #     # exit()
            # if(len(packet) == 0):
            #     os.system("python G:\\Code\\python\\alert_bot.py \"Zero packet detected. Exiting... at"+time.ctime()+"\"")
            #     death()
            #     exit()
            
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

            try:
                parser.parse_packet(packet)
            except:
                print("Packet lost")

            ###################################################################
            ### END GROUND DATA UPDATE ########################################
            ###################################################################

            ###################################################################
            ### FLIGHT DATA UPDATE ############################################
            ###################################################################
            if(parser.BOARD_ID == TARGET_ADDRESS_FLIGHT):
                flight_data_log.write(parser.log_string+'\n')
                flight_serial_log.write(time.strftime("%H:%M:%S"))
                flight_serial_log.write(str(packet)+'length = '+str(len(packet))+'\n')



                if(parser.ebatt < 10):
                    print(len(packet))
                else:
                    # print(str(len(packet))+" - good")
                    pass

                for n in range(parser.num_items-1):
                    flight_y[n].append(parser.dict[parser.items[n]])
                    flight_x[n].append(time.strftime("%H:%M:%S"))
                    flight_y[n] = flight_y[n][-BUFFER_SIZE:]
                    flight_x[n] = flight_x[n][-BUFFER_SIZE:]
                    if flight_active_plots[n]:
                        flight_curves[n].setData(flight_x[n][:], flight_y[n][:])
                        app.processEvents()


                mask = 1
                # Update valve state feedback
                for n in range(0, 32):
                    state = 0
                    if(mask & parser.valve_states):
                        state = 1

                    vlv_id = 'f'+'vlv'+str(n)
                    if vlv_id in alias.keys():
                        vlv_id = str(n) + ': ' + alias[vlv_id]

                    flight_valve_buttons[n][0].setText(vlv_id+" is "+str(state))
                    flight_valve_buttons[n][3].setText(str(parser.ivlv[n])+"A / "+str(parser.evlv[n])+"V")
                    mask = mask << 1
                for n in range(0, 22): # update pressures
                    this_press = parser.pressure[n]
                    if(alias['fpressure'+str(n)] in cal_offset.keys()):
                        this_press -= cal_offset[alias['fpressure'+str(n)]]
                        this_press *= cal_slope[alias['fpressure'+str(n)]]
                    f_press[n] = this_press
                    this_press -= f_ambient[n]
                    this_press = int(this_press)
                    flight_pressure_labels[n][1].setText(str(this_press)+"psi")

                # # Board health
                # flight_ebatt_value.setText(str(parser.ebatt))
                # flight_ibus_value.setText(str(parser.ibus))
                # flight_e5v_value.setText(str(parser.e5v))
                # flight_e3v_value.setText(str(parser.e3v))
                # flight_tbrd_value.setText(str(parser.tbrd))
                # flight_tvlv_value.setText(str(parser.tvlv))
                # flight_tmtr_value.setText(str(parser.tmtr))

                # last_packet_flight.setText(str(parser.last_packet_number))
                # last_command_flight.setText(str(parser.last_command_id))
                # if(parser.last_packet_number == flight_packet_number):
                #     flight_BOARD_HEALTH_LABEL.setText("PACKET GOOD")
                #     pass
                # else:
                #     flight_BOARD_HEALTH_LABEL.setText("PACKET BAD")
                #     pass
                # flight_ibridge0.setText("")
                # flight_ibridge1.setText("")

                # flight_samplerate_send.setText("Samplerate: "+str(parser.samplerate)+"hz")
                # flight_telemrate_send.setText("Telemrate: "+str(parser.telemetry_rate)+"hz")

                state_label.setText("STATE = "+state_dict[parser.STATE])

                # kpfb.setText(str(parser.motor_control_gain[0]))
                # kifb.setText(str(parser.motor_control_gain[1]))
                # kdfb.setText(str(parser.motor_control_gain[2]))

                for n in range(parser.num_items - 1):
                    flight_table.setItem(n, 1, QtGui.QTableWidgetItem(str(parser.dict[parser.items[n]])))



            ###################################################################
            ### END FLIGHT DATA UPDATE ########################################
            ###################################################################

    # except Exception:
    #     pass
        # raise Exception




flight_valve_buttons = []
flight_pressure_labels = []

# lol
for valve_buttons, pressure_labels, abbrev in [(flight_valve_buttons, flight_pressure_labels, 'f')]:
    for n in range(0, 32):

        # Valve wdgets init
        temp = []
        vlv_id = abbrev+'vlv'+str(n)
        if vlv_id in alias.keys():
            vlv_id = alias[vlv_id]
        temp.append(QtGui.QPushButton(str(vlv_id)+' = OFF'))
        temp.append(QtGui.QPushButton(str(vlv_id)+' = ON'))
        temp.append(QtGui.QLabel())
        temp.append(QtGui.QLabel())
        temp.append(QtGui.QLabel())
        valve_buttons.append(temp)

        # Pressure reading widgets init
    for n in range(0, 22):
        ptemp = []
        ptemp.append(QtGui.QLabel())
        ptemp.append(QtGui.QLabel())
        pressure_labels.append(ptemp)
        press_id = abbrev+"pressure"+str(n)
        if press_id in alias.keys():
            press_id = str(n) + ': ' + alias[press_id]
        pressure_labels[n][0].setText(press_id+":")
        pressure_labels[n][1].setText(str(0)+"psi")
    #for n in range(0, 4):
        # Valve wdgets init
        #temp = []
        #vlv_id = abbrev+'vlv'+str(n+8)
        #if vlv_id in alias.keys():
            #vlv_id = alias[vlv_id]
        #temp.append(QtGui.QPushButton(str(vlv_id)+' = OFF'))
        #temp.append(QtGui.QPushButton(str(vlv_id)+' = ON'))
        #temp.append(QtGui.QLabel())
        #temp.append(QtGui.QLabel())
        #temp.append(QtGui.QLabel())
        #valve_buttons.append(temp)



# Board Health
# flight_BOARD_HEALTH_LABEL = QtGui.QLabel("Board Health")
# flight_ebatt_label =  QtGui.QLabel("BUS VOLTAGE:")
# flight_ibus_label =  QtGui.QLabel("BUS CURRENT:")
# flight_e5v_label = QtGui.QLabel("5 VOLT")
# flight_e3v_label = QtGui.QLabel("3.3 VOLT")
# flight_ebatt_value =  QtGui.QLabel("ebatt")
# flight_ibus_value = QtGui.QLabel("ibus")
# flight_e5v_value =  QtGui.QLabel("e5v")
# flight_e3v_value =  QtGui.QLabel("e3v")
# flight_tbrd_label = QtGui.QLabel("BOARD TEMP:")
# flight_tvlv_label = QtGui.QLabel("VALVE TEMP:")
# flight_tmtr_label = QtGui.QLabel("MOTOR TEMP:")
# flight_tbrd_value = QtGui.QLabel("tbrd")
# flight_tvlv_value = QtGui.QLabel("tvlv")
# flight_tmtr_value = QtGui.QLabel("tmtr")
# flight_layout.addWidget(flight_BOARD_HEALTH_LABEL, zr+9+5, zc+1-1-2)
# flight_layout.addWidget(flight_ebatt_label, zr+10+5, zc+1-1-2)
# flight_layout.addWidget(flight_ibus_label, zr+11+5, zc+1-1-2)
# flight_layout.addWidget(flight_ebatt_value, zr+10+5, zc+2-1-2)
# flight_layout.addWidget(flight_ibus_value, zr+11+5, zc+2-1-2)
# flight_layout.addWidget(flight_e5v_label, zr+12+5, zc+1-1-2)
# flight_layout.addWidget(flight_e3v_label, zr+13+5, zc+1-1-2)
# flight_layout.addWidget(flight_e5v_value, zr+12+5, zc+2-1-2)
# flight_layout.addWidget(flight_e3v_value, zr+13+5, zc+2-1-2)
# flight_layout.addWidget(flight_tbrd_label, zr+14+5, zc+1-1-2)
# flight_layout.addWidget(flight_tvlv_label, zr+15+5, zc+1-1-2)
# flight_layout.addWidget(flight_tmtr_label, zr+16+5, zc+1-1-2)
# flight_layout.addWidget(flight_tbrd_value, zr+14+5, zc+2-1-2)
# flight_layout.addWidget(flight_tvlv_value, zr+15+5, zc+2-1-2)
# flight_layout.addWidget(flight_tmtr_value, zr+16+5, zc+2-1-2)

# Last packet

# last_packet_flight_label = QtGui.QLabel("Packet # TX/RX")
# last_command_flight_label = QtGui.QLabel("Last Command id")
# flight_layout.addWidget(last_packet_flight_label, zr+16+5+1, zc-2)
# flight_layout.addWidget(last_command_flight_label, zr+16+5+2, zc-2)

# last_packet_flight = QtGui.QLabel("last packet")
# last_command_flight = QtGui.QLabel("last command")
# flight_layout.addWidget(last_packet_flight, zr+16+5+1, zc-1)
# flight_layout.addWidget(last_command_flight, zr+16+5+2, zc-1)



for n in range(30):
    flight_layout.setRowStretch(zr+n, 10)
############ COLUMN WIDTH FORMATTING
flight_layout.setColumnStretch(0, 10)
flight_layout.setColumnStretch(1, 10)
flight_layout.setColumnStretch(2, 10)
flight_layout.setColumnStretch(3, 10)
flight_layout.setColumnStretch(4, 10)
flight_layout.setColumnStretch(5, 10)
flight_layout.setColumnStretch(6, 10)
flight_layout.setColumnStretch(7, 10)
flight_layout.setColumnStretch(8, 10)
flight_layout.setColumnStretch(9, 10)
flight_layout.setColumnStretch(10, 90)
flight_layout.setColumnStretch(11, 90)
flight_layout.setColumnStretch(12, 90)
flight_layout.setColumnStretch(13, 90)
flight_layout.setColumnStretch(14, 90)
flight_layout.setColumnStretch(15, 90)
flight_layout.setColumnStretch(16, 90)
    # layout.setColumnStretch(zc+17, 10)
    # layout.setColumnStretch(zc+18, 10)
    # layout.setColumnStretch(zc+19, 10)
    # layout.setColumnStretch(zc+20, 10)
    # layout.setColumnStretch(zc+21, 10)

def layout_common_widgets(layout, vlv_buttons, p_labels):
    for n in range(0, 32):
        layout.addWidget(vlv_buttons[n][0], zr+n+1, zc+0-2)
        # layout.addWidget(vlv_buttons[n][1], zr+n+1, zc+1-2)
        # layout.addWidget(vlv_buttons[n][2], zr+n+1, zc+2-2)
        layout.addWidget(vlv_buttons[n][3], zr+n+1, zc-1)
        # layout.addWidget(vlv_buttons[n][4], zr+n+1, zc+0)
    for n in range(0, 22):
        layout.addWidget(p_labels[n][0], zr+n+1, zc+0)
        layout.addWidget(p_labels[n][1], zr+n+1, zc+1)
    # for n in range(0, 4):
    #     layout.addWidget(vlv_buttons[n+32][0], zr+n+1+32, zc+0-2)
    #     layout.addWidget(vlv_buttons[n+32][1], zr+n+1+32, zc+1-2)
    #     layout.addWidget(vlv_buttons[n+32][2], zr+n+1+32, zc+2-2)
    #     layout.addWidget(vlv_buttons[n+32][3], zr+n+1+32, zc+3-2)
    #     layout.addWidget(vlv_buttons[n+32][4], zr+n+1+32, zc+4-2)



layout_common_widgets(flight_layout, flight_valve_buttons, flight_pressure_labels)

#qd_ox_release = QtGui.QPushButton("Release Ox")
#qd_ox_connect = QtGui.QPushButton("Connect Ox")
#qd_ox_off = QtGui.QPushButton("OFF Ox")
#qd_fuel_release = QtGui.QPushButton("Release Fuel")
#qd_fuel_connect = QtGui.QPushButton("Connect Fuel")
#qd_fuel_off = QtGui.QPushButton("OFF Fuel")

### GROUND ###
# init_fs = QtGui.QPushButton("Init Filesystem")
# log_start = QtGui.QPushButton("Log Start")
# log_end = QtGui.QPushButton("Log Stop")
# telem_pause = QtGui.QPushButton("Pause Telemetry")
# telem_resume = QtGui.QPushButton("Resume Telemetry")
# file_download = QtGui.QPushButton("Download file")

# ### Flight ###
# init_fs = QtGui.QPushButton("Init Filesystem")
# log_start = QtGui.QPushButton("Log Start")
# log_end = QtGui.QPushButton("Log Stop")
# telem_pause = QtGui.QPushButton("Pause Telemetry")
# telem_resume = QtGui.QPushButton("Resume Telemetry")
# file_download = QtGui.QPushButton("Download file")

# flight_layout.addWidget(init_fs,zr+14, zc+0)
# flight_layout.addWidget(log_start,zr+15, zc+0)
# flight_layout.addWidget(log_end,zr+16, zc+0)
# flight_layout.addWidget(telem_pause,zr+17, zc+0)
# flight_layout.addWidget(telem_resume,zr+18, zc+0)
# flight_layout.addWidget(file_download,zr+19, zc+0)

# init_fs.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_INIT_FS, 0, []))
# log_start.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LOG_START, 0, []))
# log_end.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LOG_END, 0, []))
# telem_pause.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_TELEM_PAUSE, 0, []))
# telem_resume.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_TELEM_RESUME, 0, []))
# file_download.clicked.connect(lambda: get_file(TARGET_ADDRESS_FLIGHT));

# flight_layout.addWidget(kp_set, zr+9-2, zc+5)
# flight_layout.addWidget(ki_set, zr+10-2, zc+5)
# flight_layout.addWidget(kd_set, zr+11-2, zc+5)
# flight_layout.addWidget(kp_input, zr+9-2, zc+6)
# flight_layout.addWidget(ki_input, zr+10-2, zc+6)
# flight_layout.addWidget(kd_input, zr+11-2, zc+6)
# flight_layout.addWidget(kpfb, zr+9-2, zc+7)
# flight_layout.addWidget(kifb, zr+10-2, zc+7)
# flight_layout.addWidget(kdfb, zr+11-2, zc+7)

# Bridge current
#flight_ibridge0 = QtGui.QLabel("flight_ibridge0")
#flight_ibridge1 = QtGui.QLabel("flight_ibridge1")
# flight_layout.addWidget(flight_ibridge0, zr+24, zc-2, 1, 2)
# flight_layout.addWidget(flight_ibridge1, zr+25, zc-2, 1, 2)

# ambientize buttons
ambientize_button = QtGui.QPushButton("Ambientize")
flight_layout.addWidget(ambientize_button, zr+23, zc+0)
ambientize_button.clicked.connect(ambientize)

load_ambients_button = QtGui.QPushButton("Load Ambients")
flight_layout.addWidget(load_ambients_button, zr+24, zc-0)
load_ambients_button.clicked.connect(read_ambient)

# State Feedback
state_label = QtGui.QLabel("STATE = N/A")
arm_button = QtGui.QPushButton("ARM")
disarm_button = QtGui.QPushButton("DISARM")
hotfire_button = QtGui.QPushButton("HOTFIRE")
flight_layout.addWidget(state_label, zr+26, zc+0)
flight_layout.addWidget(arm_button, zr+27, zc+0)
flight_layout.addWidget(disarm_button, zr+28, zc+0)
flight_layout.addWidget(hotfire_button, zr+29, zc+0)

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

#kill runt
def death():
    flight_command_log.close()
    flight_info_log.close()
    flight_serial_log.close()
    flight_data_log.close()
    app.quit()

KILL2 = QtGui.QPushButton("End Run")
flight_layout.addWidget(KILL2, zr+0, zc-2)
KILL2.clicked.connect(death)

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


# font_list = []
# font_list.append(flight_BOARD_HEALTH_LABEL)
# font_list.append(flight_ebatt_label)
# font_list.append(flight_ibus_label)
# font_list.append(flight_e5v_label )
# font_list.append(flight_e3v_label )
# font_list.append(flight_ebatt_value)
# font_list.append(flight_ibus_value)
# font_list.append(flight_e5v_value )
# font_list.append(flight_e3v_value )
# font_list.append(flight_tbrd_label)
# font_list.append(flight_tvlv_label)
# font_list.append(flight_tmtr_label)
# font_list.append(flight_tbrd_value)
# font_list.append(flight_tvlv_value)
# font_list.append(flight_tmtr_value)
# font_list.append(last_packet_flight)
# font_list.append(last_command_flight)
# font_list.append(last_packet_flight_label)
# font_list.append(last_command_flight_label)
# font_list.append(flight_ibridge0)
# font_list.append(flight_ibridge1)

# for thing in font_list:
#     thing.setFont(QtGui.QFont('SansSerif', 14))


###############################################################################
### FUNCTIONAL CONNECTIONS ####################################################
###############################################################################


arm_button.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_ARM, 0, []))
disarm_button.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_DISARM, 0, []))
hotfire_button.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_MAIN_AUTO_START, 0, []))

def toggle_valve(board, vlv_id):
    if board is 'flight':
        state = int(flight_valve_buttons[vlv_id][0].text()[-1])
        state = int(not state)
        s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_DIGITAL_WRITE, 2, [vlv_id, state])

 # GROUND VALVES

# # FLIGHT VALVES
flight_valve_buttons[0][0].clicked.connect(lambda: toggle_valve('flight', 0))
flight_valve_buttons[1][0].clicked.connect(lambda: toggle_valve('flight', 1))
flight_valve_buttons[2][0].clicked.connect(lambda: toggle_valve('flight', 2))
flight_valve_buttons[3][0].clicked.connect(lambda: toggle_valve('flight', 3))
flight_valve_buttons[4][0].clicked.connect(lambda: toggle_valve('flight', 4))
flight_valve_buttons[5][0].clicked.connect(lambda: toggle_valve('flight', 5))
flight_valve_buttons[6][0].clicked.connect(lambda: toggle_valve('flight', 6))
flight_valve_buttons[7][0].clicked.connect(lambda: toggle_valve('flight', 7))
flight_valve_buttons[8][0].clicked.connect(lambda: toggle_valve('flight', 8))
flight_valve_buttons[9][0].clicked.connect(lambda: toggle_valve('flight', 9))
flight_valve_buttons[10][0].clicked.connect(lambda: toggle_valve('flight', 10))
flight_valve_buttons[11][0].clicked.connect(lambda: toggle_valve('flight', 11))
flight_valve_buttons[12][0].clicked.connect(lambda: toggle_valve('flight', 12))
flight_valve_buttons[13][0].clicked.connect(lambda: toggle_valve('flight', 13))
flight_valve_buttons[14][0].clicked.connect(lambda: toggle_valve('flight', 14))
flight_valve_buttons[15][0].clicked.connect(lambda: toggle_valve('flight', 15))
flight_valve_buttons[16][0].clicked.connect(lambda: toggle_valve('flight', 16))
flight_valve_buttons[17][0].clicked.connect(lambda: toggle_valve('flight', 17))
flight_valve_buttons[18][0].clicked.connect(lambda: toggle_valve('flight', 18))
flight_valve_buttons[19][0].clicked.connect(lambda: toggle_valve('flight', 19))
flight_valve_buttons[20][0].clicked.connect(lambda: toggle_valve('flight', 20))
flight_valve_buttons[21][0].clicked.connect(lambda: toggle_valve('flight', 21))
flight_valve_buttons[22][0].clicked.connect(lambda: toggle_valve('flight', 22))
flight_valve_buttons[23][0].clicked.connect(lambda: toggle_valve('flight', 23))
flight_valve_buttons[24][0].clicked.connect(lambda: toggle_valve('flight', 24))
flight_valve_buttons[25][0].clicked.connect(lambda: toggle_valve('flight', 25))
flight_valve_buttons[26][0].clicked.connect(lambda: toggle_valve('flight', 26))
flight_valve_buttons[27][0].clicked.connect(lambda: toggle_valve('flight', 27))
flight_valve_buttons[28][0].clicked.connect(lambda: toggle_valve('flight', 28))
flight_valve_buttons[29][0].clicked.connect(lambda: toggle_valve('flight', 29))
flight_valve_buttons[30][0].clicked.connect(lambda: toggle_valve('flight', 30))
flight_valve_buttons[31][0].clicked.connect(lambda: toggle_valve('flight', 31))

flight_valve_buttons[0][1].clicked.connect(lambda: toggle_valve('flight', 0))
flight_valve_buttons[1][1].clicked.connect(lambda: toggle_valve('flight', 1))
flight_valve_buttons[2][1].clicked.connect(lambda: toggle_valve('flight', 2))
flight_valve_buttons[3][1].clicked.connect(lambda: toggle_valve('flight', 3))
flight_valve_buttons[4][1].clicked.connect(lambda: toggle_valve('flight', 4))
flight_valve_buttons[5][1].clicked.connect(lambda: toggle_valve('flight', 5))
flight_valve_buttons[6][1].clicked.connect(lambda: toggle_valve('flight', 6))
flight_valve_buttons[7][1].clicked.connect(lambda: toggle_valve('flight', 7))
flight_valve_buttons[8][1].clicked.connect(lambda: toggle_valve('flight', 8))
flight_valve_buttons[9][1].clicked.connect(lambda: toggle_valve('flight', 9))
flight_valve_buttons[10][1].clicked.connect(lambda: toggle_valve('flight', 10))
flight_valve_buttons[11][1].clicked.connect(lambda: toggle_valve('flight', 11))
flight_valve_buttons[12][1].clicked.connect(lambda: toggle_valve('flight', 12))
flight_valve_buttons[13][1].clicked.connect(lambda: toggle_valve('flight', 13))
flight_valve_buttons[14][1].clicked.connect(lambda: toggle_valve('flight', 14))
flight_valve_buttons[15][1].clicked.connect(lambda: toggle_valve('flight', 15))
flight_valve_buttons[16][1].clicked.connect(lambda: toggle_valve('flight', 16))
flight_valve_buttons[17][1].clicked.connect(lambda: toggle_valve('flight', 17))
flight_valve_buttons[18][1].clicked.connect(lambda: toggle_valve('flight', 18))
flight_valve_buttons[19][1].clicked.connect(lambda: toggle_valve('flight', 19))
flight_valve_buttons[20][1].clicked.connect(lambda: toggle_valve('flight', 20))
flight_valve_buttons[21][1].clicked.connect(lambda: toggle_valve('flight', 21))
flight_valve_buttons[22][1].clicked.connect(lambda: toggle_valve('flight', 22))
flight_valve_buttons[23][1].clicked.connect(lambda: toggle_valve('flight', 23))
flight_valve_buttons[24][1].clicked.connect(lambda: toggle_valve('flight', 24))
flight_valve_buttons[25][1].clicked.connect(lambda: toggle_valve('flight', 25))
flight_valve_buttons[26][1].clicked.connect(lambda: toggle_valve('flight', 26))
flight_valve_buttons[27][1].clicked.connect(lambda: toggle_valve('flight', 27))
flight_valve_buttons[28][1].clicked.connect(lambda: toggle_valve('flight', 28))
flight_valve_buttons[29][1].clicked.connect(lambda: toggle_valve('flight', 29))
flight_valve_buttons[30][1].clicked.connect(lambda: toggle_valve('flight', 30))
flight_valve_buttons[31][1].clicked.connect(lambda: toggle_valve('flight', 31))
# # GROUND LEDS1
# # FLIGHT LEDS
# flight_valve_buttons[8][1].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [0,1]))
# flight_valve_buttons[9][1].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [1,1]))
# flight_valve_buttons[10][1].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [2,1]))
# flight_valve_buttons[11][1].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [3,1]))
# flight_valve_buttons[8][0].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [0,0]))
# flight_valve_buttons[9][0].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [1,0]))
# flight_valve_buttons[10][0].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [2,0]))
# flight_valve_buttons[11][0].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [3,0]))


##############################################################################
### END FUNCTIONAL CONNECTIONS ###############################################
##############################################################################

# Start
flight_window.showMaximized()



timer2 = pg.QtCore.QTimer()
timer2.timeout.connect(parse_serial)
timer2.start(10) # 100hz for 10 as arg
# test = True
# def test_qd():
#     global test
#     if(parser.motor_pwm[1] == 20000 and test):
#         if(parser.ibus < 0.4):
#             test = False
#         s2_command(TARGET_ADDRESS_GROUND, COMMAND_QD_SET, 2, [1, -1])
#     elif(parser.motor_pwm[1] == -20000 and test):
#         if(parser.ibus < 0.4):
#             test = False
#         s2_command(TARGET_ADDRESS_GROUND, COMMAND_QD_SET, 2, [1, 0])
#     elif(test):
#         s2_command(TARGET_ADDRESS_GROUND, COMMAND_QD_SET, 2, [1, 1])
#     else:
#         pass
# timer3 = pg.QtCore.QTimer()
# timer3.timeout.connect(test_qd)
# timer3.start(2000) # 100hz for 10 as arg
parity = True
def reset_watchdog():
    try:
        global parity
        if parity:
            s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [2,0])
            parity = False
        else:
            s2_command(TARGET_ADDRESS_GROUND, COMMAND_LED_WRITE, 2, [2,0])
            parity = True
    except:
        pass

watchdog_reset_timer = pg.QtCore.QTimer()
watchdog_reset_timer.timeout.connect(reset_watchdog)
watchdog_reset_timer.start(250) 

# def flight_reset_watchdog():
#     s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [2,0])

# flight_timer = pg.QtCore.QTimer()
# flight_timer.timeout.connect(flight_reset_watchdog)
# flight_timer.start(500) 

app.exec_()