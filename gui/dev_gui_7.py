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

    if(target_id == TARGET_ADDRESS_GROUND):
        ground_packet_number += 1
        ground_command_log.write(str(packet)+'\n')
    if(target_id == TARGET_ADDRESS_FLIGHT):
        flight_packet_number += 1
        ground_command_log.write(str(packet)+'\n')


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
mtr = ['mtr0', 'mtr1', 'mtr2', 'mtr3']
mtr_toggle = []
mtr_setpoint = []
mtr_position = []
mtr_pwm = []
mtr_send = []
mtr_setpointfb = []

run_name = sys.argv[1]
ground_serial_log = open('data/'+run_name+"_ground_serial_log.csv", "w+")
ground_info_log = open('data/'+run_name+"_ground_python_log.csv", "w+")
ground_command_log = open('data/'+run_name+"_ground_command_log.csv", "w+")
ground_data_log = open('data/'+run_name+"_ground_datalog.csv", "w+")
ground_command_log.write("Time, Command/info\n")
ground_data_log.write(parser.csv_header)
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
ground_window = QtGui.QWidget()
ground_window.setWindowTitle("Ground")
ground_layout = QtGui.QGridLayout()
ground_window.setLayout(ground_layout)

flight_window = QtGui.QWidget();
flight_window.setWindowTitle("Flight")
flight_layout = QtGui.QGridLayout()
flight_window.setLayout(flight_layout)

zr = 2
zc = 2


################### PLOT DEV ########################

ground_active_plots = [0]*parser.num_items
flight_active_plots = [0]*parser.num_items


def ground_plots_update(item_clicked):
    if(not item_clicked.column() == 2):
        return
    if(ground_active_plots[item_clicked.row()] == ground_num_plots):
        ground_active_plots[item_clicked.row()] = 0
        temp_item = QtGui.QTableWidgetItem('off')
        temp_item.setBackground(QtGui.QColor(250,150,150))
        ground_table.setItem(item_clicked.row(), item_clicked.column(), temp_item)
        ground_plots[ground_active_plots[item_clicked.row()]-1].removeItem(ground_curves[item_clicked.row()])
    else:
        ground_active_plots[item_clicked.row()] += 1
        temp_item = QtGui.QTableWidgetItem(str(ground_active_plots[item_clicked.row()]))
        temp_item.setBackground(QtGui.QColor(150, 250, 150))
        ground_table.setItem(item_clicked.row(), item_clicked.column(), temp_item)
        ground_curves[item_clicked.row()] = (ground_plots[ground_active_plots[item_clicked.row()]-1].plot(ground_x[n], ground_y[n], pen=(0,0,255)))



ground_clear_plot = QtGui.QPushButton("Clear plot")
ground_layout.addWidget(ground_clear_plot, zr+0, zc-1)
ground_clear_plot.clicked.connect(lambda: ground_reset_plot())

pg.setConfigOption('background', 'w')
ground_plots = []
for n in range(ground_num_plots):
    ground_plots.append(pg.PlotWidget(title='Ground Plots '+str(n+1)))
    ground_layout.addWidget(ground_plots[n], zr+(plot_area_height/ground_num_plots)*n, zc+10, int(plot_area_height/ground_num_plots), 7)

def reset_plot():
    global ground_x, ground_y, BUFFER_SIZE, ground_curves
    ground_y = [[]]*parser.num_items
    ground_x = [[]]*parser.num_items


ground_y = [[]]*parser.num_items
ground_x = [[]]*parser.num_items
ground_curves = [None]*parser.num_items


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

def ground_reset_plot():
    global flight_x, flight_y, BUFFER_SIZE, ground_curves
    ground_y = [[]]*parser.num_items
    ground_x = [[]]*parser.num_items

def flight_reset_plot():
    global flight_x, flight_y, BUFFER_SIZE, flight_curves
    flight_y = [[]]*parser.num_items
    flight_x = [[]]*parser.num_items

flight_y = [[]]*parser.num_items
flight_x = [[]]*parser.num_items
flight_curves = [None]*parser.num_items



##################### END PLOT DEV ########################

# Full telemetry table
ground_table = QtGui.QTableWidget()
ground_table.setRowCount(parser.num_items)
ground_table.setColumnCount(3)
for n in range(parser.num_items):
    ground_table.setItem(n,0, QtGui.QTableWidgetItem(parser.items[n]))
    temp_item = QtGui.QTableWidgetItem("off")
    temp_item.setBackground(QtGui.QColor(250,150,150))
    ground_table.setItem(n, 2, temp_item)
ground_layout.addWidget(ground_table, zr+0, zc+2, 25, 8)
ground_table.clicked.connect(ground_plots_update)

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

for info_log in [ground_info_log, flight_info_log]:
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
ser = serial.Serial(port=None, baudrate=int(alias["BAUDRATE"]), timeout=0.2)
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
            # print(len(packet))
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

            parser.parse_packet(packet)


            ###################################################################
            ### GROUND DATA UPDATE ############################################
            ###################################################################
            if(parser.BOARD_ID == TARGET_ADDRESS_GROUND):
                ground_data_log.write(parser.log_string+'\n')
                ground_serial_log.write(time.strftime("%H:%M:%S"))
                ground_serial_log.write(str(packet)+'length = '+str(len(packet))+'\n')

                for n in range(parser.num_items-1):
                    ground_y[n].append(parser.dict[parser.items[n]])
                    ground_x[n].append(time.strftime("%H:%M:%S"))
                    ground_y[n] = ground_y[n][-BUFFER_SIZE:]
                    ground_x[n] = ground_x[n][-BUFFER_SIZE:]
                    if ground_active_plots[n]:
                        ground_curves[n].setData(ground_x[n][:], ground_y[n][:])
                        app.processEvents()

                mask = 1
                # Update valve state feedback
                for n in range(0, 8):
                    state = 0
                    if(mask & parser.valve_states):
                        state = 1

                    vlv_id = 'g'+'vlv'+str(n)
                    if vlv_id in alias.keys():
                        vlv_id = alias[vlv_id]

                    ground_valve_buttons[n][0].setText(vlv_id+" is "+str(state))
                    ground_valve_buttons[n][3].setText(str(parser.ivlv[n])+"A / "+str(parser.evlv[n])+"V")
                    
                    mask = mask << 1

                    ground_pressure_labels[n][1].setText(str(parser.pressure[n])+"psi")
                # Board health
                ground_ebatt_value.setText(str(parser.ebatt))
                ground_ibus_value.setText(str(parser.ibus))
                ground_e5v_value.setText(str(parser.e5v))
                ground_e3v_value.setText(str(parser.e3v))
                ground_tbrd_value.setText(str(parser.tbrd))
                ground_tvlv_value.setText(str(parser.tvlv))
                ground_tmtr_value.setText(str(parser.tmtr))

                last_packet_ground.setText(str(parser.last_packet_number))
                last_command_ground.setText(str(parser.last_command_id))

                ground_ibridge0.setText("Motor 0 Current: "+str(parser.imtr[0]))
                ground_ibridge1.setText("Motor 1 Current: "+str(parser.imtr[1]))

                ground_samplerate_send.setText("Samplerate: "+str(parser.samplerate)+"hz")
                ground_telemrate_send.setText("Telemrate: "+str(parser.telemetry_rate)+"hz")

                for n in range(parser.num_items - 1):
                    ground_table.setItem(n, 1, QtGui.QTableWidgetItem(str(parser.dict[parser.items[n]])))


                if(parser.last_packet_number == ground_packet_number):
                    ground_BOARD_HEALTH_LABEL.setText("PACKET GOOD")
                    pass
                else:
                    ground_BOARD_HEALTH_LABEL.setText("PACKET BAD")
                    pass
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
                for n in range(0, 8):
                    state = 0
                    if(mask & parser.valve_states):
                        state = 1

                    vlv_id = 'f'+'vlv'+str(n)
                    if vlv_id in alias.keys():
                        vlv_id = alias[vlv_id]

                    flight_valve_buttons[n][0].setText(vlv_id+" is "+str(state))
                    flight_valve_buttons[n][3].setText(str(parser.ivlv[n])+"A / "+str(parser.evlv[n])+"V")
                    mask = mask << 1

                    flight_pressure_labels[n][1].setText(str(parser.pressure[n])+"psi")

                # Board health
                flight_ebatt_value.setText(str(parser.ebatt))
                flight_ibus_value.setText(str(parser.ibus))
                flight_e5v_value.setText(str(parser.e5v))
                flight_e3v_value.setText(str(parser.e3v))
                flight_tbrd_value.setText(str(parser.tbrd))
                flight_tvlv_value.setText(str(parser.tvlv))
                flight_tmtr_value.setText(str(parser.tmtr))

                last_packet_flight.setText(str(parser.last_packet_number))
                last_command_flight.setText(str(parser.last_command_id))
                if(parser.last_packet_number == flight_packet_number):
                    flight_BOARD_HEALTH_LABEL.setText("PACKET GOOD")
                    pass
                else:
                    flight_BOARD_HEALTH_LABEL.setText("PACKET BAD")
                    pass
                flight_ibridge0.setText("Motor 0 Current: "+str(parser.imtr[0]))
                flight_ibridge1.setText("Motor 1 Current: "+str(parser.imtr[1]))

                flight_samplerate_send.setText("Samplerate: "+str(parser.samplerate)+"hz")
                flight_telemrate_send.setText("Telemrate: "+str(parser.telemetry_rate)+"hz")

                state_label.setText("STATE = "+state_dict[parser.STATE])

                kpfb.setText(str(parser.motor_control_gain[0]))
                kifb.setText(str(parser.motor_control_gain[1]))
                kdfb.setText(str(parser.motor_control_gain[2]))

                for n in range(parser.num_items - 1):
                    flight_table.setItem(n, 1, QtGui.QTableWidgetItem(str(parser.dict[parser.items[n]])))


                for mtrx in range(0, 2):
                    try:
                        mtr_send[mtrx].setText("Update "+str(parser.motor_setpoint[mtrx]))
                        mtr_position[mtrx].setText(str(parser.motor_position[mtrx]))
                        mtr_toggle[mtrx].setText(str(parser.motor_pwm[mtrx]))
                    except:
                        pass
                try:
                    motor_cycle_rate.setText(str(round(1000000/parser.motor_cycle_time, 3)))
                    adc_cycle_rate.setText(str(round(1000000/parser.adc_cycle_time, 3)))
                    telemetry_cycle_rate.setText(str(round(1000000/parser.telemetry_cycle_time, 3)))
                except:
                    pass

            ###################################################################
            ### END FLIGHT DATA UPDATE ########################################
            ###################################################################

    # except Exception:
    #     pass
        # raise Exception



ground_valve_buttons = []
flight_valve_buttons = []
ground_pressure_labels = []
flight_pressure_labels = []

# lol
for valve_buttons, pressure_labels, abbrev in [(ground_valve_buttons, ground_pressure_labels, 'g'), (flight_valve_buttons, flight_pressure_labels, 'f')]:
    for n in range(0, 8):

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
        ptemp = []
        ptemp.append(QtGui.QLabel())
        ptemp.append(QtGui.QLabel())
        pressure_labels.append(ptemp)
        press_id = abbrev+"pressure"+str(n)
        if press_id in alias.keys():
            press_id = alias[press_id]
        pressure_labels[n][0].setText(press_id+":")
        pressure_labels[n][1].setText(str(0)+"psi")
    for n in range(0, 4):
        # Valve wdgets init
        temp = []
        vlv_id = abbrev+'vlv'+str(n+8)
        if vlv_id in alias.keys():
            vlv_id = alias[vlv_id]
        temp.append(QtGui.QPushButton(str(vlv_id)+' = OFF'))
        temp.append(QtGui.QPushButton(str(vlv_id)+' = ON'))
        temp.append(QtGui.QLabel())
        temp.append(QtGui.QLabel())
        temp.append(QtGui.QLabel())
        valve_buttons.append(temp)

# # Board Health
ground_BOARD_HEALTH_LABEL = QtGui.QLabel("Board Health")
# ground_BOARD_HEALTH_LABEL.setStyleSheet('color: red')
ground_ebatt_label =  QtGui.QLabel("BUS VOLTAGE:")
ground_ibus_label =  QtGui.QLabel("BUS CURRENT:")
ground_e5v_label = QtGui.QLabel("5 VOLT")
ground_e3v_label = QtGui.QLabel("3.3 VOLT")
ground_ebatt_value =  QtGui.QLabel("ebatt")
ground_ibus_value = QtGui.QLabel("ibus")
ground_e5v_value =  QtGui.QLabel("e5v")
ground_e3v_value =  QtGui.QLabel("e3v")
ground_tbrd_label = QtGui.QLabel("BOARD TEMP:")
ground_tvlv_label = QtGui.QLabel("VALVE TEMP:")
ground_tmtr_label = QtGui.QLabel("MOTOR TEMP:")
ground_tbrd_value = QtGui.QLabel("tbrd")
ground_tvlv_value = QtGui.QLabel("tvlv")
ground_tmtr_value = QtGui.QLabel("tmtr")
ground_layout.addWidget(ground_BOARD_HEALTH_LABEL, zr+9+5, zc+1-1-2)
ground_layout.addWidget(ground_ebatt_label, zr+10+5, zc+1-1-2)
ground_layout.addWidget(ground_ibus_label, zr+11+5, zc+1-1-2)
ground_layout.addWidget(ground_ebatt_value, zr+10+5, zc+2-1-2)
ground_layout.addWidget(ground_ibus_value, zr+11+5, zc+2-1-2)
ground_layout.addWidget(ground_e5v_label, zr+12+5, zc+1-1-2)
ground_layout.addWidget(ground_e3v_label, zr+13+5, zc+1-1-2)
ground_layout.addWidget(ground_e5v_value, zr+12+5, zc+2-1-2)
ground_layout.addWidget(ground_e3v_value, zr+13+5, zc+2-1-2)
ground_layout.addWidget(ground_tbrd_label, zr+14+5, zc+1-1-2)
ground_layout.addWidget(ground_tvlv_label, zr+15+5, zc+1-1-2)
ground_layout.addWidget(ground_tmtr_label, zr+16+5, zc+1-1-2)
ground_layout.addWidget(ground_tbrd_value, zr+14+5, zc+2-1-2)
ground_layout.addWidget(ground_tvlv_value, zr+15+5, zc+2-1-2)
ground_layout.addWidget(ground_tmtr_value, zr+16+5, zc+2-1-2)

# Board Health
flight_BOARD_HEALTH_LABEL = QtGui.QLabel("Board Health")
flight_ebatt_label =  QtGui.QLabel("BUS VOLTAGE:")
flight_ibus_label =  QtGui.QLabel("BUS CURRENT:")
flight_e5v_label = QtGui.QLabel("5 VOLT")
flight_e3v_label = QtGui.QLabel("3.3 VOLT")
flight_ebatt_value =  QtGui.QLabel("ebatt")
flight_ibus_value = QtGui.QLabel("ibus")
flight_e5v_value =  QtGui.QLabel("e5v")
flight_e3v_value =  QtGui.QLabel("e3v")
flight_tbrd_label = QtGui.QLabel("BOARD TEMP:")
flight_tvlv_label = QtGui.QLabel("VALVE TEMP:")
flight_tmtr_label = QtGui.QLabel("MOTOR TEMP:")
flight_tbrd_value = QtGui.QLabel("tbrd")
flight_tvlv_value = QtGui.QLabel("tvlv")
flight_tmtr_value = QtGui.QLabel("tmtr")
flight_layout.addWidget(flight_BOARD_HEALTH_LABEL, zr+9+5, zc+1-1-2)
flight_layout.addWidget(flight_ebatt_label, zr+10+5, zc+1-1-2)
flight_layout.addWidget(flight_ibus_label, zr+11+5, zc+1-1-2)
flight_layout.addWidget(flight_ebatt_value, zr+10+5, zc+2-1-2)
flight_layout.addWidget(flight_ibus_value, zr+11+5, zc+2-1-2)
flight_layout.addWidget(flight_e5v_label, zr+12+5, zc+1-1-2)
flight_layout.addWidget(flight_e3v_label, zr+13+5, zc+1-1-2)
flight_layout.addWidget(flight_e5v_value, zr+12+5, zc+2-1-2)
flight_layout.addWidget(flight_e3v_value, zr+13+5, zc+2-1-2)
flight_layout.addWidget(flight_tbrd_label, zr+14+5, zc+1-1-2)
flight_layout.addWidget(flight_tvlv_label, zr+15+5, zc+1-1-2)
flight_layout.addWidget(flight_tmtr_label, zr+16+5, zc+1-1-2)
flight_layout.addWidget(flight_tbrd_value, zr+14+5, zc+2-1-2)
flight_layout.addWidget(flight_tvlv_value, zr+15+5, zc+2-1-2)
flight_layout.addWidget(flight_tmtr_value, zr+16+5, zc+2-1-2)

# Last packet
last_packet_ground_label = QtGui.QLabel("Packet # TX/RX")
last_command_ground_label = QtGui.QLabel("Last Command id")
ground_layout.addWidget(last_packet_ground_label, zr+16+5+1, zc-2)
ground_layout.addWidget(last_command_ground_label, zr+16+5+2, zc-2)
last_packet_flight_label = QtGui.QLabel("Packet # TX/RX")
last_command_flight_label = QtGui.QLabel("Last Command id")
flight_layout.addWidget(last_packet_flight_label, zr+16+5+1, zc-2)
flight_layout.addWidget(last_command_flight_label, zr+16+5+2, zc-2)

last_packet_ground = QtGui.QLabel("last packet")
last_command_ground = QtGui.QLabel("last command")
ground_layout.addWidget(last_packet_ground, zr+16+5+1, zc-1)
ground_layout.addWidget(last_command_ground, zr+16+5+2, zc-1)
last_packet_flight = QtGui.QLabel("last packet")
last_command_flight = QtGui.QLabel("last command")
flight_layout.addWidget(last_packet_flight, zr+16+5+1, zc-1)
flight_layout.addWidget(last_command_flight, zr+16+5+2, zc-1)




for layout in (ground_layout, flight_layout):
    for n in range(30):
        layout.setRowStretch(zr+n, 10)
    ############ COLUMN WIDTH FORMATTING
    layout.setColumnStretch(0, 10)
    layout.setColumnStretch(1, 10)
    layout.setColumnStretch(2, 10)
    layout.setColumnStretch(3, 10)
    layout.setColumnStretch(4, 10)
    layout.setColumnStretch(5, 10)
    layout.setColumnStretch(6, 10)
    layout.setColumnStretch(7, 10)
    layout.setColumnStretch(8, 10)
    layout.setColumnStretch(9, 10)
    layout.setColumnStretch(10, 90)
    layout.setColumnStretch(11, 90)
    layout.setColumnStretch(12, 90)
    layout.setColumnStretch(13, 90)
    layout.setColumnStretch(14, 90)
    layout.setColumnStretch(15, 90)
    layout.setColumnStretch(16, 90)
    # layout.setColumnStretch(zc+17, 10)
    # layout.setColumnStretch(zc+18, 10)
    # layout.setColumnStretch(zc+19, 10)
    # layout.setColumnStretch(zc+20, 10)
    # layout.setColumnStretch(zc+21, 10)

def layout_common_widgets(layout, vlv_buttons, p_labels):
    for n in range(0, 8):
        layout.addWidget(vlv_buttons[n][0], zr+n+1, zc+0-2)
        # layout.addWidget(vlv_buttons[n][1], zr+n+1, zc+1-2)
        # layout.addWidget(vlv_buttons[n][2], zr+n+1, zc+2-2)
        layout.addWidget(vlv_buttons[n][3], zr+n+1, zc-1)
        # layout.addWidget(vlv_buttons[n][4], zr+n+1, zc+0)
        layout.addWidget(p_labels[n][0], zr+n+1, zc+0)
        layout.addWidget(p_labels[n][1], zr+n+1, zc+1)
    for n in range(0, 4):
        layout.addWidget(vlv_buttons[n+8][0], zr+n+1+8, zc+0-2)
        layout.addWidget(vlv_buttons[n+8][1], zr+n+1+8, zc+1-2)
        layout.addWidget(vlv_buttons[n+8][2], zr+n+1+8, zc+2-2)
        layout.addWidget(vlv_buttons[n+8][3], zr+n+1+8, zc+3-2)
        layout.addWidget(vlv_buttons[n+8][4], zr+n+1+8, zc+4-2)


layout_common_widgets(ground_layout, ground_valve_buttons, ground_pressure_labels)
layout_common_widgets(flight_layout, flight_valve_buttons, flight_pressure_labels)

qd_ox_release = QtGui.QPushButton("Release Ox")
qd_ox_connect = QtGui.QPushButton("Connect Ox")
qd_ox_off = QtGui.QPushButton("OFF Ox")
qd_fuel_release = QtGui.QPushButton("Release Fuel")
qd_fuel_connect = QtGui.QPushButton("Connect Fuel")
qd_fuel_off = QtGui.QPushButton("OFF Fuel")
ground_layout.addWidget(qd_ox_connect,zr+9, zc+0)
ground_layout.addWidget(qd_ox_release,zr+10, zc+0)
ground_layout.addWidget(qd_fuel_connect,zr+9, zc+1)
ground_layout.addWidget(qd_fuel_release,zr+10, zc+1)
ground_layout.addWidget(qd_ox_off,zr+11, zc+0)
ground_layout.addWidget(qd_fuel_off,zr+11, zc+1)





### GROUND ###
init_fs = QtGui.QPushButton("Init Filesystem")
log_start = QtGui.QPushButton("Log Start")
log_end = QtGui.QPushButton("Log Stop")
telem_pause = QtGui.QPushButton("Pause Telemetry")
telem_resume = QtGui.QPushButton("Resume Telemetry")
file_download = QtGui.QPushButton("Download file")

ground_layout.addWidget(init_fs,zr+14, zc+0)
ground_layout.addWidget(log_start,zr+15, zc+0)
ground_layout.addWidget(log_end,zr+16, zc+0)
ground_layout.addWidget(telem_pause,zr+17, zc+0)
ground_layout.addWidget(telem_resume,zr+18, zc+0)
ground_layout.addWidget(file_download,zr+19, zc+0)

init_fs.clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_INIT_FS, 0, []))
log_start.clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_LOG_START, 0, []))
log_end.clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_LOG_END, 0, []))
telem_pause.clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_TELEM_PAUSE, 0, []))
telem_resume.clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_TELEM_RESUME, 0, []))
file_download.clicked.connect(lambda: get_file(TARGET_ADDRESS_GROUND));

### Flight ###
init_fs = QtGui.QPushButton("Init Filesystem")
log_start = QtGui.QPushButton("Log Start")
log_end = QtGui.QPushButton("Log Stop")
telem_pause = QtGui.QPushButton("Pause Telemetry")
telem_resume = QtGui.QPushButton("Resume Telemetry")
file_download = QtGui.QPushButton("Download file")

flight_layout.addWidget(init_fs,zr+14, zc+0)
flight_layout.addWidget(log_start,zr+15, zc+0)
flight_layout.addWidget(log_end,zr+16, zc+0)
flight_layout.addWidget(telem_pause,zr+17, zc+0)
flight_layout.addWidget(telem_resume,zr+18, zc+0)
flight_layout.addWidget(file_download,zr+19, zc+0)

init_fs.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_INIT_FS, 0, []))
log_start.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LOG_START, 0, []))
log_end.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LOG_END, 0, []))
telem_pause.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_TELEM_PAUSE, 0, []))
telem_resume.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_TELEM_RESUME, 0, []))
file_download.clicked.connect(lambda: get_file(TARGET_ADDRESS_FLIGHT));


# motor control
for mtrx in range(0, 2):
    mtr_toggle.append(QtGui.QPushButton(mtr[mtrx]+" ENABLE"))
    mtr_setpoint.append(QtGui.QLineEdit())
    mtr_position.append(QtGui.QLabel("POSITION FB"))
    mtr_pwm.append(QtGui.QLabel("pwm FB"))
    mtr_send.append(QtGui.QPushButton("Command Setpoint"))
    mtr_setpointfb.append(QtGui.QLabel("SETPOINT FB"))

    if mtr[mtrx] in alias.keys():
        mtr_toggle[mtrx].setText(alias[mtr[mtrx]]+"TOGGLE_EN")

    flight_layout.addWidget(mtr_toggle[mtrx], zr+9+(2*mtrx), zc+0)
    flight_layout.addWidget(mtr_send[mtrx],zr+10+(2*mtrx), zc+0)
    # flight_layout.addWidget(mtr_pwm[mtrx], zr+1+(2*mtrx), zc+8)
    flight_layout.addWidget(mtr_setpoint[mtrx], zr+10+(2*mtrx), zc+1)
    # flight_layout.addWidget(mtr_setpointfb[mtrx], zr+2+(2*mtrx), zc+7)
    flight_layout.addWidget(mtr_position[mtrx], zr+9    +(2*mtrx), zc+1)

# spacer = QtGui.QSpacerItem(1, 1)
# ground_layout.addItem(spacer, zr+6, zc+6)
# Samplerate Set
ground_samplerate_setpoint = QtGui.QLineEdit('50')
# ground_samplerate_setpointfb = QtGui.QLabel("SAMPLERATE FB")
ground_samplerate_send = QtGui.QPushButton("Update samplerate (Hz)")
ground_layout.addWidget(ground_samplerate_send, zr+13, zc+0)
ground_layout.addWidget(ground_samplerate_setpoint, zr+13, zc+1)
# ground_layout.addWidget(ground_samplerate_setpointfb, zr+11, zc+2)
flight_samplerate_setpoint = QtGui.QLineEdit('50')
# flight_samplerate_setpointfb = QtGui.QLabel("SAMPLERATE FB")
flight_samplerate_send = QtGui.QPushButton("Update samplerate (Hz)")
flight_layout.addWidget(flight_samplerate_send, zr+13, zc+0)
flight_layout.addWidget(flight_samplerate_setpoint, zr+13, zc+1)
# flight_layout.addWidget(flight_samplerate_setpointfb, zr+11, zc+2)

# Telemrate set
ground_telemrate_setpoint = QtGui.QLineEdit('10')
# ground_telemrate_setpointfb = QtGui.QLabel("TELEMRATE FB")
ground_telemrate_send = QtGui.QPushButton("Update telemrate (Hz)")
ground_layout.addWidget(ground_telemrate_send, zr+13, zc+0)
ground_layout.addWidget(ground_telemrate_setpoint, zr+13, zc+1)
# ground_layout.addWidget(ground_telemrate_setpointfb, zr+12, zc+2)
flight_telemrate_setpoint = QtGui.QLineEdit('10')
# flight_telemrate_setpointfb = QtGui.QLabel("TELEMRATE FB")
flight_telemrate_send = QtGui.QPushButton("Update telemrate (Hz)")
flight_layout.addWidget(flight_telemrate_send, zr+13, zc+0)
flight_layout.addWidget(flight_telemrate_setpoint, zr+13, zc+1)
# flight_layout.addWidget(flight_telemrate_setpointfb, zr+12, zc+2)

# Motor gains set
MOTOR_GAINS_LABEL = QtGui.QLabel("Motor Gains")
kp_set = QtGui.QPushButton("Update Kp")
ki_set = QtGui.QPushButton("Update Ki")
kd_set = QtGui.QPushButton("Update Kd")
kp_input = QtGui.QLineEdit()
ki_input = QtGui.QLineEdit()
kd_input = QtGui.QLineEdit()
kpfb = QtGui.QLabel("kpfb")
kifb = QtGui.QLabel("kifb")
kdfb = QtGui.QLabel("kdfb")
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
ground_ibridge0 = QtGui.QLabel("ground_ibridge0")
ground_ibridge1 = QtGui.QLabel("ground_ibridge1")
ground_layout.addWidget(ground_ibridge0, zr+24, zc-2, 1, 2)
ground_layout.addWidget(ground_ibridge1, zr+25, zc-2, 1, 2)
flight_ibridge0 = QtGui.QLabel("flight_ibridge0")
flight_ibridge1 = QtGui.QLabel("flight_ibridge1")
flight_layout.addWidget(flight_ibridge0, zr+24, zc-2, 1, 2)
flight_layout.addWidget(flight_ibridge1, zr+25, zc-2, 1, 2)

# State Feedback
state_label = QtGui.QLabel("STATE = N/A")
arm_button = QtGui.QPushButton("ARM")
disarm_button = QtGui.QPushButton("DISARM")
hotfire_button = QtGui.QPushButton("HOTFIRE")
flight_layout.addWidget(state_label, zr+21, zc+0)
flight_layout.addWidget(arm_button, zr+22, zc+0)
flight_layout.addWidget(disarm_button, zr+23, zc+0)
flight_layout.addWidget(hotfire_button, zr+24, zc+0)

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


def death():
    ground_command_log.close()
    ground_info_log.close()
    ground_serial_log.close()
    ground_data_log.close()
    flight_command_log.close()
    flight_info_log.close()
    flight_serial_log.close()
    flight_data_log.close()
    app.quit()

KILL1 = QtGui.QPushButton("End Run")
ground_layout.addWidget(KILL1, zr+0, zc-2)
KILL2 = QtGui.QPushButton("End Run")
flight_layout.addWidget(KILL2, zr+0, zc-2)
KILL1.clicked.connect(death)
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


font_list = []
font_list.append(ground_BOARD_HEALTH_LABEL)
font_list.append(flight_BOARD_HEALTH_LABEL)
font_list.append(ground_ebatt_label)
font_list.append(ground_ibus_label)
font_list.append(ground_e5v_label )
font_list.append(ground_e3v_label )
font_list.append(ground_ebatt_value)
font_list.append(ground_ibus_value)
font_list.append(ground_e5v_value )
font_list.append(ground_e3v_value )
font_list.append(ground_tbrd_label)
font_list.append(ground_tvlv_label)
font_list.append(ground_tmtr_label)
font_list.append(ground_tbrd_value)
font_list.append(ground_tvlv_value)
font_list.append(ground_tmtr_value)
font_list.append(flight_ebatt_label)
font_list.append(flight_ibus_label)
font_list.append(flight_e5v_label )
font_list.append(flight_e3v_label )
font_list.append(flight_ebatt_value)
font_list.append(flight_ibus_value)
font_list.append(flight_e5v_value )
font_list.append(flight_e3v_value )
font_list.append(flight_tbrd_label)
font_list.append(flight_tvlv_label)
font_list.append(flight_tmtr_label)
font_list.append(flight_tbrd_value)
font_list.append(flight_tvlv_value)
font_list.append(flight_tmtr_value)
font_list.append(last_packet_ground)
font_list.append(last_command_ground)
font_list.append(last_packet_flight)
font_list.append(last_command_flight)
font_list.append(last_packet_ground_label)
font_list.append(last_command_ground_label)
font_list.append(last_packet_flight_label)
font_list.append(last_command_flight_label)
font_list.append(ground_ibridge0)
font_list.append(ground_ibridge1)
font_list.append(flight_ibridge0)
font_list.append(flight_ibridge1)

for thing in font_list:
    thing.setFont(QtGui.QFont('SansSerif', 14))


###############################################################################
### FUNCTIONAL CONNECTIONS ####################################################
###############################################################################


ground_samplerate_send.clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_SAMPLERATE_SET, 1, [int(ground_samplerate_setpoint.text())]))
ground_telemrate_send.clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_TELEMRATE_SET, 1, [int(ground_telemrate_setpoint.text())]))
flight_samplerate_send.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_SAMPLERATE_SET, 1, [int(flight_samplerate_setpoint.text())]))
flight_telemrate_send.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_TELEMRATE_SET, 1, [int(flight_telemrate_setpoint.text())]))

arm_button.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_ARM, 0, []))
disarm_button.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_DISARM, 0, []))
hotfire_button.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_MAIN_AUTO_START, 0, []))

def toggle_valve(board, vlv_id):
    if board is 'ground':
        state = int(ground_valve_buttons[vlv_id][0].text()[-1])
        state = int(not state)
        s2_command(TARGET_ADDRESS_GROUND, COMMAND_DIGITAL_WRITE, 2, [vlv_id, state])
    if board is 'flight':
        state = int(flight_valve_buttons[vlv_id][0].text()[-1])
        state = int(not state)
        s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_DIGITAL_WRITE, 2, [vlv_id, state])

 # GROUND VALVES
ground_valve_buttons[0][0].clicked.connect(lambda: toggle_valve('ground', 0))
ground_valve_buttons[1][0].clicked.connect(lambda: toggle_valve('ground', 1))
ground_valve_buttons[2][0].clicked.connect(lambda: toggle_valve('ground', 2))
ground_valve_buttons[3][0].clicked.connect(lambda: toggle_valve('ground', 3))
ground_valve_buttons[4][0].clicked.connect(lambda: toggle_valve('ground', 4))
ground_valve_buttons[5][0].clicked.connect(lambda: toggle_valve('ground', 5))
ground_valve_buttons[6][0].clicked.connect(lambda: toggle_valve('ground', 6))
ground_valve_buttons[7][0].clicked.connect(lambda: toggle_valve('ground', 7))
ground_valve_buttons[0][1].clicked.connect(lambda: toggle_valve('ground', 0))
ground_valve_buttons[1][1].clicked.connect(lambda: toggle_valve('ground', 1))
ground_valve_buttons[2][1].clicked.connect(lambda: toggle_valve('ground', 2))
ground_valve_buttons[3][1].clicked.connect(lambda: toggle_valve('ground', 3))
ground_valve_buttons[4][1].clicked.connect(lambda: toggle_valve('ground', 4))
ground_valve_buttons[5][1].clicked.connect(lambda: toggle_valve('ground', 5))
ground_valve_buttons[6][1].clicked.connect(lambda: toggle_valve('ground', 6))
ground_valve_buttons[7][1].clicked.connect(lambda: toggle_valve('ground', 7))
# # FLIGHT VALVES
flight_valve_buttons[0][0].clicked.connect(lambda: toggle_valve('flight', 0))
flight_valve_buttons[1][0].clicked.connect(lambda: toggle_valve('flight', 1))
flight_valve_buttons[2][0].clicked.connect(lambda: toggle_valve('flight', 2))
flight_valve_buttons[3][0].clicked.connect(lambda: toggle_valve('flight', 3))
flight_valve_buttons[4][0].clicked.connect(lambda: toggle_valve('flight', 4))
flight_valve_buttons[5][0].clicked.connect(lambda: toggle_valve('flight', 5))
flight_valve_buttons[6][0].clicked.connect(lambda: toggle_valve('flight', 6))
# flight_valve_buttons[7][0].clicked.connect(lambda: toggle_valve('flight', 7))
flight_valve_buttons[0][1].clicked.connect(lambda: toggle_valve('flight', 0))
flight_valve_buttons[1][1].clicked.connect(lambda: toggle_valve('flight', 1))
flight_valve_buttons[2][1].clicked.connect(lambda: toggle_valve('flight', 2))
flight_valve_buttons[3][1].clicked.connect(lambda: toggle_valve('flight', 3))
flight_valve_buttons[4][1].clicked.connect(lambda: toggle_valve('flight', 4))
flight_valve_buttons[5][1].clicked.connect(lambda: toggle_valve('flight', 5))
flight_valve_buttons[6][1].clicked.connect(lambda: toggle_valve('flight', 6))
# flight_valve_buttons[7][1].clicked.connect(lambda: toggle_valve('flight', 7))
# # GROUND LEDS
ground_valve_buttons[8][1].clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_LED_WRITE, 2, [0,1]))
ground_valve_buttons[9][1].clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_LED_WRITE, 2, [1,1]))
ground_valve_buttons[10][1].clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_LED_WRITE, 2, [2,1]))
ground_valve_buttons[11][1].clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_LED_WRITE, 2, [3,1]))
ground_valve_buttons[8][0].clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_LED_WRITE, 2, [0,0]))
ground_valve_buttons[9][0].clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_LED_WRITE, 2, [1,0]))
ground_valve_buttons[10][0].clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_LED_WRITE, 2, [2,0]))
ground_valve_buttons[11][0].clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_LED_WRITE, 2, [3,0]))
# # FLIGHT LEDS
flight_valve_buttons[8][1].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [0,1]))
flight_valve_buttons[9][1].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [1,1]))
flight_valve_buttons[10][1].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [2,1]))
flight_valve_buttons[11][1].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [3,1]))
flight_valve_buttons[8][0].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [0,0]))
flight_valve_buttons[9][0].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [1,0]))
flight_valve_buttons[10][0].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [2,0]))
flight_valve_buttons[11][0].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_LED_WRITE, 2, [3,0]))

### CONTROL GAINS
kp_set.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_SET_KP, 1, [int(kp_input.text())]))
ki_set.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_SET_KI, 1, [int(ki_input.text())]))
kd_set.clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_SET_KD, 1, [int(kd_input.text())]))

### MOTORS
def toggle_motor(mtrx):
    if(int(mtr_toggle[mtrx].text()) == 0):
       s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_MOTOR_ENABLE, 1, [mtrx])
    else:
        s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_MOTOR_DISABLE, 1, [mtrx])

mtr_send[0].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_MOTOR_WRITE, 2, [0, int(mtr_setpoint[0].text())]))
mtr_send[1].clicked.connect(lambda: s2_command(TARGET_ADDRESS_FLIGHT, COMMAND_MOTOR_WRITE, 2, [1, int(mtr_setpoint[1].text())]))
mtr_toggle[0].clicked.connect(lambda: toggle_motor(0))
mtr_toggle[1].clicked.connect(lambda: toggle_motor(1))

### QICK DISCONNECTS
qd_ox_release.clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_QD_SET, 2, [0, -1]))
qd_ox_connect.clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_QD_SET, 2, [0, 1]))
qd_ox_off.clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_QD_SET, 2, [0, 0]))
qd_fuel_release.clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_QD_SET, 2, [1, -1]))
qd_fuel_connect.clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_QD_SET, 2, [1, 1]))
qd_fuel_off.clicked.connect(lambda: s2_command(TARGET_ADDRESS_GROUND, COMMAND_QD_SET, 2, [1, 0]))


##############################################################################
### END FUNCTIONAL CONNECTIONS ###############################################
##############################################################################

# Start
flight_window.showMaximized()
ground_window.showMaximized()
# plot_window.show()


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