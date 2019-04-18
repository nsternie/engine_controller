import serial, sys, os, time
import time

COMMAND_TARE            =   30
COMMAND_AMBIENTIZE      =   31
COMMAND_DIGITAL_WRITE   =   50
COMMAND_LED_WRITE       =   51
COMMAND_MOTOR_WRITE     =   52
COMMAND_MOTOR_DISABLE   =   53
COMMAND_MOTOR_ENABLE    =   54
COMMAND_QD_SET          =   55
COMMAND_SET_KP          =   60
COMMAND_SET_KI          =   61
COMMAND_SET_KD          =   62
COMMAND_TELEMRATE_SET   =   63
COMMAND_SAMPLERATE_SET  =   64
COMAND_LOGRATE_SET      =   65
COMMAND_ARM             =   100
COMMAND_DISARM          =   101
COMMAND_MAIN_AUTO_START =   102


COMMAND_PRINT_FILE      =   40
COMMAND_NUMFILES        =   41
COMMAND_LOG_START       =   42
COMMAND_LOG_END         =   43
COMMAND_INIT_FS         =   44
COMMAND_TELEM_PAUSE     =   45
COMMAND_TELEM_RESUME   =   46


TARGET_ADDRESS_GROUND   =   100
TARGET_ADDRESS_FLIGHT   =   101

packet_number = 0

def s2_command(target_id, command_id, argc, argv):
    global packet_number;
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
    # print(packet)
    tosend = bytes(packet)
    # print("Packet "+str(packet_number)+", target_id "+str(target_id)+", command_id "+str(command_id))
    ser.write(tosend)
    # TODO: Log command

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

port = sys.argv[1]
baud = int(sys.argv[2])
try:
    ser = serial.Serial(port, baudrate=baud, timeout=0.25)
    print("Connected on "+port+" at "+str(baud)+"bps")
except:
    print("Could not open "+port)
    print("exiting...")
    exit(1)

s2_command(TARGET_ADDRESS_GROUND, COMMAND_TELEM_PAUSE, 0, []);
time.sleep(100)
s2_command(TARGET_ADDRESS_GROUND, COMMAND_INIT_FS, 0, []);
time.sleep(100)
try:
	while(1):
		print(ser.readline())
finally:
	s2_command(TARGET_ADDRESS_GROUND, COMMAND_TELEM_RESUME, 0, []);