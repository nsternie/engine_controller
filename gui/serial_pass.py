import serial
ser = serial.Serial(port="COM12", baudrate=115200, timeout=0.5)
while(1):
	print(ser.readline())