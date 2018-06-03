import sys
from hotfire_packet import ECParse
parser = ECParse()

file = open(sys.argv[1], 'rb')
datalog = open(sys.argv[1]+'_datalog.csv', 'w')
datalog.write(parser.csv_header)

packets = []
this_line = []
n = 0
while True:
	b = file.read(1)
	# print(b)
	if not b:
		break;
	if b == b'\x0a':
		n += 1
		packets.append(this_line)
		this_line = []
	else:
		this_line += b

print(n)

for packet in packets:
	# print(len(packet))
	packet = list(packet)
	unstuffed = b''
	index = int(packet[0])
	for n in range(1, len(packet)):
		temp = bytes(packet[n:n+1])
		if(n == index):
			index = int(packet[n])+n
			temp = b'\n'
		unstuffed = unstuffed + temp
	packet = unstuffed
	# print(list(unstuffed))
	try:
		parser.parse_packet(packet)
	except:
		pass
	if not(parser.valve_states == 65535):
		datalog.write(parser.log_string+'\n')