filename = input("Template file: ")

template_file = open(filename)

format_string = ""
argument_string = ""
python_string = ""
globals_string = "\t## GLOBALS ##\n"
device_list = []
col = {}
packet_byte_length = 1; # Always will have at least \n

type_byte_lengths = {
	"char"		:	1,
	"uint8_t"	:	1,
	"int8_t"	:	1,
	"uint16_t"	:	2,
	"int16_t"	:	2,
	"uint32_t"	:	4,
	"int32_t"	:	4,
	"uint64_t"	:	8,
	"int64_t"	:	8
}
		

n = 0
for line in template_file:
	if(n == 0):	# First line of the file
		split_string = line.split('\t')
		c = 0
		for arg in split_string:
			col[arg] = c;
			c += 1;

	else:	# Rest of the file with real data

		split_string = line.split('\t')

		name = split_string[col['name']]
		firmware_variable  = split_string[col['firmware_variable']]
		min_val  = split_string[col['min_val']]
		max_val  = split_string[col['max_val']]
		unit  = split_string[col['unit']]
		firmware_type  = split_string[col['firmware_type']]
		printf_format  = split_string[col['printf_format']]
		type_cast  = split_string[col['type_cast']]
		xmit_scale  = split_string[col['xmit_scale']]
		device  = split_string[col['device']]
		python_variable_override  = split_string[col['python_variable_override']]
		python_type  = split_string[col['python_globals']]
		python_globals  = split_string[col['firmware_variable']]

		byte_length = type_byte_lengths[type_cast]
		packet_byte_length += byte_length

		format_string = format_string + printf_format.rstrip('\n')+','
		argument_string = argument_string + (","+firmware_variable)

		if(python_globals):
			globals_string = globals_string + "\tglobal "+python_globals+'\n'

		if(python_variable_override):
			python_string = python_string + "\t"+python_variable_override+" = "+python_type+"(split_line["+str(n-1)+"])\n"
		else:
			python_string = python_string + "\t"+firmware_variable+" = "+python_type+"(split_line["+str(n-1)+"])\n"

		device_list.append(split_string[5])

	n += 1;


# csv_header = "csv_header = \"Time(s),\""
# parse_csv_header = "\tif(write_csv_header):\n"
# parse_csv_header +=  "\t\tdevice_list = [\'"
# for m in range(0, n-1):
# 	parse_csv_header += device_list[m]
# 	parse_csv_header += "\', \'"
# parse_csv_header += "\']\n"
# parse_csv_header +=  "\t\tfor device in range(0, len(device_list)):\n"
# parse_csv_header +=  "\t\t\tif device_list[device] in alias.keys():\n"
# parse_csv_header +=  "\t\t\t\tdevice_list[device] = alias[device_list[device]]\n"
# parse_csv_header +=  "\t\tcsv_header = \"Time (s),\"\n"
# parse_csv_header +=  "\t\t\tcsv_header.append(device, \",\")\n"
# parse_csv_header +=  "\t\twrite_csv_header = False\n"

csv_header = ""#csv_header + "\"\n\n"
parsed_printf_file = open((filename+"_sprintf-call_.c"),"w+")
parsed_python_file = open(("hotfire_packet.py"),"w+")

parsed_printf_file.write("snprintf(line, sizeof(line), \""+format_string+"\\r\\n\""+argument_string+");")
parsed_python_file.write("write_csv_header = True\ndef parse_packet(split_line):\n"+globals_string+python_string)

print("Packet statistics:")
print("Packet items: "+str(n))
print("Packet length (bytes): "+str(packet_byte_length))

