import time

filename = input("Template file: ")

template_file = open(filename)
print("Reading "+filename+"...")

format_string = ""
argument_string = ""
python_string = ""
globals_string = "\t\t## GLOBALS ##\n"
device_list = []
col = {}
packet_byte_length = 0; 

pack_telem_defines_h_string = 	"/// pack_telem_defines.h\n" + \
						"/// Last autogenerated: " + time.ctime() + "\n\n" + \
						"#include \"globals.h\"\n" + \
						"#include \"calibrations.h\"\n" + \
						"\n"

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

type_range_positive = {
	"char"		:	255,
	"uint8_t"	:	(2**8)-1,
	"int8_t"	:	((2**8)/2)-1,
	"uint16_t"	:	(2**16)-1,
	"int16_t"	:	((2**16)/2)-1,
	"uint32_t"	:	(2**32)-1,
	"int32_t"	:	((2**32)/2)-1,
	"uint64_t"	:	(2**64)-1,
	"int64_t"	:	((2**64)/2)-1
}

type_range_negative = {
	"char"		:	0,
	"uint8_t"	:	0,
	"int8_t"	:	-((2**8)/2),
	"uint16_t"	:	0,
	"int16_t"	:	-((2**16)/2),
	"uint32_t"	:	0,
	"int32_t"	:	-((2**32)/2),
	"uint64_t"	:	0,
	"int64_t"	:	-((2**64)/2)
}

type_unpack_arg = {
	"char"		:	"\"<c\"",
	"uint8_t"	:	"\"<B\"",
	"int8_t"	:	"\"<b\"",
	"uint16_t"	:	"\"<H\"",
	"int16_t"	:	"\"<h\"",
	"uint32_t"	:	"\"<I\"",
	"int32_t"	:	"\"<i\"",
	"uint64_t"	:	"\"<L\"",
	"int64_t"	:	"\"<l\"",
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
		python_type  = split_string[col['python_type']]
		python_globals  = split_string[col['python_globals']]

		try:
			assert type_cast in type_byte_lengths.keys();
		except:
			print("Invalid type cast. Valid Types are:\n"+str(type_byte_lengths.keys()))

		## Check xmit limits
		try:
			if(min_val):
				assert (float(min_val)*float(xmit_scale) >= type_range_negative[type_cast])
		except:
			print("Invalid type cast for given range on item "+name	)
			print("Min val: "+str(float(min_val)*float(xmit_scale)))
			print("Type limit: "+str(type_range_negative[type_cast]))
		try:
			if(max_val):
				assert (float(max_val)*float(xmit_scale) <= type_range_positive[type_cast])
		except:
			print("Invalid type cast for given range on item "+name	)
			print("Max val: "+str(float(max_val)*float(xmit_scale)))
			print("Type limit: "+str(type_range_positive[type_cast]))
		## End check xmit limits

		byte_length = type_byte_lengths[type_cast]
		packet_byte_length += byte_length

		format_string = format_string + printf_format.rstrip('\n')+','
		argument_string = argument_string + (","+firmware_variable)


		# Parse Globals
		if(python_globals):
			globals_string = globals_string + "\t\tglobal "+python_globals+'\n'

		python_variable = firmware_variable
		if(python_variable_override):
			python_variable = python_variable_override

		python_string += "\t\tbyte_rep = "
		python_string += "packet["+str(packet_byte_length-byte_length)+":"+ str(packet_byte_length)+"]"
		python_string += "\n"
			
			
		python_string += 	"\t\t"+python_variable+" = "+python_type+ \
							"((float(struct.unpack("+type_unpack_arg[type_cast] + \
							", byte_rep)[0]))/"+xmit_scale+")\n"


		device_list.append(split_string[5])
		for m in range(0, byte_length):
			pack_telem_defines_h_string += "#define\tTELEM_ITEM_"+str(packet_byte_length-byte_length+m)+ \
			"\t(("+type_cast+") ("+str(firmware_variable)+"*"+str(xmit_scale)+")) >> "+str(8*m)+" \n"

	n += 1;

for m in range(packet_byte_length, 254):
	pack_telem_defines_h_string += "#define\tTELEM_ITEM_"+str(m)+"\t0\n"
pack_telem_defines_h_string += "#define\tPACKET_SIZE\t"+str(packet_byte_length)+"\n"


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
pack_telem_defines_h = open("../firmware/Engine_Controller_V1/src/pack_telem_defines.h", "w+")

parsed_printf_file.write("snprintf(line, sizeof(line), \""+format_string+"\\r\\n\""+argument_string+");")
parsed_python_file.write(	"import struct\nclass ECParse:\n\n" + \
							"\tdef parse_packet(packet):\n" + \
							globals_string+python_string)
pack_telem_defines_h.write(pack_telem_defines_h_string)

print(filename+" Successfully Parsed!")
print(" --- Packet statistics --- ")
print("Packet items: "+str(n))
print("Packet length (bytes): "+str(packet_byte_length))