

filename = input("Template file: ")

template_file = open(filename)

format_string = ""
argument_string = ""
python_string = ""
globals_string = ""
device_list = []

n = 0
for line in template_file:
	if(n):
		split_string = line.split('\t')
		format_string = format_string + split_string[2].rstrip('\n')+','
		argument_string = argument_string + (","+split_string[1])

		if(split_string[7]):
			globals_string = globals_string + "\tglobal "+split_string[7]+'\n'

		if(split_string[6]):
			python_string = python_string + "\t"+split_string[6]+" = "+split_string[4]+"(split_line["+str(split_string[0])+"])\n"
		else:
			python_string = python_string + "\t"+split_string[1]+" = "+split_string[4]+"(split_line["+str(split_string[0])+"])\n"

		device_list.append(split_string[5])
	n += 1;

csv_header = "csv_header = \"Time(s),\""
parse_csv_header = "\tif(write_csv_header):\n"
parse_csv_header +=  "\t\tdevice_list = [\'"
for m in range(0, n-1):
	parse_csv_header += device_list[m]
	parse_csv_header += "\', \'"
parse_csv_header += "\']\n"
parse_csv_header +=  "\t\tfor device in range(0, len(device_list)):\n"
parse_csv_header +=  "\t\t\tif device_list[device] in alias.keys():\n"
parse_csv_header +=  "\t\t\t\tdevice_list[device] = alias[device_list[device]]\n"
parse_csv_header +=  "\t\tcsv_header = \"Time (s),\"\n"
parse_csv_header +=  "\t\t\tcsv_header.append(device, \",\")\n"
parse_csv_header +=  "\t\twrite_csv_header = False\n"

csv_header = csv_header + "\"\n\n"
parsed_c_file = open((filename+"_PARSED_C.txt"),"w+")
parsed_python_file = open(("hotfire_packet.py"),"w+")

parsed_c_file.write("snprintf(line, sizeof(line), \""+format_string+"\\r\\n\""+argument_string+");")
parsed_python_file.write("write_csv_header = True\ndef parse_packet(split_line):\n"+parse_csv_header+globals_string+python_string)