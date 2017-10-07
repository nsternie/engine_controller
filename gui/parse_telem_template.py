filename = input("Template file: ")

template_file = open(filename)

format_string = ""
argument_string = ""
python_string = ""
globals_string = ""
csv_header = "csv_header = \"Time(s),"
n = 0
for line in template_file:
	if(n):
		split_string = line.split('\t')
		format_string = format_string + split_string[2].rstrip('\n')+','
		argument_string = argument_string + (","+split_string[1])
		if(split_string[5]):
			globals_string = globals_string + "\tglobal "+split_string[5]+'\n'
		python_string = python_string + "\t"+split_string[1]+" = "+split_string[4]+"(split_line["+str(split_string[0])+"])\n"
		csv_header = csv_header + split_string[1] + ","
	n = 1

csv_header = csv_header + "\"\n\n"
parsed_c_file = open((filename+"_PARSED_C.txt"),"w+")
parsed_python_file = open(("hotfire_packet.py"),"w+")

parsed_c_file.write("snprintf(line, sizeof(line), \""+format_string+"\\r\\n\""+argument_string+");")
parsed_python_file.write(csv_header+"def parse_packet(split_line):\n"+globals_string+python_string)