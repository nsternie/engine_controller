import sys

##### SCRIPT CONFIGURATION #####
threshold = 0.5 	## Chamber trigger voltage
pre_burn = 1000		## Samples kept before burn
post_burn = 1000	## Samples kept after burn
##### END CONFIGURATION ########

filename = input("Enter filename (.csv): ")
#chamber_channel = int(input("Enter the channel number that has chamber pressure data: "))
cal_filename = input("Enter calibration file (tab-seperated): ")
sample_rate = int(input("Enter sample rate (Hz): "))

data_in = open(filename, 'r')
cal_file = open(cal_filename, 'r')

slope = [0,0,0,0,0,0,0,0]
offset = [0,0,0,0,0,0,0,0]
name = ["Sample","Time","","","","","","","",""]
chamber_channel = -1
channel_dict = {}
n = 0
for line in cal_file:
	if(n == 0):
		pass	# Skip the header
	else:
		split_line = line.split('\t')
		slope[int(split_line[0])] = float(split_line[1])
		offset[int(split_line[0])] = float(split_line[2])
		name[int(split_line[0])+2] = split_line[3].rstrip('\n')
		channel_dict[name[int(split_line[0])+2]] = int(split_line[0])
	n += 1


for n in range(0, len(name)):
	temp = name[n].lower()
	if(temp == 'chamber' or temp == "chamber pressure" or temp == "chamber_pressure"):
		chamber_channel = n
# Make sure we have a trigger channel
if(chamber_channel == -1):
	print("ERROR: Chamber pressure not found in cal file. Valid aliases are: ")
	print("chamber")
	sys.exit()



line_num = 0
start = -1
end = -1
# Parse the source file
for line in data_in:
	split_line = line.split(',')

	if(line_num > 50):
		if(float(split_line[chamber_channel].strip("\"")) > threshold and start == -1):
			start = line_num - pre_burn
		if(float(split_line[chamber_channel].strip("\"")) < threshold and end == -1 and start != -1):
			end = line_num + post_burn


	line_num += 1

# Refresh the data_in
data_in.close()
data_in = open(filename, 'r')

# Write out the header
data_out = open(filename.rstrip('.csv')+'_out.csv', 'w+')
for n in range(0, len(name)):
	data_out.write(name[n]+',')
data_out.write('\n')

# Write out all the data
line_num = 0
for line in data_in:
	
	# Grab better offset for things that have dont have pressure before startup
	if(line_num == start):
		split_line = line.split(',')
		temp_chan = []
		temp_chan.append(channel_dict['chamber'])
		temp_chan.append(channel_dict['ox_man'])
		temp_chan.append(channel_dict['fuel_man'])
		for chan in temp_chan:
			offset[chan] = float(split_line[chan+2].rstrip('\n').strip("\""))-(14.7/slope[chan])

	if(line_num > start and line_num < end):
		split_line = line.split(',')
		split_data = [float(i.rstrip('\n').strip("\"")) for i in split_line[2:]]
		# Cal all the data
		data_string = ""
		for n in range(0, 8):
			split_data[n] -= offset[n]
			split_data[n] *= slope[n]
			data_string += str(split_data[n])+','

		calibrated_line = split_line[0]+','+str(((float(split_line[0].strip("\""))-(start+pre_burn))/sample_rate))+','+data_string+'\n'
		data_out.write(calibrated_line)
	line_num += 1

data_in.close()
cal_file.close()
data_out.close()

print("Done")