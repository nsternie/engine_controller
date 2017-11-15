import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import math
import pandas as pd

data_file_name = input("Enter datafile name:")
pdf = PdfPages(data_file_name.rstrip('.csv')+'plots.pdf')

mnemonic_file = open('mnemonics.csv', 'r')
line = mnemonic_file.readline()
split_line = line.split(',')
col_dict = {}
n = 0
for col in split_line:
	col_dict[col.rstrip('\n')] = n
	n += 1

mnemonic_dict = {}

for line in mnemonic_file:
	split_line = line.rstrip('\n').split(',')
	mnemonic_dict[split_line[col_dict['mnemonic']]] = split_line[col_dict['name']]

mnemonic_file.close()

## END MNEMONIC PARSER ##

plot_templates_file = open("plot_template.csv")

plot_name = []		
independent = []
primary_dependents = []
secondary_dependants = []
time_zoom = []

line = plot_templates_file.readline()
for line in plot_templates_file:
	split_line = line.rstrip('\n').split(',')
	plot_name.append(split_line[0])
	independent.append(split_line[1])
	time_zoom.append(split_line[4])
	temp_dep = []
	for dependent in split_line[2].split('|'):
		if(dependent):
			temp_dep.append(dependent)
	primary_dependents.append(temp_dep)

# data_file = open('test_data.csv', 'r')
df = pd.read_csv(data_file_name)

plot_num = 0
for plot in plot_name:
	plt.figure(plot_num)
	indep_list = df[independent[plot_num]].tolist()
	for d in primary_dependents[plot_num]:
		print(d)
		dep_list = df[d].tolist()
		plt.plot(indep_list, dep_list, label=mnemonic_dict[d])
	if(independent[plot_num] in mnemonic_dict.keys()):
		plt.xlabel(mnemonic_dict[independent[plot_num]])
	else:
		plt.xlabel(independent[plot_num])
	plt.legend()
	plt.title(plot)
	if(time_zoom[plot_num]):
		split_string = time_zoom[plot_num].split(' to ')
		start = float(split_string[0])
		end = float(split_string[1])
		plt.xlim(start, end)
	plt.savefig(pdf, format='pdf')
	plot_num += 1

pdf.close()