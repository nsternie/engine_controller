import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import math
import pandas as pd
import re

debug = 0

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
x_zoom = []
y_zoom = []

line = plot_templates_file.readline()
for line in plot_templates_file:
	split_line = line.rstrip('\n').split(',')
	plot_name.append(split_line[0])
	independent.append(split_line[1])
	x_zoom.append(split_line[4])
	y_zoom.append(split_line[5])
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
		if any(c in d for c in "+-*/"):
			# Need to do some math
			orig = d
			args = re.split("\+|-|\*|\/|\(|\)|[0-9]", d)
			args = [item for item in args if item != '']
			temp = [arg for arg in args]
			args = []
			for arg in temp:
				if arg not in args:
					args.append(arg)
			dep_lists = []
			for arg in args:
				dep_lists.append(df[arg].tolist())
				d = d.replace(arg, "df[\'"+arg+'\']')
			eval_list = (eval(d))
			plt.plot(indep_list, eval_list, label=orig)

		else:		# Just plot a single var, no math
			dep_list = df[d].tolist()
			plt.plot(indep_list, dep_list, label=mnemonic_dict[d])
	if(independent[plot_num] in mnemonic_dict.keys()):
		plt.xlabel(mnemonic_dict[independent[plot_num]])
	else:
		plt.xlabel(independent[plot_num])
	plt.legend()
	plt.title(plot)
	# Zoom in on region of intrerest
	if(x_zoom[plot_num]):
		split_string = x_zoom[plot_num].split(' to ')
		start = float(split_string[0])
		end = float(split_string[1])
		plt.xlim(start, end)
	if(y_zoom[plot_num]):
		split_string = y_zoom[plot_num].split(' to ')
		start = float(split_string[0])
		end = float(split_string[1])
		plt.ylim(start, end)
	# Save the plot to pdf
	plt.savefig(pdf, format='pdf')
	plot_num += 1

pdf.close()