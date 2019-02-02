import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import pandas as pd
import sys

df = pd.read_csv(sys.argv[1])
pdf = PdfPages(sys.argv[1].rstrip('.csv')+'_plots.pdf')

temp = 1
plot_num = 1
time = []
for col in df:
	if temp:
		time = df[col]
		temp = 0
	else:
		data = df[col]

		# Global data plot
		plt.figure(0)
		try:
			plt.plot(time, data, label=data.name)
		except:
			pass
		# Column data
		plt.figure(plot_num)
		plot_num = plot_num + 1
		plt.plot(time, data)
		plt.grid()
		plt.title(data.name)
		

plt.figure(0)
plt.title("ALL DATA")
plt.legend()

for n in range(plot_num):
    plt.figure(n)
    plt.savefig(pdf, format='pdf')

pdf.close()
