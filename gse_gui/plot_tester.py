## Test file to generate new data and see if plotter picks it up

import csv
import random
import time

data_files = ['data.csv','data1.csv','data2.csv','data3.csv']
t = 0
dt = 0.2

while t < 10:
    for data_file in data_files:
        with open(data_file,'a') as f:
            wr = csv.writer(f)
            wr.writerow([t,random.randrange(0,100,step=1)])

    t = t + dt
    time.sleep(0.2)
