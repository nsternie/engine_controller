## Test file to generate new data and see if plotter picks it up

import csv
import random
import time

data_file = 'data.csv'
t = 0
dt = 0.2

while t < 10:
    with open(data_file,'a') as f:
        wr = csv.writer(f)
        wr.writerow([t,random.randint(0,1)])

    t = t + dt
    time.sleep(0.2)
