import scipy
from scipy import signal
import numpy as np
import matplotlib.pyplot as plt
import math
import pandas as pd
import wave

x = np.arange(0, 100, 0.01).tolist()
y = [math.sin(z) for z in x]

df = pd.read_csv('test_data.csv')
data = df['pc']
index = np.linspace(0, 100, len(data))

data = 50*np.sin(index)# + 100*np.sin(2*index)


f = scipy.fft(data)
f = f/len(data)

plt.figure(0)
plt.plot(np.linspace(0, 100, len(data)).tolist(), np.abs(f))
plt.xlim(0, 50)
plt.figure(1)
plt.plot(data)
plt.show()