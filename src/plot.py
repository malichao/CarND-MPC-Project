import csv
from os.path import abspath, exists
import matplotlib.pyplot as plt

f_path = "/home/mma/udacity/Model-Predictive-Control/build/log.txt"
content = []
x1= []
y1 = []
psi1 = []
x2= []
y2 = []
psi2 = []
with open(f_path, 'rb') as f:
    reader = csv.reader(f)
    content = list(reader)

for row in content:
    x1.append(float(row[0]))
    y1.append(float(row[1]))
    psi1.append(float(row[2]))
    x2.append(float(row[3]))
    y2.append(float(row[4]))
    psi2.append(float(row[5]))

plt.plot(x1, y1, 'r--', x2, y2, 'r', linewidth=2.0)
#plt.plot(t, z, linewidth=2.0)
plt.grid(True)
plt.show()
