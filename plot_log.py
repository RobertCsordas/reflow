from matplotlib import pyplot as plt
import numpy as np

with open('log.txt', 'r') as f:
    lines = f.readlines()

x = []
y_curr = []
y_target = []

delay = 9.244475

for i, line in enumerate(lines):
    l = [p.strip() for p in line.split(',')]
    l = [p.split(" ") for p in l]
    print(l)
    for p in l:
        if p[0].startswith("Time"):
            t = float(p[1])/1000.0
            if i==0:
                t0 = t
            x.append(t - t0)
        elif p[0].startswith("Curr"):
            y_curr.append(float(p[1]))
        elif p[0].startswith("Target"):
            y_target.append(float(p[1]))


plt.figure()
plt.plot(x, y_curr, label='Current')
plt.plot([a + delay for a in x], y_target, label='Target')
plt.xlabel('Time (s)')
plt.ylabel('Temperateure (C)')
plt.legend()
plt.savefig("log.pdf", bbox_inches='tight')