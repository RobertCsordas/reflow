from matplotlib import pyplot as plt
import numpy as np

with open('tuning_log.txt', 'r') as f:
    lines = f.readlines()

y_curr = []

for i, line in enumerate(lines):
   y_curr.append(float(line.split("temp: ")[1].strip().split(" ")[0]))

dt =  5 * 60 / len(y_curr) 
TUNE_POWER = 0.15
x_curr = [i * dt for i in range(len(y_curr))]

t_ambient = y_curr[0]

yt = (y_curr[-1] - t_ambient) * 0.02 + t_ambient

for i in range(len(y_curr)):
    if y_curr[i] > yt:
        istart = i
        break
else:
    raise ValueError("No temperature rise detected")

L_est_tresh = istart * dt;

xmid = len(y_curr) // 2
print(((len(y_curr) - 1 - xmid) * dt), dt, len(y_curr), len(y_curr) - 1 - xmid)
m = (y_curr[-1] - y_curr[xmid]) / ((len(y_curr) - 1 - xmid) * dt) / TUNE_POWER
mid = int((xmid + len(y_curr) - 1) / 2)

tmid = mid * dt
ymid = y_curr[mid] - y_curr[0]
L_est = tmid - ((y_curr[mid] - t_ambient) / (m*TUNE_POWER));

L_intersect_factor = 0.5
L_est = L_est * L_intersect_factor + L_est_tresh * (1-L_intersect_factor)

istart = int(L_est / dt)
m = (y_curr[-1] - y_curr[istart]) / ((len(y_curr) - 1 - istart) * dt) / TUNE_POWER

lambda_factor = 0.2

lambd = L_est * lambda_factor
Kp = 1.0 / (m * lambd);
Ki = Kp / (4 * lambd);

print(f"Estimated L: {L_est:.6f} s, m: {m:.6f}, Kp: {Kp:.6f}, Ki: {Ki:.6f}")

plt.figure()
plt.plot(x_curr, y_curr, label='Current')
plt.axvline(x=L_est, color='gray', linestyle='--', linewidth=2, label=f'x = {L_est:.2f} s')
plt.plot([L_est, len(y_curr) * dt], [t_ambient, t_ambient + m * TUNE_POWER * (len(y_curr) * dt - L_est)], color='red', linestyle='--', linewidth=2, label="m= {:.2f} C/s".format(m))
plt.xlabel('Time (s)')
plt.ylabel('Temperateure (C)')
plt.legend()
plt.savefig("log_tune.pdf", bbox_inches='tight')