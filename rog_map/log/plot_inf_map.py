import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt('inf_map.txt')
inf_time = data[:,0]
inf_cnt = data[:,1]
update_t = data[:,2]



fig, axs = plt.subplots(3)
fig.suptitle('Vertically stacked subplots')
axs[0].plot(inf_time, label='inf_time')
axs[1].plot(inf_cnt, label='inf_cnt')
axs[2].plot(update_t, label = 'update_t')

print("inf_time_per_point:",np.mean(inf_time)/np.mean(inf_cnt) * 1e9, " ns")
print("average inf time", np.mean(inf_time))
for i in range(3):
    axs[i].legend()
plt.show()