import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

samples = np.random.rand(64, 50)  # 64 samples, 50 frames

x = np.arange(samples.shape[0])  # sample index
y = np.arange(samples.shape[1])  # frame/time or frequency
x, y = np.meshgrid(x, y)
z = samples.T  # transpose เพื่อให้สอดคล้องกับ meshgrid

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# แสดงผล 3D histogram (surface)
surf = ax.plot_surface(x, y, z, cmap='viridis')

ax.set_xlabel('Sample Index')
ax.set_ylabel('Frame/Time')
ax.set_zlabel('Amplitude')

plt.title("3D Histogram (Spectrum Data)")
plt.colorbar(surf)
plt.show()