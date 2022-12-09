import numpy as np
import matplotlib.pyplot as plt
X = 0
Y = 1
Z = 2

data = np.load("DATA.npy")
t = np.arange(0, data.shape[1]/2, 1/2)

x,y,z = data

dist = data[:,1:] - data[:,:-1]

index_peak = np.max(np.argmax(np.abs(dist), axis=1))
# index_peak = np.max(np.argmax(np.abs(dist[X]))) # Select axis X, Y or Z as the reference for the perturbation

error = data[:,index_peak+1]

rising_time = np.abs(error) * 2 + t[index_peak]
overshoot = np.abs(error) * 0.1
settling_time = np.abs(error) * 3 + t[index_peak]


plt.figure(figsize=(15,5))


"""
    X
"""
plt.subplot(1,3,1)

# Time response
plt.plot(t, x, label="error x", linewidth=1, color="#EC407A")

# Overshoot
plt.plot([t[0], t[-1]], [overshoot[0], overshoot[0]], "--", linewidth=1, color="#EC407A")
plt.plot([t[0], t[-1]], [-overshoot[0], -overshoot[0]], "--", linewidth=1, color="#EC407A")

# Rising time
plt.plot([rising_time[0], rising_time[0]], [np.min(x), np.max(x)], "--", linewidth=1, color="#EC407A")

# Settling time
plt.plot([settling_time[0], settling_time[0]], [np.min(x), np.max(x)], "--", linewidth=1, color="#EC407A")   


plt.grid()
plt.xlim([t[0], t[-1]])
plt.ylabel("x (m)")
plt.xlabel("Time (s)")
plt.title("Time response on x-axis")


"""
    Y
"""
plt.subplot(1,3,2)

# Time response
plt.plot(t, y, label="error y", linewidth=1, color="#26C6DA")

# Overshoot
plt.plot([t[0], t[-1]], [overshoot[1], overshoot[1]], "--", linewidth=1, color="#26C6DA")
plt.plot([t[0], t[-1]], [-overshoot[1], -overshoot[1]], "--", linewidth=1, color="#26C6DA")

# Rising time
plt.plot([rising_time[1], rising_time[1]], [np.min(y), np.max(y)], "--", linewidth=1, color="#26C6DA")

# Settling time
plt.plot([settling_time[1], settling_time[1]], [np.min(y), np.max(y)], "--", linewidth=1, color="#26C6DA")   

plt.grid()
plt.xlim([t[0], t[-1]])
plt.ylabel("y (m)")
plt.xlabel("Time (s)")
plt.title("Time response on y-axis")


"""
    Z
"""
plt.subplot(1,3,3)

# Time response
plt.plot(t, z, label="error z", linewidth=1, color="#2C3E50")

# Overshoot
plt.plot([t[0], t[-1]], [overshoot[2], overshoot[2]], "--", linewidth=1, color="#2C3E50")
plt.plot([t[0], t[-1]], [-overshoot[2], -overshoot[2]], "--", linewidth=1, color="#2C3E50")

# Rising time
plt.plot([rising_time[2], rising_time[2]], [np.min(z), np.max(z)], "--", linewidth=1, color="#2C3E50")

# Settling time
plt.plot([settling_time[2], settling_time[2]], [np.min(z), np.max(z)], "--", linewidth=1, color="#2C3E50")   

plt.grid()
plt.xlim([t[0], t[-1]])
plt.ylabel("z (m)")
plt.xlabel("Time (s)")
plt.title("Time response on z-axis")


plt.show()
