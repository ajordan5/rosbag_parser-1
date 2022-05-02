import matplotlib.pyplot as plt
import numpy as np

data = np.load("./flight_data/outdoor_w_boat_2022-02-05_moving3_less_accurate.npz")


base_time = data["base_time"]
rover_time = data["rover_time"]

length = np.min((len(rover_time), len(base_time)))
rover_vel = data["rover_vel"][:, 0:length]
base_vel = data["base_vel"][:, 0:length]
base_time = base_time[0:length]
rover_time = rover_time[0:length]

fig, ax = plt.subplots(3,2)
ax[0][0].plot(base_time, base_vel[0,:])
ax[0][1].plot(rover_time, rover_vel[0,:])
ax[1][0].plot(base_time, base_vel[1,:])
ax[1][1].plot(rover_time, rover_vel[1,:])
ax[2][0].plot(base_time, base_vel[2,:])
ax[2][1].plot(rover_time, rover_vel[2,:])

plt.figure(1)
relVel = base_vel - rover_vel
plt.plot(rover_time, relVel[0,:])
plt.show()





