from readline import append_history_file
from rosbag_parser import Parser
import numpy as np
import matplotlib.pyplot as plt


baseGpsTopic = '/base/PosVelEcef'
roverRelPosTopic = '/rover/RelPos'
roverGpsTopic = '/rover/PosVelEcef'
baseOdomTopic = '/base_odom'
baseMocapTopic = '/boat_landing_platform_ned'
roverMocapTopic = '/rhodey_ned'
tagTopic = '/tag_detections'
tagOdomTopic = '/april_odom'
tagHatTopic = '/at_hat'

bagfile = 'at_psi2'

bagdir = '/home/tpool2/bags/at_test/{}.bag'.format(bagfile)

parser = Parser(bagdir, baseGpsTopic, roverRelPosTopic, roverGpsTopic, baseOdomTopic, baseMocapTopic, roverMocapTopic, tagTopic, tagOdomTopic, tagHatTopic)

# parser.parse_and_save()

mocap_data = np.load("{}_rhodey_ned.npz".format(bagfile))
mocap_base_data = np.load("{}_boat_landing_platform_ned.npz".format(bagfile))
odom_data = np.load("{}_base_odom.npz".format(bagfile))


# Plot truth vs estimate
fig = plt.figure(1)
ax = plt.axes(projection='3d')

ax.plot3D(-mocap_data["pn"],-mocap_data["pe"], np.array(-mocap_data["pd"])-.18)
ax.plot3D(odom_data["pn"], odom_data["pe"], odom_data["pd"])


# Plot odom estimate vs odom based on apriltag
fig = plt.figure(2)
ax = plt.axes(projection='3d')
tag_odom_data = np.load("{}_april_odom.npz".format(bagfile))

ax.plot3D(tag_odom_data["pn"], tag_odom_data["pe"], tag_odom_data["pd"])
ax.plot3D(odom_data["pn"], odom_data["pe"], odom_data["pd"])

fig, ax = plt.subplots(3,1)
size = 5
time = np.array(tag_odom_data["sec"]) + np.array(tag_odom_data["nsec"]) * 1E-9
time_odom = np.array(odom_data["sec"]) + np.array(odom_data["nsec"]) * 1E-9
time_mocap = np.array(mocap_data["sec"]) + np.array(mocap_data["nsec"]) * 1E-9
ax[0].scatter(time, tag_odom_data["pn"], color='r', s=size, label='fiducial')
ax[0].plot(time_odom, odom_data["pn"], label='estimate')
ax[0].plot(time_mocap, -mocap_data["pn"], label='truth')
ax[1].scatter(time, tag_odom_data["pe"], color='r', s=size)
ax[1].plot(time_odom, odom_data["pe"])
ax[1].plot(time_mocap, -mocap_data["pe"])
ax[2].scatter(time, tag_odom_data["pd"], color='r', s=size)
ax[2].plot(time_odom, odom_data["pd"])
ax[2].plot(time_mocap, np.array(-mocap_data["pd"])-.18)
ax[0].legend(loc='upper right')

# Plot apriltag vs expected apriltag
fig, ax = plt.subplots(3,1)

tag_hat_data = np.load("{}_at_hat.npz".format(bagfile))
tag_data = np.load("{}_tag_detections.npz".format(bagfile))

ax[0].scatter(tag_data["seq"], tag_data["x"], label="measured")
ax[0].scatter(np.array(tag_data["seq"]), tag_hat_data["x"], label="expected")
ax[1].scatter(tag_data["seq"], tag_data["y"])
ax[1].scatter(np.array(tag_data["seq"]), tag_hat_data["y"])
ax[2].scatter(tag_data["seq"], tag_data["z"])
ax[2].scatter(np.array(tag_data["seq"]), tag_hat_data["z"])
ax[0].legend(loc='upper right')
plt.show()

