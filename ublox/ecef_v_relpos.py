from rosbag_parser import Parser
import matplotlib.pyplot as plt
# from IPython.core.debugger import set_trace
import rosbag
import numpy as np
from collections import namedtuple
import os

def main():

	data = Parser()
	
	filename = 'outdoor_w_boat_2022-02-05_moving1.bag'
	path = os.path.join('../data/tests0205', filename)
	bag = rosbag.Bag(path)

	data_type = 'outdoor'
	# data_type = 'mocap'
	# data_type = 'm2u'
	# data_type = 'outdoor'
	# data_type = 'sim'
	roverNED, boatNED, relpos = get_data(data, bag, data_type)
	get_north_data(roverNED, boatNED, relpos)
	get_east_data(roverNED, boatNED, relpos)
	get_down_data(roverNED, boatNED, relpos)

def get_data(data, bag, data_type):

	if data_type == 'outdoor':

		roverPVE = data.get_PosVelECEF(bag)
		boatPVE = data.get_boat_PosVelECEF(bag)
		relpos = data.get_RelPos(bag)

		boatNED = pve2ned(boatPVE,roverPVE)
		roverNED = pve2ned(roverPVE,roverPVE)
		# relpos = -relpos

	else:
		print('invalid data_type')

		bag.close()

	return roverNED, boatNED, relpos

def pve2ned(pve, originSet):
	ned = pve
	length = len(pve.time)
	position = np.array(pve.position).T
	origin = np.array(originSet.position).T[0]
	lla = np.array(originSet.lla).T[0]
	for i in range(length-1):
		positionNED = ecef2ned(position[i], origin, lla)
		ned.position[0][i] = positionNED[0]
		ned.position[1][i] = positionNED[1]
		ned.position[2][i] = positionNED[2]
	return ned

def ecef2ned(ecef, ecefOrigin, refLla):

	lon = refLla[1]*np.pi/180.0
	lat = refLla[0]*np.pi/180.0
	Clon = np.cos(lon)
	Slon = np.sin(lon)
	Clat = np.cos(-lat) #because ublox reports it positive when it should be negative
	Slat = np.sin(-lat)
	C90 = np.cos(-np.pi/2.0)
	S90 = np.sin(-np.pi/2.0)

	Hecef = np.array([[1.0, 0.0, 0.0, ecef[0]],
				   [0.0, 1.0, 0.0, ecef[1]],
				   [0.0, 0.0, 1.0, ecef[2]],
				   [0.0, 0.0, 0.0, 1.0]])

	Ht = np.array([[1.0, 0.0, 0.0, -ecefOrigin[0]],
				   [0.0, 1.0, 0.0, -ecefOrigin[1]],
				   [0.0, 0.0, 1.0, -ecefOrigin[2]],
				   [0.0, 0.0, 0.0, 1.0]])

	Hz = np.array([[Clon, Slon, 0.0, 0.0],
				   [-Slon, Clon, 0.0, 0.0],
				   [0.0, 0.0, 1.0, 0.0],
				   [0.0, 0.0, 0.0, 1.0]])

	Hy =  np.array([[Clat, 0.0, -Slat, 0.0],
				   [0.0, 1.0, 0.0, 0.0],
				   [Slat, 0.0, Clat, 0.0],
				   [0.0, 0.0, 0.0, 1.0]])

	H90 = np.array([[C90, 0.0, -S90, 0.0],
				   [0.0, 1.0, 0.0, 0.0],
				   [S90, 0.0, C90, 0.0],
				   [0.0, 0.0, 0.0, 1.0]])

	Hned = H90@Hy@Hz@Ht@Hecef
	ned = np.array([Hned[0][3], Hned[1][3], Hned[2][3]])

	return ned


def get_north_data(roverEcef, boatEcef, relpos):

	delta_time = []
	delta_n = []
	roverEcef_time = roverEcef.time - roverEcef.time[0]
	boatEcef_time = boatEcef.time - boatEcef.time[0]

	j = 0
	for i in range(len(boatEcef_time)-1):
		while (roverEcef_time[j] < boatEcef_time[i]):
			j = j+1
		if (j == len(roverEcef_time)-1):
			break
		delta_time.append(roverEcef_time[j])
		delta_n.append(roverEcef.position[0][j] - boatEcef.position[0][i])

	delta_time = delta_time - delta_time[0]
	relpos_time = relpos.time - relpos.time[0]

	fig_num = 1
	plot_2(fig_num, delta_time, delta_n, 'delta', relpos_time, np.array(relpos.position[0]), 'relpos')


def get_east_data(roverEcef, boatEcef, relpos):

	delta_time = []
	delta_e = []
	roverEcef_time = roverEcef.time - roverEcef.time[0]
	boatEcef_time = boatEcef.time - boatEcef.time[0]

	j = 0
	for i in range(len(boatEcef_time)-1):
		while (roverEcef_time[j] < boatEcef_time[i]):
			j = j+1
		if (j == len(roverEcef_time)-1):
			break
		delta_time.append(roverEcef_time[j])
		delta_e.append(roverEcef.position[1][j] - boatEcef.position[1][i])

	delta_time = delta_time - delta_time[0]
	relpos_time = relpos.time - relpos.time[0]

	fig_num = 2
	plot_2(fig_num, delta_time, delta_e, 'delta', relpos_time, np.array(relpos.position[1]), 'relpos')


def get_down_data(roverEcef, boatEcef, relpos):

	delta_time = []
	delta_d = []
	roverEcef_time = roverEcef.time - roverEcef.time[0]
	boatEcef_time = boatEcef.time - boatEcef.time[0]

	j = 0
	for i in range(len(boatEcef_time)-1):
		while (roverEcef_time[j] < boatEcef_time[i]):
			j = j+1
		if (j == len(roverEcef_time)-1):
			break
		delta_time.append(roverEcef_time[j])
		delta_d.append(roverEcef.position[2][j] - boatEcef.position[2][i])

	delta_time = delta_time - delta_time[0]
	relpos_time = relpos.time - relpos.time[0]

	fig_num = 3
	plot_2(fig_num, delta_time, delta_d, 'delta', relpos_time, np.array(relpos.position[2]), 'relpos')


def plot_2(fig_num, t_x, x, xlabel, t_y, y, ylabel):

	plt.figure(fig_num)
	plt.plot(t_x, x, label = xlabel)
	plt.plot(t_y, y, label = ylabel)
	plt.legend(loc = "upper right")
	plt.show()


if __name__ == '__main__':
	main()
