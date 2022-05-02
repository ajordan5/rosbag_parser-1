from rosbag_parser import Parser
import matplotlib.pyplot as plt
# from IPython.core.debugger import set_trace
import rosbag
import numpy as np
from collections import namedtuple

def main():
	# These are the topics that will be pushed into the parser
	mocapTopic = 'm2u/base/PosVelEcef' #TODO: Check this topic
	baseOdomTopic = '/base_odom'
	roverOdomTopic = '/rover_odom'
	roverNEDTopic = '/rhodey_ned'
	boatNEDTopic = '/boat_landing_platform_ned'
	roverEulerTopic = '/m2u/mocap_euler'
	baseEulerTopic = '/base_euler'

	# The topics specified above are passed into 'rosbag_parse.py'
	data = Parser(mocapTopic, baseOdomTopic, roverOdomTopic, roverNEDTopic, boatNEDTopic,roverEulerTopic, baseEulerTopic)

	# Specify the file and location of the bag file you want to extract data from
	filename = 'anomally.bag'
	bag = rosbag.Bag('../../mocap_anom/' + filename)

	odom, boatOdom, truth, boatTruth = get_data(data, bag)

	fig_num = 1
	fig_num = get_north_data(odom[0], boatOdom, truth, boatTruth, fig_num)
	fig_num = get_east_data(odom[0], boatOdom, truth, boatTruth, fig_num)
	fig_num = get_down_data(odom[0], boatOdom, truth, boatTruth, fig_num)
	plt.show()
	#get_down_data(odom, boatOdom, truth, boatTruth)




def get_data(data, bag):
	"""Function to gather data from the topics selected above
	
	Inputs: 
		data (class): contains member variables that correspond to the topics you specified in line 17
		bag: the bag file you specified to analyze (lines 20-21)
	Returns:
		odom: data from '/rover_odom'
		boatOdom: data from '/base_odom'
		truth: data from 'rhodey_ned'
		boatTruth: data from '/boat_landing_platform_ned'
	"""
	
	odom = data.get_odom(bag)
	boatOdom = data.get_boat_odom(bag)
	truth = data.get_rhodey_ned(bag)
	boatTruth = data.get_boat_landing_platform_ned(bag)

	return odom, boatOdom, truth, boatTruth


def get_north_data(odom, boatOdom, truth, boatTruth, fig_num):
	"""Function to plot the north data of the rover and boat

	Inputs:
		odom: data from '/rover_odom'
		boatOdom: data from '/base_odom'
		truth: data from 'rhodey_ned'
		boatTruth: data from '/boat_landing_platform_ned'

	"""
	odom_n = odom.position[0] - odom.position[0][0]
	boat_n = boatOdom.position[0] - odom.position[0][0]

	fig_num = plot_2(fig_num, odom.time, odom_n, 'N Rover Odom', truth.time, truth.n, 'N Rover Truth')
	fig_num = plot_2(fig_num, boatOdom.time, boat_n, 'N Boat Odom', boatTruth.time, boatTruth.position[0], 'N Boat Truth')
	fig_num = plot_1(fig_num, odom.time, odom_n, 'N Rover Odom')
	fig_num = plot_1(fig_num, truth.time, truth.n, 'N Rover Truth')

	return fig_num
	
	
def get_east_data(odom, boatOdom, truth, boatTruth, fig_num):
	"""Function to plot the east data of the rover and boat

	Inputs:
		odom: data from '/rover_odom'
		boatOdom: data from '/base_odom'
		truth: data from 'rhodey_ned'
		boatTruth: data from '/boat_landing_platform_ned'

	"""
	odom_e = odom.position[1] - odom.position[1][0]
	boat_e = boatOdom.position[1] - odom.position[1][0]

	
	fig_num = plot_2(fig_num, odom.time, odom_e, 'E Rover Odom', truth.time, truth.e, 'E Rover Truth')
	fig_num = plot_2(fig_num, boatOdom.time, boat_e, 'E Boat Odom', boatTruth.time, boatTruth.position[1], 'E Boat Truth')
	
	# plot_1(fig_num, odom.time, odom_e, 'E Rover Odom')
	# plot_1(fig_num, truth.time, truth.e, 'E Rover Truth')

	return fig_num

def get_down_data(odom, boatOdom, truth, boatTruth, fig_num):
	"""Function to plot the down data of the rover and boat

	Inputs:
		odom: data from '/rover_odom'
		boatOdom: data from '/base_odom'
		truth: data from 'rhodey_ned'
		boatTruth: data from '/boat_landing_platform_ned'

	"""
	odom_d = odom.position[2] - odom.position[2][0]
	boat_d = boatOdom.position[2] - odom.position[2][0]

	
	fig_num = plot_2(fig_num, odom.time, odom_d, 'D Rover Odom', truth.time, truth.d, 'D Rover Truth')
	fig_num = plot_2(fig_num, boatOdom.time, boat_d, 'D Boat Odom', boatTruth.time, boatTruth.position[2], 'D Boat Truth')
	
	# plot_1(fig_num, odom.time, odom_e, 'E Rover Odom')
	# plot_1(fig_num, truth.time, truth.e, 'E Rover Truth')

	return fig_num
	

# def get_east_data(odom, boatOdom, relpos):

# 	delta_time = []
# 	delta_e = []
# 	odom_time = odom.time - odom.time[0]
# 	odom_e = np.array(odom.position[1])
# 	boatOdom_time = boatOdom.time - boatOdom.time[0]
# 	boatOdom_e = np.array(boatOdom.position[1])
# 	j = 0
# 	for i in range(len(boatOdom_time)-1):
# 		while (odom_time[j] < boatOdom_time[i]):
# 			if (j == len(odom_time)-1):
# 				break
# 			j = j+1
# 		delta_time.append(odom_time[j])
# 		delta_e.append(odom_e[j] - boatOdom_e[i])

# 	relpos_time = np.array(relpos[0])+np.array(relpos[1])*1E-9
# 	relpos_time = relpos_time - relpos_time[0]
# 	relpos_e = np.array(relpos[3])


# 	fig_num = 3
# 	plot_2(fig_num, delta_time, delta_e, 'delta', relpos_time, relpos_e, 'relpos')
# 	plt.show()
#
#
# def get_down_data(odom, boatOdom, relpos):
#
# 	delta_time = []
# 	delta_d = []
# 	odom_time = odom.time - odom.time[0]
# 	odom_d = np.array(odom.position[2])
# 	boatOdom_time = boatOdom.time - boatOdom.time[0]
# 	boatOdom_d = np.array(boatOdom.position[2])
# 	j = 0
# 	for i in range(len(boatOdom_time)-1):
# 		while (odom_time[j] < boatOdom_time[i]):
# 			if (j == len(odom_time)-1):
# 				break
# 			j = j+1
# 		delta_time.append(odom_time[j])
# 		delta_d.append(odom_d[j] - boatOdom_d[i])
#
# 	relpos_time = np.array(relpos[0])+np.array(relpos[1])*1E-9
# 	relpos_time = relpos_time - relpos_time[0]
# 	relpos_d = np.array(relpos[4])
#
#
# 	fig_num = 3
# 	plot_2(fig_num, delta_time, delta_d, 'delta', relpos_time, relpos_d, 'relpos')


def plot_2(fig_num, t_x, x, xlabel, t_y, y, ylabel):
	"""Function to plot 2 quantities
	
	Inputs: 
		fig_num (int): figure number
		t_x (array[float]): array of time stamps associated with x
		x (array[float]): array of first data set to plot
		xlabel (string): string label for first data set
		t_y (array[float]): array of time stamps associated with y
		y (array[float]): array of second data set to plot
		ylabel (string): string label for second data set

		"""
	plt.figure(fig_num)
	plt.plot(t_x, x, label = xlabel)
	plt.plot(t_y, y, label = ylabel)
	plt.legend(loc = "upper right")
	#plt.show()

	fig_num+=1
	return fig_num

def plot_1(fig_num, t_x, x, xlabel):
	"""Function to plot 1 quantity
	
	Inputs: 
		fig_num (int): figure number
		t_x (array[float]): array of time stamps associated with x
		x (array[float]): array of first data set to plot
		xlabel (string): string label for first data set

		"""
	plt.figure(fig_num)
	plt.plot(t_x, x, label = xlabel)
	plt.legend(loc = "upper right")
	#plt.show()

	fig_num+=1
	return fig_num


if __name__ == '__main__':
	main()
