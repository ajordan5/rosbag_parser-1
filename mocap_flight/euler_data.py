from estimates import plot_2
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
    mocapEulerTopic = '/m2u/mocap_euler'
    baseEulerTopic = '/base_euler'

	# The topics specified above are passed into 'rosbag_parse.py'
    data = Parser(mocapTopic, baseOdomTopic, roverOdomTopic, roverNEDTopic, boatNEDTopic,mocapEulerTopic, baseEulerTopic)

	# Specify the file and location of the bag file you want to extract data from
    filename = 'cf-kp04-ki008.bag'
    bag = rosbag.Bag('../../gainsTesting/boat-motion-11-24/kp04-ki0-008/' + filename)

    odom, boatOdom, truth, boatTruth, mocapEuler, baseEuler = get_data(data, bag)

    fig_num = 1
    fig_num = get_euler_data(mocapEuler, baseEuler, fig_num)
	# fig_num = get_east_data(odom[0], boatOdom, truth, boatTruth, fig_num)
	# fig_num = get_down_data(odom[0], boatOdom, truth, boatTruth, fig_num)
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
    mocapEuler = data.get_mocap_euler(bag)
    baseEuler = data.get_base_euler(bag)
    

    return odom, boatOdom, truth, boatTruth, mocapEuler, baseEuler


def get_euler_data(mocapEuler, baseEuler, fig_num):
    """Function to plot the north data of the rover and boat

    Inputs:
	    mocapEuler: data from '/m2u/mocap_euler'
        baseEuler: data from /base_euler

    """
    
  
    mocap_roll = [element * 180 / np.pi for element in mocapEuler.euler[0]] 
    base_roll = [element * 180 / np.pi for element in baseEuler.euler[0]] 
    mocap_pitch = [element * 180 / np.pi for element in mocapEuler.euler[1]] 
    base_pitch = [element * 180 / np.pi for element in baseEuler.euler[1]] 
    mocap_yaw = [element * 180 / np.pi for element in mocapEuler.euler[2]] 
    base_yaw = [element * 180 / np.pi for element in baseEuler.euler[2]] 

    
    mocap_time = [element - mocapEuler.time[0] for element in mocapEuler.time]
    base_time = [element - mocapEuler.time[0] for element in baseEuler.time]

    
    fig_num = plot_2(fig_num, mocap_time, mocap_roll, 'Mocap Roll', base_time, base_roll, 'Base Roll')
    fig_num = plot_2(fig_num, mocap_time, mocap_pitch, 'Mocap Pitch', base_time, base_pitch, 'Base Pitch')
    # fig_num = plot_2(fig_num, mocapEuler.time, mocap_yaw, 'Mocap Yaw', baseEuler.time, base_yaw, 'Base Yaw')




    return fig_num
	


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
