from rosbag_parser import Parser
import matplotlib.pyplot as plt
from IPython.core.debugger import set_trace
import rosbag
import numpy as np
from collections import namedtuple

def main():
	
	data = Parser()

	filename = 'm2u_w_boat_2021-10-15-21-28-51.bag'
	bag = rosbag.Bag('/home/magicc_undergrad/LandonShumway/rosbag_parser-1/' + filename)

	data_type = 'mocap_master_branch'
	# data_type = 'mocap'
	# data_type = 'm2u'
	# data_type = 'outdoor'
	# data_type = 'sim'
	odom, hlc = get_data(data, bag, data_type)
	get_north_data(odom, hlc)
	get_east_data(odom, hlc)
	get_height_data(odom, hlc)


def get_data(data, bag, data_type):

	if data_type == 'mocap_master_branch':
		
		odom = data.get_odom(bag)

		hlc = data.get_master_branch_high_level_command(bag) #used for master branch (byu) mocap

	elif data_type == 'mocap':

		odom = data.get_odom(bag)

		hlc = data.get_high_level_command(bag) #used for mocap

	elif data_type == 'm2u':

		odom = data.get_odom(bag)

		hlc = data.get_high_level_command(bag) #used for mocap2ublox

	elif data_type == 'outdoor':

		odom = data.get_odom(bag)		

		hlc = data.get_high_level_command(bag)

	elif data_type == 'sim':

		odom = data.get_multirotor_odom(bag)

		hlc = data.get_multirotor_high_level_command(bag)

	else:
		print('invalid data_type')

		bag.close()

	return odom, hlc


def get_north_data(odom, hlc):

	odom_time = odom.time
	odom_n = np.array(odom.position[0])

	hlc_time_points = hlc.time
	hlc_n_points = np.array(hlc.position[0])
	hlc_time, hlc_n = level_hlc_data(hlc_time_points, hlc_n_points)

	fig_num = 1
	plot_2(fig_num, odom_time, odom_n, 'odom', hlc_time, hlc_n, 'hlc')


def get_east_data(odom, hlc):

	odom_time = odom.time
	odom_e = np.array(odom.position[1])

	hlc_time_points = hlc.time
	hlc_e_points = np.array(hlc.position[1])
	hlc_time, hlc_e = level_hlc_data(hlc_time_points, hlc_e_points)

	fig_num = 2
	plot_2(fig_num, odom_time, odom_e, 'odom', hlc_time, hlc_e, 'hlc')


def get_height_data(odom, hlc):

	odom_time = odom.time
	odom_h = -np.array(odom.position[2])

	hlc_time_points = hlc.time
	hlc_h_points = np.array(hlc.position[2])
	hlc_time, hlc_h = level_hlc_data(hlc_time_points, hlc_h_points)

	fig_num = 3
	plot_2(fig_num, odom_time, odom_h, 'odom', hlc_time, hlc_h, 'hlc')


def level_hlc_data(time, pos):

	#function inserts data points just before a change in hlc, so that plots have level lines for the durration of that command
	new_time = []
	new_pos = []
	back_step = 0.001 #this is the time back from a point that we insert a value
	for i in range(len(time)):
		if i == 0:
			new_time.append(time[i])
			new_pos.append(pos[i])
		else:
			new_time.append(time[i]-back_step)
			new_time.append(time[i])
			new_pos.append(pos[i-1])
			new_pos.append(pos[i])
	
	return new_time, new_pos


def plot_2(fig_num, t_x, x, xlabel, t_y, y, ylabel):

	plt.figure(fig_num)
	plt.plot(t_x, x, label = xlabel)
	plt.plot(t_y, y, label = ylabel)
	plt.legend(loc = "upper right")
	plt.show()


if __name__ == '__main__':
	main()
