from rosbag_parser import Parser
import matplotlib.pyplot as plt
# from IPython.core.debugger import set_trace
import rosbag
import numpy as np
from collections import namedtuple

def main():
	
	data = Parser()

	filename = 'anomally.bag'
	bag = rosbag.Bag('../../mocap_anom/' + filename)

	odom, truth= get_data(data, bag)
	plot_2(1, odom.time, odom.position[0], 'odom', truth.time, truth.n, 'truth')
	plot_2(2, odom.time, odom.position[1], 'odom', truth.time, truth.e, 'truth')
	plot_2(3, odom.time, odom.position[2], 'odom', truth.time, truth.d, 'truth')

def get_data(data, bag):

	odom = data.get_odom(bag)
	truth = data.get_ragnarok_ned(bag)
	odom.position[0] = np.array(odom.position[0]) + truth.n[0]
	odom.position[1] = np.array(odom.position[1]) + truth.e[0]
	odom.position[2] = np.array(odom.position[2]) + truth.d[0]

	return odom, truth

def plot_2(fig_num, t_x, x, xlabel, t_y, y, ylabel):

	plt.figure(fig_num)
	plt.plot(t_x, x, label = xlabel)
	plt.plot(t_y, y, label = ylabel)
	plt.legend(loc = "upper right")
	plt.show()


if __name__ == '__main__':
	main()
