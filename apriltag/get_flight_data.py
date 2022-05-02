from rosbag_parser import Parser
#from IPython.core.debugger import set_trace 
import matplotlib.pyplot as plt
import rosbag
import numpy as np
import statistics as stats
import os
import argparse
from scipy.spatial.transform import Rotation as R

def main(path):
	""" This script gathers the selected data (defined by the nodes below by calling rosbag_parser.py.
	If there are many bag files, statistics of will be gathered from all the bagfiles in a list (i.e. bagNames1, bagNames2). 
	If there is just one bag file to be looked at, the baseOdomTopic, baseGpsTopic, and roverGpsTopic are plotted
	"""
	# Set topics you wish to extract
	flightModeTopic = '/flight_mode'
	missionStateTopic = '/mission_state'
	baseGpsTopic = '/base/PosVelEcef'
	baseImuTopic = '/base/imu'
	roverRelPosTopic = '/rover/RelPos'
	roverGpsTopic = '/rover/PosVelEcef'
	baseOdomTopic = '/base_odom'
	baseEulerTopic = '/base_euler'
	data = Parser(flightModeTopic,missionStateTopic,baseGpsTopic,baseImuTopic,roverRelPosTopic,roverGpsTopic,baseOdomTopic, baseEulerTopic)
	
	# If there is only one bag file in the list, stats won't be given. If there is more, stats will be given
	bagNames1 = [file for file in os.listdir(path)]
	#bagNames1 = ['outdoor_w_boat_2021-11-11-16-39-38.bag.active'] #, 'outdoor_w_boat_2021-10-30-16-54-00.bag']
	# bagNames1 = ['flightAFast.bag','flightB.bag','flightD.bag','flightE.bag','flightg.bag','flightH.bag']
	# bagNames2 = ['flightA.bag','flightB.bag','flightC.bag','flightD.bag','flightE.bag','flightF.bag','flightH.bag','flightI.bag']

	timeOfFlightList = []
	startDistanceList = []
	speedList = []
	eulerList = []
	accelList = []
	angRatesList = []
	angRatesListX = []
	angRatesListY = []
	maxPitch = []
	maxRoll = []
	for bagName in os.listdir(path):
		print("Stats for ", bagName)
		bag = rosbag.Bag(path + bagName)
		flightMode,missionState,baseGps,roverGps,imu,relPos,boatOdom, odom = get_data(data,bag)
		

		timeOfFlight,timeInterval = calc_time_of_flight(flightMode,missionState)
		timeOfFlightList.append(timeOfFlight)
		print('time of flight = ', timeOfFlight)
		relPostimeIndices = find_time_interval_indeces(relPos,timeInterval)
		startDistance = calc_start_distance_data(relPos,relPostimeIndices)
		startDistanceList.append(startDistance)

		speedtimeIndices = find_time_interval_indeces(baseGps,timeInterval)
		speed = calc_speed_data(baseGps,speedtimeIndices)
		speedList.extend(speed)
		mean,stdDev = get_stats(speed)
		print('speed:')
		print('		mean = ', mean)
		print('		std dev = ', stdDev)

		acceltimeIndices = find_time_interval_indeces(imu,timeInterval)
		accel = calc_accel_data(imu,acceltimeIndices)
		accelList.extend(accel)

		odomtimeIndices = find_time_interval_indeces(boatOdom,timeInterval)
		euler = calc_odom_data(boatOdom,odomtimeIndices)
		eulerList.extend(euler)
		eulerArr = np.array(euler)
		pitch = eulerArr[:,0]
		mean,stdDev = get_stats(abs(pitch))
		print('pitch:')
		print('		mean = ', mean)
		print('		std dev = ', stdDev)
		print('     max = ', pitch.max())
		print('     min = ', pitch.min())
		maxPitch.append(max(pitch.max(), pitch.min()))
		roll = eulerArr[:,1]
		mean,stdDev = get_stats(abs(roll))
		print('roll:')
		print('		mean = ', mean)
		print('		std dev = ', stdDev)
		print('     max = ', roll.max())
		print('     min = ', roll.min())
		maxRoll.append(max(roll.max(), roll.min()))
		
		fig_num = 1
		fig_num = get_north_data(odom[0], boatOdom, fig_num)
		fig_num = get_east_data(odom[0], boatOdom, fig_num)
		fig_num = get_down_data(odom[0], boatOdom, fig_num)
		fig_num = plot_2(fig_num,roverGps[0].time,roverGps[0].position[0], "N Rover GPS", baseGps.time, baseGps.position[0], "N BaseGPS")
		fig_num = plot_2(fig_num,roverGps[0].time,roverGps[0].position[1], "E Rover GPS", baseGps.time, baseGps.position[1], "E BaseGPS")
		fig_num = plot_2(fig_num,roverGps[0].time,roverGps[0].position[2], "D Rover GPS", baseGps.time, baseGps.position[2], "D BaseGPS")
		plt.waitforbuttonpress()
		plt.show()


		angRatestimeIndices = find_time_interval_indeces(imu,timeInterval)
		angRates, angRatesX, angRatesY = calc_ang_rates_data(imu,angRatestimeIndices)
		angRatesList.extend(angRates)
		angRatesListX.extend(angRatesX)
		angRatesListY.extend(angRatesY)
		save_data(flightMode,missionState,baseGps,roverGps,imu,relPos,boatOdom, odom, bagName, speedtimeIndices, 
					speedList, acceltimeIndices, accelList, angRatestimeIndices, angRatesList)

	#if more than one bag file is being analyzed in a list, gather the statistics
	if len(bagNames1) > 1:
		startDistanceMean,startDistanceStdDev = get_stats(startDistanceList)
		speedMean,speedStdDev = get_stats(speedList)
		rollMean,rollStdDev = get_stats(maxRoll)
		pitchMean,pitchStdDev = get_stats(maxPitch)
		accelMean,accelStdDev = get_stats(accelList)
		angRatesMean,angRatesStdDev = get_stats(angRatesList)
		angRatesMeanX,angRatesStdDevX = get_stats(angRatesListX)
		angRatesMeanY,angRatesStdDevY = get_stats(angRatesListY)
		timeOfFlightMean,timeOfFlightStdDev = get_stats(timeOfFlightList)

		print('starting distance away data:')
		print('		mean = ', startDistanceMean)
		print('		std dev = ', startDistanceStdDev)

		print('speed data:')
		print('		mean = ', speedMean)
		print('		std dev = ', speedStdDev)

		print('accel data:')
		print('		mean = ', accelMean)
		print('		std dev = ', accelStdDev)

		print('angular rates data:')
		print('		mean = ', angRatesMean)
		print('		std dev = ', angRatesStdDev)
		print('		meanx = ', angRatesMeanX)
		print('		std dev = ', angRatesStdDevX)
		print('		meany = ', angRatesMeanY)
		print('		std dev = ', angRatesStdDevY)

		print('pitch data:')
		print('		mean = ', pitchMean)
		print('		std dev = ', pitchStdDev)

		print('roll data:')
		print('		mean = ', rollMean)
		print('		std dev = ', rollStdDev)
			
		print('time of flight data:')
		print('		mean =', timeOfFlightMean)
		print('		std dev = ', timeOfFlightStdDev)

	# get and plot data
	fig_num = 1
	fig_num = get_north_data(odom[0], boatOdom, fig_num)
	fig_num = get_east_data(odom[0], boatOdom, fig_num)
	fig_num = get_down_data(odom[0], boatOdom, fig_num)
	fig_num = plot_2(fig_num,roverGps[0].time,roverGps[0].position[0], "N Rover GPS", baseGps.time, baseGps.position[0], "N BaseGPS")
	fig_num = plot_2(fig_num,roverGps[0].time,roverGps[0].position[1], "E Rover GPS", baseGps.time, baseGps.position[1], "E BaseGPS")
	fig_num = plot_2(fig_num,roverGps[0].time,roverGps[0].position[2], "D Rover GPS", baseGps.time, baseGps.position[2], "D BaseGPS")
	plt.waitforbuttonpress()
	plt.show()

def calc_time_of_flight(flightMode,missionState):
	startTime = 0.0
	endTime = 0.0
	for i in range(len(flightMode.time)):
		if flightMode.flightMode[i] == 7:
			startTime = flightMode.time[i]
			break
	for j in range(len(missionState.time)):
		if missionState.missionState[j] == 4:
			endTime = missionState.time[j]
			break

	if endTime == 0.0:
		for i in range(len(flightMode.time)):
			if flightMode.flightMode[i] == 11:
				endTime = flightMode.time[i]
				break


	timeOfFlight = endTime - startTime
	timeInterval = [startTime,endTime]
	return timeOfFlight, timeInterval

def find_time_interval_indeces(x,timeInterval):
	for i in range(len(x.time)):
		if x.time[i] >= timeInterval[0]:
			startIndex = i
			break
	for j in range(i,len(x.time)):
		if x.time[j] >= timeInterval[1]:
			endIndex = j
			break
	timeIntervalIndeces = [startIndex,endIndex]
	return timeIntervalIndeces

def calc_start_distance_data(relPos,timeIndices):
	relPosStart = np.linalg.norm(relPos.position[:,timeIndices[0]])
	return relPosStart

def calc_speed_data(baseGps,timeIndices):
	speedList = []
	for i in range(timeIndices[0],timeIndices[1]):
		normVelocityI = np.linalg.norm(baseGps.velocity[:,i])
		speedList.append(normVelocityI)
	return speedList

def calc_odom_data(baseOdom,timeIndices):
	eulerList = []
	for i in range(timeIndices[0],timeIndices[1]):
		quat = baseOdom.orientation[i]
		base_attitude = R.from_quat(np.array([quat.x, quat.y,quat.z,quat.w]))
		euler = base_attitude.as_euler('xyz', degrees=True)
		eulerList.append(euler.tolist())

	return eulerList

def calc_accel_data(imu,timeIndices):
	accelList = []
	alpha = 0.02
	filteredAccelX = low_pass_filter(imu.accel[0,:],alpha)
	filteredAccelY = low_pass_filter(imu.accel[1,:],alpha)
	filteredAccelZ = low_pass_filter(imu.accel[2,:],alpha)
	for i in range(timeIndices[0],timeIndices[1]):
		accelINoGravity = [filteredAccelX[i],filteredAccelY[i],filteredAccelZ[i]+9.81]
		normAccelI = np.linalg.norm(accelINoGravity)
		accelList.append(normAccelI)
	return accelList

def calc_ang_rates_data(imu,timeIndices):
	omegaList = []
	omegaListX = []
	omegaListY = []

	alpha = 0.02
	filteredOmegaX = low_pass_filter(imu.omega[0,:],alpha)
	filteredOmegaY = low_pass_filter(imu.omega[1,:],alpha)
	for i in range(timeIndices[0],timeIndices[1]):
		omegaIXY = [filteredOmegaX[i],filteredOmegaY[i]]
		normOmegaI = np.linalg.norm(omegaIXY)
		omegaList.append(normOmegaI)
		omegaListY.append(abs(filteredOmegaY[i]))
		omegaListX.append(abs(filteredOmegaX[i]))
	return omegaList, omegaListX, omegaListY

def low_pass_filter(signal,alpha):
	filteredSignal = []
	filteredSignal0 = signal[0]*alpha
	filteredSignal.append(filteredSignal0)
	for i in range(1,len(signal)):
		filteredSignalI = signal[i]*alpha + filteredSignal[-1]*(1.0-alpha)
		filteredSignal.append(filteredSignalI)
	return filteredSignal

def get_stats(x):
	mean = stats.mean(x)
	stdDev = stats.stdev(x)
	return mean,stdDev

def get_data(data, bag):
	flightMode = data.get_flight_mode(bag)
	missionState = data.get_mission_state(bag)
	baseGps = data.get_base_gps(bag)
	roverGps = data.get_rover_gps(bag)
	imu = data.get_imu(bag)
	relPos = data.get_rover_relPos(bag)
	boatOdom = data.get_boat_odom(bag)
	odom = data.get_odom(bag)

	return flightMode,missionState,baseGps, roverGps, imu,relPos,boatOdom, odom

def save_data(flightMode,missionState,baseGps,roverGps,imu,relPos,boatOdom, odom, bagName, speedtimeIndices, 
					speedList, acceltimeIndices, accelList, angRatestimeIndices, angRatesList):
	"""Save all ros data as np arrays for future use"""
	data = {}
	data['mode']=flightMode.flightMode
	data['mode_time']=flightMode.time
	data['state']=missionState.missionState
	data['state_time'] = missionState.time
	data['base_gps']=baseGps.position
	data['base_time']=baseGps.time
	data['base_vel']=baseGps.velocity
	data['rover_gps']=roverGps[0].position
	data['rover_time']=roverGps[0].time
	data['rover_vel']=roverGps[0].velocity
	data['accel'] = imu.accel
	data['gyro'] = imu.omega
	data['imu_time'] = imu.time
	data['relPos_pos'] = relPos.position
	data['relPos_time'] = relPos.time
	data['boatOdom_orient'] = boatOdom.orientation
	data['boatOdom_pos'] = boatOdom.position
	data['boatOdom_time'] = boatOdom.time
	data['odom_pos'] = odom[0].position
	data['odom_time'] = odom[0].time
	data['odom_velocity'] = odom[1].velocity
	data['odom_euler'] = odom[1].euler
	data['speedtimeIndices'] = speedtimeIndices
	data['speedList'] = speedList
	data['acceltimeIndices'] = acceltimeIndices
	data['accelList'] = accelList
	data['angRatestimeIndices'] = angRatestimeIndices
	data['angRatesList'] = angRatesList
	filename = './flight_data/' + os.path.splitext(bagName)[0] + '.npz'
	np.savez(filename, **data)


def get_north_data(odom, boatOdom, fig_num):
	"""Function to plot the north data of the rover and boat

	Inputs:
		odom: data from '/rover_odom'
		boatOdom: data from '/base_odom'
		truth: data from 'rhodey_ned'
		boatTruth: data from '/boat_landing_platform_ned'

	"""
	#odom_n = odom.position[0] - odom.position[0][0]
	boat_n = boatOdom.position[0] - odom.position[0][0]

	#fig_num = plot_1(fig_num, odom.time, odom_n, 'N Rover Odom')
	fig_num = plot_1(fig_num, boatOdom.time, boat_n, 'N Boat Odom')

	return fig_num
	
	
def get_east_data(odom, boatOdom, fig_num):
	"""Function to plot the east data of the rover and boat

	Inputs:
		odom: data from '/rover_odom'
		boatOdom: data from '/base_odom'
		truth: data from 'rhodey_ned'
		boatTruth: data from '/boat_landing_platform_ned'

	"""
	#odom_e = odom.position[1] - odom.position[1][0]
	boat_e = boatOdom.position[1] - odom.position[1][0]

	
	#fig_num = plot_2(fig_num, odom.time, odom_e, 'E Rover Odom', truth.time, truth.e, 'E Rover Truth')
	fig_num = plot_1(fig_num, boatOdom.time, boat_e, 'E Boat Odom')
	
	# plot_1(fig_num, odom.time, odom_e, 'E Rover Odom')
	# plot_1(fig_num, truth.time, truth.e, 'E Rover Truth')

	return fig_num

def get_down_data(odom, boatOdom, fig_num):
	"""Function to plot the down data of the rover and boat

	Inputs:
		odom: data from '/rover_odom'
		boatOdom: data from '/base_odom'
		truth: data from 'rhodey_ned'
		boatTruth: data from '/boat_landing_platform_ned'

	"""
	#odom_d = odom.position[2] - odom.position[2][0]
	boat_d = boatOdom.position[2] - odom.position[2][0]

	
	#fig_num = plot_2(fig_num, odom.time, odom_d, 'D Rover Odom', truth.time, truth.d, 'D Rover Truth')
	fig_num = plot_1(fig_num, boatOdom.time, boat_d, 'D Boat Odom')
	
	# plot_1(fig_num, odom.time, odom_e, 'E Rover Odom')
	# plot_1(fig_num, truth.time, truth.e, 'E Rover Truth')

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
	parser = argparse.ArgumentParser(description="Get flight data")
	parser.add_argument("-p", "--path", default="../data/tests_03_11/")
	args = vars(parser.parse_args())
	
	main(**args)
