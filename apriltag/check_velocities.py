from rosbag_parser import Parser
from IPython.core.debugger import set_trace 
import matplotlib.pyplot as plt
import rosbag
import numpy as np
import navpy

def main():
	flightModeTopic = '/dummyTopic'
	missionStateTopic = '/dummyTopic'
	baseGpsTopic = '/base/PosVelEcef'
	baseImuTopic = '/dummyTopic'
	roverRelPosTopic = '/rover/RelPos'
	roverGpsTopic = '/rover/PosVelEcef'
	baseOdomTopic = '/dummyTopic'
	data = Parser(flightModeTopic,missionStateTopic,baseGpsTopic,baseImuTopic,roverRelPosTopic,roverGpsTopic,baseOdomTopic)
	filename = 'flightA.bag'
	bag = rosbag.Bag('/home/matt/data/px4flight/outdoor/0424/' + filename)

	flightMode,missionState,estOdom,estRelPos,baseGps,roverGps,refLla = get_data(data,bag)

	baseGpsNed = ecef2ned(baseGps,refLla)
	roverGpsNed = ecef2ned(roverGps,refLla)

	# timeOfFlight,timeInterval = calc_time_of_flight(flightMode,missionState)
	# timeOfFlightList.append(timeOfFlight)

	# get_pn_data(ubloxRelPos,estRelPos)
	# get_pe_data(ubloxRelPos,estRelPos)
	# get_pd_data(ubloxRelPos,estRelPos)
	get_ub_data(estOdom,baseGpsNed)
	get_vb_data(estOdom,baseGpsNed)
	get_wb_data(estOdom,baseGpsNed)

	check_odom_vs_rover_u(estOdom,roverGpsNed,estRelPos)
	check_odom_vs_rover_v(estOdom,roverGpsNed)
	check_odom_vs_rover_w(estOdom,roverGpsNed)
	check_base_vs_rover_u(baseGpsNed,roverGpsNed)
	check_base_vs_rover_v(baseGpsNed,roverGpsNed)
	check_base_vs_rover_w(baseGpsNed,roverGpsNed)

	# get_attitude_data(estOdom)

	plt.show()

def calc_time_of_flight(flightMode,missionState):
	startTime = 0.0
	endTime = 0.0
	for i in range(len(flightMode.time)):
		if flightMode.flightMode[i] == 7:
			startTime = flightMode.time[i]
			break
	for j in range(len(missionState.time)):
		if missionState.missionState[j] == 3:
			endTime = missionState.time[j]
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

def get_data(data, bag):
	flightMode = data.get_flight_mode(bag)
	missionState = data.get_mission_state(bag)

	estRelPos, estOdom = data.get_odom(bag)
	# estRelPos.time -= estRelPos.time[0]
	# estRelPos.time += timeOffset[0]

	# imu = data.get_imu(bag)
	# imu.time -= imu.time[0]
	# imu.time += timeOffset[4]

	# measuredRelPos = data.get_ublox_relPos(bag)
	# measuredRelPos.time -= measuredRelPos.time[0]
	# measuredRelPos.time += timeOffset[5]

	baseGps = data.get_base_gps(bag)

	roverGps,refLla = data.get_rover_gps(bag)
	# roverGps.time -= roverGps.time[0]
	# roverGps.time += timeOffset[7]

	# rtkCompass = data.get_rtk_compass(bag)
	# rtkCompass.time -= rtkCompass.time[0]
	# rtkCompass.time += timeOffset[8]

	return flightMode, missionState, estOdom, estRelPos, baseGps, roverGps, refLla

def ecef2ned(gps,refLla):
	ecefOrigin1D = navpy.lla2ecef(refLla.lat,refLla.lon,refLla.alt)
	ecefOrigin = np.array([ecefOrigin1D]).T
	ecefPositionLocal = gps.position - ecefOrigin
	gpsPositionNed = navpy.ecef2ned(ecefPositionLocal,refLla.lat,refLla.lon,refLla.alt)
	gpsVelocityNed = navpy.ecef2ned(gps.velocity,refLla.lat,refLla.lon,refLla.alt)
	gpsNed = GpsNed(gps.time,gpsPositionNed,gpsVelocityNed)
	return gpsNed

def get_pn_data(ubloxRelPos, estRelPos):
	fig_num = 1
	plot_2(fig_num, ubloxRelPos.time, -ubloxRelPos.position[0], 'ublox n', estRelPos.time, estRelPos.position[0], 'estimate n')
	finalError = ubloxRelPos.position[0][-1] - estRelPos.position[0][-1]
	print('relative north final error = ', finalError, ' meters')

def get_pe_data(ubloxRelPos, estRelPos):
	fig_num = 2
	plot_2(fig_num, ubloxRelPos.time, -ubloxRelPos.position[1], 'ublox e', estRelPos.time, estRelPos.position[1], 'estimate e')
	finalError = ubloxRelPos.position[1][-1] - estRelPos.position[1][-1]
	print('relative east final error = ', finalError, ' meters')

def get_pd_data(ubloxRelPos, estRelPos):
	fig_num = 3
	plot_2(fig_num, ubloxRelPos.time, -ubloxRelPos.position[2], 'ublox d', estRelPos.time, estRelPos.position[2], 'estimate d')
	finalError = ubloxRelPos.position[2][-1] - estRelPos.position[2][-1]
	print('relative down final error = ', finalError, ' meters')

def get_ub_data(odom, gpsNed):
	fig_num = 4
	plot_2(fig_num, odom.time, odom.velocity[0], 'odom u', gpsNed.time, gpsNed.velocity[0], 'gps u')
	finalError = odom.velocity[0][-1] - gpsNed.velocity[0][-1]
	print('u final error = ', finalError, ' m/s')

def get_vb_data(odom, gpsNed):
	fig_num = 5
	plot_2(fig_num, odom.time, odom.velocity[1], 'odom v', gpsNed.time, gpsNed.velocity[1], 'gps v')
	finalError = odom.velocity[1][-1] - gpsNed.velocity[1][-1]
	print('v final error = ', finalError, ' m/s')

def get_wb_data(odom, gpsNed):
	fig_num = 6
	plot_2(fig_num, odom.time, odom.velocity[2], 'odom w', gpsNed.time, gpsNed.velocity[2], 'gps w')
	finalError = odom.velocity[2][-1] - gpsNed.velocity[2][-1]
	print('w final error = ', finalError, ' m/s')

def check_odom_vs_rover_u(baseOdom, roverGpsNed, relPos):
	fig_num = 7
	# set_trace()
	plot_3(fig_num, baseOdom.time, baseOdom.velocity[0], 'base odom u', roverGpsNed.time, roverGpsNed.velocity[0], 'rover u', relPos.time, relPos.position[0], 'base relPos x')

def check_odom_vs_rover_v(baseOdom, roverGpsNed):
	fig_num = 8
	plot_2(fig_num, baseOdom.time, baseOdom.velocity[1], 'base odom v', roverGpsNed.time, roverGpsNed.velocity[1], 'rover v')

def check_odom_vs_rover_w(baseOdom, roverGpsNed):
	fig_num = 9
	plot_2(fig_num, baseOdom.time, baseOdom.velocity[2], 'base odom w', roverGpsNed.time, roverGpsNed.velocity[2], 'rover w')

def check_base_vs_rover_u(baseGpsNed, roverGpsNed):
	fig_num = 10
	plot_2(fig_num, baseGpsNed.time, baseGpsNed.velocity[0], 'base u', roverGpsNed.time, roverGpsNed.velocity[0], 'rover u')

def check_base_vs_rover_v(baseGpsNed, roverGpsNed):
	fig_num = 11
	plot_2(fig_num, baseGpsNed.time, baseGpsNed.velocity[1], 'base v', roverGpsNed.time, roverGpsNed.velocity[1], 'rover v')

def check_base_vs_rover_w(baseGpsNed, roverGpsNed):
	fig_num = 12
	plot_2(fig_num, baseGpsNed.time, baseGpsNed.velocity[2], 'base w', roverGpsNed.time, roverGpsNed.velocity[2], 'rover w')

def get_attitude_data(odom):
	fig_num = 13
	plot_3(fig_num, odom.time, odom.euler, 'phi deg', 'theta deg', 'psi deg')

def plot_2(fig_num, t_x, x, xlabel, t_y, y, ylabel):

	plt.figure(fig_num)
	plt.plot(t_x, x, label = xlabel)
	plt.plot(t_y, y, label = ylabel)
	plt.legend(loc = "upper right")

def plot_3(fig_num, t_x, x, label1, t_y, y, label2, t_z, z, label3):
	plt.figure(fig_num)
	plt.plot(t_x,x,label = label1)
	plt.plot(t_y,y,label = label2)
	plt.plot(t_z,z,label = label3)
	plt.legend(loc = "upper right")

class GpsNed:
	def __init__(self,time,position,velocity):
		self.time = time
		self.position = position.T
		self.velocity = velocity.T

if __name__ == '__main__':
	main()
