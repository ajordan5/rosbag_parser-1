#be sure to specify filename in this file
# from IPython.core.debugger import set_trace
import rosbag
import pickle
from collections import namedtuple
import numpy as np
from geometry_msgs.msg import Pose

def main():

	filename = 'flight_2020-03-17-09-47-43.bag'
	bag = rosbag.Bag('../../../data/mocap/' + filename)
	data = Parser()
	plt = data.get_boat_landing_platform_ned(bag)
	bag.close()

	MyStruct = namedtuple("mystruct", "plt")#, plt_virt, drone, hl_cmd, is_flying, is_landing, odom")
	vals = MyStruct(plt)#, plt_virt, drone, hl_cmd, is_flying, is_landing, odom)

	return vals

class Parser: 


	def get_multirotor_error(self, bag):
		err_x = []
		err_y = []
		err_z = []

		for topic, msg, t in bag.read_messages(topics=['/multirotor/error']):  
			err_x.append(msg.position.x)
			err_y.append(msg.position.y)
			err_z.append(msg.position.z)

		error = [err_x, err_y, err_z]

		return error

	def get_boat_odom(self, bag):
		boat_sec = []
		boat_nsec = []
		boat_x = []
		boat_y = []
		boat_z = []
		boat_orientation = []

		for topic, msg, t in bag.read_messages(topics=['/boat/odom']):  
			boat_sec.append(msg.header.stamp.secs)
			boat_nsec.append(msg.header.stamp.nsecs)			
			boat_x.append(msg.pose.pose.position.x)
			boat_y.append(msg.pose.pose.position.y)
			boat_z.append(msg.pose.pose.position.z)
			boat_orientation.append(msg.pose.pose.orientation)

		boat = pos_orient_time(boat_sec, boat_nsec, boat_x, boat_y, boat_z, boat_orientation)

		return boat

	def get_multirotor_platform_virtual_odometry(self, bag):
		
		plt_virt_sec = []
		plt_virt_nsec = []
		plt_virt_x = []
		plt_virt_y = []
		plt_virt_z = []
		plt_virt_orientation = []

		#parse bag and convert from NWU position to NED
		for topic, msg, t in bag.read_messages(topics=['/multirotor/platform_virtual_odometry']):  
			plt_virt_sec.append(msg.header.stamp.secs)
			plt_virt_nsec.append(msg.header.stamp.nsecs)			
			plt_virt_x.append(msg.pose.pose.position.x)
			plt_virt_y.append(-msg.pose.pose.position.y)
			plt_virt_z.append(-msg.pose.pose.position.z)
			plt_virt_orientation.append(msg.pose.pose.orientation)
		
		plt_virt = pos_orient_time(plt_virt_sec, plt_virt_nsec, plt_virt_x, plt_virt_y, plt_virt_z, plt_virt_orientation)

		return plt_virt

	def get_multirotor_odom(self, bag):
		
		drone_sec = []
		drone_nsec = []
		drone_x = []
		drone_y = []
		drone_z = []
		drone_orientation = []

		for topic, msg, t in bag.read_messages(topics=['/multirotor/odom']):
			drone_sec.append(msg.header.stamp.secs)
			drone_nsec.append(msg.header.stamp.nsecs)			
			drone_x.append(msg.pose.pose.position.x)
			drone_y.append(msg.pose.pose.position.y)
			drone_z.append(msg.pose.pose.position.z)
			drone_orientation.append(msg.pose.pose.orientation)
		
		drone = pos_orient_time(drone_sec, drone_nsec, drone_x, drone_y, drone_z, drone_orientation)
	
		return drone

	
	def get_multirotor_ned(self, bag):
		
		drone_sec = []
		drone_nsec = []
		drone_x = []
		drone_y = []
		drone_z = []
		drone_orientation = []

		for topic, msg, t in bag.read_messages(topics=['/multirotor/ground_truth/odometry/NED']):
			drone_sec.append(msg.header.stamp.secs)
			drone_nsec.append(msg.header.stamp.nsecs)			
			drone_x.append(msg.pose.pose.position.x)
			drone_y.append(msg.pose.pose.position.y)
			drone_z.append(msg.pose.pose.position.z)
		
		ned = nedTime(drone_sec, drone_nsec, drone_x, drone_y, drone_z)
	
		return ned

	def get_multirotor_imu(self, bag):

		drone_sec = []
		drone_nsec = []
		drone_x = []
		drone_y = []
		drone_z = []
		drone_orientation = []

		for topic, msg, t in bag.read_messages(topics=['/multirotor/imu/data']):
			drone_sec.append(msg.header.stamp.secs)
			drone_nsec.append(msg.header.stamp.nsecs)			
			drone_x.append(msg.linear_acceleration.x)
			drone_y.append(msg.linear_acceleration.y)
			drone_z.append(msg.linear_acceleration.z)
		
		imu_data = nedTime(drone_sec, drone_nsec, drone_x, drone_y, drone_z)
	
		return imu_data


	def get_multirotor_high_level_command(self, bag):
		
		hl_cmd_sec = []
		hl_cmd_nsec = []
		hl_cmd_x = []
		hl_cmd_y = []
		hl_cmd_z = []
		hl_cmd_orientation = []

		#Parse bag and convert from NEU position to NED position
		for topic, msg, t in bag.read_messages(topics=['/multirotor/high_level_command']):  
			hl_cmd_sec.append(msg.header.stamp.secs)
			hl_cmd_nsec.append(msg.header.stamp.nsecs)			
			hl_cmd_x.append(msg.x)
			hl_cmd_y.append(msg.y)
			hl_cmd_z.append(-msg.F)
			hl_cmd_orientation.append(msg.z)
		
		hl_cmd = pos_orient_time(hl_cmd_sec, hl_cmd_nsec, hl_cmd_x, hl_cmd_y, hl_cmd_z, hl_cmd_orientation)

		return hl_cmd

	def get_boat_landing_platform_ned(self, bag):
		plt_sec = []
		plt_nsec = []
		plt_x = []
		plt_y = []
		plt_z = []
		plt_orientation = []

		for topic, msg, t in bag.read_messages(topics=['/boat_landing_platform_ned']):  
			plt_sec.append(msg.header.stamp.secs)
			plt_nsec.append(msg.header.stamp.nsecs)			
			plt_x.append(msg.pose.position.x)
			plt_y.append(msg.pose.position.y)
			plt_z.append(msg.pose.position.z)
			plt_orientation.append(msg.pose.orientation)
		
		plt = pos_orient_time(plt_sec, plt_nsec, plt_x, plt_y, plt_z, plt_orientation)

		return plt

	def get_platform_virtual_odometry(self, bag):

		plt_virt_sec = []
		plt_virt_nsec = []
		plt_virt_x = []
		plt_virt_y = []
		plt_virt_z = []
		plt_virt_orientation = []

		for topic, msg, t in bag.read_messages(topics=['/platform_virtual_odometry']):  
			plt_virt_sec.append(msg.header.stamp.secs)
			plt_virt_nsec.append(msg.header.stamp.nsecs)			
			plt_virt_x.append(msg.pose.pose.position.x)
			plt_virt_y.append(msg.pose.pose.position.y)
			plt_virt_z.append(msg.pose.pose.position.z)
			plt_virt_orientation.append(msg.pose.pose.orientation)
		
		plt_virt = pos_orient_time(plt_virt_sec, plt_virt_nsec, plt_virt_x, plt_virt_y, plt_virt_z, plt_virt_orientation)

		return plt_virt

	def get_ragnarok_ned(self, bag):
		drone_sec = []
		drone_nsec = []
		drone_x = []
		drone_y = []
		drone_z = []
		drone_orientation = []

		for topic, msg, t in bag.read_messages(topics=['/ragnarok_ned']):  
			drone_sec.append(msg.header.stamp.secs)
			drone_nsec.append(msg.header.stamp.nsecs)			
			drone_x.append(msg.pose.position.x)
			drone_y.append(msg.pose.position.y)
			drone_z.append(msg.pose.position.z)
			drone_orientation.append(msg.pose.orientation)
		
		drone = nedTime(drone_sec, drone_nsec, drone_x, drone_y, drone_z)
	
		return drone

	def get_high_level_command(self, bag):
		hl_cmd_sec = []
		hl_cmd_nsec = []
		hl_cmd_x = []
		hl_cmd_y = []
		hl_cmd_z = []
		hl_cmd_orientation = []

		for topic, msg, t in bag.read_messages(topics=['/high_level_command']):  
			hl_cmd_sec.append(msg.header.stamp.secs)
			hl_cmd_nsec.append(msg.header.stamp.nsecs)		
			hl_cmd_x.append(msg.x)
			hl_cmd_y.append(msg.y)
			hl_cmd_z.append(msg.F)
			hl_cmd_orientation.append(msg.z)
		
		hl_cmd = pos_orient_time(hl_cmd_sec, hl_cmd_nsec, hl_cmd_x, hl_cmd_y, hl_cmd_z, hl_cmd_orientation)

		return hl_cmd

	def get_master_branch_high_level_command(self, bag):
		hl_cmd_sec = []
		hl_cmd_nsec = []
		hl_cmd_x = []
		hl_cmd_y = []
		hl_cmd_z = []
		hl_cmd_orientation = []

		for topic, msg, t in bag.read_messages(topics=['/high_level_command']):  
			hl_cmd_sec.append(msg.stamp.secs)
			hl_cmd_nsec.append(msg.stamp.nsecs)		
			hl_cmd_x.append(msg.cmd1)
			hl_cmd_y.append(msg.cmd2)
			hl_cmd_z.append(-msg.cmd3) #need to negate/ convert ned to neu
			hl_cmd_orientation.append(msg.cmd4)
		
		hl_cmd = pos_orient_time(hl_cmd_sec, hl_cmd_nsec, hl_cmd_x, hl_cmd_y, hl_cmd_z, hl_cmd_orientation)

		return hl_cmd

	def get_is_flying(self, bag):
		is_flying = []

		for topic, msg, t in bag.read_messages(topics=['/is_flying']):  
			is_flying.append(msg.data)

		return is_flying

	def get_is_landing(self, bag):
		is_landing = []

		for topic, msg, t in bag.read_messages(topics=['/is_landing']):  
			is_landing.append(msg.data)
		
		return is_landing

	def get_odom(self, bag):
		odom_sec = []
		odom_nsec = []
		odom_x = []
		odom_y = []
		odom_z = []
		odom_orientation = []

		for topic, msg, t in bag.read_messages(topics=['/odom']):  
			odom_sec.append(msg.header.stamp.secs)
			odom_nsec.append(msg.header.stamp.nsecs)			
			odom_x.append(msg.pose.pose.position.x)
			odom_y.append(msg.pose.pose.position.y)
			odom_z.append(msg.pose.pose.position.z)
			odom_orientation.append(msg.pose.pose.orientation)
		
		odom = pos_orient_time(odom_sec, odom_nsec, odom_x, odom_y, odom_z, odom_orientation)

		return odom

	def get_multirotor_gps(self, bag):
		gps_n = []
		gps_e = []
		gps_d = []

		for topic, msg, t in bag.read_messages(topics=['/gps_ned_cov']):  			
			gps_n.append(msg.pose.position.x)
			gps_e.append(msg.pose.position.y)
			gps_d.append(msg.pose.position.z)

		gps = ned(gps_n, gps_e, gps_d)

		return gps

	def get_PosVelECEF(self, bag):
		sec = []
		nsec = []
		lon = []
		lat = []
		alt = []
		pn = []
		pe = []
		pd = []

		for topic, msg, t in bag.read_messages(topics=['/rover/PosVelEcef']):
			sec.append(msg.header.stamp.secs)
			nsec.append(msg.header.stamp.nsecs)
			lon.append(msg.lla[0])
			lat.append(msg.lla[1])
			alt.append(msg.lla[2])
			pn.append(msg.position[0])
			pe.append(msg.position[1])
			pd.append(msg.position[2])

		return gps_time(sec,nsec,lon,lat,alt,pn,pe,pd)

	def get_boat_PosVelECEF(self, bag):
		sec = []
		nsec = []
		lon = []
		lat = []
		alt = []
		pn = []
		pe = []
		pd = []

		for topic, msg, t in bag.read_messages(topics=['/boat/PosVelEcef']):
			sec.append(msg.header.stamp.secs)
			nsec.append(msg.header.stamp.nsecs)
			lon.append(msg.lla[0])
			lat.append(msg.lla[1])
			alt.append(msg.lla[2])
			pn.append(msg.position[0])
			pe.append(msg.position[1])
			pd.append(msg.position[2])

		return gps_time(sec,nsec,lon,lat,alt,pn,pe,pd)

	def get_imu(self, bag):

		sec = []
		nsec = []
		accel_x = []
		accel_y = []
		accel_z = []

		for topic, msg, t in bag.read_messages(topics=['/imu/data']):  
			sec.append(msg.header.stamp.secs)
			nsec.append(msg.header.stamp.nsecs)			
			accel_x.append(msg.linear_acceleration.x)
			accel_y.append(msg.linear_acceleration.y)
			accel_z.append(msg.linear_acceleration.z)

		imu_data = nedTime(sec, nsec, accel_x, accel_y, accel_z)

		return imu_data

	def get_RelPos(self, bag):
		sec = []
		nsec = []
		RP_N = []
		RP_E = []
		RP_D = []
		N_hp = []
		E_hp = []
		D_hp = []

		flag = []

		for topic, msg, t in bag.read_messages(topics=['/rover/RelPos']):
			sec.append(msg.header.stamp.secs)
			nsec.append(msg.header.stamp.nsecs)
			RP_N.append(msg.relPosNED[0])
			RP_E.append(msg.relPosNED[1])
			RP_D.append(msg.relPosNED[2])
			N_hp.append(msg.relPosHPNED[0])
			E_hp.append(msg.relPosHPNED[1])
			D_hp.append(msg.relPosHPNED[2])

			flag.append(msg.flags)

		return sec, nsec, RP_N, RP_E, RP_D, N_hp, E_hp, D_hp, flag

class pos_orient_time:
	def __init__(self, sec, nsec, x, y, z, orientation):
		
		self.time = np.array(sec)+np.array(nsec)*1E-9
		
		self.position = [x, y, z]
		self.orientation = orientation

class nedTime:
	def __init__(self, sec, nsec, north, east, down):
		
		self.time = np.array(sec)+np.array(nsec)*1E-9
		
		self.n = north
		self.e = east
		self.d = down

class llaTime:
	def __init__(self, sec, nsec, latitude, longitude, altitude):

		self.time = np.array(sec)+np.array(nsec)*1E-9

		self.lon = longitude
		self.lat = latitude
		self.alt = altitude


class ecefTime:
	def __init__(self, sec, nsec, x, y, z):

		self.time = np.array(sec)+np.array(nsec)*1E-9

		self.x = x
		self.y = y
		self.z = z

class gps_time:
	def __init__(self, sec, nsec, lon, lat, alt, x, y, z):
		self.time = np.array(sec)+np.array(nsec)*1E-9
		self.lla = [lon,lat,alt]
		self.position = [x,y,z]

if __name__ == '__main__':
    vals = main()
