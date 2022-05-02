import rosbag
import pickle
from collections import namedtuple
import numpy as np
from scipy.spatial.transform import Rotation as R

class Parser:
	def __init__(self, mocapTopic, baseOdomTopic, roverOdomTopic, roverNEDTopic, boatNEDTopic, mocapEulerTopic, baseEulerTopic):
		self.mocapTopic = mocapTopic
		self.baseOdomTopic = baseOdomTopic
		self.roverOdomTopic = roverOdomTopic
		self.roverNEDTopic = roverNEDTopic
		self.boatNEDTopic = boatNEDTopic
		self.mocapEulerTopic = mocapEulerTopic
		self.baseEulerTopic = baseEulerTopic
	

	def get_mocap(self, bag):
		sec = []
		nsec = []
		qx = []
		qy = []
		qz = []
		qw = []

		for topic, msg, t in bag.read_messages(topics=[self.mocapTopic]):
				sec.append(t.secs)
				nsec.append(t.nsecs)
				qx.append(msg.pose.orientation.x)
				qy.append(msg.pose.orientation.y)
				qz.append(msg.pose.orientation.z)
				qw.append(msg.pose.orientation.w)
		return Quat(sec,nsec,qx,qy,qz,qw)

	def get_mocap_euler(self, bag):
		sec = []
		nsec = []
		phi = []
		theta = []
		psi = []

		for topic, msg, t in bag.read_messages(topics=[self.mocapEulerTopic]):
				sec.append(t.secs)
				nsec.append(t.nsecs)
				phi.append(msg.vector.x)
				theta.append(msg.vector.y)
				psi.append(msg.vector.z)

		return Euler(sec,nsec,phi,theta,psi)

	def get_base_euler(self, bag):
		sec = []
		nsec = []
		phi = []
		theta = []
		psi = []

		for topic, msg, t in bag.read_messages(topics=[self.baseEulerTopic]):
				sec.append(t.secs)
				nsec.append(t.nsecs)
				phi.append(msg.vector.x)
				theta.append(msg.vector.y)
				psi.append(msg.vector.z)

		return Euler(sec,nsec,phi,theta,psi)

	def get_odom(self, bag):
                sec = []
                nsec = []
                pn = []
                pe = []
                pd = []
                qx = []
                qy = []
                qz = []
                qw = []
                vx = []
                vy = []
                vz = []

                for topic, msg, t in bag.read_messages(topics=[self.roverOdomTopic]):
                        sec.append(msg.header.stamp.secs)
                        nsec.append(msg.header.stamp.nsecs)
                        pn.append(msg.pose.pose.position.x)
                        pe.append(msg.pose.pose.position.y)
                        pd.append(msg.pose.pose.position.z)
                        qx.append(msg.pose.pose.orientation.x)
                        qy.append(msg.pose.pose.orientation.y)
                        qz.append(msg.pose.pose.orientation.z)
                        qw.append(msg.pose.pose.orientation.w)
                        vx.append(msg.twist.twist.linear.x)
                        vy.append(msg.twist.twist.linear.y)
                        vz.append(msg.twist.twist.linear.z)

                return RelPos(sec,nsec,pn,pe,pd), Odom(sec,nsec,0.0,0.0,0.0,qx,qy,qz,qw,vx,vy,vz)

	def get_boat_odom(self, bag):
			boat_sec = []
			boat_nsec = []
			boat_x = []
			boat_y = []
			boat_z = []
			boat_orientation = []

			for topic, msg, t in bag.read_messages(topics=[self.baseOdomTopic]):  
				boat_sec.append(msg.header.stamp.secs)
				boat_nsec.append(msg.header.stamp.nsecs)			
				boat_x.append(msg.pose.pose.position.x)
				boat_y.append(msg.pose.pose.position.y)
				boat_z.append(msg.pose.pose.position.z)
				boat_orientation.append(msg.pose.pose.orientation)

			boat = pos_orient_time(boat_sec, boat_nsec, boat_x, boat_y, boat_z, boat_orientation)

			return boat

	def get_rhodey_ned(self, bag):
		drone_sec = []
		drone_nsec = []
		drone_x = []
		drone_y = []
		drone_z = []
		drone_orientation = []

		for topic, msg, t in bag.read_messages(topics=[self.roverNEDTopic]):  
			drone_sec.append(msg.header.stamp.secs)
			drone_nsec.append(msg.header.stamp.nsecs)			
			drone_x.append(msg.pose.position.x)
			drone_y.append(msg.pose.position.y)
			drone_z.append(msg.pose.position.z)
			drone_orientation.append(msg.pose.orientation)
		
		drone = nedTime(drone_sec, drone_nsec, drone_x, drone_y, drone_z)
	
		return drone

	def get_boat_landing_platform_ned(self, bag):
		plt_sec = []
		plt_nsec = []
		plt_x = []
		plt_y = []
		plt_z = []
		plt_orientation = []

		for topic, msg, t in bag.read_messages(topics=[self.boatNEDTopic]):  
			plt_sec.append(msg.header.stamp.secs)
			plt_nsec.append(msg.header.stamp.nsecs)			
			plt_x.append(msg.pose.position.x)
			plt_y.append(msg.pose.position.y)
			plt_z.append(msg.pose.position.z)
			plt_orientation.append(msg.pose.orientation)
		
		plt = pos_orient_time(plt_sec, plt_nsec, plt_x, plt_y, plt_z, plt_orientation)

		return plt


class Quat:
	def __init__(self,sec,nsec,qx,qy,qz,qw):
		self.time = np.array(sec)+np.array(nsec)*1E-9
		self.quat = [qx,qy,qz,qw]

class Euler:
	def __init__(self,sec,nsec,phi,theta,psi):
		self.time = np.array(sec)+np.array(nsec)*1E-9
		self.euler = [phi,theta,psi]

class RelPos:
	def __init__(self,sec,nsec,pn,pe,pd,pnHp=0.0,peHp=0.0,pdHp=0.0):
		self.time = np.array(sec)+np.array(nsec)*1E-9
		# self.time -= self.time[0]
		pn = np.array(pn) + np.array(pnHp)
		pe = np.array(pe) + np.array(peHp)
		pd = np.array(pd) + np.array(pdHp)
		self.position = np.array([pn,pe,pd])

class Odom:
	def __init__(self,sec,nsec,pn,pe,pd,qx,qy,qz,qw,vx,vy,vz):
		self.time = np.array(sec)+np.array(nsec)*1E-9
		# self.time -= self.time[0]
		self.position = np.array([pn,pe,pd])
		quat = np.array([qx,qy,qz,qw]).T
		eulerRad = R.from_quat(quat).as_euler('xyz').T
		self.euler = eulerRad*180.0/np.pi
		self.velocity = np.array([vx,vy,vz])

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
