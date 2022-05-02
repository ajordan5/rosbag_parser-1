import rosbag
import pickle
from collections import namedtuple
import numpy as np
from scipy.spatial.transform import Rotation as R

class Parser:
	def __init__(self, bagfile,baseGpsTopic,roverRelPosTopic,roverGpsTopic,baseOdomTopic,
				baseMocapTopic,roverMocapTopic,tagDetectTopic,tagOdomTopic, tagHatTopic, roverAirsimTopic=None):

		self.tagDetectTopic = tagDetectTopic
		self.tagOdomTopic = tagOdomTopic
		self.tagHatTopic = tagHatTopic
		self.roverMocapTopic = roverMocapTopic
		self.roverAirsimTopic = roverAirsimTopic
		self.baseGpsTopic = baseGpsTopic
		self.baseMocapTopic = baseMocapTopic
		self.roverRelPosTopic = roverRelPosTopic
		self.roverGpsTopic = roverGpsTopic
		self.baseOdomTopic = baseOdomTopic

		self.bag = rosbag.Bag(bagfile)


	def parse_and_save(self):
		self.get_base_mocap()	
		self.get_rover_mocap()	
		self.get_airsim()
		self.get_odom()
		self.get_tag()
		self.get_tag_odom()
		self.get_tag()
		self.get_tag_hat()


	def get_base_gps(self, bag):

		data = {}
		data["px"] = []
		data["py"] = []
		data["pz"] = []
		data["vx"] = []
		data["vy"] = []
		data["vz"] = []
		data["lat"] = []
		data["lon"] = []
		data["alt"] = []
		data["sec"] = []
		data["nsec"] = []

		for topic, msg, t in bag.read_messages(topics=[self.baseGpsTopic]):
				data["sec"].append(msg.header.stamp.secs)
				data["nsec"].append(msg.header.stamp.nsecs)
				data["px"].append(msg.position[0])
				data["py"].append(msg.position[1])
				data["pz"].append(msg.position[2])
				data["vx"].append(msg.velocity[0])
				data["vy"].append(msg.velocity[1])
				data["vz"].append(msg.velocity[2])
				data["lat"].append(msg.lla[0])
				data["lon"].append(msg.lla[1])
				data["alt"].append(msg.lla[2])

		npz_file = self.bag.filename.split("/")[-1].split(".")[0] + "_"+ self.roverGpsTopic.split("/")[1] +".npz"
		np.savez(npz_file, **data)

	def get_rover_relPos(self, bag):

		sec = []
		nsec = []
		pn = []
		pe = []
		pd = []
		pnHp = []
		peHp = []
		pdHp = []

		for topic, msg, t in bag.read_messages(topics=[self.roverRelPosTopic]):
			sec.append(t.secs)
			nsec.append(t.nsecs)
			pn.append(msg.relPosNED[0])
			pe.append(msg.relPosNED[1])
			pd.append(msg.relPosNED[2])
			pnHp.append(msg.relPosHPNED[0])
			peHp.append(msg.relPosHPNED[1])
			pdHp.append(msg.relPosHPNED[2])

		return RelPos(sec,nsec,pn,pe,pd,pnHp,peHp,pdHp)

	def get_rover_mocap(self):
		if self.roverMocapTopic is not None:
			data = {}										
			data["pn"] = []
			data["pe"] = []
			data["pd"] = []
			data["qx"] = []
			data["qy"] = []
			data["qz"] = []
			data["qw"] = []
			data["sec"] = []
			data["nsec"] = []
			for topic, msg, t in self.bag.read_messages(topics=[self.roverMocapTopic]):
				data["sec"].append(t.secs)
				data["nsec"].append(t.nsecs)
				data["pn"].append(msg.pose.position.x)
				data["pe"].append(msg.pose.position.y)
				data["pd"].append(msg.pose.position.z)
				data["qx"].append(msg.pose.orientation.x)
				data["qy"].append(msg.pose.orientation.y)
				data["qz"].append(msg.pose.orientation.z)
				data["qw"].append(msg.pose.orientation.w)
			npz_file = self.bag.filename.split("/")[-1].split(".")[0] + "_"+ self.roverMocapTopic.split("/")[1] +".npz"
			np.savez(npz_file, **data)

	def get_base_mocap(self):
		if self.baseMocapTopic is not None:
			data = {}
			data["pn"] = []
			data["pe"] = []
			data["pd"] = []
			data["qx"] = []
			data["qy"] = []
			data["qz"] = []
			data["qw"] = []
			data["sec"] = []
			data["nsec"] = []
			for topic, msg, t in self.bag.read_messages(topics=[self.baseMocapTopic]):
				data["sec"].append(t.secs)
				data["nsec"].append(t.nsecs)
				data["pn"].append(msg.pose.position.x)
				data["pe"].append(msg.pose.position.y)
				data["pd"].append(msg.pose.position.z)
				data["qx"].append(msg.pose.orientation.x)
				data["qy"].append(msg.pose.orientation.y)
				data["qz"].append(msg.pose.orientation.z)
				data["qw"].append(msg.pose.orientation.w)
			npz_file = self.bag.filename.split("/")[-1].split(".")[0] + "_"+ self.baseMocapTopic.split("/")[1] +".npz"
			np.savez(npz_file, **data)

		


	def get_rover_gps(self):

		data = {}
		data["px"] = []
		data["py"] = []
		data["pz"] = []
		data["vx"] = []
		data["vy"] = []
		data["vz"] = []
		data["lat"] = []
		data["lon"] = []
		data["alt"] = []
		data["sec"] = []
		data["nsec"] = []

		for topic, msg, t in self.bag.read_messages(topics=[self.roverGpsTopic]):
				data["sec"].append(msg.header.stamp.secs)
				data["nsec"].append(msg.header.stamp.nsecs)
				data["px"].append(msg.position[0])
				data["py"].append(msg.position[1])
				data["pz"].append(msg.position[2])
				data["vx"].append(msg.velocity[0])
				data["vy"].append(msg.velocity[1])
				data["vz"].append(msg.velocity[2])
				data["lat"].append(msg.lla[0])
				data["lon"].append(msg.lla[1])
				data["alt"].append(msg.lla[2])

		npz_file = self.bag.filename.split("/")[-1].split(".")[0] + "_"+ self.roverGpsTopic.split("/")[1] +".npz"
		np.savez(npz_file, **data)


	def get_odom(self):

		data = {}
		data["pn"] = []
		data["pe"] = []
		data["pd"] = []
		data["qx"] = []
		data["qy"] = []
		data["qz"] = []
		data["qw"] = []
		data["sec"] = []
		data["nsec"] = []
		data["vx"] = []
		data["vy"] = []
		data["vz"] = []

		for topic, msg, t in self.bag.read_messages(topics=[self.baseOdomTopic]):
				data["sec"].append(t.secs)
				data["nsec"].append(t.nsecs)
				data["pn"].append(msg.pose.pose.position.x)
				data["pe"].append(msg.pose.pose.position.y)
				data["pd"].append(msg.pose.pose.position.z)
				data["qx"].append(msg.pose.pose.orientation.x)
				data["qy"].append(msg.pose.pose.orientation.y)
				data["qz"].append(msg.pose.pose.orientation.z)
				data["qw"].append(msg.pose.pose.orientation.w)
				data["vx"].append(msg.twist.twist.linear.x)
				data["vy"].append(msg.twist.twist.linear.y)
				data["vz"].append(msg.twist.twist.linear.z)

		npz_file = self.bag.filename.split("/")[-1].split(".")[0] + "_"+ self.baseOdomTopic.split("/")[1] +".npz"
		np.savez(npz_file, **data)

	def get_tag_odom(self):

		data = {}
		data["pn"] = []
		data["pe"] = []
		data["pd"] = []
		data["sec"] = []
		data["nsec"] = []
		
		for topic, msg, t in self.bag.read_messages(topics=[self.tagOdomTopic]):
				data["sec"].append(t.secs)
				data["nsec"].append(t.nsecs)
				data["pn"].append(msg.pose.pose.position.x)
				data["pe"].append(msg.pose.pose.position.y)
				data["pd"].append(msg.pose.pose.position.z)

		npz_file = self.bag.filename.split("/")[-1].split(".")[0] + "_"+ self.tagOdomTopic.split("/")[1] +".npz"
		np.savez(npz_file, **data)


	
	def get_tag_hat(self):

		data = {}
		data["x"] = []
		data["y"] = []
		data["z"] = []
		data["seq"] = []

		for topic, msg, t in self.bag.read_messages(topics=[self.tagHatTopic]):
				data["x"].append(msg.vector.x)
				data["y"].append(msg.vector.y)
				data["z"].append(msg.vector.z)
				data["seq"].append(msg.header.seq)

		npz_file = self.bag.filename.split("/")[-1].split(".")[0] + "_"+ self.tagHatTopic.split("/")[1] +".npz"
		np.savez(npz_file, **data)


	def get_tag(self):

		data = {}
		data["x"] = []
		data["y"] = []
		data["z"] = []
		data["seq"] = []
		seq = 1

		for topic, msg, t in self.bag.read_messages(topics=[self.tagDetectTopic]):
				if msg.detections:
					data["x"].append(msg.detections[0].pose.pose.pose.position.x)
					data["y"].append(msg.detections[0].pose.pose.pose.position.y)
					data["z"].append(msg.detections[0].pose.pose.pose.position.z)
					data["seq"].append(seq)
					seq+=1

		npz_file = self.bag.filename.split("/")[-1].split(".")[0] + "_"+ self.tagDetectTopic.split("/")[1] +".npz"
		np.savez(npz_file, **data)

		
		
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

	def get_airsim(self):
		if self.roverAirsimTopic is not None:
			data = {}	
			data["pn"] = []
			data["pe"] = []
			data["pd"] = []
			data["qx"] = []
			data["qy"] = []
			data["qz"] = []
			data["qw"] = []
			data["sec"] = []
			data["nsec"] = []
			data["vx"] = []
			data["vy"] = []
			data["vz"] = []

			for topic, msg, t in self.bag.read_messages(topics=[self.baseOdomTopic]):
					data["sec"].append(t.secs)
					data["nsec"].append(t.nsecs)
					data["pn"].append(msg.pose.pose.position.x)
					data["pe"].append(msg.pose.pose.position.y)
					data["pd"].append(msg.pose.pose.position.z)
					data["qx"].append(msg.pose.pose.orientation.x)
					data["qy"].append(msg.pose.pose.orientation.y)
					data["qz"].append(msg.pose.pose.orientation.z)
					data["qw"].append(msg.pose.pose.orientation.w)
					data["vx"].append(msg.twist.twist.linear.x)
					data["vy"].append(msg.twist.twist.linear.y)
					data["vz"].append(msg.twist.twist.linear.z)

			npz_file = self.bag.filename.split("/")[-1].split(".")[0] + "_"+ self.roverAirsimTopic.split("/")[1] +".npz"
			np.savez(npz_file, **data)


class FlightMode:
	def __init__(self,sec,nsec,flightMode):
		self.time = np.array(sec)+np.array(nsec)*1E-9
		self.flightMode = flightMode

class MissionState:
	def __init__(self,sec,nsec,missionState):
		self.time = np.array(sec)+np.array(nsec)*1E-9
		self.missionState = missionState

class Gps:
	def __init__(self,sec,nsec,px,py,pz,vx,vy,vz):
		self.time = np.array(sec)+np.array(nsec)*1E-9
		# self.time -= self.time[0]
		self.position = np.array([px,py,pz])
		self.velocity = np.array([vx,vy,vz])

class Imu:
	def __init__(self,sec,nsec,ax,ay,az,wx,wy,wz):
		self.time = np.array(sec)+np.array(nsec)*1E-9
		# self.time -= self.time[0]
		self.accel = np.array([ax,ay,az])
		self.omega = np.array([wx,wy,wz])

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

class refLla:
	def __init__(self,lat,lon,alt):
		self.lat = lat[0]
		self.lon = lon[0]
		self.alt = alt[0]

class pos_orient_time:
	def __init__(self, sec, nsec, x, y, z, orientation):
		
		self.time = np.array(sec)+np.array(nsec)*1E-9
		
		self.position = [x, y, z]
		self.orientation = orientation


