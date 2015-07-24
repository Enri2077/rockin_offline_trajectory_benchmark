#!/usr/bin/python

#TODO?	configuration file, output file
#TODO?	slice and merge the rockin_mocap bags to a temporary one in order to work on just one smaller bag
#TODO	adapt for waypoints
#TODO	robot's pose topic name in mocap bags to be generalised

# tests:
# D	very long robot bag (might there be overflows in the sums for the error?)
#	when more than one mocap bag are used
# ?		all the right mocap bags are selected
# D		bags are concatenated in the right order
# D		when mocap bags are missing
# D	the mocap poses mp1, mp2 and the robot pose rp with which the error is calculated, are always such that mp1.t <= rp.t <= mp2.t
# D	the tracking was lost when a robot's pose was being logged

import sys, os
import subprocess, yaml, itertools, math
import rosbag
import tf
from std_msgs.msg import Int32, String

class TrackingLost(Exception):
	pass

class EndOfBag(Exception):
	pass

def get_pose_time(pose):
#	print pose
	return pose.header.stamp

def pose_equal_position(p1, p2):
	return	p1.pose.position.x == p2.pose.position.x and \
			p1.pose.position.y == p2.pose.position.y and \
			p1.pose.position.z == p2.pose.position.z 

###	Get summary information about a bag;
###	arguments:
###		bag_pos:	the position of the bag file
def get_bag_info(bag_pos):
	try:
		if bag_pos.endswith('.bag'):
			return yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bag_pos], stdout=subprocess.PIPE).communicate()[0])
	except:
		return None

def get_bag_info_from_object(bag_object):
	return yaml.load(bag_object._get_yaml_info())

###	TODO
def interpolate(robot_pose, mocap_pose_1, mocap_pose_2):
	return mocap_pose_1

def seek_mocap_pose_at(t, dual_iterator):
	try:
		mocap_pose_1, mocap_pose_2 = dual_iterator.current()
		
		t1 = get_pose_time(mocap_pose_1).to_sec()
		t2 = get_pose_time(mocap_pose_2).to_sec()
		
		while(get_pose_time(mocap_pose_2) < t):
			mocap_pose_1, mocap_pose_2 = dual_iterator.next()
			
			#print "\n\n\n\nmp1:\n", mocap_pose_1, "\n\nt: "+str(t)+"\n\n", "\n\nmp2:\n", mocap_pose_2
			
			t1 = get_pose_time(mocap_pose_1).to_sec()
			t2 = get_pose_time(mocap_pose_2).to_sec()
			
			if t2 - t1 > 0.1:
				print "[WARNING] consecutive mocap poses found to be very distant in time: t1,t2 = "+str(t1)+"[s],"+str(t2)+"[s] difference: "+str((t2-t1)*1000)+" milliseconds"
			if t1 > t2:
				print "[WARNING] poses out of order: result may be invalid. t1,t2 = "+str(t1)+"[s],"+str(t2)+"[s]"

	except StopIteration:
		raise EndOfBag
	
	if not (t1 <= t.to_sec() <= t2):
		print "[ERROR] mocap poses mp1, mp2 and the robot pose rp with which the error is calculated, weren't such that mp1.t <= rp.t <= mp2.t\n\tThis should never happen. mp1.t,rp.t,mp2.t = "+str(t1)+"[s],"+str(t.to_sec())+"[s],"+str(t2)+"[s]"
			
	if pose_equal_position(mocap_pose_1, mocap_pose_2):	# if two poses are exactly the same, almost certainly the tracking is lost
		raise TrackingLost
	else:
		return mocap_pose_1, mocap_pose_2

class DualIterator:
	def __init__(self, iterable_list):
		self.iterator	= itertools.chain.from_iterable(iterable_list)
		_, self.i1, _ = self.iterator.next()
		_, self.i2, _ = self.iterator.next()
	
	def __iter__(self):
		return self
	
	def next(self):
		self.i1 = self.i2
		_, self.i2, _ = self.iterator.next()
		return self.i1, self.i2
	
	def current(self):
		return self.i1, self.i2

class Error:
	def __init__(self):
		self.n = 0
		self.sum_d = 0
		self.sum_sin = 0
		self.sum_cos = 0
	
	def update(self, robot_pose, mocap_pose):
		rp = robot_pose.pose.position	# robot position
		mp = mocap_pose.pose.position	# mocap position
		
		rq = (	robot_pose.pose.orientation.x,
				robot_pose.pose.orientation.y,
				robot_pose.pose.orientation.z,
				robot_pose.pose.orientation.w)		
		mq = (	mocap_pose.pose.orientation.x,
				mocap_pose.pose.orientation.y,
				mocap_pose.pose.orientation.z,
				mocap_pose.pose.orientation.w)
		
		ro = tf.transformations.euler_from_quaternion(rq)
		mo = tf.transformations.euler_from_quaternion(mq)
		r_theta = ro[2]	# robot yaw
		m_theta = mo[2]	# mocap yaw
		
		self.n = self.n + 1
		
		self.sum_d = self.sum_d + math.sqrt((rp.x-mp.x)**2 + (rp.y-mp.y)**2 + (rp.z-mp.z)**2)
		
		self.sum_sin = self.sum_sin + math.sin(math.fabs(r_theta - m_theta))
		self.sum_cos = self.sum_cos + math.cos(math.fabs(r_theta - m_theta))
	
	def get_position_error(self):
		return self.sum_d / self.n
		
	def get_orientation_error(self):
		return math.atan2(self.sum_sin, self.sum_cos)

if len(sys.argv) < 4:
	print "Two arguments must be provided: rockin_trajectory_benchmark.py rockin_logger_bag_file.bag rockin_mocap_bags_directory teamname\n\trockin_logger_bag_file.bag:\tthe bag produced by rockin_logger\n\trockin_mocap_bags_directory:\tthe directory in which are found all the bags produced by rockin_mocap while rockin_logger was being executed\n\tteamname:\t\t\tthe name that was set in the configuration of rockin_logger, should be the same as the one in the bag's name"
	sys.exit(1)

robot_bag_pos	= sys.argv[1]
mocap_bags_dir	= sys.argv[2]
teamname		= sys.argv[3]

## get summary information about robot_bag
robot_bag_info = get_bag_info(robot_bag_pos)

## check that the robot's bag is available
if not robot_bag_info:
	print robot_bag_pos+" is not a bagfile or can't be opened"
	sys.exit(2)

## check that the robot's topic is in the bag
marker_pose_found = False
for t in robot_bag_info["topics"]:
	if t["topic"] == "/rockin/"+teamname+"/marker_pose":
		marker_pose_found = True
if not marker_pose_found:
	print "/rockin/"+teamname+"/marker_pose topic not found in the robot's bag;\nrockin_logger_bag_file must contain this topic"
	sys.exit(3)

robot_bag_start	= robot_bag_info["start"]
robot_bag_end	= robot_bag_info["end"]

## find all the bags produced by rockin_mocap while rockin_logger was being executed
mocap_bags_pos_list	= []
for f in os.listdir(mocap_bags_dir):
	mocap_bag_pos	= os.path.join(mocap_bags_dir, f)
	mocap_bag_info	= get_bag_info(mocap_bag_pos)
	if mocap_bag_info and mocap_bag_info["start"] <= robot_bag_end and mocap_bag_info["end"] >= robot_bag_start: 
		mocap_bags_pos_list.append(mocap_bag_pos)

## check that the mocap's bags are available
if len(mocap_bags_pos_list) == 0:
	print "NO suitable rockin_mocap bags have been found in "+mocap_bags_dir
	sys.exit(4)
else:
	print "The following rockin_mocap bags will be used:"
	for b in mocap_bags_pos_list:
		print b

#TODO? check for overlap 


## open the bags
print "\nOpening bags..."
robot_bag = rosbag.Bag(robot_bag_pos)
mocap_bags_list = []
mocap_messages_list = []
for mocap_bag_pos in mocap_bags_pos_list:
	b = rosbag.Bag(mocap_bag_pos)
	mocap_bags_list.append(b)
	mocap_messages_list.append(b.read_messages("/airlab/robot/pose"))
print "Bags opened."



## order mocap_bags_list by start time (the bags must be in order, otherwise the seek function doesn't work)
mocap_bags_list = sorted(mocap_bags_list, key=lambda bag: get_bag_info_from_object(bag)["start"])

mocap_bag_iterator = DualIterator(mocap_messages_list)
error = Error()

## iterate through the robot's poses and the respective mocap's poses, and update the error
for _, robot_pose, _ in robot_bag.read_messages("/rockin/"+teamname+"/marker_pose"):
	
	try:
		mocap_pose_1, mocap_pose_2 = seek_mocap_pose_at(get_pose_time(robot_pose), mocap_bag_iterator)
	except TrackingLost:
		print "ignoring pose at time "+str(get_pose_time(robot_pose).to_sec())+"[s] (tracking was lost)"
		continue
	except EndOfBag:
		print "mocap bags are missing: there wasn't available a mocap pose corresponding to the robot's pose:\n", robot_pose
		sys.exit(5)
	
	mocap_pose = interpolate(robot_pose, mocap_pose_1, mocap_pose_2)
	
	# update error
	error.update(robot_pose, mocap_pose)


print error.get_position_error(), error.get_orientation_error()
print error.sum_d, error.sum_sin, error.sum_cos

## close the bags
for mb in mocap_bags_list:
	mb.close()
robot_bag.close()
