#!/usr/bin/python

import sys, os
import subprocess, yaml, itertools, math
import rosbag
import tf
from std_msgs.msg import UInt8
from datetime import datetime

ROCKIN_MOCAP_TOPIC = "/airlab/robot/pose"




class EndOfBag(Exception):
	pass

def get_pose_time(pose):
	return pose.header.stamp

def pose_equal_position(p1, p2):
	return	p1.pose.position.x == p2.pose.position.x and \
			p1.pose.position.y == p2.pose.position.y and \
			p1.pose.position.z == p2.pose.position.z 

###	Get summary information about a bag file, specified through the bag's posision;
###	bag_pos:	the position of the bag file
def get_bag_info(bag_pos):
	try:
		if bag_pos.endswith('.bag'):
			return yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bag_pos], stdout=subprocess.PIPE).communicate()[0])
	except:
		return None

###	Get summary information about a bag object (already opened);
###	bag_object: already opened bag object
def get_bag_info_from_object(bag_object):
	return yaml.load(bag_object._get_yaml_info())

###	not used: interpolation should not be necessary since the mocap poses are much more frequent than the robot poses
def interpolate(robot_pose, mocap_pose_1, mocap_pose_2):
	return mocap_pose_1

### go through the iterator until the couples of poses p1, p2 are such that p1.t <= t <= p2.t, and returns p1, p2
### t instance of rospy.Time
### raise EndOfBag when the poses run out
def seek_mocap_pose_at(t, dual_iterator):
	try:
		mocap_pose_1, mocap_pose_2 = dual_iterator.current()
		
		t1 = get_pose_time(mocap_pose_1).to_sec()
		t2 = get_pose_time(mocap_pose_2).to_sec()
		
		while(get_pose_time(mocap_pose_2) < t):
			mocap_pose_1, mocap_pose_2 = dual_iterator.next()
			
			t1 = get_pose_time(mocap_pose_1).to_sec()
			t2 = get_pose_time(mocap_pose_2).to_sec()
			
			if t2 - t1 > 0.1:
				output( "[WARNING] consecutive mocap poses very distant in time: t1,t2 = "+str(t1)+"[s],"+str(t2)+"[s] difference: "+str((t2-t1)*1000)+" milliseconds" )
			if t1 > t2:
				output( "[WARNING] poses out of order: result may be invalid. t1,t2 = "+str(t1)+"[s],"+str(t2)+"[s]" )

	except StopIteration:
		raise EndOfBag
	
	if not (t1 <= t.to_sec() <= t2):
		output( "[ERROR] mocap poses mp1, mp2 and the robot pose rp with which the error is calculated, weren't such that mp1.t <= rp.t <= mp2.t\n\tThis should never happen. mp1.t,rp.t,mp2.t = "+str(t1)+"[s],"+str(t.to_sec())+"[s],"+str(t2)+"[s]" )
			
	return mocap_pose_1, mocap_pose_2

##### Iterator that iterates throuh couples of (mocap) poses.
##### iterable_list: list of messages iterators (from the mocap bags)
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
		if self.n > 0:
			return self.sum_d / self.n
		else:
			return None
		
	def get_orientation_error(self):
		if self.n > 0:
			return math.atan2(self.sum_sin, self.sum_cos)
		else:
			return None

def output(s):
	with open("rtb_result_for_"+os.path.split(robot_bag_pos)[1]+".txt", 'a') as output_file:
		print s
		output_file.write(str(s)+'\n')





if len(sys.argv) < 4:
	print "The following arguments must be provided: rockin_trajectory_benchmark.py teamname rockin_logger_bag_file.bag rockin_mocap_bags_directory\n\tteamname:\t\t\tthe name that was set in the configuration of rockin_logger, should be the same as the one in the bag's name\n\trockin_logger_bag_file.bag:\tthe bag produced by rockin_logger\n\trockin_mocap_bags_directory:\tthe directory in which are found all the bags produced by rockin_mocap while rockin_logger was being executed"
	sys.exit(1)

teamname		= sys.argv[1]
robot_bag_pos	= sys.argv[2]
mocap_bags_dir	= sys.argv[3]

output(datetime.now().isoformat()+" "+" ".join(sys.argv))

## get summary information about robot_bag
robot_bag_info = get_bag_info(robot_bag_pos)

## check that the robot's bag is available
if not robot_bag_info:
	output( robot_bag_pos+" is not a bagfile or can't be opened" )
	sys.exit(2)

## check that the robot's topic is in the bag
marker_pose_found = False
for t in robot_bag_info["topics"]:
	if t["topic"] == "/rockin/"+teamname+"/marker_pose":
		marker_pose_found = True
if not marker_pose_found:
	output( "/rockin/"+teamname+"/marker_pose topic not found in the robot's bag;\nrockin_logger_bag_file must contain this topic" )
	sys.exit(3)

robot_bag_start	= robot_bag_info["start"]
robot_bag_end	= robot_bag_info["end"]

#### BEGIN auto-selection of mocap bags to be opened; comment to specify every mocap bag individually instead of just providing the directory
## find all the bags produced by rockin_mocap while rockin_logger was being executed
mocap_bags_pos_list	= []
for f in os.listdir(mocap_bags_dir):
	mocap_bag_pos	= os.path.join(mocap_bags_dir, f)
	mocap_bag_info	= get_bag_info(mocap_bag_pos)
	if mocap_bag_info and mocap_bag_info["start"] <= robot_bag_end and mocap_bag_info["end"] >= robot_bag_start: 
		mocap_bags_pos_list.append(mocap_bag_pos)

## check that the mocap's bags are available
if len(mocap_bags_pos_list) == 0:
	output( "NO suitable rockin_mocap bags have been found in "+mocap_bags_dir )
	sys.exit(4)
else:
	output( "The following rockin_mocap bags will be used:" )
	for b in mocap_bags_pos_list:
		output( " "+b )

#### END auto-selection of mocap bags to be opened; comment to specify every mocap bag individually instead of just providing the directory

#### BEGIN without mocap bags auto-selection; uncomment to specify every mocap bag individually instead of just providing the directory
#mocap_bags_pos_list = sys.argv[3:] # the mocap bags pos are the arguments from the 4th forward
#
#output( "The following rockin_mocap bags will be used:" )
#for b in mocap_bags_pos_list:
#	output( " "+b )
#### END without mocap bags auto-selection; uncomment to specify every mocap bag individually instead of just providing the directory

## open the bags
output( "Opening bags..." )
robot_bag = rosbag.Bag(robot_bag_pos)
mocap_bags_list = []
for mocap_bag_pos in mocap_bags_pos_list:
	b = rosbag.Bag(mocap_bag_pos)
	mocap_bags_list.append(b)
output( "Bags opened." )

## order mocap_bags_list by start time (the bags must be in order, otherwise the seek function doesn't work)
mocap_bags_list = sorted(mocap_bags_list, key=lambda bag: get_bag_info_from_object(bag)["start"])

###### Calculate the waypoints error
output( "waypoints error:" )
## init mocap bags iterators
mocap_messages_list = []
for b in mocap_bags_list:
	mocap_messages_list.append(b.read_messages(ROCKIN_MOCAP_TOPIC))

mocap_bag_iterator = DualIterator(mocap_messages_list)
waypoints_error  = Error()

## TODO find all waypoints
waypoints = [] # = something_something(robot_bag.read_messages("/roah_rsbb/goal")) # type: geometry_msgs/Pose2D

for _, waypoint_index, t in robot_bag.read_messages("/roah_rsbb/reached_waypoint"):
	# get the waypoint pose2D from waypoints
	wp_expected_pose = waypoints[waypoint_index]
	
	try:
		# get the respective mocap_pose
		mocap_pose_1, mocap_pose_2 = seek_mocap_pose_at(get_pose_time(robot_pose), mocap_bag_iterator)
		
		# if tracking was lost, just warn about it
		if pose_equal_position(mocap_pose_1, mocap_pose_2):	# if two poses are exactly the same, almost certainly the tracking is lost
			output( "[WARNING] tracking was lost at time "+str(get_pose_time(robot_pose).to_sec())+"[s], in waypoint "+str(waypoint_index) )
			
	except EndOfBag:
		output( "[ERROR] mocap bags are missing: there wasn't available a mocap pose corresponding to the robot's pose:\n" + str(robot_pose) )
		sys.exit(5)
	
	mocap_pose = interpolate(robot_pose, mocap_pose_1, mocap_pose_2)
	
	waypoints_error.update(robot_pose, mocap_pose)

	
output( "waypoints mean distance error    A = " + str(waypoints_error.get_position_error()) )
output( "waypoints mean orientation error B = " + str(waypoints_error.get_orientation_error()) )

output("")



###### Calculate the trajectory error
output( "trajectory error:" )

## init mocap bags iterators
mocap_messages_list = []
for b in mocap_bags_list:
	mocap_messages_list.append(b.read_messages(ROCKIN_MOCAP_TOPIC))

mocap_bag_iterator = DualIterator(mocap_messages_list)
trajectory_error = Error()

## iterate through the robot's poses and the respective mocap's poses, and update the error
for _, robot_pose, _ in robot_bag.read_messages("/rockin/"+teamname+"/marker_pose"):
	
	try:
		mocap_pose_1, mocap_pose_2 = seek_mocap_pose_at(get_pose_time(robot_pose), mocap_bag_iterator)
		
		# check for a maximum delay between the mocap poses and robot_pose
		if get_pose_time(mocap_pose_2).to_sec() - get_pose_time(mocap_pose_1).to_sec() > 0.1:
		    output( "[WARNING] consecutive mocap poses very distant in time, used to calculate the error (result may be invalid): t1,t2 = "+str(get_pose_time(mocap_pose_1).to_sec())+"[s],"+str(get_pose_time(mocap_pose_2).to_sec())+"[s] difference: "+str((get_pose_time(mocap_pose_2).to_sec() - get_pose_time(mocap_pose_1).to_sec())*1000)+" milliseconds" )
		
		# if tracking was lost, ignore this robot_pose
		if pose_equal_position(mocap_pose_1, mocap_pose_2):	# if two poses are exactly the same, almost certainly the tracking is lost
			output( "[INFO] ignoring pose at time "+str(get_pose_time(robot_pose).to_sec())+"[s] (tracking was lost)" )
			continue
	except EndOfBag:
		output( "[ERROR] mocap bags are missing: there wasn't available a mocap pose corresponding to the robot's pose:\n"+str(robot_pose) )
		sys.exit(6)
	
	mocap_pose = interpolate(robot_pose, mocap_pose_1, mocap_pose_2)
	
	trajectory_error.update(robot_pose, mocap_pose)

output( "trajectory mean distance error     = "+str(trajectory_error.get_position_error()) )
output( "trajectory mean orientation error  = "+str(trajectory_error.get_orientation_error()) )

output("")

## close the bags
for mb in mocap_bags_list:
	mb.close()
robot_bag.close()
