#!/usr/bin/python

import sys, yaml
import rosbag, rospy, geometry_msgs.msg, tf
#from geometry_msgs.msg import PoseStamped

def create_pose(x, y, z, roll, pitch, yaw, seq, frame_id, stamp):
	pose = geometry_msgs.msg.PoseStamped()
	pose.pose.position.x = x
	pose.pose.position.y = y
	pose.pose.position.z = z
	quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
	pose.pose.orientation.x = quat[0]
	pose.pose.orientation.y = quat[1]
	pose.pose.orientation.z = quat[2]
	pose.pose.orientation.w = quat[3]
	pose.header.seq = seq
	pose.header.frame_id = frame_id
	pose.header.stamp = stamp
	return pose

def create_pose_from_yaml(y):
	return create_pose(	y['x'], y['y'], y['z'],
						y['roll'], y['pitch'], y['yaw'],
						y['seq'], y['frame_id'], rospy.Time(y['stamp']))

if len(sys.argv) < 2:
	print "An argument must be provided: yaml file with description of bags that will be generated"
	sys.exit(1)

bags_yaml = sys.argv[1]
messages = yaml.load(open(bags_yaml, 'r'))['messages']

bags = {
	'rockin_mocap_bag_1': rosbag.Bag('rockin_mocap_bag_1.bag', 'w'),
	'robot_bag': rosbag.Bag('robot_bag.bag', 'w')
}

bags = {}

for m in messages:
	bag_name = m['bag']
	if not bag_name in bags.keys():
		bags[bag_name] = rosbag.Bag(bag_name+'.bag', 'w')
	
	bags[bag_name].write(m['topic'], create_pose_from_yaml(m['msg']), rospy.Time(m['msg']['stamp']))

for b in bags.values():
	b.close()


for bag_name in bags.keys():
	with rosbag.Bag(bag_name+'.bag') as b:
		messages_read = b.read_messages()
		print bag_name
		for m in messages_read:
			print m
