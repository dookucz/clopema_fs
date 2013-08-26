#!/usr/bin/env python2
import roslib; roslib.load_manifest('clopema_fs')
import rospy
import math
import tf
import numpy
from sys import exit
from time import sleep
from tf_conversions import posemath
from geometry_msgs.msg import Pose
from clopema_utilities import np_bridge
from geometry_msgs.msg import WrenchStamped

# const
MAX_VALS = 1000


def quat2rot(quat):
	w = quat[3]
	x = quat[0]
	y = quat[1]
	z = quat[2]
	Nq = w*w + x*x + y*y + z*z
	if Nq > 0.0:
		s = 2/Nq
	else:
		s = 0.0
	X = x*s; Y = y*s; Z = z*s
	wX = w*X; wY = w*Y; wZ = w*Z
	xX = x*X; xY = x*Y; xZ = x*Z
	yY = y*Y; yZ = y*Z; zZ = z*Z
	return numpy.matrix([[1.0-(yY+zZ), xY-wZ, xZ+wY],[xY+wZ,1.0-(xX+zZ),yZ-wX],[xZ-wY,yZ+wX,1.0-(xX+yY)]])

def write_data(msg):
	global count
	global d
	if count < MAX_VALS:
		d.write("%15.5f" % msg.header.stamp.to_sec() + ";")
		d.write(str(msg.wrench.force.x) + ";")
		d.write(str(msg.wrench.force.y) + ";")
		d.write(str(msg.wrench.force.z) + ";")
		d.write(str(msg.wrench.torque.x) + ";")
		d.write(str(msg.wrench.torque.y) + ";")
		d.write(str(msg.wrench.torque.z) + ";\n")
		count += 1
	else:
		d.close()
		rospy.signal_shutdown('Data acquired, shutting down')

def main():
	global count
	global d
	rospy.init_node('centroid_data_collector')
	try:
		d = open('centroid_calc_data','w')
	except IOError:
		print 'Error: couldn\'t open file'
		exit(1)
	listener = tf.TransformListener()
	now = rospy.Time(0)
	listener.waitForTransform('/base_link','/r2_force_sensor',now,rospy.Duration(1.5))
	(trans,rot) = listener.lookupTransform('/base_link','/r2_force_sensor',now)
	d.write(str(quat2rot(rot)) + '\n')
	count = 0
	rospy.Subscriber('/netft_data',WrenchStamped,write_data)
	rospy.spin()

if __name__ == '__main__':
	main()
