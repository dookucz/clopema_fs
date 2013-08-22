#!/usr/bin/env python2
import roslib; roslib.load_manifest('clopema_fs')
import rospy
import math
import tf2_ros
import numpy
from tf_conversions import posemath
from geometry_msgs.msg import Pose
from clopema_utilities import np_bridge

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
	

if __name__ == '__main__':
	rospy.init_node('tf_listener_test')
	print rospy.rostime.is_rostime_initialized()
	#rospy.sleep(500)
	print 'pred listenerem'
	tf_buffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tf_buffer)
	print 'po listeneru'
	while not rospy.is_shutdown():
		print 'pred rospy.Time(0)'
		now = rospy.Time(0)
		print 'po rospy.Time(0)'
		print 'pred waitForTransform()'
		listener.waitForTransform('/base_link','/r2_force_sensor',now,rospy.Duration(1.5))
		print 'po waitForTransform()'
		(trans,rot) = listener.lookupTransform('/base_link','/r2_force_sensor',now)
		print str(quat2rot(rot)) + '\n\n'
		rospy.sleep(3)
