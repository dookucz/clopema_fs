#!/usr/bin/env python2
# Calculates:
#	gripper center of mass position
#	gripper mass
#	force and torque offset
# Robot _MUST_ be still when running this script!
import roslib; roslib.load_manifest('clopema_calibration')
import rospy
import tf
import sys
import numpy as np

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

def get_wrench_data(msg):
	global count
	global F,T
	F[3*count,1] = msg.wrench.force.x
	F[3*count+1,1] = msg.wrench.force.y
	F[3*count+2,1] = msg.wrench.force.z
	T[3*count,1] = msg.wrench.torque.x
	T[3*count+1,1] = msg.wrench.torque.y
	T[3*count+2,1] = msg.wrench.torque.z

if __name__ == '__main__':
	global count
	global F,T

	# number of readings to make
	SENSOR_READINGS = 4 

	F = np.empty(shape = (3*SENSOR_READINGS,1))
	T = np.empty(shape = (3*SENSOR_READINGS,1))

	rospy.init_node('gripper_centroid_calculator')
	listener = tf.TransformListener()
	try:
		rospy.Subscriber(sys.argv[1],WrenchStamped,get_wrench_data)
	except IndexError:
		print 'Error: argument needed' 
		print 'USAGE: centroid.py NAME_OF_TOPIC'
		sys.exit(1)
	except ROSException:
		print 'Non-existent topic (try /netft_data)'
		print 'USAGE: centroid.py NAME_OF_TOPIC'
		sys.exit(1)

	while not rospy.is_shutdown():

		# getting sensor data
		count = 0
		while count < SENSOR_READINGS:
			rospy.spinOnce()
			count += 1

		# getting transformation frame data
		now = rospy.Time(0)
		listener.waitForTransform('/base_link','/r2_gripper',now,rospy.Duration(1.5))
		(trans,quat) = listener.lookupTransform('/base_link','/r2_gripper',now)
		R = quat2rot(quat)

		# creating matrices for calculation
		A = np.empty(shape=(3*SENSOR_READINGS,4))
		i = 0
		while i < SENSOR_READINGS:
			A[3*i] = np.matrix([[1,0,0,-R[0,2]*g]])
			A[3*i+1] = np.matrix([A,[0,1,0,-R[1,2]*g]])
			A[3*i+2] = np.matrix([A,[0,0,1,-R[2,2]*g]])
			i += 1
			
		# calculating
		U,s,V = np.linalg.svd(A,full_matrices=1)
		V = V.getH()
		S = np.zeros(shape=(len(b),4))
		S[0,0] = s[0]
		S[1,1] = s[1]
		S[2,2] = s[2]
		S[3,3] = s[3]
		X = V*np.linalg.pinv(S)*U.getH()*b
		Fo = X[0:2]
		m = X[3]

		# printing output
		print 'R =\n' + str(R)
		print 'm = ' + str(m)
		print 'Fo =\n' + str(Fo)
		rospy.sleep(5)
