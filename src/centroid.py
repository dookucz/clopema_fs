#!/usr/bin/env python2
# Calculates:
#	gripper center of mass position
#	gripper mass
#	force and torque offset
# Robot _MUST_ be still when running this script!
import roslib; roslib.load_manifest('clopema_fs')
import rospy
import tf
import sys
import numpy as np
from geometry_msgs.msg import WrenchStamped



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
	return np.matrix([[1.0-(yY+zZ), xY-wZ, xZ+wY],[xY+wZ,1.0-(xX+zZ),yZ-wX],[xZ-wY,yZ+wX,1.0-(xX+yY)]])

def get_wrench_data(msg):
	global count
	global F,T
	global SENSOR_READINGS
	if count < SENSOR_READINGS:
		F[3*count,0] = msg.wrench.force.x
		F[3*count+1,0] = msg.wrench.force.y
		F[3*count+2,0] = msg.wrench.force.z
		T[3*count,0] = msg.wrench.torque.x
		T[3*count+1,0] = msg.wrench.torque.y
		T[3*count+2,0] = msg.wrench.torque.z
		count += 1

if __name__ == '__main__':
	global count
	global F,T
	global SENSOR_READINGS
	# number of readings to make
	SENSOR_READINGS = 4
	
	# gravity force
	g = 9.81

	F = np.matrix(np.empty(shape = (3*SENSOR_READINGS,1)))
	T = np.matrix(np.empty(shape = (3*SENSOR_READINGS,1)))

	rospy.init_node('gripper_centroid_calculator')
	listener = tf.TransformListener()
	try:
		rospy.Subscriber(sys.argv[1],WrenchStamped,get_wrench_data)
	except IndexError:
		print 'Error: argument needed' 
		print 'USAGE: centroid.py NAME_OF_TOPIC_FOR_SENSOR_DATA'
		sys.exit(1)
	except rospy.exceptions.ROSException:
		print 'Non-existent topic (try /netft_data)'
		print 'USAGE: centroid.py NAME_OF_TOPIC_FOR_SENSOR_DATA'
		sys.exit(1)

	while not rospy.is_shutdown():
		# (re)load data from sensor
		data_loaded = False
		count = 0
		
		# getting transformation frame data
		now = rospy.Time(0)
		listener.waitForTransform('/base_link','/r2_force_sensor',now,rospy.Duration(1.5))
		(trans,quat) = listener.lookupTransform('/base_link','/r2_force_sensor',now)
		R = quat2rot(quat)

		# creating matrix A for force calculation
		A = np.matrix(np.empty(shape=(3*SENSOR_READINGS,4)))
		i = 0
		while i < SENSOR_READINGS:
			A[3*i] = np.matrix([[1,0,0,-R[0,2]*g]])
			A[3*i+1] = np.matrix([[0,1,0,-R[1,2]*g]])
			A[3*i+2] = np.matrix([[0,0,1,-R[2,2]*g]])
			i += 1
			
		# waiting until sensor data are loaded
		while count < SENSOR_READINGS:
			rospy.sleep(0.5)
			
		# calculating force offset and mass
		U,s,V = np.linalg.svd(A,full_matrices=1)
		V = V.getH()
		S = np.matrix(np.zeros(shape=(len(F),4)))
		S[0,0] = s[0]
		S[1,1] = s[1]
		S[2,2] = s[2]
		S[3,3] = s[3]
		X = V*np.linalg.pinv(S)*U.getH()*F
		Fo = X[0:3]
		m = float(X[3])

		# creating matrix A for torque calculation
		A = np.matrix(np.empty(shape=(3*SENSOR_READINGS,6)))
		i = 0
		while i < SENSOR_READINGS:
			A[3*i] = np.matrix([[1,0,0,0,-m*R[2,2]*g,m*R[1,2]*g]])
			A[3*i+1] = np.matrix([[0,1,0,m*R[2,2]*g,0,-m*R[0,2]*g]])
			A[3*i+2] = np.matrix([[0,0,1,-m*R[1,2]*g,m*R[0,2]*g,0]])
			i += 1
			
		# calculating torque offset and centroid position (r)
		U,s,V = np.linalg.svd(A,full_matrices=1)
		V = V.getH()
		S = np.matrix(np.zeros(shape=(len(T),6)))
		S[0,0] = s[0]
		S[1,1] = s[1]
		S[2,2] = s[2]
		S[3,3] = s[3]
		S[4,4] = s[4]
		S[5,5] = s[5]
		X = V*np.linalg.pinv(S)*U.getH()*T
		To = X[0:3]
		centroid_pos = X[3:6]
		
		# printing output
		print 'R =\n' + str(R)
		print 'm = ' + str(m)
		print 'Fo =\n' + str(Fo)
		print 'To =\n' + str(To)
		print 'r =\n' + str(centroid_pos)
		print '\n'
		
		### DEBUG OUT
		try:
			f = open('centroid.out','a')
			f.write('R =\n' + str(R) + '\n')
			f.write('F =\n' + str(F) + '\n')
			f.write('T =\n' + str(T) + '\n')
			f.write('Fo =\n' + str(Fo) + '\n')
			f.write('To =\n' + str(To) + '\n')
			f.write('r =\n' + str(centroid_pos) + '\n')
			f.write('m = ' + str(m) + '\n')
			f.write('\n\n')
			f.close()
		except IOError:
			print 'DEBUG ERR: can\'t write to file centroid.out'
		###
		
		
		# sleep for some time and recalculate
		rospy.sleep(1)
