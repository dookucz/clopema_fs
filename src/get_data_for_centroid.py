#!/usr/bin/env python2
import roslib; roslib.load_manifest('clopema_fs')
import rospy
import math
import tf
import numpy

if __name__ == '__main__':
	rospy.Subscriber('/netft_data',WrenchStamped,write_force)