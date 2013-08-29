#!/usr/bin/env python2
import re
import pyqtgraph as pg
import numpy
import math
import scipy
import sys
from PyQt4 import QtGui,QtCore
import roslib; roslib.load_manifest('clopema_fs')
import rospy, smach, smach_ros, copy, tf, PyKDL, os, shutil
from geometry_msgs.msg import WrenchStamped

def netf_cb(msg):
	global FREQ_CONST
	global td_data
	global td_curve
	global fd_curve
	
	if len(td_data) == FREQ_CONST:
		td_curve.setData(td_data)
		fd_curve.setData(numpy.fft.rfft(td_data).real)
		td_data = []
	else:
		td_data.append(msg.wrench.force.z)

def main():
	global FREQ_CONST
	global td_data
	global td_curve
	global fd_curve
	FREQ_CONST = 1000
	td_data = []
	rospy.init_node('sm_test')
	# init default data
	i = 0
	while i < FREQ_CONST:
		td_data.append(0)
		i += 1
	win = pg.GraphicsWindow(title="Fourier transformation of force sensor signal")
	win.resize(1000,600)
	win.setWindowTitle('Fourier transformation of force sensor signal')
	time_domain = win.addPlot(title = 'Z-axis time domain')
	td_curve = time_domain.plot(td_data, pen = 'b')
	win.nextRow()
	freq_domain = win.addPlot(title = 'Z-axis freq domain')
	fd_curve = freq_domain.plot(td_data, pen = 'b')

	rospy.Subscriber('/netft_data',WrenchStamped,netf_cb)
	QtGui.QApplication.instance().exec_()
	rospy.spin()

if __name__ == '__main__':
	main()
