#!/usr/bin/env python2
import roslib; roslib.load_manifest('clopema_fs')
import rospy
import numpy as np
import pyqtgraph as pg
import sys
from PyQt4 import QtGui
from geometry_msgs.msg import WrenchStamped

def print_help():
	print 'Usage: running_fourier DATA_TOPIC DATA_FREQ_IN_HZ SAMPLE_COUNT'

def data_acquisition(msg):
	global data
	global count
	global sampleCount
	global fftPlot
	global dataPlot
	if count < sampleCount:
		# TODO make all 3 forces
		data[count] = msg.wrench.force.x
		count += 1
	elif count == sampleCount:
		# data acquired; calculate fft
		Y = np.fft.fft(y)
		Pyy = np.real(Y*np.conj(Y)/sampleCount)
		# getting rid of DC gain; makes graph less readable
		Pyy[0] = 0
		# replotting graph
		dataPlot.setData(data)
		fftPlot.setData(Pyy)
		# resetting count
		count = 0


def main():
	# global vars
	global count
	global data
	global sampleCount
	global fftPlot
	global dataPlot
	# getting args
	try:
		dataTopic = str(sys.argv[1])
		freq = int(sys.argv[2])
		sampleCount = int(sys.argv[3])
	except IndexError:
		print 'Error: this script needs three arguments'
		print_help()
		sys.exit(1)
	except ValueError:
		print 'Error: invalid value for freq/sample count'
		print_help()
		sys.exit(1)
	# preparing data array for data_acquisition()
	data = [0.0]*sampleCount
	# calculating frequency axis array (defines step size on freq axis)
	freqAxis = np.arange(0,sampleCount/2)*freq/float(sampleCount)
	# calculating time domain axis
	timeAxis = np.arange(0,sampleCount)*1/float(freq)
	# initiating node
	rospy.init_node('running_fourier')
	# creating gui
	win = pg.GraphicsWindow(title="FFT force")
	win.resize(1000,600)
	win.setWindowTitle('FFT force')
	dataPlot = win.addPlot(title = 'Force Z time domain')
	# plot sample time domain data
	dataCurve = dataPlot.plot([0]*sampleCount,timeAxis,pen = 'g')
	# stop rescaling x axis
	dataPlot.enableAutoRange('x', False)
	dataPlot.setLabel('bottom', "Time", units='s')
	win.nextRow()
	# plot sample freq domain data
	fftPlot = win.addPlot(title = 'Force Z freq domain')
	fftCurve = fftPlot.plot([0]*sampleCount/2, freqAxis, pen = 'r')
	# stop rescaling x axis
	fftPlot.enableAutoRange('x', False)
	fftPlot.setLabel('bottom', "Frequency", units='Hz')
	# creating subscriber; data won't be acquired until count < sample count
	count = 0
	rospy.Subscriber(dataTopic,WrenchStamped,data_acquisition)
	# QT loop; from now on, all work is done within data_acquisition()
	QtGui.QApplication.instance().exec_()
