#!/usr/bin/env python2
import re
import pyqtgraph as pg
import numpy
import math
import scipy
import sys
from PyQt4 import QtGui

def main():
	print 'Loading force sensor data from file \'data\'... ',
	with open('data') as f:
		force_data = f.readlines()
	f.close()
	data_time = []
	f_x = []
	f_y = []
	f_z = []
	m_x = []
	m_y = []
	m_z = []
	# time;f_x;f_y;f_z;m_x;m_y;m_z;
	regex_force = re.compile('''\
(?P<time>[^;]*);\
(?P<force_x>[^;]*);\
(?P<force_y>[^;]*);\
(?P<force_z>[^;]*);\
(?P<torque_x>[^;]*);\
(?P<torque_y>[^;]*);\
(?P<torque_z>[^;]*);\
''')
	for line in force_data:
		res = regex_force.search(line)
		data_time.append(float(res.group('time')))
		f_x.append(float(res.group('force_x')))
		f_y.append(float(res.group('force_y')))
		f_z.append(float(res.group('force_z')))
		m_x.append(float(res.group('torque_x')))
		m_y.append(float(res.group('torque_y')))
		m_z.append(float(res.group('torque_z')))
	print str(len(data_time)) + ' values loaded'
	print 'Loading script output from file \'output\'... ',
	with open('output') as f:
		output = f.readlines()
	f.close()
	start_time = []
	stop_time = []
	# compiling regex
	regex_start = re.compile('\[INFO\] \[WallTime: (\d*\.\d*).*VISUALIZE.*EXECUTE.*')
	regex_stop = re.compile('\[INFO\] \[WallTime: (\d*\.\d*).*EXECUTE.*succeeded.*succeeded.*')
	# searching for start&stop times
	for line in output:
		res = regex_start.search(line)
		if res:
			start_time.append(float(res.group(1)))
		else:
			res = regex_stop.search(line)
			if res:
				stop_time.append(float(res.group(1)))
	if len(start_time) != len(stop_time):
		print 'ERROR: stop and start times mismatch'
		sys.exit(1)
	print str(len(start_time)) + ' values loaded'
	# unix time -> time since starting measure
	measure_start = data_time[0]
	i = 0
	while i < len(data_time):
		data_time[i] -= measure_start
		i += 1
	i = 0
	while i < len(start_time):
		start_time[i] -= measure_start
		stop_time[i] -= measure_start
		i += 1
	print 'Plotting...'
	# Here starts the plotting
	win = pg.GraphicsWindow(title="Force and torque measure")
	win.resize(1000,600)
	win.setWindowTitle('Force and torque measure')
	force_p = win.addPlot(title = 'Force measure')
	force_p.plot(data_time,f_x, pen = 'r')
	force_p.plot(data_time,f_y, pen = 'g')
	force_p.plot(data_time,f_z, pen = 'b')
	for time in start_time:
		force_p.plot([time, time],[-40,20],pen = 'w')
	for time in stop_time:
		force_p.plot([time, time],[-40,20],pen = (100,100,100))
	win.nextRow()
	torque_p = win.addPlot(title = 'Torque measure')
	torque_p.plot(data_time,m_x, pen = 'r')
	torque_p.plot(data_time,m_y, pen = 'g')
	torque_p.plot(data_time,m_z, pen = 'b')
	for time in start_time:
		torque_p.plot([time, time],[-2,2],pen = 'w')
	for time in stop_time:
		torque_p.plot([time, time],[-2,2],pen = (100,100,100))
	QtGui.QApplication.instance().exec_()

if __name__ == '__main__':
	main()