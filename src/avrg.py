#!/usr/bin/env python2
import re
import pyqtgraph as pg
import numpy
import math
import scipy
import sys

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
	# calculating avrg
	start_pos = 0
	stop_pos = 0
	i = 0
	while i < len(start_time)-1:
		print 'Step ' + str(i+1)
		print 'time: ' + str(stop_time[i]) + ' to ' + str(start_time[i+1])

		j = start_pos
		while data_time[j] < stop_time[i]:
			j += 1
		stop_pos = j
		while data_time[j] < start_time[i+1]:
			j += 1
		start_pos = j
		#print 'DEBUG: stop at index ' + str(stop_pos) + ', next start at index ' + str(start_pos)
		# Force X
		sum = 0.0
		for val in f_x[stop_pos:start_pos]:
			sum += val
		avrg = sum / (start_pos - stop_pos)
		print '\tAverage values for force:\n\t\tX = ' + str(avrg)
		# Force Y
		sum = 0.0
		for val in f_y[stop_pos:start_pos]:
			sum += val
		avrg = sum / (start_pos - stop_pos)
		print '\t\tY = ' + str(avrg)
		# Force X
		sum = 0.0
		for val in f_z[stop_pos:start_pos]:
			sum += val
		avrg = sum / (start_pos - stop_pos)
		print '\t\tZ = ' + str(avrg)
		# doplnit momenty
		i += 1
		
if __name__ == '__main__':
	main()