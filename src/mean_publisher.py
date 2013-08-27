#!/usr/bin/env python2
import roslib; roslib.load_manifest('clopema_calibration')
import rospy, math, numpy, sys
from geometry_msgs.msg import WrenchStamped

class RunningStatistic(object):
	def __init__(self, count=2000):
		self.F = []
		self.N = count
		self.sum = 0
		self.mean = 0
		self.sq_sum = 0
		self.std = 0

	def add(self, value):
		self.F.append(value)
		self.sum += value
		self.sq_sum += value**2
		if len(self.F) > self.N:
			self.sum -= self.F[0]
			self.sq_sum -= self.F[0]**2
			del self.F[0]
			self.mean = self.sum/float(self.N)
			self.std = sqrt(self.N*self.sq_sum-self.sum**2)/float(self.N)
		return self.mean, self.std

count = 0
def resend_data(msg):
	global count
	global Fx, Fy, Fz, Tx, Ty, Tz
	global mean_pub,dev_pub
	global publish_each

	# filtering
	val_fx = Fx.add(msg.wrench.force.x)
	val_fy = Fy.add(msg.wrench.force.y)
	val_fz = Fz.add(msg.wrench.force.z)
	val_tx = Tx.add(msg.wrench.torque.x)
	val_ty = Ty.add(msg.wrench.torque.y)
	val_tz = Tz.add(msg.wrench.torque.z)
	if count == publish_each:
		mean_wr = WrenchStamped()
		dev_wr = WrenchStamped()
		# copying acquired msg header to new msg
		mean_wr.header = msg.header
		dev_wr.header = msg.header
		# copying force data X
		mean_wr.wrench.force.x = val_fx[0]
		dev_wr.wrench.force.x = val_fx[1]
		# copying force data Y
		mean_wr.wrench.force.y = val_fy[0]
		dev_wr.wrench.force.y = val_fy[1]
		# copying force data Z
		mean_wr.wrench.force.z = val_fz[0]
		dev_wr.wrench.force.z = val_fz[1]
		# copying torque data X
		mean_wr.wrench.torque.x = val_tx[0]
		dev_wr.wrench.torque.x = val_tx[1]
		# copying torque data Y
		mean_wr.wrench.torque.y = val_ty[0]
		dev_wr.wrench.torque.y = val_ty[1]
		# copying torque data Z
		mean_wr.wrench.torque.z = val_tz[0]
		dev_wr.wrench.torque.z = val_tz[1]
		# publishing
		mean_pub.publish(mean_wr)
		dev_pub.publish(dev_wr)
		real_pub.publish(msg)
		count = -1
	count += 1

def main():
	global Fx, Fy, Fz, Tx, Ty, Tz
	global mean_pub,dev_pub,real_pub
	global publish_each

	# topic names
	real_pub_name='ft_data'
	dev_pub_name='ft_std_dev'
	mean_pub_name='ft_mean_val'

	try:
		avrg_count = int(sys.argv[1])
		publish_each = int(sys.argv[2])
		if publish_each < 1  or avrg_count < 1:
			print 'Error: arguments must be positive integers'
			sys.exit(1)
	except ValueError:
		print 'Error: argument is non-integer type'
		sys.exit(1)
	except IndexError:
		print 'Error: two arguments needed'
		print 'USAGE: mean_publisher filter_count each_val_to_print'
		sys.exit(1)

	Fx = RunningStatistic(count=avrg_count)
	Fy = RunningStatistic(count=avrg_count)
	Fz = RunningStatistic(count=avrg_count)
	Tx = RunningStatistic(count=avrg_count)
	Ty = RunningStatistic(count=avrg_count)
	Tz = RunningStatistic(count=avrg_count)

	# initiating node
	rospy.init_node('mean_publisher')
	# subscribing to topic netft_data (sensor data)
	rospy.Subscriber('/netft_data',WrenchStamped,resend_data)
	# creating publishers: mean value and standard deviation
	mean_pub = rospy.Publisher(mean_pub_name,WrenchStamped)
	dev_pub = rospy.Publisher(dev_pub_name,WrenchStamped)
	real_pub = rospy.Publisher(real_pub_name,WrenchStamped)
	
	# our work is done here; waiting
	print 'Filtering through ' + str(avrg_count) + ' values'
	print 'Publishing each ' + str(publish_each) + '. value'
	print 'Publishing topics:'
	print 'Real data out: ' + real_pub_name
	print 'Mean values data out: ' + mean_pub_name
	print 'Standard deviation data out: ' + dev_pub_name 
	rospy.spin()

if __name__ == '__main__':
	main()
