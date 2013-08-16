#!/usr/bin/env python2
import roslib; roslib.load_manifest('clopema_calibration')
import rospy, math, numpy, sys
from geometry_msgs.msg import WrenchStamped

class RunningStatistic(object):
	def __init__(self, count=2000):
		self.F=[]
		self.counter=0
		self.N=count
		self.mean_old=0
		self.var_old=0
		self.Fo=0

	def add(self, value):
		self.F.append(value) 
		self.counter+=1  
		if  self.counter>self.N: 
			self.Fo=self.F[0]
			del self.F[0] 

		pom=0
		pom2=0
		if self.counter==self.N:
			for i in range(0,self.N-1):
				pom=pom+self.F[i]
				pom2=pom2+self.F[i]*self.F[i]
			self.mean_old=pom/self.N
			self.var_old=pom2/self.N-self.mean_old*self.mean_old

		if self.counter>self.N:
			mean_new=self.mean_old+(-self.Fo+self.F[-1])/self.N
			var_new=self.var_old+(-self.Fo*self.Fo+self.F[-1]*self.F[-1])/self.N+self.mean_old*self.mean_old-mean_new*mean_new

			self.mean_old=mean_new
			self.var_old=var_new

		return self.mean_old, self.var_old      

count = 0
def resend_data(msg):
	global count
	global Fx, Fy, Fz, Tx, Ty, Tz
	global mean_pub,dev_pub

	# const
	SHOW_EACH = 10

	# TODO pridelat if na cekani nez se nabere dost dat?

	if count == SHOW_EACH:
		mean_wr = WrenchStamped()
		dev_wr = WrenchStamped()
		# copying acquired msg header to new msg
		mean_wr.header = msg.header # will this work?
		dev_wr.header = msg.header
		# copying force data X
		val = Fx.add(msg.wrench.force.x)
		mean_wr.wrench.force.x = val[0]
		dev_wr.wrench.force.x = val[1]
		# copying force data Y
		val = Fy.add(msg.wrench.force.y)
		mean_wr.wrench.force.y = val[0]
		dev_wr.wrench.force.y = val[1]
		# copying force data Z
		val = Fz.add(msg.wrench.force.z)
		mean_wr.wrench.force.z = val[0]
		dev_wr.wrench.force.z = val[1]
		# copying torque data X
		val = Tx.add(msg.wrench.torque.x)
		mean_wr.wrench.torque.x = val[0]
		dev_wr.wrench.torque.x = val[1]
		# copying torque data Y
		val = Ty.add(msg.wrench.torque.y)
		mean_wr.wrench.torque.y = val[0]
		dev_wr.wrench.torque.y = val[1]
		# copying torque data Z
		val = Tz.add(msg.wrench.torque.z)
		mean_wr.wrench.torque.z = val[0]
		dev_wr.wrench.torque.z = val[1]
		# publishing
		mean_pub.publish(mean_wr)
		dev_pub.publish(dev_wr)
		real_pub.publish(msg)
		count = 0
	count += 1

def main():
	global Fx, Fy, Fz, Tx, Ty, Tz
	global mean_pub,dev_pub,real_pub

	# topic names
	real_pub_name='ft_data'
	dev_pub_name='ft_std_dev'
	mean_pub_name='ft_mean_val'

	try:
		avrg_count = int(sys.argv[1])
	except ValueError:
		print 'Error: argument is non-integer type'
		sys.exit(1)
	except IndexError:
		print 'Error: no arguments given'
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
	print 'Publishing topics:'
	print 'Real data out: ' + real_pub_name
	print 'Mean values data out: ' + mean_pub_name
	print 'Standard deviation data out: ' + dev_pub_name 
	rospy.spin()

if __name__ == '__main__':
	main()
