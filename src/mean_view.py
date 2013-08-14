#!/usr/bin/env python
import roslib; roslib.load_manifest('clopema_calibration')
import rospy, smach, smach_ros, math, copy, tf, PyKDL, os, shutil, numpy
from tf.transformations import quaternion_from_euler, quaternion_about_axis
from tf_conversions import posemath
from geometry_msgs.msg import *
from clopema_planning_actions.msg import MA1400JointState
import sysimport pyqtgraph as pg
from PyQt4 import QtGui


wrench = WrenchStamped()

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


def netf_cb(msg):
    global f
    global fx_mean_curve
    global fx_std_curve
    global fx_mean, fx_std
    global MAX_DATA
    global Fx, Fy, Fz, Tx, Ty, Tz

    if count >= MAX_DATA:
    	fx_std = []
    	fx_mean = []
    	count = 0

    val = Fx.add(msg.wrench.force.x)
    fx_std.append(val[1])
    fx_mean.append(val[0])
    fx_std_curve.setData(fx_std)
    fx_mean_curve.setData(fx_mean)
    count += 1

    f.write("%15.5f" % msg.header.stamp.to_sec() + ";")
    f.write(str(val[0]) + ';')
    f.write(str(val[1]) + ';')
   	f.write('\n')
    #print '%9.5f %9.5f'% Fx.add(msg.wrench.force.x), '%9.5f %9.5f'% Fy.add(msg.wrench.force.y), '%9.5f %9.5f'% Fz.add(msg.wrench.force.z), \
    #     '%9.5f %9.5f'%Tx.add(msg.wrench.torque.x), '%9.5f %9.5f'% Ty.add(msg.wrench.torque.y),'%9.5f %9.5f\r'% Tz.add(msg.wrench.torque.z), 


def main():
    global f
    global count
    global MAX_DATA
    global fx_mean
    global fx_std
    global fx_mean_curve
    global fx_std_curve
    global Fx, Fy, Fz, Tx, Ty, Tz 

    Fx = RunningStatistic(int(sys.argv[1]))
    Fy = RunningStatistic(int(sys.argv[1]))
    Fz = RunningStatistic(int(sys.argv[1]))
    Tx = RunningStatistic(int(sys.argv[1]))
    Ty = RunningStatistic(int(sys.argv[1]))
    Tz = RunningStatistic(int(sys.argv[1]))
   
    MAX_DATA = 1000
    count = 0
    rospy.init_node('sm_test')
    fx_mean = []
    fx_std = []
    def_data = []
    x_len = 1000
    y_len = 40
    # init default data - for aspect ratio
    i = 0
    while i < x_len:
    	if i > x_len/2:
    		def_data.append(-y_len/2)
    	else:
    		def_data.append(y_len/2)
    	i += 1
    win = pg.GraphicsWindow(title="Mean and std deviation")
    win.resize(1000,600)
    win.setWindowTitle('Mean and std deviation')
    mean_row = win.addPlot(title = 'mean')
    fx_mean_curve = mean_row.plot(def_data, pen = 'b')
    mean_row.enableAutoRange('xy', False)
    win.nextRow()
    std_row = win.addPlot(title = 'std')
    fx_std_curve = std_row.plot(def_data, pen = 'b')
    std_row.enableAutoRange('xy', False)
    f = open('mean_view.output', "w")
    f.write('t;fx_mean;fx_var;fy_mean;fy_var;fz_mean;fz_var;\n')
    #f.write('mean; var\n')
   

    rospy.Subscriber('/netft_data',WrenchStamped,netf_cb)
    QtGui.QApplication.instance().exec_()
    rospy.spin()

    f.close()

if __name__ == '__main__':
    main()
