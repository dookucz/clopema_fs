#!/usr/bin/env python
import roslib; roslib.load_manifest('clopema_calibration')
import rospy, smach, smach_ros, math, copy, tf, PyKDL, os, shutil, numpy
from tf.transformations import quaternion_from_euler, quaternion_about_axis
from tf_conversions import posemath
from clopema_smach import *
from geometry_msgs.msg import *
from smach import State
from clopema_planning_actions.msg import MA1400JointState
import sys

wrench = WrenchStamped()

f = None

class RunnigStatistic(object):

    def __init__(self):
        self.F=[]
        self.counter=0
        self.N=2000
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
    global Fx, Fy, Fz, Tx, Ty, Tz
    print '%9.5f %9.5f'% Fx.add(msg.wrench.force.x), '%9.5f %9.5f'% Fy.add(msg.wrench.force.y), '%9.5f %9.5f'% Fz.add(msg.wrench.force.z), \
          '%9.5f %9.5f'%Tx.add(msg.wrench.torque.x), '%9.5f %9.5f'% Ty.add(msg.wrench.torque.y),'%9.5f %9.5f\r'% Tz.add(msg.wrench.torque.z), 
  
  
    

         
        

def main():
    global f
    global Fx, Fy, Fz, Tx, Ty, Tz 
    Fx = RunnigStatistic()
    Fy = RunnigStatistic()
    Fz = RunnigStatistic()
    Tx = RunnigStatistic()
    Ty = RunnigStatistic()
    Tz = RunnigStatistic()
   
    rospy.init_node('sm_test')
    
    f = open(sys.argv[1], "w")
    #f.write('t;fx;fy;F;tx;ty;tz\n')
    #f.write('mean; var\n')
    print ' Fx_mean   Fx_std    Fy_mean   Fy_std    Fz_mean   Fz_std    Tx_mean   Tx_std    Ty_mean   Ty_std    Tz_mean   Tz_std'
   

    rospy.Subscriber('/netft_data',WrenchStamped,netf_cb)

    rospy.spin()

    f.close()

if __name__ == '__main__':
    main()
