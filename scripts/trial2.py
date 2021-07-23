#! /usr/bin/env python

import rospy
import math
import time
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
vi=0.3
l=0.14
k=0.3
beta=0.45
K=1
deltac=0.3 
### define goal point
#class Point:
 #   def __init__(self,x,y):
  #      self.x=x
   #     self.y=y

goal=Point()
goal.x=5
goal.y=5


#p=LaserScan()
p0=np.array([])
p1=np.array([])
p2=LaserScan().ranges
#print(p0)



controllast_free_0=0
controllast_free_1=0
controllast_free_2=0
t_e_1=0
t_e_2=0
t_e_0=0
x0=0
y0=0
x1=0
y1=0
x2=0
y2=0
theta2=0
theta1=0
theta0=0
#delta=0
delta0=0
delta1=0
delta2=0

###########

def inside(ranges, deltac): 
    #print(ranges) 
    for x in range(55,125):
        if ranges[x]<deltac:
            return 1
        else:
            continue      
    return 0 

def caldelta(lc,minvalue_of_ranges):
    deltaij=(lc*minvalue_of_ranges*180)/(math.pi)
    return deltaij


def distance(x,y):
    dist=math.sqrt(x**2+y**2) 
    return dist



##Odometry

def newOdom_tb0(msg0):
    global x0
    global y0
    global theta0                                             #### for getting the angle of the bot wrt the goal
    x0=msg0.pose.pose.position.x
    y0=msg0.pose.pose.position.y
   
    rot_q0=msg0.pose.pose.orientation
    (roll0,pitch0,theta0)=euler_from_quaternion([rot_q0.x,rot_q0.y,rot_q0.z,rot_q0.w])

def newOdom_tb1(msg1):
    global x1
    global y1
    global theta1
    x1=msg1.pose.pose.position.x
    y1=msg1.pose.pose.position.y

    rot_q1=msg1.pose.pose.orientation
    (roll1,pitch1,theta1)=euler_from_quaternion([rot_q1.x,rot_q1.y,rot_q1.z,rot_q1.w])

def newOdom_tb2(msg2):
    global x2
    global y2
    global theta2
    x2=msg2.pose.pose.position.x
    y2=msg2.pose.pose.position.y

    rot_q2=msg2.pose.pose.orientation
    (roll2,pitch2,theta2)=euler_from_quaternion([rot_q2.x,rot_q2.y,rot_q2.z,rot_q2.w])


####### surroundings data

def surr_data_tb0(data0):
        global center,left,right,delta0,minvalue_of_ranges0,p0
        #p0=[]
        
        #center0=data.ranges[320]
        #left0=data.ranges[639]
        lc0=data0.angle_increment
        p0=np.array(data0.ranges)
        for g in range(len(p0)):
            if p0[g]==float('inf'):
                p0[g]=30
            else:
                continue
        print(len(p0))
        minvalue_of_ranges0=(np.argmin(p0))-320      ## have to find the index of the minimum value then find the degree deltaij
        
        delta0=caldelta(lc0,minvalue_of_ranges0)
        #print(left,center,right)
def surr_data_tb1(data1):
        global center,left,right,minvalue_of_ranges1,delta1,p1
        p1=[]
        #center1=data.ranges[320]
        #left1=data.ranges[639]
        lc1=data1.angle_increment
        p1=data1.ranges
        minvalue_of_ranges1=p1.index(min(p1))-320    ## have to find the index of the minimum value then find the degree deltaij
        delta1=caldelta(lc1,minvalue_of_ranges1)
        #print(left,center,right)
def surr_data_tb2(data2):
        global center,left,right,delta2,minvalue_of_ranges2,p2
        p2=[]
        #center2=data.ranges[320]
        #left2=data.ranges[639]
        lc2=data2.angle_increment
        p2=data2.ranges
        minvalue_of_ranges2=p2.index(min(p2))-320    ## have to find the index of the minimum value then find the degree deltaij
        delta2=caldelta(lc2,minvalue_of_ranges2)
############################
rospy.init_node('vel')
pub_0 = rospy.Publisher('tb3_0/cmd_vel', Twist,queue_size=1)
pub_1 = rospy.Publisher('tb3_1/cmd_vel', Twist,queue_size=1)
pub_2 = rospy.Publisher('tb3_2/cmd_vel', Twist,queue_size=1)
rate = rospy.Rate(1)

##############angle_min/max  is in 'radian' ,,, ranges is in 'm'
        #print(left,center,right) minimum range value is in  [m]
        # maximum range value is in  [m]
###################################   

  
### publisher object  defined 
velocity0=Twist()
velocity1=Twist()
velocity2=Twist()



while not rospy.is_shutdown(): 
    
    # definging the subscriber
    rospy.Subscriber("tb3_0/odom",Odometry, newOdom_tb0)
    rospy.Subscriber("tb3_1/odom",Odometry, newOdom_tb1)
    rospy.Subscriber("tb3_2/odom",Odometry, newOdom_tb2)
    rospy.Subscriber("tb3_0/scan", LaserScan , surr_data_tb0)
    rospy.Subscriber("tb3_1/scan", LaserScan , surr_data_tb1)
    rospy.Subscriber("tb3_2/scan", LaserScan , surr_data_tb2)   

    

    ### finding the misalignment angle   and deltaij  
    dif_x0=goal.x-x0
    dif_y0=goal.y-y0
    err0=distance(dif_x0,dif_y0)
    angle_wrt_goal0=math.atan2(dif_y0,dif_x0)
    z0=180-(theta0-angle_wrt_goal0)              ### misalignment angle

    dif_x1=goal.x-x1
    dif_y1=goal.y-y1
    err1=distance(dif_x1,dif_y1)
    angle_wrt_goal1=math.atan2(dif_y1,dif_x1)
    z1=180-(theta1-angle_wrt_goal1)
 
    dif_x2=goal.x-x2
    dif_y2=goal.y-y2
    err2=distance(dif_x2,dif_y2)
    angle_wrt_goal2=math.atan2(dif_y2,dif_x2)
    z2=180-(theta2-angle_wrt_goal2)


#For Bot0
    if inside(p0, deltac):
        if controllast_free_0==0:
            controllast_free_0=1
            t_e_0=time.time()  
        if delta0<0 and z0<0:
            t= time.time()-t_e_0
            velocity0.linear.x= max(vi-l*t,0)
            velocity0.angular.z=-beta+K
        elif delta0>0 and z0<0:
            t= time.time()-t_e_0
            velocity0.linear.x=max(vi-l*t,0)
            velocity0.angular.z=-beta-K
        elif delta0<0 and z0>0:
            t= time.time()-t_e_0
            velocity0.linear.x=max(vi-l*t,0)
            velocity0.angular.z=beta+K
        elif delta0>0 and z0>0:
            t= time.time()-t_e_0
            velocity0.linear.x=max(vi-l*t,0)
            velocity0.angular.z=beta-K


    elif err0<0.1:
        velocity0.linear.x=0
        velocity0.angular.z=0                                      
            
    else:
        controllast_free_0=0        
        if z0==0:
            velocity0.linear.x=vi    # free subsystem linear v is constant
            velocity0.angular.z=0     # sgn misalignment angle is 0 so angular z is 0
        
        elif z0<0:
            velocity0.linear.x=vi    # free subsystem linear v is constant
            velocity0.angular.z=-k     # sgn misalignment angle is -1 so angular z is -k
        
        elif z0>01:
            velocity0.linear.x=vi    # free subsystem linear v is constant
            velocity0.angular.z=k    # sgn misalignment angle is 1 so angular z is k



#For bot 1
    if inside(p1, deltac):
        if controllast_free_1==0:
            controllast_free_1=1
            t_e_1=time.time()
            print(t_e_1)         
        if delta1<0 and z1<0:
            t= time.time()-t_e_1
            velocity1.linear.x= max(vi-l*t,0)
            velocity1.angular.z=-beta+K
            
        elif delta1>0 and z0<0:
            t= time.time()-t_e_1
            velocity1.linear.x=max(vi-l*t,0)
            velocity1.angular.z=-beta-K
        elif delta1<0 and z1>0:
            t= time.time()-t_e_1
            velocity1.linear.x=max(vi-l*t,0)
            velocity1.angular.z=beta+K
        elif delta1>0 and z1>0:
            t= time.time()-t_e_1
            velocity1.linear.x=max(vi-l*t,0)
            velocity1.angular.z=beta-K
                            
    elif err1<0.1:
        velocity1.linear.x=0
        velocity1.angular.z=0                                                         
   
    else:
        controllast_free_1=0
        if z1==0:
            velocity1.linear.x=vi    # free subsystem linear v is constant
            velocity1.angular.z=0     # sgn misalignment angle is 0 so angular z is 0
        
        elif z1<0:
            velocity1.linear.x=vi    # free subsystem linear v is constant
            velocity1.angular.z=-k     # sgn misalignment angle is -1 so angular z is -k
        
        elif z1>0:
            velocity1.linear.x=vi    # free subsystem linear v is constant
            velocity1.angular.z=k    # sgn misalignment angle is 1 so angular z is k


#For bot 2
    if inside(p2, deltac):
        if controllast_free_2==0:
            controllast_free_2=1
            t_e_2=time.time()
            print(t_e_2)         

        if delta2<0 and z2<0:
            t= time.time()-t_e_2
            velocity2.linear.x= max(vi-l*t,0)
            velocity2.angular.z=-beta+K
            
        elif delta2>0 and z2<0:
            t= time.time()-t_e_2
            velocity2.linear.x=max(vi-l*t,0)
            velocity2.angular.z=-beta-K
        elif delta2<0 and z2>0:
            t= time.time()-t_e_2
            velocity2.linear.x=max(vi-l*t,0)
            velocity2.angular.z=beta+K
        elif delta2>0 and z2>0:
            t= time.time()-t_e_2
            velocity2.linear.x=max(vi-l*t,0)
            velocity2.angular.z=beta-K

 
    elif err2<0.1:
        velocity2.linear.x=0
        velocity2.angular.z=0                                                                           
            # engaged subsystem   
           
   
    else:
        controllast_free_2=0
        if z2==0:
            velocity2.linear.x=vi    # free subsystem linear v is constant
            velocity0.angular.z=0     # sgn misalignment angle is 0 so angular z is 0
        
        elif z2<0:
            velocity2.linear.x=vi    # free subsystem linear v is constant
            velocity2.angular.z=-k     # sgn misalignment angle is -1 so angular z is -k
        
        elif z2>01:
            velocity2.linear.x=vi    # free subsystem linear v is constant
            velocity2.angular.z=k    # sgn misalignment angle is 1 so angular z is k
    # publish/ print velocity
    pub_0.publish(velocity0);
    pub_1.publish(velocity1);
    pub_2.publish(velocity2);
    #print(velocity0)
    rate.sleep()  
