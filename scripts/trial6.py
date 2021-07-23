#! /usr/bin/env python



#### issue that minimum value is calculated for 360 degrees need to reduce it to 180
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
l=0.2
k=0.2
beta=0.45
K=1
deltac=0.3
### define goal point
#class Point:
 #   def __init__(self,x,y):
  #      self.x=x
   #     self.y=y

goal=Point()
goal.x=4.00
goal.y=1.00


#p=LaserScan().ranges
p0=LaserScan().ranges
p1=LaserScan().ranges
p2=LaserScan().ranges
#inview=[]


for x in range(360):
    p0.append(0)
    p1.append(0)
    p2.append(0)
#print(len(p0))
## control variable , we switch system and change its vallue
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
     
    for x in range(len(ranges)):
        if ranges[x]<deltac:
            return 1
        else:
            continue      
    return 0 
def veryclose(ranges, deltac): 
    #print(len(ranges)) 
    for x in range(0,15):
        if ranges[x]<0.5:
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
    global theta0                                             #### for getting the angle of the bot wrt to x axis
    x0=msg0.pose.pose.position.x
    y0=msg0.pose.pose.position.y
   
    rot_q0=msg0.pose.pose.orientation
    (roll0,pitch0,theta0)=euler_from_quaternion([rot_q0.x,rot_q0.y,rot_q0.z,rot_q0.w])
    theta0=theta0*(180/math.pi)
    if theta0>0 or theta0==0:
        theta0=theta0  
    elif theta0<0:
        theta0=360+theta0

def newOdom_tb1(msg1):
    global x1
    global y1
    global theta1
    x1=msg1.pose.pose.position.x
    y1=msg1.pose.pose.position.y

    rot_q1=msg1.pose.pose.orientation
    (roll1,pitch1,theta1)=euler_from_quaternion([rot_q1.x,rot_q1.y,rot_q1.z,rot_q1.w])
    theta1=theta1*(180/math.pi)

    if theta1>0 or theta1==0:
        theta1=theta1  
    elif theta1<0:
        theta1=360+theta1


def newOdom_tb2(msg2):
    global x2
    global y2
    global theta2
    x2=msg2.pose.pose.position.x
    y2=msg2.pose.pose.position.y

    rot_q2=msg2.pose.pose.orientation
    (roll2,pitch2,theta2)=euler_from_quaternion([rot_q2.x,rot_q2.y,rot_q2.z,rot_q2.w])
    theta2=theta2*(180/math.pi)
    if theta2>0 or theta2==0:
        theta2=theta2  
    elif theta2<0:
        theta2=360+theta2

####### surroundings data

def surr_data_tb0(data0):
        global delta0,minvalue_of_ranges0,p0,inview
       
        lc0=data0.angle_increment
        p0=np.array(data0.ranges)
        # omit the inf value
        for g in range(len(p0)):
            if p0[g]==float('inf'):
                p0[g]=3.5
            else:
                continue
        
        #print(type(inview))	
        #print(len(p0))
        p0=np.concatenate((p0[-35:-1],p0[0:35]),axis=None)
        minvalue_of_ranges0=np.argmin(p0)-35    ## have to find the index of the minimum value then find the degree deltaij
       # print(minvalue_of_ranges0)

        delta0=caldelta(lc0,minvalue_of_ranges0)
        #print(left,center,right)
def surr_data_tb1(data1):
        global center,left,right,minvalue_of_ranges1,delta1,p1
 
        lc1=data1.angle_increment
        p1=np.array(data1.ranges)
        for g in range(len(p1)):
            if p1[g]==float('inf'):
                p1[g]=3.5
            else:
                continue
        #print(len(p1))
        p1=np.concatenate((p1[-35:-1],p1[0:35]),axis=None)
        minvalue_of_ranges1=np.argmin(p1)-35    ## have to find the index of the minimum value then      ## have to value then find the degree deltaij
        delta1=caldelta(lc1,minvalue_of_ranges1)
        #print(left,center,right)
def surr_data_tb2(data2):
        global center,left,right,delta2,minvalue_of_ranges2,p2
        
        #center2=data.ranges[320]
        #left2=data.ranges[639]
        lc2=data2.angle_increment
        p2=np.array(data2.ranges)
        for g in range(len(p2)):
            if p2[g]==float('inf'):
                p2[g]=3.5
            else:
                continue
        #print(len(p2))
        p2=np.concatenate((p2[-35:-1],p2[0:35]),axis=None)
        minvalue_of_ranges2=np.argmin(p2)-35    ## have to find the index of the minimum value then      ## have to then find the degree deltaij
        delta2=caldelta(lc2,minvalue_of_ranges2)
        #print(delta2)
############################
rospy.init_node('vel')
pub_0 = rospy.Publisher('tb3_0/cmd_vel', Twist,queue_size=1)
pub_1 = rospy.Publisher('tb3_1/cmd_vel', Twist,queue_size=1)
pub_2 = rospy.Publisher('tb3_2/cmd_vel', Twist,queue_size=1)
rate = rospy.Rate(2)

##############angle_min/max  is in 'radian' ,,, ranges is in 'm'
        #print(left,center,right) minimum range value is in  [m]
        # maximum range value is in  [m]
###################################   

  
        ### publisher object  defined 
velocity0=Twist()
velocity1=Twist()
velocity2=Twist()
 

def velo():
    global controllast_free_0,controllast_free_1,controllast_free_2

    while not rospy.is_shutdown(): 
    
    # definging the subscriber
        rospy.Subscriber("tb3_0/odom",Odometry, newOdom_tb0)
        rospy.Subscriber("tb3_1/odom",Odometry, newOdom_tb1)
        rospy.Subscriber("tb3_2/odom",Odometry, newOdom_tb2)
        rospy.Subscriber("tb3_0/scan", LaserScan , surr_data_tb0)
        rospy.Subscriber("tb3_1/scan", LaserScan , surr_data_tb1)
        rospy.Subscriber("tb3_2/scan", LaserScan , surr_data_tb2)   



        ### finding the misalignment angle   and deltaij  
        dif_x0=x0-goal.x
        dif_y0=y0-goal.y
        err0=distance(dif_x0,dif_y0)
        angle_wrt_goal0=math.atan2(dif_y0,dif_x0)
        angle_wrt_goal0=(180/math.pi)*angle_wrt_goal0
        z0=180-(theta0-angle_wrt_goal0)
                         ### misalignment angle
        #print(x0)
        #print(dif_y0)
        #print(goal.y)
        #print(x0)
        #print(y0)
        

        dif_x1=x1-goal.x
        dif_y1=y1-goal.y
        err1=distance(dif_x1,dif_y1)
        angle_wrt_goal1=math.atan2(dif_y1,dif_x1)
        angle_wrt_goal1=(180/math.pi)*angle_wrt_goal1
        z1=180-(theta1-angle_wrt_goal1)
 
        dif_x2=x2-goal.x
        dif_y2=y2-goal.y
        err2=distance(dif_x2,dif_y2)
        angle_wrt_goal2=math.atan2(dif_y2,dif_x2)  
        angle_wrt_goal2=(180/math.pi)*angle_wrt_goal2
        z2=180-(theta2-angle_wrt_goal2)
        print(delta2)
        print(z2)

    #For Bot0
        if inside(p0, deltac):
            if controllast_free_0==0:
                controllast_free_0=1
                t_e_0=time.time()  
                           
            if delta0<0 and z0<0:
                t0= time.time()-t_e_0
                velocity0.linear.x= max(vi-l*t0,0)    
                velocity0.angular.z=-beta+K
            elif delta0>0 and z0<0:
                t0= time.time()-t_e_0
                velocity0.linear.x=max(vi-l*t0,0)
                velocity0.angular.z=-beta-K
            elif delta0<0 and z0>0:
                t0= time.time()-t_e_0
                velocity0.linear.x=max(vi-l*t0,0)
                velocity0.angular.z=beta+K
            elif delta0>0 and z0>0:
                t0= time.time()-t_e_0
                velocity0.linear.x=max(vi-l*t0,0)
                velocity0.angular.z=beta-K
    

        elif err0<0.1:
            velocity0.linear.x=0
            velocity0.angular.z=0                                      
            
        else:
            controllast_free_0=0        
            if -1.5<z0 and z0<1.5:
                #print(z0)
                velocity0.linear.x=vi   # free subsystem linear v is constant
                velocity0.angular.z=0     # sgn misalignment angle is 0 so angular z is 0
        
            elif z0<-1.5:
                velocity0.linear.x=0    # free subsystem linear v is constant
                velocity0.angular.z=-k     # sgn misalignment angle is -1 so angular z is -k
        
            elif z0>1.5:
                velocity0.linear.x=0    # free subsystem linear v is constant
                velocity0.angular.z=k    # sgn misalignment angle is 1 so angular z is k
    
    


    #For bot 1
        if inside(p1, deltac):
            if controllast_free_1==0:
                controllast_free_1=1
                t_e_1=time.time()
                #print(t_e_1)         
            if delta1<0 and z1<0:
                t1= time.time()-t_e_1
                velocity1.linear.x= max(vi-l*t1,0)
                velocity1.angular.z=-beta+K
                
            elif delta1>0 and z0<0:
                t1= time.time()-t_e_1
                velocity1.linear.x=max(vi-l*t1,0)
                velocity1.angular.z=-beta-K
            elif delta1<0 and z1>0:
                t1= time.time()-t_e_1
                velocity1.linear.x=max(vi-l*t1,0)
                velocity1.angular.z=beta+K
            elif delta1>0 and z1>0:
                t1= time.time()-t_e_1
                velocity1.linear.x=max(vi-l*t1,0)
                velocity1.angular.z=beta-K
                                
        elif err1<0.1:
            velocity1.linear.x=0
            velocity1.angular.z=0                                                         
   
        else:
            controllast_free_1=0
            if -1.5<z1 and z1<1.50:
                velocity1.linear.x=vi    # free subsystem linear v is constant
                velocity1.angular.z=0     # sgn misalignment angle is 0 so angular z is 0
            
            elif z1<-1.5:
                velocity1.linear.x=vi    # free subsystem linear v is constant
                velocity1.angular.z=-k     # sgn misalignment angle is -1 so angular z is -k
            
            elif z1>1.5:
                velocity1.linear.x=vi    # free subsystem linear v is constant
                velocity1.angular.z=k    # sgn misalignment angle is 1 so angular z is k


    #For bot 2
        if inside(p2, deltac):
            if controllast_free_2==0:
                controllast_free_2=1
                t_e_2=time.time()
                #print(t_e_2)         
    
            if delta2<0 and z2<0:
                t2= time.time()-t_e_2
                velocity2.linear.x= max(vi-l*t2,0)
                velocity2.angular.z=-beta+K
            
            elif delta2>0 and z2<0:
                t2= time.time()-t_e_2
                velocity2.linear.x=max(vi-l*t2,0)
                velocity2.angular.z=-beta-K
            elif delta2<0 and z2>0:
                t2= time.time()-t_e_2    
                velocity2.linear.x=max(vi-l*t2,0)
                velocity2.angular.z=beta+K
            elif delta2>0 and z2>0:
                t2= time.time()-t_e_2    
                velocity2.linear.x=max(vi-l*t2,0)
                velocity2.angular.z=beta-K

 
        elif err2<0.1:
            velocity2.linear.x=0
            velocity2.angular.z=0                                                                           
                # engaged subsystem   
               
       
        else:
            controllast_free_2=0
            if -1.5<z2 and z2<1.5:
                velocity2.linear.x=vi    # free subsystem linear v is constant
                velocity0.angular.z=0     # sgn misalignment angle is 0 so angular z is 0    
        
            elif z2<-1.5:
                velocity2.linear.x=vi    # free subsystem linear v is constant
                velocity2.angular.z=-k     # sgn misalignment angle is -1 so angular z is -k
        
            elif z2>1.5:
                velocity2.linear.x=vi    # free subsystem linear v is constant
                velocity2.angular.z=k    # sgn misalignment angle is 1 so angular z is k
        # publish/ print velocity
        print(velocity2)
        pub_0.publish(velocity0);
        pub_1.publish(velocity1);
        pub_2.publish(velocity2);
        #print(velocity0)
        rate.sleep()  

velo()
