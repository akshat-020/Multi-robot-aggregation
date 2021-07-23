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
vi=0.1
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
goal.x=4.00
goal.y=4.00


#p=LaserScan().ranges
p0=LaserScan().ranges
p1=LaserScan().ranges
p2=LaserScan().ranges
#inview=[]


for x in range(360):
    p0.append(0)
#print(len(p0))
## control variable , we switch system and change its vallue
controllast_free_0=0
t_e_0=0
x0=0
y0=0
theta0=0
#delta=0
delta0=0

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
    deltaij=((lc*180)/(math.pi))*minvalue_of_ranges
    return deltaij


def distance(x,y):
    dist=math.sqrt(x**2+y**2) 
    return dist


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

def surr_data_tb0(data0):
        global delta0,minvalue_of_ranges0,p0,inview
       
        lc0=data0.angle_increment
        p0=np.array(data0.ranges)
        # omit the inf value
        for g in range(len(p0)):
            if p0[g]==float('inf'):
                p0[g]=3.5
            elif math.isnan(p0[g]):
                p0[g]=0.05
            else:
                continue
        
        #print(type(inview))	
        #print(len(p0))
        #p0=np.concatenate((p0[-35:-1],p0[0:35]),axis=None)
        minvalue_of_ranges0=np.argmin(np.concatenate((p0[-85:-1],p0[0:85]),axis=None))-85    ## have to find the index of the minimum value then find the degree deltaij
       # print(minvalue_of_ranges0)

        delta0=caldelta(lc0,minvalue_of_ranges0)
        
############################
rospy.init_node('vel0')
pub_0 = rospy.Publisher('tb3_0/cmd_vel', Twist,queue_size=1)
#pub_1 = rospy.Publisher('tb3_1/cmd_vel', Twist,queue_size=1)
#pub_2 = rospy.Publisher('tb3_2/cmd_vel', Twist,queue_size=1)
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
    global controllast_free_0

    while not rospy.is_shutdown(): 
    
    # definging the subscriber
        rospy.Subscriber("tb3_0/odom",Odometry, newOdom_tb0) 
        rospy.Subscriber("tb3_0/scan", LaserScan , surr_data_tb0)
        

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
    

        #elif err0<0.1:
         ##   velocity0.linear.x=0
           # velocity0.angular.z=0                                      
            
        else:
            controllast_free_0=0        
            if -1.5<z0 and z0<1.5:
                print(z0)
                velocity0.linear.x=vi   # free subsystem linear v is constant
                velocity0.angular.z=0     # sgn misalignment angle is 0 so angular z is 0
        
            elif z0<-1.5:
                velocity0.linear.x=vi    # free subsystem linear v is constant
                velocity0.angular.z=-k     # sgn misalignment angle is -1 so angular z is -k
        
            elif z0>1.5:
                velocity0.linear.x=vi   # free subsystem linear v is constant
                velocity0.angular.z=k    # sgn misalignment angle is 1 so angular z is k
    
    

        pub_0.publish(velocity0);
        #print(velocity0)
        rate.sleep()  

velo()

