#!/usr/bin/python
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import numpy as np 
import math 
import cmath
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi
from numpy import linalg
from itertools import chain

###############################################################[UR5-Manipulator]####################################################################
"""This script aims to manipulate the UR5 using a position input list and it applies Inverse Kinmatics to the the input list in order to find the 
    coresponding joints values and then it operates by publishing the joint commands to the '/trajectory_controller/command' in order to move the 
    simulated arm. The script runs in two different modes:
    
    (1) Control mode:
        In this mode the the program waits for a list of the desired position in the form of <x,y,z>. 
        The points are entered one by one. After entering each point, The inverse kinmatics is applied to the entered point
        to calculate the joint values and appends all the possible solutions (same desired position but with different joints configurations) 
        to the Waypointlist.

    (2) Operational Mode:
        In this mode we can traverese the Waypoint list and publish joints values to the topic '/trajectory_controller/command'
        traveresing the list is done by using (n) for the next waypoint or (p) for previuos waypoint. 

    Changing between the modes is done by entering (t) to toggle between the modes. 
"""






####################################################################[init]#############################################################################
""" In this part the JointTrajectory() message is declared globally to be used prepareMsg() to prepare the message that should be sent to the 
    the '/trajectory_controller/command'. The Waypoint list is initilized to have the home position with joint values [0.0, -1.57, 0.0, 0.0, 0.0, 0.0].
    This list is modified by operating in the Control mode where it appends the other waypoints to this list. 
"""
global traj
traj = JointTrajectory()
traj.header = Header()
global wp
wp=0;
waypoints=[
    [0.0, -1.57, 0.0, 0.0, 0.0, 0.0],
]





###############################################################[ROS-Joint-Values-Publisher]####################################################################
""" the declared JointTrajectory() message is used here at first to preapre the message by initilizing the message fields. The prepared message fields 
    include the list of the joint values which is set the Waypoint[Wp] where the Wp is used to index the waypoins (incremented by entering (n) for next waypoint
    , or decremented by entering (p) for the previous waypoint).
"""


def prepareMsg():
    global wp
    global waypoints

    traj.header.stamp = rospy.Time.now()
    pts = JointTrajectoryPoint()
    pts.time_from_start = rospy.Duration(1.0)

    
    pts.positions = waypoints[wp];
    
       

    # Set the points to the trajectory
    traj.points = []
    traj.points.append(pts)

    return traj



###############################################################[UR5-Manipulator]###########################################################################
""" The main loop of the script starts by initlizing the ROS node send_joints and creating a publisher to the topic '/trajectory_controller/command'
    it completes the fields for the of the 'traj' message, where the remaining fields is set during the message prepartion in the prepareMsg().
    The script starts in the Control Mode. where to options are availble. The first is enter a position list in the form of <x,y,z> (i.e. 0.2 0.1 0.2) 
    sperated by spaces. To start moving the robot enter (t) to toggle the mode to the Operational mode. Where (n) and (p) are used to move between next 
    and previous the waypoints. NOTE: The Waypoints added to the list include all the possible configuration for the robotic arm.

"""



def main():
    global traj
    global wp
    global waypoints
    rospy.init_node('send_joints')
    pub = rospy.Publisher('/trajectory_controller/command',
                          JointTrajectory,
                          queue_size=1)

    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint', 'wrist_1_joint', 'wrist_2_joint','wrist_3_joint']
    rate = rospy.Rate(10)
    manual =True;  #True: Control Mode, False: Operational mode 





    while not rospy.is_shutdown():
        traj = prepareMsg() #prepare the Ros message
        pub.publish(traj)   #publish the Ros message

        if(manual):
            command=raw_input("[Manual]: Enter (t)toggle, or (p) to input a pose list:")
            if(command=='t'):  #toggle the mode
                manual= not manual
            else:
                
                while(True):
                    input_string = raw_input('Enter end-effector pose seperated by spaces : ') #input in the for of x y z
                    print("\n")
                    user_list = input_string.split()

                    if(user_list[0]=='t'):  #check if a toggle request made
                        manual=not manual
                        wp=0
                        break
                    else:                   #take the pose list and apply inverse kinmatics to it



                        for i in range(len(user_list)):
                            # convert each item to int type
                            user_list[i] = float(user_list[i])
                        
                        #the dp is the desired position, the last column is for the pos, the others are for the arm configuration 
                        dp= [[ 3.06161700e-17, 8.66025404e-01, -5.00000000e-01, 0.0],
                            [-1.00000000e+00, 0.00000000e+00, -5.55111512e-17, 0.0],
                            [-5.55111512e-17, 5.00000000e-01, 8.66025404e-01, 0.0],
                            [ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.0]]

                        #append the user pose
                        dp[0][3]=user_list[0]
                        dp[1][3]=user_list[1]
                        dp[2][3]=user_list[2]

                        np.matrix(dp)
                        


                        #apply the inverse kinmatic to the the list, then appends all the eight possible solutions
                        for i in range(0,8):
                            joints=invKine(dp)
                            jlist = list(chain.from_iterable(joints[:,i].tolist()))
                            print(jlist)
                            waypoints.append(jlist)

                       

                    #prepare and publish the ROS message
                    traj = prepareMsg()
                    pub.publish(traj)

        else:       #Operational mode

            command=raw_input("[waypoints-mode]: Enter (t)toggle, (n) for next waypoint, (p) for previous waypoint:")
            if(command=='t'): #to toggle the mode
                manual= not manual
            elif (command=='n'): #to next waypoint
                if (wp<len(waypoints)-1):
                    wp+=1
            elif (command=='p'): #to previuos waypoint
                if(wp>0):
                    wp-=1






###############################################################[Inverse-Kinamtics]####################################################################
"""The Implementation of the inverse kinmatics for the UR5 arm, This Implementation is done by @mc-capolei with slight modifications to match the UR5.
    please refer to the following PDF file for both the Forward and Inverse kinamtics for the UR5:
    (https://smartech.gatech.edu/bitstream/handle/1853/50782/ur_kin_tech_report_1.pdf)
    refere to the original code by @mc-capolei (https://github.com/mc-capolei/python-Universal-robot-kinematics).
"""

global mat
mat=np.matrix

global d1, a2, a3, a7, d4, d5, d6
d1 =  0.1273
a2 = -0.612
a3 = -0.5723
a7 = 0.075
d4 =  0.163941
d5 =  0.1157
d6 =  0.0922
a =mat([0 ,-0.425 ,-0.39225 ,0 ,0 ,0])
d = mat([0.089159, 0, 0, 0.10915, 0.09465, 0.0823])
alph = mat([math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0 ])
def invKine(desired_pos):# T60
  th = mat(np.zeros((6, 8)))
  P_05 = (desired_pos * mat([0,0, -d6, 1]).T-mat([0,0,0,1 ]).T)
  
  # **** theta1 ****
  
  psi = atan2(P_05[2-1,0], P_05[1-1,0])
  phi = acos(d4 /sqrt(P_05[2-1,0]*P_05[2-1,0] + P_05[1-1,0]*P_05[1-1,0]))
  #The two solutions for theta1 correspond to the shoulder
  #being either left or right
  th[0, 0:4] = pi/2 + psi + phi
  th[0, 4:8] = pi/2 + psi - phi
  th = th.real
  
  # **** theta5 ****
  
  cl = [0, 4]# wrist up or down
  for i in range(0,len(cl)):
	      c = cl[i]
	      T_10 = linalg.inv(AH(1,th,c))
	      T_16 = T_10 * desired_pos
	      th[4, c:c+2] = + acos((T_16[2,3]-d4)/d6);
	      th[4, c+2:c+4] = - acos((T_16[2,3]-d4)/d6);

  th = th.real
  
  # **** theta6 ****
  # theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.

  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
	      c = cl[i]
	      T_10 = linalg.inv(AH(1,th,c))
	      T_16 = linalg.inv( T_10 * desired_pos )
	      th[5, c:c+2] = atan2((-T_16[1,2]/sin(th[4, c])),(T_16[0,2]/sin(th[4, c])))
		  
  th = th.real

  # **** theta3 ****
  cl = [0, 2, 4, 6]
  for i in range(0,len(cl)):
	      c = cl[i]
	      T_10 = linalg.inv(AH(1,th,c))
	      T_65 = AH( 6,th,c)
	      T_54 = AH( 5,th,c)
	      T_14 = ( T_10 * desired_pos) * linalg.inv(T_54 * T_65)
	      P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T
	      t3 = cmath.acos((linalg.norm(P_13)**2 - a2**2 - a3**2 )/(2 * a2 * a3)) # norm ?
	      th[2, c] = t3.real
	      th[2, c+1] = -t3.real

  # **** theta2 and theta 4 ****

  cl = [0, 1, 2, 3, 4, 5, 6, 7]
  for i in range(0,len(cl)):
	      c = cl[i]
	      T_10 = linalg.inv(AH( 1,th,c ))
	      T_65 = linalg.inv(AH( 6,th,c))
	      T_54 = linalg.inv(AH( 5,th,c))
	      T_14 = (T_10 * desired_pos) * T_65 * T_54
	      P_13 = T_14 * mat([0, -d4, 0, 1]).T - mat([0,0,0,1]).T
	      
	      # theta 2
	      th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(a3* sin(th[2,c])/linalg.norm(P_13))
	      # theta 4
	      T_32 = linalg.inv(AH( 3,th,c))
	      T_21 = linalg.inv(AH( 2,th,c))
	      T_34 = T_32 * T_21 * T_14
	      th[3, c] = atan2(T_34[1,0], T_34[0,0])
  th = th.real

  return th

def AH( n,th,c  ):

  T_a = mat(np.identity(4), copy=False)
  T_a[0,3] = a[0,n-1]
  T_d = mat(np.identity(4), copy=False)
  T_d[2,3] = d[0,n-1]

  Rzt = mat([[cos(th[n-1,c]), -sin(th[n-1,c]), 0 ,0],
	         [sin(th[n-1,c]),  cos(th[n-1,c]), 0, 0],
	         [0,               0,              1, 0],
	         [0,               0,              0, 1]],copy=False)
      

  Rxa = mat([[1, 0,                 0,                  0],
			 [0, cos(alph[0,n-1]), -sin(alph[0,n-1]),   0],
			 [0, sin(alph[0,n-1]),  cos(alph[0,n-1]),   0],
			 [0, 0,                 0,                  1]],copy=False)

  A_i = T_d * Rzt * T_a * Rxa
	    

  return A_i





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
