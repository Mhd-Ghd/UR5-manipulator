#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import copy
# import pynput
# from pynput import keyboard
#!/usr/bin/python
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



# Create the topic message
global traj
traj = JointTrajectory()
traj.header = Header()
global wp
wp=0;
waypoints=[
    [0.0, -1.57, 0.0, 0.0, 0.0, 0.0],
    [-0.19834809 ,-2.71798325 , 1.1418184 , -2.62115693 ,-1.47211123 ,-2.96925616],
    [-0.19834809 ,-1.62762785 ,-1.1418184 , -1.4278754  ,-1.47211123 ,-2.96925616]
]
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

    manual =True;

    while not rospy.is_shutdown():
        traj = prepareMsg()
        pub.publish(traj)

        if(manual):
            command=raw_input("[Manual]: Enter (t)toggle, or (p) to input a pose list:")
            if(command=='t'):
                manual= not manual
            else:
                
                while(True):
                    input_string = raw_input('Enter end-effector pose seperated by spaces : ')
                    print("\n")
                    user_list = input_string.split()

                    if(user_list[0]=='t'):
                        manual=not manual
                        wp=0
                        break
                    else:
                        for i in range(len(user_list)):
                            # convert each item to int type
                            user_list[i] = float(user_list[i])
                        
                        dp= [[ 3.06161700e-17, 8.66025404e-01, -5.00000000e-01, 0.4470],
                            [-1.00000000e+00, 0.00000000e+00, -5.55111512e-17, -0.2235],
                            [-5.55111512e-17, 5.00000000e-01, 8.66025404e-01, 0.8661],
                            [ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.0008900]]
                        dp[0][3]=user_list[0]
                        dp[1][3]=user_list[1]
                        dp[2][3]=user_list[2]

                        np.matrix(dp)

                        for i in range(0,1):
                            joints=invKine(dp)
                            jlist = list(chain.from_iterable(joints[:,i].tolist()))
                            print(jlist)
                            waypoints.append(jlist)

                       

                    
                    traj = prepareMsg()
                    pub.publish(traj)

        else:
            # for way in waypoints:
            #     for j in way:
            #         print(j)
            #     print("\n")

            command=raw_input("[waypoints-mode]: Enter (t)toggle, (n) for next waypoint, (p) for previous waypoint:")
            if(command=='t'):
                manual= not manual
            elif (command=='n'):
                if (wp<len(waypoints)-1):
                    wp+=1
            elif (command=='p'):
                if(wp>0):
                    wp-=1




            
        

        

    
# def on_press(key):
#     global wp
#     if key == keyboard.Key.up:
#         wp+=1;
    # elif key == keyboard.Key.down:
    #     lin_vel = lin_vel-0.1
    # elif key == keyboard.Key.left:
    #     ang_vel = ang_vel+0.05
    # elif key == keyboard.Key.right:
    #     ang_vel = ang_vel-0.05
    # elif key == keyboard.Key.space:  
    #     lin_vel = 0
    #     ang_vel = 0  
    # elif key == keyboard.Key.tab:
    #     print('mode switch')
    #     calibrate= not calibrate
  



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
