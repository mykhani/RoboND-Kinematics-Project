#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

### Function for extracting a small matrix from large one
def submatrix( matrix, startRow, startCol, size):
    return matrix[startRow:startRow+size,startCol:startCol+size]

### Generic function for creating Homogenous Transform Matrix
def homogeneous_transform(q, d, a, alpha):
    Transform = Matrix([[            cos(q),           -sin(q),           0,             a],
                        [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                        [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                        [                 0,                 0,           0,             1]])
    return Transform

class RobotTransform:
    def init(self):
	### Create symbols for joint variables
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') 
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        
        ### Kuka KR210 ###
        # DH Parameters
        DH = { alpha0:      0,  a0:      0,  d1:   0.75,  q1:  q1,
               alpha1:  -pi/2,  a1:   0.35,  d2:      0,  q2:  q2-pi/2,  
               alpha2:      0,  a2:   1.25,  d3:      0,  q3:  q3,
               alpha3:  -pi/2,  a3: -0.054,  d4:    1.5,  q4:  q4,
               alpha4:   pi/2,  a4:      0,  d5:      0,  q5:  q5,
               alpha5:  -pi/2,  a5:      0,  d6:      0,  q6:  q6,
               alpha6:      0,  a6:      0,  d7:  0.303,  q7:  0}
        
        # Homogenous transforms for individual joints
        T0_1 = homogeneous_transform(q1, d1, a0, alpha0).subs(DH)
        T1_2 = homogeneous_transform(q2, d2, a1, alpha1).subs(DH)
        T2_3 = homogeneous_transform(q3, d3, a2, alpha2).subs(DH)
        T3_4 = homogeneous_transform(q4, d4, a3, alpha3).subs(DH)
        T4_5 = homogeneous_transform(q5, d5, a4, alpha4).subs(DH)
        T5_6 = homogeneous_transform(q6, d6, a5, alpha5).subs(DH)
        T6_G = homogeneous_transform(q7, d7, a6, alpha6).subs(DH)
        
        print("Computed individual transforms")
        
        # Composition of Homogeneous Transforms from base link to link_3
        T0_3 = simplify(T0_1 * T1_2 * T2_3)
        # Extract 3x3 Rotation matrix from Homogeneous Transform matrix for
        # base_link to link_3
        self.R0_3 = submatrix(T0_3, 0 , 0, 3)
        
        print("Computed composition of transforms")
        
        r, p, y = symbols('r p y')
        
        Rot_x = Matrix([[       1,       0,        0],
                        [       0,  cos(r),  -sin(r)],
                        [       0,  sin(r),   cos(r)]])  # Roll

        Rot_y = Matrix([[  cos(p),       0,   sin(p)],
                        [       0,       1,        0],
                        [ -sin(p),       0,   cos(p)]])  # Pitch

        Rot_z = Matrix([[  cos(y), -sin(y),       0],
                        [  sin(y),  cos(y),       0],
                        [       0,       0,       1]])   # Yaw
        
        R_corr = Rot_z.subs(y, pi) * Rot_y.subs(p, -pi/2)
        self.Rrpy = Rot_z * Rot_y * Rot_x * R_corr

    
    def getGripperOrientation(self, roll, pitch, yaw):
        # put in angles and return matrix
        R_G = self.Rrpy.subs({'r': roll, 'p': pitch, 'y': yaw})
        return R_G

    def getWristCenterOrientation(self, theta1, theta2, theta3):
        R_W = self.R0_3.subs({'q1': theta1, 'q2': theta2, 'q3': theta3})
        return R_W
        

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
       # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    #
            Rrpy = robotTransform.getGripperOrientation(roll, pitch, yaw)
            
            # End-effector position
            EE = Matrix([[px],
                         [py],
                         [pz]])

            # Calculating wrist-center position. 0.303 is the distance between
            # origin of end-effector and origin of wrist-center
            WC = EE - 0.303 * Rrpy[:,2]

	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            theta1 = atan2(WC[1], WC[0])

            # SSS triangle for theta2 and theta3
            side_a = 1.50
            side_b = sqrt(pow(sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35, 2) + pow(WC[2] - 0.75, 2))
            side_c = 1.25

            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
            angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))
            
            theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
            theta3 = pi/2 - angle_b - atan2(0.054, 1.5) 


            R_WC = robotTransform.getWristCenterOrientation(theta1, theta2, theta3)
            #R3_6 = R_WC.inv("LU") * R_EE            
            R3_6 = R_WC.T * Rrpy            
            
            ### Euler Angles from Rotation Matrix
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5  = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    robotTransform = RobotTransform()
    robotTransform.init()
    IK_server()
