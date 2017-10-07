#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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

def create_TM(alpha, a, d, q):
    T = Matrix([[            cos(q),           -sin(q),           0,             a],   
                [ cos(alpha)*sin(q), cos(alpha)*cos(q), -sin(alpha), -sin(alpha)*d],
                [ sin(alpha)*sin(q), sin(alpha)*cos(q),  cos(alpha),  cos(alpha)*d],
                [                 0,                 0,           0,             1]])

    return T

def rotation_matrix():
    r, p, y = symbols('r p y')

    R_x = Matrix([[   1,      0,       0],
                  [   0, cos(r), -sin(r)],
                  [   0, sin(r),  cos(r)]])

    R_y = Matrix([[ cos(p),  0, sin(p)],
                  [      0,  1,      0],
                  [-sin(p),  0, cos(p)]])

    R_z = Matrix([[ cos(y), -sin(y),  0],
                  [ sin(y),  cos(y),  0],
                  [      0,       0,  1]])

    R_G = R_z * R_y * R_x
    R_corr = R_z.subs(y, pi) * R_y.subs(p, -pi/2)
    R_G = R_G * R_corr
    
    return R_G

def find_first_joints(WC):
    a1, a2, a3 = 0.35, 1.25, -0.054
    d1, d4 = 0.75, 1.5

    theta1 = atan2(WC[1], WC[0])

    line_ab = a2 
    line_bc = sqrt(d4**2 + a3**2) 
    line_ca = sqrt((sqrt(WC[0]**2 + WC[1]**2) - a1)**2 + (WC[2] - d1)**2)

    angle_A = acos((line_ca**2 + line_ab**2 - line_bc**2) / (2 * line_ca * line_ab))
    angle_B = acos((line_bc**2 + line_ab**2 - line_ca**2) / (2 * line_bc * line_ab))

    gamma = atan2(WC[2] - d1, sqrt(WC[0]**2 + WC[1]**2) - a1)
    beta = atan2(d4, -a3)

    theta2 = pi/2 - angle_A - gamma
    theta3 = -(angle_B - beta)

    return theta1, theta2, theta3

def find_last_joints(R):
    theta6 = atan2(-R[1,1], R[1,0])
    theta5 = atan2(sqrt(R[0,2]**2 + R[2,2]**2), R[1,2])
    theta4 = atan2(R[2,2], -R[0,2])

    return theta4, theta5, theta6

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') 
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') 
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') 

        # Create Modified DH parameters
        dh_table = {alpha0:     0, a0:      0, d1:  0.75, q1:        q1,
                    alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2 - pi/2,
                    alpha2:     0, a2:   1.25, d3:     0, q3:        q3,
                    alpha3: -pi/2, a3: -0.054, d4:   1.5, q4:        q4,
                    alpha4:  pi/2, a4:      0, d5:     0, q5:        q5,
                    alpha5: -pi/2, a5:      0, d6:     0, q6:        q6,
                    alpha6:     0, a6:      0, d7: 0.303, q7:         0} 

        # Define Modified DH Transformation matrix

        T0_1 = create_TM(alpha0, a0, d1, q1).subs(dh_table)
        T1_2 = create_TM(alpha1, a1, d2, q2).subs(dh_table)
        T2_3 = create_TM(alpha2, a2, d3, q3).subs(dh_table)
        T3_4 = create_TM(alpha3, a3, d4, q4).subs(dh_table)
        T4_5 = create_TM(alpha4, a4, d5, q5).subs(dh_table)
        T5_6 = create_TM(alpha5, a5, d6, q6).subs(dh_table)
        T6_G = create_TM(alpha6, a6, d7, q7).subs(dh_table)

        # Create individual transformation matrices
        T0_3 = T0_1 * T1_2 * T2_3
        T0_G = T0_3 * T3_4 * T4_5 * T5_6 * T6_G
        
        # Extract rotation matrices from the transformation matrices
        R_G = rotation_matrix()

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
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
            R_G = R_G.subs({'r': roll, 'p': pitch, 'y': yaw}) 
            EE = Matrix([[px], [py], [pz]])
            WC = EE - 0.303 * R_G[:,2]
            
	    # Calculate joint angles using Geometric IK method
            theta1, theta2, theta3 = find_first_joints(WC)
            R0_3 = T0_3[:3,:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.inv('LU') * R_G

            theta4, theta5, theta6 = find_last_joints(R3_6)

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
    IK_server()
