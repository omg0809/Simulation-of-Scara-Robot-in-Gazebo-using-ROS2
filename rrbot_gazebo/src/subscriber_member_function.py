# Subscriber for RBE500 Group Assignment Part 1 Question 2
# Author: Marissa Langille
# 11/10/22
# Takes in the joint variables [q1, q2, and q3] from the gazebo simulator and solves the forward kinematics of the manipulator.The topic it subscribes to is joint_states. The simulator publishes a JointState message from the sensor_msgs library. The message has a float64 array for the position, which is used to isolate each joint variable. 
# Each joint variable is either the angle of rotation (revolute) or dispacement (prismatic)
# In this problem, there are two revolute joints and one prismatic joint: q1 = theta1, q2 = theta2, and q3 = d3
# This node calculates the end effector pose and publishes the pose to a ROS topic

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np # Scientific computing library
import math as m

class End_effector_pose(Node):
# Creates node to run subscriber

    # Subscriber listens for an array of the joint variables [q1,q2,q3] on topic. It then goes to listener_callback

    def __init__(self):
        super().__init__('end_effector_pose')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
    	# Extract the joint variables from the array
    	q1 = msg.position[0] # theta 1, in radians
    	q2 = msg.position[1] # theta 2, in radians
    	q3 = msg.position[2] # d3
    	
    	# Define link lengths
    	l1a = 0.9
    	l1d = 2
    	l2 = 1
    	l3 = 0.25
    	
    	# Solve forward kinematics of the manipulator. Define DH Parameters:
    	a1 = l1a
    	theta1 = q1
    	d1 = l1d
    	alpha1 = 0
    	
    	a2 = l2
    	theta2 = q2
    	d2 = 0
    	alpha2 = 0
    	
    	a3 = 0
    	theta3 = 0
    	d3 = q3-l3
    	alpha3 = 0
    	
    	# Calculate Tranformation matrix of the end-effector:
    	
    	T = np.array([[(np.cos(theta1)*np.cos(theta2)-np.sin(theta1)*np.sin(theta2)),(-1)*np.cos(theta1)*np.sin(theta2)-np.cos(theta2)*np.sin(theta1),0,l1a*np.cos(theta1)+l2*np.cos(theta1)*np.cos(theta2)-l2*np.sin(theta1)*np.sin(theta2)],
    	[np.cos(theta1)*np.sin(theta2)+np.cos(theta2)*np.sin(theta1),np.cos(theta1)*np.cos(theta2)-np.sin(theta1)*np.sin(theta2),0,l1a*np.sin(theta1)+l2*np.cos(theta1)*np.sin(theta2)+l2*np.cos(theta2)*np.sin(theta1)],
    	[0,0,1,d3+l1d],
    	[0,0,0,1]])
    	
    	#print('['+'\n'.join([''.join(['{:25}'.format(item) for item in row]) for row in T])+']')
    	self.get_logger().info('"%s"' % (T))
    	#self.get_logger().info(print('['+'\n'.join([''.join(['{:25}'.format(item) for item in row]) for row in T])+']'))

def main(args=None):
    rclpy.init(args=args)
    
    end_effector_pose = End_effector_pose()

    rclpy.spin(end_effector_pose)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    end_effector_pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
