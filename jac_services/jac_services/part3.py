#RBE500 - Foundations of Robotics
#Group Assignment Part 3
from services_package.srv import Pose1
from services_package.srv import Pose2
from std_msgs.msg import Float64MultiArray
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Jacobian_Node(Node):
	global q1_ref, q2_ref, q3_ref, jac

	def __init__(self):
		#Initiliaze all variables to 0 and defining them global
		global q1_ref, q2_ref, q3_ref, jac,q1_pos,q2_pos,q3_pos,v1,v2,v3,e_prev1,e_prev2,e_prev3,old_t,eta,a,b,c
		q1_ref=0
		q2_ref=0	
		q3_ref=0
		q1_pos=0
		q2_pos=0
		q3_pos=0
		jac=np.zeros((3, 3))
		eta = np.matrix([0,0,0]) 
		a=0.0
		b=0.0
		c=0.0
		v1=0
		v2=0
		v3=0
		e_prev1=0
		e_prev2=0
		e_prev3=0
		old_t=0
		
		super().__init__('jacobian_node')
		self.subscription = self.create_subscription(JointState,'joint_states',self.vel_control,10)
		self.publisher_ = self.create_publisher(Float64MultiArray,'forward_effort_controller/commands',10)
		
		self.srv = self.create_service(Pose1, 'joint2end', self.joint2end)
		self.srv = self.create_service(Pose2, 'end2joint', self.end2joint)

    
	def joint2end(self, request, response):
		# Takes joint velocities, converts to end effector velocities
		global q1_ref, q2_ref, q3_ref,q1_pos,q2_pos
		# Values enterted by the user
		q1_ref = request.q1_ref
		q2_ref = request.q2_ref
		q3_ref = request.q3_ref
		self.get_logger().info('Incoming request\nx: %f y: %f z: %f' % (request.q1_ref, request.q2_ref, request.q3_ref))

		jac = self.jacobian(q1_pos, q2_pos)

		# Calculates end effector velocities 
		eta = jac*np.transpose(np.matrix([q1_ref,q2_ref,q3_ref])) 

		#Output end-effector velocities
		x = eta.item(0)
		y = eta.item(1)
		z = eta.item(2)
		ang_x = eta.item(3)
		ang_y = eta.item(4)
		ang_z = eta.item(5)

		response.x = x # velocity in cartesian coordinates
		response.y = y
		response.z = z

		response.ang_x = ang_x # angular velocity
		response.ang_y = ang_y
		response.ang_z  = ang_z

		return response

	def end2joint(self, request, response):
		global q1_ref, q2_ref, q3_ref,v1,v2,v3,q1_pos,q2_pos,q3_pos,eta,a,b,c

		# Takes end effector velocities, converts to joint velocities
		a=request.x
		b=request.y
		c=request.z
		eta = np.matrix([request.x,request.y,request.z]) 
		
		self.get_logger().info('Incoming request\nx: %f y: %f z: %f' % (a, b, c))
		self.get_logger().info('Incoming request\nang_x: %f ang_y: %f ang_z: %f' % (request.ang_x, request.ang_y, request.ang_z))

		#Defining variables for the inverse Jacobian matrix
		theta1 = q1_pos
		theta2 = q2_pos

		# This condition avoids singularity and take angle of joint2 as 0.01 when theta2 goes less than 0.01
		if abs(np.sin(theta2))<=0.01:
			theta2=0.01

		#Calculate Jaobian Inverse
		jac_inv = np.matrix([[(10*np.cos(theta1 + theta2))/(9*np.sin(theta2)),(10*np.sin(theta1 + theta2))/(9*np.sin(theta2)),0],
		[-(2*(5*np.cos(theta1 + theta2) + 9*np.cos(theta1)))/(9*np.sin(theta2)),-(2*(5*np.sin(theta1 + theta2) + 9*np.sin(theta1)))/(9*np.sin(theta2)),0],
		[0,0,1]])

		
        #Resultant joint velocities 
		q_dot = jac_inv*np.transpose(eta) 
		
	    #Respective joint velocites of each joint
		v1 = q_dot.item(0)
		v2= q_dot.item(1)
		v3= q_dot.item(2)
		
		response.v1 = v1
		response.v2 = v2
		response.v3 = v3			
		return response

	def read_joints_Jac(self,msg):
		global jac, true_q1, true_q2, true_q3, q1_pos, q2_pos, q3_pos
	
		true_q1 = msg.velocity[0] # theta 1 velocity
		true_q2 = msg.velocity[1] # theta 2 velocity
		true_q3 = msg.velocity[2] # d3 velocity
		
		# Reads in joint position values 
		q1_pos = msg.position[0] 
		q2_pos = msg.position[1] 
		q3_pos = msg.position[2] 

		jac = self.jacobian(q1_pos, q2_pos)

	def jacobian(self, q1_pos, q2_pos):
		# Calculates the Jacobian
		theta1 = q1_pos
		alpha1 = 0
		theta2 = q2_pos
		d3 = q3_pos
		
		#Link lenghts

		a1 = 0.9 # l1a
		d1 = 2 # l1d

		
		Z_0 = np.matrix([0, 0, 1])
		O_3 = np.matrix([0.9*np.cos(theta1)+0.5*np.cos(theta1)*np.cos(theta2)-0.5*np.sin(theta1)*np.sin(theta2),0.9*np.sin(theta1)+0.5*np.cos(theta1)*np.sin(theta2)+0.5*np.cos(theta2)*np.sin(theta1),d3-0.35+2])
		O_0 = np.matrix([0,0,0])
		Z_1 = np.matrix([(-1)*np.sin(theta1)*np.sin(alpha1),(-1)*np.cos(theta1)*np.sin(alpha1),np.cos(alpha1)])
		O_1 = np.matrix([a1*np.cos(theta1),a1*np.sin(theta1),d1])
		Z_2 = np.matrix([(-1)*np.cos(theta1)*np.sin(theta1)-np.cos(theta1)*np.sin(theta1),np.cos(theta1)*np.cos(theta2)-np.sin(theta1)*np.sin(theta2),0])

		z0_dot = np.cross(Z_0,O_3-O_0)
		z1_dot = np.cross(Z_1,O_3-O_1) 
		
		# Jacobian saved to global variable
		Jac = np.matrix([[z0_dot.item(0),z1_dot.item(0),Z_2.item(0)],\
			[z0_dot.item(1),z1_dot.item(1),Z_2.item(1)],\
				[z0_dot.item(2),z1_dot.item(2),Z_2.item(2)],\
					[Z_0.item(0),Z_1.item(0),O_0.item(0)],\
						[Z_0.item(1),Z_1.item(1),O_0.item(1)],\
							[Z_0.item(2),Z_1.item(2),O_0.item(2)]])

		return Jac

# Implementing velocity controller

	def vel_control(self,msg):
	
		global q1_ref,q2_ref,q3_ref,v1,v2,v3,e_prev1,e_prev2,e_prev3,jac_inv,old_t,true_q1,true_q2,true_q3,eta, q1_pos, q2_pos, q3_pos,a,b,c

		# Gain values for each joint
		kp1 = 10
		kp2 = 10
		kp3 = 10
		kd1 = 0.01
		kd2=  0.0001
		kd3 = 0.001
# Reading joint values
		self.read_joints_Jac(msg)
		theta1 = msg.position[0]
		theta2 = msg.position[1]
		true_q1 = msg.velocity[0] 
		true_q2 = msg.velocity[1] 
		true_q3 = msg.velocity[2] 

# Creating text file for matlab plots
        # Taking the end-effector velocities values in the file to compare them in the matlab  
		file1 = open("final.txt","a")
		L1 = str(a)+',' 
		L2 = str(b)+','
		L3 = str(c)+','
		file1.write(L1)
		file1.write(L2)
		file1.write(L3)
		

		# Adjusting the theta2 of joint 2 if  it comeas to a singularity condition while performing the trajectory
		if abs(np.sin(theta2))<=0.01:
			theta2=0.01

# Calculation the end-effector velocities        
		p=jac*np.transpose(np.matrix([true_q1,true_q2,true_q3]))

		end1_actual=p.item(0)
		end2_actual=p.item(1)
		end3_actual=p.item(2)

		L1_ref = str(end1_actual)+','
		L2_ref = str(end2_actual)+','
		L3_ref = str(end3_actual)+'\n'
		file1.write(L1_ref)
		file1.write(L2_ref)
		file1.write(L3_ref)  
		file1.close()
		
		# Taking the jacobian inverse to find the joint velocities which are used to calculate the error in the controller function

		jac_inv = np.matrix([[(10*np.cos(theta1 + theta2))/(9*np.sin(theta2)),(10*np.sin(theta1 + theta2))/(9*np.sin(theta2)),0],
		[-(2*(5*np.cos(theta1 + theta2) + 9*np.cos(theta1)))/(9*np.sin(theta2)),-(2*(5*np.sin(theta1 + theta2) + 9*np.sin(theta1)))/(9*np.sin(theta2)),0],
		[0,0,1]])

		t = msg.header.stamp.sec+(msg.header.stamp.nanosec/1000000000)
		q_dot = jac_inv*np.transpose(eta)
	 
		v1 = q_dot.item(0)
		v2= q_dot.item(1)
		v3= q_dot.item(2)
		
		# Controller error
		e1 = v1 - true_q1
		e2 = v2 - true_q2
		e3 = v3 - true_q3

        # delta error
		s1 = e1-e_prev1
		s2 = e2-e_prev2
		s3 = e3-e_prev3
        # e_prev is taken as the error generated in the previous iteration
		e_prev1 = e1
		e_prev2 = e2
		e_prev3 = e3

        #  change in time
		dt = t - old_t
		# Controller equation
		v1_contr = (kp1*e1)+((kd1*s1)/dt)
		v2_contr = (kp2*e2)+((kd2*s2)/dt)
		v3_contr = 9.8+ (kp3*e3)+((kd3*s3)/dt)

		#old time value assinged to t to get the previous time
		old_t=t
		
		# Publishing the values to effort controller
		msg = Float64MultiArray()
		msg.data = v1_contr,v2_contr,v3_contr
		self.publisher_.publish(msg)
		# print(msg.data)
		self.get_logger().info('v1, v2, v3: %f, %f, %f'%(v1_contr,v2_contr,v3_contr))
	
def main():
	rclpy.init()

	jacobian_node = Jacobian_Node()
	rclpy.spin(jacobian_node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()