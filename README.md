# Simulation-of-Scara-Robot-in-Gazebo-using-ROS2
RBE-500 Final Project

Implemented a forward kinematics node that
• Subscribes to the joint values topic and reads them from the gazebo simulator
• Calculate the end effector pose
• publishes the pose as a ROS topic 

Implemented an inverse kinematics node (a separate node) that has a
service client that takes a (desired) pose of the end effector from the user and returns joint
positions as a response.

![image](https://user-images.githubusercontent.com/98101801/209718265-f13efacc-07a3-43cd-97fc-30c86a7301b5.png)


Implemented a position controller for the robot joints. Here we wrote PD Controllers from scratch. The package will read the joint values from the Gazebo simulator, receive a reference value for the joints through a service, and publish joint efforts (continuously with high sampling rates) to make the joints move to these locations.

For three different sets of joint position references, recorded the reference positions and current positions of the joints in a text file for a period of time and plotted them in MATLAB.

![image](https://user-images.githubusercontent.com/98101801/209718424-90acef3a-db14-40e2-a9b2-4e7dbe46d088.png)

![image](https://user-images.githubusercontent.com/98101801/209718475-cc150594-4176-4bd3-988e-f0896b9704c5.png)

Velocity Level Kinematics: 

Implemented a node with two services. One takes joint velocities and converts them to end effector velocities, and the second one takes end effector velocities and converts them to joint velocities.

Velocity controllers from scratch and provided a constant velocity reference in the positive ‘y’ direction of the Cartesian space. Converted this velocity to the joint space velocities using your Jacobian and feed it as a reference to your velocity controllers. This should moves the robot on a straight line in the +y direction. Recorded the generated velocity references together with the actual velocity of the system over time, and plotted via Matlab.






https://user-images.githubusercontent.com/98101801/209719028-7f902d1a-70e3-4c87-a657-3f7a09f42b52.mp4

