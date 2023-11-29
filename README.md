# How to use

You should be able to import the project within unity. Testred with Unity 2022.3.14f1 under linux.

It makes use of an environment and prefabs provided by Udacity, under the MIT license, available at this repository : [https://github.com/udacity/self-driving-car-sim](https://github.com/udacity/self-driving-car-sim).

For testing it, you must 

1) Run the ros_tcp_endpoint which you need to clone/colcon build in your workspace from [this repository](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/main-ros2) 

	ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1

2) Start the JungleTrack scene

You should be able to view the image from within ros2 (e.g. rqt_image_view) and control the car by sending commands to `/command` topic. The command is a pair throttle/steering angle.

3) you can then control the simulation from within ROS driving_unity package :

	ros2 run driving_unity keyboard_teleop

and record the images and commands with :

	ros2 run driving_unity recorder

the last will dump the commands and frames in compressed npz files. The chunks all contain, by default, 500 frames/commands. For example :

	import numpy
	truc = numpy.load('chunk-00000.npz')
	import matplotlib.pyplot as plt
	img = truc['frames'][-1]
	plt.imshow(img[:,:,::-1])
	plt.show()

# If you were to do it from scratch

Open the project

- Add ROS-TCP-Connector from Window/Package Manager/Add from GIT url : https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector

- Customize the Robotics/ROS settings to use ROS2 ; Keep the default ROS ip address as 127.0.0.1 and port 10000

- In your ROS2 workspace , you must clone the repository

- Then run the endpoint : 

	ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1

- You can then drag and drop some ros2 publishers/subscribers. Usually with a top level RosConnector empty object to which the publisher/subscriber scripts are attached
