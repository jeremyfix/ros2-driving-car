# How to

Open the project

- Add ROS-TCP-Connector from Window/Package Manager/Add from GIT url : https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector

- Customize the Robotics/ROS settings to use ROS2 ; Keep the default ROS ip address as 127.0.0.1 and port 10000

- In your ROS2 workspace , you must clone the repository

- Then run the endpoint : 

	ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
