# My Workspace - Robotics Software Engineer course Udacity
See source (src) folder to access the packages!
# Package ball_chaser:
This package is related to first and second course projects. The project tasks list is placed at each root package folder.
The ROS structure is based on two nodes:
- ball_chaser/process_image.cpp
Subscribe to the camera image topic in order to identify the white ball at the frame. When it is detected, the process_image_callback function calculates the ball's position at the frame and it becomes the Output Variable in the feedback control structure. It was developed a Proportional and Integral controller for both linear and angular movements.
- ball_chaser/drive_bot.cpp
This node is responsible for creating a ROS service and call the handle_drive_request function to move the robot towards its requested linear and angular direction.
# Package my_robot:
This package includes mainly the urdf model for the robot. It was written from scratch based on the course lessons. It imports two open-source mesh models (one for the Lidar sensor, other for the camera visual). Besides that, this package contains the launch file that provide instructions for launching the Rviz, ROS Packages and Gazebo simultaneously. 
# Package simple_arm:
This package contains the codes used in the course lessons related to subscribing and Publishing at topics, using a gazebo world template. 
- simple_arm/look_away.cpp
The node look_away.cpp moves automatically towards the dices when the camera detects a boting image (such as the sky of the simulation environment).
- simple_arm/simple_mover
This node just create a ROS server to command both joint1 and joint2 movements.

