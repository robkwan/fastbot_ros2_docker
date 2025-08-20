# fastbot_bringup

This ROS2 package is for the bringup of the ROS2 drivers for Fasbot robot car.

To use this:-

1. git clone https://github.com/robkwan/fastbot_bringup.git under the local ~/ros2_ws/src folder.

2. then inside ~/ros2_ws folder, execute "colcon build" to build all the files.

3. then "source install/setup.bash" in the ~/ros2_ws folder.

4. then "ros2 launch fastbot_bringup bringup.launch.xml" will launch all the ROS2 drivers for Fastbot.

5. then "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/fastbot/cmd_vel" in another terminal
   or check the related ros2 topic for Fastbot with "ros2 topic list" for different operations.

