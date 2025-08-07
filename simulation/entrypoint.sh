#! /bin/bash 

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

#prints each command and its arguments as they are executed for debugging
set -x

# Exit immediately if a command exits with a non-zero status
set -e

echo "$(date +'[%Y-%m-%d %T]') Starting Nginx server..."
nginx -g 'daemon off;' &

#xhost +local:root &

# Print current working directory
echo "Current working directory: $(pwd)"
echo "Container PATH: $PATH"
#Launch rosbridge server
echo "$(date +'[%Y-%m-%d %T]') Starting rosbridge server..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# Set the instance_id environment variable
#instance_id=$(curl -s http://169.254.169.254/latest/meta-data/instance-id)

#hard-coded here as it should be from rosbridge-suite build on host side originally!
#SLOT_PREFIX='4c220621-bf14-404c-a167-a341b4c48489'
#SLOT_PREFIX=$(grep 'SLOT_PREFIX=' ./ros2_ws/build/rosbridge_suite/colcon_command_prefix_build.sh.env | cut -d'=' -f2)
#echo "SLOT_PREFIX: $SLOT_PREFIX"

# Run the rosbridge_address command and capture the output
#export ROSBRIDGE_URL="wss://${instance_id}.robotigniteacademy.com/${SLOT_PREFIX}/rosbridge/"
#echo "Rosbridge URL: ${ROSBRIDGE_URL}"

#WEBPAGE_URL="https://${instance_id}.robotigniteacademy.com/${SLOT_PREFIX}/webpage/"
#echo "Webpage URL: ${WEBPAGE_URL}"

#chmod -R 666 /var/www/html
#chmod -R 666 /var/www/html/template_main.js
#chmod -R 666 /var/www/html/main.js

# Inject ROSBRIDGE_URL into main.js
#if [ -f /var/www/html/template_main.js ]; then
#    sed "s@{{ROSBRIDGE_URL}}@${ROSBRIDGE_URL}@g" /var/www/html/template_main.js > /var/www/html/main.js
#    echo "Configured main.js with ROSBRIDGE_URL."
#else
#    echo "Error: /var/www/html/main.js.template not found!"
#    exit 1
#fi

#Launch tf2_web_republisher_py
echo "$(date +'[%Y-%m-%d %T]') Starting tf2_web_republisher_py..."
ros2 run tf2_web_republisher_py tf2_web_republisher &

#Launch web video server
#echo "$(date +'[%Y-%m-%d %T]') Starting web video server..."
#ros2 run web_video_server web_video_server --ros-args -p port:=11315 &

# Keep the container running
echo "$(date +'[%Y-%m-%d %T]') All services started. Keeping container alive..."
tail -f /dev/null
