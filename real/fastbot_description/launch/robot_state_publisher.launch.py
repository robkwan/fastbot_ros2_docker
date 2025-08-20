import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction

from launch.logging import get_logger # Import for logging

_logger = get_logger('robot_state_publisher_launch') # Get a logger instance

def launch_setup(context, *args, **kwargs):
    
    ####### DATA INPUT ##########
    robot_name = LaunchConfiguration("robot_name").perform(context)
    robot_file = LaunchConfiguration("robot_file").perform(context)

        # --- Verification part ---
    # Using print (less ideal for production, but quick for debugging)
    #print(f"DEBUG: robot_name received: {robot_name}")
    #print(f"DEBUG: robot_file received: {robot_file}")

    # Using ROS 2 standard logging (preferred)
    #_logger.info(f"INFO: robot_name received: {robot_name}")
    #_logger.info(f"INFO: robot_file received: {robot_file}")
    # --- End Verification part ---

    robot_description_topic_name = "/" + robot_name + "_robot_description"
    robot_state_publisher_name = robot_name + "_robot_state_publisher"
    joint_state_topic_name = "/" + robot_name + "/joint_states"

    package_description = "fastbot_description"

    robot_desc_path = os.path.join(
        get_package_share_directory(package_description), "onshape/", robot_file
    )
    # Load XACRO file with ARGUMENTS
    robot_desc = xacro.process_file(
        robot_desc_path, mappings={"robot_name": robot_name}
    )

    #print(f"robot_desc: {robot_desc}")

    xml = robot_desc.toxml()
    #print(f"xml: {xml}")


    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=robot_state_publisher_name,
        emulate_tty=True,
        parameters=[{"use_sim_time": True, "robot_description": xml}],
        remappings=[
            ("/robot_description", robot_description_topic_name),
            ("/joint_states", joint_state_topic_name),
        ],
        output="screen",
    )

    return [robot_state_publisher_node]


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument("robot_name", default_value="fastbot")
    robot_file_arg = DeclareLaunchArgument("robot_file", default_value="fastbot_multi_sim.xacro")

    return LaunchDescription(
        [robot_name_arg, robot_file_arg, OpaqueFunction(function=launch_setup)]
    )

