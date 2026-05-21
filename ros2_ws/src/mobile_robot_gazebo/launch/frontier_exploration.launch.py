from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    frontier_explorer = Node(
        package="mobile_robot_gazebo",
        executable="frontier_explorer.py",
        name="frontier_explorer",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        frontier_explorer
    ])