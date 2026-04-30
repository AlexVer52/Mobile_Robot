from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_description_path = PathJoinSubstitution([
        FindPackageShare("mobile_robot_description"),
        "urdf",
        "mobile_robot.urdf.xacro",
    ])
    
    robot_description = {
        "robot_description": Command([
            "xacro ",
            robot_description_path,
        ])
    }

    world_path = PathJoinSubstitution([
        FindPackageShare("mobile_robot_gazebo"),
        "worlds",
        "empty.world",
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ]),
        launch_arguments={
            "gz_args": world_path,
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "mobile_robot",
            "-z", "0.0",
        ],
        output="screen",
    )

    cmd_vel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
        "/model/mobile_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
    ],)

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        cmd_vel_bridge,
    ])