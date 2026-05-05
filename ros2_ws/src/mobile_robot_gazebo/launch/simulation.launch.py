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

    ## Switch to YAML file
    #bridge = Node(
    #    package="ros_gz_bridge",
    #    executable="parameter_bridge",
    #    arguments=[
    #    "/model/mobile_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
    #    "/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
    #    "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
    #    "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
    #    "/model/mobile_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
    #    "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
    #    "/model/mobile_robot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
    #    ],
    #    remappings=[
    #        ("/model/mobile_robot/tf", "/tf"),
    #        ("/model/mobile_robot/odometry", "/odom"),
    #    ]
    #)

    ## Add a static transform between the LiDAR and the robot basefootprint
    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.22", "0.05", "0.10",  # x, y, z
            "0.0", "0.0", "0.0",  # roll, pitch, yaw
            "mobile_robot/base_footprint",
            "mobile_robot/base_footprint/lidar",
        ],
    )

    ##rviz_node = Node(
    ##    package='rviz2',
    ##    executable='rviz2',
    ##    name='rviz2',
    ##    output='screen',
    ##    arguments=['-d', LaunchConfiguration('rvizconfig')],
    ##)

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        #bridge,
        lidar_tf,
        ##rviz_node,
    ])