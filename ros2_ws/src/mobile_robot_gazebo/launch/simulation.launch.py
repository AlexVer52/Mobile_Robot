import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PythonExpression, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_bridge.actions import RosGzBridge
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory("mobile_robot_gazebo")
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge.yaml')
    world = LaunchConfiguration("world")
    mode = LaunchConfiguration("mode")
    map_file = LaunchConfiguration("map")

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
        world,
    ])

    slam_path = PathJoinSubstitution([
        FindPackageShare("mobile_robot_gazebo"),
        "config",
        "slam.yaml",
    ])

    nav_path = PathJoinSubstitution([
        FindPackageShare("mobile_robot_gazebo"),
        "config",
        "nav.yaml",
    ])

    rviz_path_navigation = PathJoinSubstitution([
        FindPackageShare("mobile_robot_gazebo"),
        "rviz",
        "nav_navigation.rviz",
    ])

    rviz_path_mapping = PathJoinSubstitution([
        FindPackageShare("mobile_robot_gazebo"),
        "rviz",
        "nav_mapping.rviz",
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
            "gz_args": [TextSubstitution(text="-r "), world_path],
            "use_sim_time": "true",
        }.items(),
    )

    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        use_composition='False',
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

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml'), {'use_sim_time': True}]
    )

    ## Automatiser le lancement de rivz
    rviz_navigation = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_navigation",
        output="screen",
        arguments=['-d', rviz_path_navigation],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(
              PythonExpression(["'", mode, "' == 'navigation'"])
          ),
    )
    
    rviz_mapping = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_mapping",
        output="screen",
        arguments=['-d', rviz_path_mapping],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(
              PythonExpression(["'", mode, "' == 'mapping'"])
          ),
    )

    ## Automatiser le lancement de SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            ])
        ]),
        launch_arguments={
            "slam_params_file": slam_path,
            "use_sim_time": "true",
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'mapping'"])
        ),
    )

    ## Automatiser le lancement de Nav2
    nav2_mapping = IncludeLaunchDescription(
          PythonLaunchDescriptionSource([
              PathJoinSubstitution([
                  FindPackageShare("nav2_bringup"),
                  "launch",
                  "navigation_launch.py",
              ])
          ]),
          launch_arguments={
              "params_file": nav_path,
              "use_sim_time": "true",
          }.items(),
          condition=IfCondition(
              PythonExpression(["'", mode, "' == 'mapping'"])
          ),
      )

    nav2_saved_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            ])
        ]),
        launch_arguments={
            "params_file": nav_path,
            "map": map_file,
            "use_sim_time": "true",
            "slam": "False",
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'navigation'"])
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="simple_test.world",
            description="World file from mobile_robot_gazebo/worlds",
        ),

        DeclareLaunchArgument(
            "mode",
            default_value="navigation",
            description="Mode of operation: 'navigation' or 'mapping'",
        ),

        DeclareLaunchArgument(
            "map",
            default_value=PathJoinSubstitution([
                FindPackageShare("mobile_robot_gazebo"),
                "maps",
                "map.yaml",
            ]),
            description="Map YAML file for navigation mode",
        ),

        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        ekf_node,
        rviz_mapping,
        rviz_navigation,
        slam,
        nav2_mapping,
        nav2_saved_map,
    ])
