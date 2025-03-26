from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch TurtleBot3 Simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('my_robot_controller'),
                    'launch',
                    #'my_turtlebot3_world.launch.py'
                    'my_turtlebot3.launch.py'
                ])
            ]),
        ),
        # Launch SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_cartographer'),
                    'launch',
                    'cartographer.launch.py'
                ])
            ]),
            launch_arguments={'use_sim_time': 'True'}.items(),
        ),
        # Launch Autonomous Mapping Node
        Node(
            package='my_robot_controller',
            executable='mapping',
            name='control'
        )
    ])