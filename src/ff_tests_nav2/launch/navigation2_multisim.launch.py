from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
import os
from ament_index_python.packages import get_package_share_directory

# Robot configurations
robots = [
    {
        'name': 'robot1',
        'x_pose': 0.28,
        'y_pose': -6.75,
        'z_pose': 0.01,
        'roll': 0.0,
        'pitch': 0.0,
        'yaw': 0.0,
    },
    {
        'name': 'robot2',
        'x_pose': 5.67,
        'y_pose': -9.5,
        'z_pose': 0.01,
        'roll': 0.0,
        'pitch': 0.0,
        'yaw': 0.0,
    },
]

def generate_launch_description():
    nav2_pkg = get_package_share_directory('ff_tests_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tb3_dir = get_package_share_directory('turtlebot3_gazebo')

    # Global launch configurations
    map_file = LaunchConfiguration('map', default=os.path.join(tb3_dir, 'maps', 'map.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # RViz config file
    rviz_config_file = os.path.join(nav2_pkg, 'rviz_configs', 'nav2_namespaced_view.rviz')

    # Param files
    params_file_robot1 = LaunchConfiguration('params_file_robot1', default=os.path.join(nav2_pkg, 'params', 'nav2_multirobot_params_1.yaml'))
    params_file_robot2 = LaunchConfiguration('params_file_robot2', default=os.path.join(nav2_pkg, 'params', 'nav2_multirobot_params_2.yaml'))

    def create_robot_group(robot, params_file):
        return GroupAction([
            PushRosNamespace(robot['name']),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb3_dir, 'launch', 'spawn_turtlebot3.launch.py')
                ),
                launch_arguments={
                    'x_pose': str(robot['x_pose']),
                    'y_pose': str(robot['y_pose']),
                    # Could also pass other launch args if spawn launch is extended
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                }.items()
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name=f"rviz2_{robot['name']}",
                output='screen',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ])

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('map', default_value=os.path.join(tb3_dir, 'maps', 'map.yaml'), description='Map file'),
        DeclareLaunchArgument('params_file_robot1', default_value=params_file_robot1, description='Param file for robot1'),
        DeclareLaunchArgument('params_file_robot2', default_value=params_file_robot2, description='Param file for robot2'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz'),

        # Robot launch groups
        create_robot_group(robots[0], params_file_robot1),
        create_robot_group(robots[1], params_file_robot2),
    ])
