
import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ============ Include the launch file for the robot description =======
    include_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("diff_robot_description"),
                "launch",
                "description.launch.py"
            ])
        )
    )

    # ============ Exceute gazebo =======
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ])

    #locate the world file 
    pkg_gazebo      = FindPackageShare('diff_robot_gazebo')   
    world_sdf = PathJoinSubstitution([pkg_gazebo, 'worlds', 'empty.sdf'])
    
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'gz_args': ['-r ', world_sdf]
        }.items()
    )

     # ============ spawn robot  =======
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'diff_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )

    # ============ GZ bridge  =======
    gz_bridge_params_path = os.path.join(
        get_package_share_directory('diff_robot_gazebo'),
        'config',
        'diff_robot_bridge.yaml'
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],    
        output='screen',
     
    )



    # === Registrar todo en LaunchDescription ===
    ld = LaunchDescription()
    ld.add_action(gazebo_node)
    ld.add_action(include_description)
    ld.add_action(spawn_robot)
    ld.add_action(gz_bridge_node)

    return ld
