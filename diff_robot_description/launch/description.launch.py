# display_robot.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Argumentos de launch
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='visualization.rviz',
        description='RViz config file name inside the rviz/ folder of the description package'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config  = LaunchConfiguration('rviz_config')

    # Rutas dentro del paquete de descripción
    robot_description_pkg = FindPackageShare('diff_robot_description')

    robot_model_path = PathJoinSubstitution([
        robot_description_pkg,
        'urdf',
        'diff_robot_description.xacro'   # <-- cambia por tu archivo
    ])

    rviz_config_path = PathJoinSubstitution([
        robot_description_pkg,
        'rviz',
        rviz_config               # permite cambiar el .rviz por argumento
    ])

    # Nodo robot_state_publisher: carga el xacro y publica /robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', robot_model_path])
        }]
    )

    # (Opcional) joint_state_publisher_gui para mover articulaciones a mano


    # Nodo RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # LaunchDescription y registro de acciones
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(robot_state_publisher_node)
    #ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld
