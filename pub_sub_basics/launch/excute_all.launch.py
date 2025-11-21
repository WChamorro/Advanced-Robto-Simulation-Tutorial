# two_nodes.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Argumento (parámetro expuesto al usuario)
    stop_arg = DeclareLaunchArgument(
        'end_value',
        default_value='20.0'
        )
    
    stop= LaunchConfiguration('end_value')


    # Nodos
    publisher_node = Node(
        package='pub_sub_basics',
        executable='pub_param_node',
        name='python_publisher',
        parameters=[{'end_value': stop}],
        output='screen',  #para node.get_logger
        emulate_tty=True, #para visualizar los prints
    )

    subscriber_node = Node(
        package='pub_sub_basics',
        executable='sub_node',
        name='python_subscriber',
        output='screen',  #para node.get_logger
        emulate_tty=True, #para visualizar los prints
    )
    
     # Crear la descripción del launch
    ld = LaunchDescription()
    
    # Registrar nodos y argumentos en el launch description
    ld.add_action(stop_arg)
    ld.add_action(publisher_node)
    ld.add_action(subscriber_node)

    return ld
