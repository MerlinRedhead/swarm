import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('swarm_control') # Замени на имя своего пакета
    config_file = os.path.join(pkg_dir, 'config', 'swarm_params.yaml')

    agent_id = LaunchConfiguration('agent_id')
    fcu_url = LaunchConfiguration('fcu_url')

    return LaunchDescription([
        DeclareLaunchArgument('agent_id', default_value='striker_1'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyAMA0:921600'),

        # 1. MAVROS
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=[{'fcu_url': fcu_url}]
        ),

        # 2. Адаптивная нода логики (выбирает тип на основе ID)
        Node(
            package='swarm_control',
            executable='striker_node',
            name=agent_id, # Важно: имя ноды должно совпадать с agent_id для YAML
            condition=PythonExpression(["'striker' in '", agent_id, "'"]),
            parameters=[config_file, {'agent_id': agent_id}]
        ),

        Node(
            package='swarm_control',
            executable='vision_node',
            name=agent_id,
            condition=PythonExpression(["'vision' in '", agent_id, "'"]),
            parameters=[config_file, {'agent_id': agent_id}]
        ),

        Node(
            package='swarm_control',
            executable='connect_node',
            name=agent_id,
            condition=PythonExpression(["'connect' in '", agent_id, "'"]),
            parameters=[config_file, {'agent_id': agent_id}]
        )
    ])
