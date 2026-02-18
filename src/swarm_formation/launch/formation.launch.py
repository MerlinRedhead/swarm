from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        # --- VISION 1: Слева от Хаба (-30м) ---
        Node(
            package='swarm_formation',
            executable='formation_member',
            name='vision_1_node',
            namespace='vision_1',
            parameters=[
                {'my_id': 'vision_1'},
                {'offset_x': -30.0}, # 30м Западнее центра
                {'offset_y': 0.0}    
            ]
        ),

        # --- VISION 2: Справа от Хаба (+30м) ---
        Node(
            package='swarm_formation',
            executable='formation_member',
            name='vision_2_node',
            namespace='vision_2',
            parameters=[
                {'my_id': 'vision_2'},
                {'offset_x': 30.0},  # 30м Восточнее центра
                {'offset_y': 0.0}
            ]
        ),

        # --- STRIKER 1: Сзади Хаба (-20м) ---
        Node(
            package='swarm_formation',
            executable='formation_member',
            name='striker_1_node',
            namespace='striker_1',
            parameters=[
                {'my_id': 'striker_1'},
                {'offset_x': 0.0},
                {'offset_y': -20.0} # 20м Южнее (сзади)
            ]
        ),
        
        # --- HUB: Сам лидер (Offset 0,0) ---
        # Хаб тоже использует этот скрипт, чтобы лететь в центр, 
        # ПЛЮС запускает скрипт hub_commander
        Node(
            package='swarm_formation',
            executable='formation_member',
            name='hub_pilot',
            namespace='hub',
            parameters=[{'offset_x': 0.0}, {'offset_y': 0.0}]
        )
    ])
