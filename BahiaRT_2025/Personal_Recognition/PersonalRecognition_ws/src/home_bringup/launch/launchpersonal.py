import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='personal_recognition',  # Substitua pelo nome do seu pacote
            executable='image_capture_node',    # Substitua pelo nome do seu executável
            name='image_capture',
            output='screen',
           
        ),        
        
        Node(
            package='personal_recognition',  # Substitua pelo nome do seu pacote
            executable='turn_around_node',    # Substitua pelo nome do seu executável
            name='turn_around',
            output='screen',
            
        ),


        Node(
            package='personal_recognition',  # Substitua pelo nome do seu pacote
            executable='people_recognition_node',    # Substitua pelo nome do seu executável
            name='people_recognition',
            output='screen',
            
        ),
        Node(
            package='personal_recognition',  # Substitua pelo nome do seu pacote
            executable='voice_node',    # Substitua pelo nome do seu executável
            name='voice_node',
            output='screen',
           
        ),
        
    ])
