import launch
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speech_pkg',  # Substitua pelo nome do seu pacote
            executable='speech_context_node',    # Substitua pelo nome do seu executável
            name='speech_context',
            output='screen',
            #parameters=[{'param1': 'value1'}] #Se necessário inicializar parâmetros
        ),

        # Adicione mais nós conforme necessário
    ])