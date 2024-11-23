from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    image_node = Node (
        package="personal_recognition",
        executable="image_capture_node",
        name="image_capture",
    )

    turn_around_node = Node (
        package="personal_recognition",
        executable="turn_around_node",
        name="turn_around",
    )

    people_recognition_node = Node (
        package="personal_recognition",
        executable="people_recognition_node",
        name="people_recognition",
    )

    voice_node = Node(
        package="personal_recognition",
        executable="voice_node",
        name="voice",
    )

    ld.add_action(image_node)
    ld.add_action(turn_around_node)
    ld.add_action(people_recognition_node)
    ld.add_action(voice_node)
    

    return ld
    