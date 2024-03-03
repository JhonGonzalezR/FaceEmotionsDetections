from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    LaunchDescription()

    camera_publisher_node = Node(
        package="opencv_tools",
        executable="img_publisher"

    )
    emotions_node = Node(
        package="opencv_tools",
        executable="emotions"
    )
    stats_node = Node(
        package="opencv_tools",
        executable="stats"
    )
  

    return LaunchDescription([camera_publisher_node,
                              emotions_node,
                              stats_node])  