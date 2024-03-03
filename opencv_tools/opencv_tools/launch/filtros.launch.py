from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    LaunchDescription()

    lowpass = Node(
        package="opencv_tools",
        executable="lowpass"

    )
    highpass = Node(
        package="opencv_tools",
        executable="highpass"
    )
    custom = Node(
        package="opencv_tools",
        executable="custom_filter"
    )
    image = Node(
        package="opencv_tools",
        executable= "img_subscriber"
    )
    current_frame = Node(
        package="opencv_tools",
        executable= "img_publisher"
    )
  

    return LaunchDescription([lowpass,
                              highpass,
                              custom,
                              image,
                              current_frame])  