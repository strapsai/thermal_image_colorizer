import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Declare the namespace argument
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the colormap_image_publisher node'
        ),
        Node(
            package='thermal_image_colorizer',  # Replace with your package name
            executable='flip_image',
            namespace=LaunchConfiguration('namespace'),
            name='colormap_image_publisher',
            parameters=[
                {
                    'input_topic': 'realsense/image_raw',
                    'output_topic': 'realsense/flip_image',
                }
            ],
            output='screen'
        )
    ])
