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

        # Declare the colormap_name argument
        DeclareLaunchArgument(
            'colormap_name',
            default_value='Inferno',  # Default colormap
            description='Colormap to apply (Inferno, Jet, Viridis, Rainbow, etc.)'
        ),

        Node(
            package='thermal_image_colorizer',  # Replace with your package name
            executable='colorizer',
            namespace=LaunchConfiguration('namespace'),
            name='colormap_image_publisher',
            parameters=[
                {
                    'input_topic': 'thermal/image_raw',
                    'output_topic': 'thermal/colormapped_image',
                    'colormap_name': LaunchConfiguration('colormap_name')
                }
            ],
            output='screen'
        )
    ])
