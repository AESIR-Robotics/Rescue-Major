import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # The Isaac ROS AprilTag node
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace='',
        parameters=[{
            'size': 0.15, # Size of the tag in meters (adjust to RoboCup specs)
            'max_tags': 10,
            'image_transport': 'raw',
            'family': 'tag36h11', # Standard tag family
        }],
        remappings=[
            # Remap these to match your camera/GStreamer node outputs
            ('image_rect', '/oak/rgb/image_rect'),
            ('camera_info', '/oak/rgb/camera_info'),
            ('tf', 'tf')
        ]
    )

    # Container to run the component
    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[apriltag_node],
        output='screen'
    )

    return launch.LaunchDescription([container])