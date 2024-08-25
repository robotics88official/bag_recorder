from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    config_directory_arg = DeclareLaunchArgument(
        'configuration_directory', default_value='../config/bag_recorder.config',
        description='Directory containing configuration files for the bag launcher to read'
    )
    data_directory_arg = DeclareLaunchArgument(
        'data_directory', default_value='/home/parallels/data/',
        description='Directory for the bags to be recorded to'
    )
    name_topic_arg = DeclareLaunchArgument(
        'name_topic', default_value='/record/bag_name',
        description='Topic to publish bag name to if publish_name is true'
    )
    heartbeat_interval_arg = DeclareLaunchArgument(
        'heartbeat_interval', default_value='5.0',
        description='Interval in seconds on which to publish bag heartbeats'
    )

    # Launch the recorder node
    recorder_node = Node(
        package='bag_recorder',
        executable='bag_recorder_node',
        name='rosbag_recorder_node',
        output='screen',
        respawn=True,
        parameters=[
            {'configuration_directory': LaunchConfiguration('configuration_directory')},
            {'data_directory': LaunchConfiguration('data_directory')},
            {'start_bag_topic': '/record/start'},
            {'stop_bag_topic': '/record/stop'},
            {'publish_name': True},
            {'name_topic': LaunchConfiguration('name_topic')},
            {'publish_heartbeat': True},
            {'heartbeat_topic': '/record/heartbeat'},
            {'heartbeat_interval': LaunchConfiguration('heartbeat_interval')},
            {'default_record_all': False}
        ]
    )

    return LaunchDescription([
        config_directory_arg,
        data_directory_arg,
        name_topic_arg,
        heartbeat_interval_arg,
        recorder_node
    ])
