from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pc_ip = LaunchConfiguration('pc_ip')
    data_port = LaunchConfiguration('data_port')
    output_bin = LaunchConfiguration('output_bin')
    publish_packets = LaunchConfiguration('publish_packets')
    input_bin = LaunchConfiguration('input_bin')

    return LaunchDescription([
        DeclareLaunchArgument('pc_ip', default_value='192.168.33.180',
                              description='PC NIC IP that receives DCA1000 UDP data'),
        DeclareLaunchArgument('data_port', default_value='4098',
                              description='UDP port for DCA1000 data stream'),
        DeclareLaunchArgument('output_bin', default_value='/tmp/awr2243_capture.bin',
                              description='Path to write the captured raw bytes'),
        DeclareLaunchArgument('publish_packets', default_value='false',
                              description='Also publish awr2243/raw_packets (ByteMultiArray)'),
        DeclareLaunchArgument('input_bin', default_value='/tmp/awr2243_capture.bin',
                              description='Path of .bin file to process for RD map'),

        Node(
            package='awr2243_ros2',
            executable='raw_capture_node',
            name='awr2243_raw_capture',
            output='screen',
            parameters=[{
                'pc_ip': pc_ip,
                'data_port': LaunchConfiguration('data_port'),
                'output_bin': output_bin,
                'publish_packets': publish_packets,
            }],
        ),

        Node(
            package='awr2243_ros2',
            executable='rd_offline_node',
            name='awr2243_rd_offline',
            output='screen',
            parameters=[{'input_bin': input_bin}],
        ),
    ])
