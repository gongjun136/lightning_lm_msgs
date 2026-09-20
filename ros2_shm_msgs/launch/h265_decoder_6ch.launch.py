from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='shm_msgs',
            executable='h265_decoder_6ch',
            name='h265_decoder_6ch',
            output='screen',
            parameters=[{
                'decoder_threads_per_camera': 1,
                'frame_flush_timeout_ms': 250,
            }],
        ),
    ])
