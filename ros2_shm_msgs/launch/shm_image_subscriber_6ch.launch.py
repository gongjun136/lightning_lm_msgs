# shm_image_subscriber_6ch.launch.py
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='shm_msgs',
            executable='shm_image6m_subscriber_6ch',
            output='screen',
        ),
    ])