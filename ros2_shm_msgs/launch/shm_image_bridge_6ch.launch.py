# shm_image_bridge_6ch.launch.py
import launch
import launch_ros.actions

def generate_launch_description():
    # shm_image6m_bridge_6ch 单进程内已创建 6 路 pub/sub，
    # topic 名在 C++ 中硬编码：
    #   sub: /camera/<cam>/shm_image_6m
    #   pub: <cam>/sensor_image_6m
    node = launch_ros.actions.Node(
        package='shm_msgs',
        executable='shm_image6m_bridge_6ch',
        output='screen',
    )

    return launch.LaunchDescription([node])
