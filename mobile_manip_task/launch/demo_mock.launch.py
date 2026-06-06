from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    fake_vision_node = Node(
        package='mobile_manip_task',
        executable='fake_vision_node',
        name='fake_vision_node',
        output='screen',
        parameters=[
            {
                'publish_rate': 1.0,
                'frame_id': 'camera_frame',
                'object_x': 1.0,
                'object_y': 0.2,
                'object_z': 0.7,
            }
        ]
    )

    robot_state_node = Node(
        package='mobile_manip_task',
        executable='robot_state_node',
        name='robot_state_node',
        output='screen',
        parameters=[
            {
                'initial_state': 'READY',
                'publish_rate': 1.0,
            }
        ]
    )

    task_executor_node = Node(
        package='mobile_manip_task',
        executable='task_executor_node',
        name='task_executor_node',
        output='screen'
    )

    return LaunchDescription([
        fake_vision_node,
        robot_state_node,
        task_executor_node,
    ])