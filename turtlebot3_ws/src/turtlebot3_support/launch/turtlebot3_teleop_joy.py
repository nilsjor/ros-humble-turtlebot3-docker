from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'autorepeat_rate': 20.0,
                'deadzone': 0.2
            }]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[{
                'enable_button': 4,
                'enable_turbo_button': 5,
                'axis_linear.x': 1,
                'axis_angular.yaw': 0,
                'scale_linear.x': 0.13,
                'scale_angular.yaw': 0.91,
                'scale_linear_turbo.x': 0.26,
                'scale_angular_turbo.yaw': 1.74
            }]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
