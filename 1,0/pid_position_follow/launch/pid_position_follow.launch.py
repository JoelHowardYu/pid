from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pid_position_follow',
            executable='pid_position_follow_node',
            name='pid_position_follow',
            output='screen',
            respawn=True,
            parameters=[
                {'global_frame': 'map'},
                {'plan_frequency': 50},
                {'max_x_speed': 2.0},
                {'max_y_speed': 2.0},
                {'set_yaw_speed': 3.0},
                {'goal_dist_tolerance': 0.25},
                {'prune_ahead_distance': 0.6},
                {'p_value': 1.5},
                {'i_value': 1.0},
                {'d_value': 1.0}
            ],
            remappings=[
                ('/base_vel', '/cmd_vel')
            ]
        )
    ])