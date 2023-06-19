from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wayp_plan_tools',
            executable='single_goal_pursuit',
            name='pure_pursuit',
            namespace='sim1',
            output='screen',
            parameters=[
                    {"cmd_topic": "/model/vehicle_blue/cmd_vel"},
                    {"wheelbase": 1.0}, # from the /usr/share/ignition/ignition-gazebo6/worlds/ackermann_steering.sdf file wheel_base parameter
                    {"waypoint_topic": "targetpoints"},
                ],
        )
    ])
