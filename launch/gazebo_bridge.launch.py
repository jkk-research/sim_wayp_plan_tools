from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        # ros2 run ros_gz_bridge parameter_bridge /world/ackermann_steering/pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            #name='ros_gz_bridge_1',
            arguments=[
                '/world/ackermann_steering/pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V',  
                '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',  
            ],
            output='screen',
        ),
        Node(
            package='sim_wayp_plan_tools',
            executable='pose_arr_to_tf',
            #name='pose_arr_to_tf1',
            output='screen',
            parameters=[
                {"pose_topic": "/world/ackermann_steering/pose/info"},
            ],
        )
        

    ])