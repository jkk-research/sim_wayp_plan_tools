from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("sim_wayp_plan_tools"), '/launch/', 'gazebo_bridge.launch.py'])
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("sim_wayp_plan_tools"), '/launch/', 'rviz1.launch.py'])
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("sim_wayp_plan_tools"), '/launch/', 'waypoint_loader.launch.py'])
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("sim_wayp_plan_tools"), '/launch/', 'csv_saver.launch.py'])
            ),            
            TimerAction(
                period=4.0, # delay / wait in seconds
                actions=[
                    Node(
                        package='wayp_plan_tools',
                        executable='waypoint_to_target',
                        #name='wayp_to_target',
                        output='screen',
                        namespace='sim1',
                        parameters=[
                            {"lookahead_min": 2.5},
                            {"lookahead_max": 4.5},
                            {"mps_alpha": 1.5},
                            {"mps_beta": 4.5}, 
                            {"waypoint_topic": "waypointarray"},
                            {"tf_frame_id": "base_link"},
                            {"tf_child_frame_id": "map"},
                            {"interpolate": True},
                        ],
                    ),
                ]
            ),    
            TimerAction(
                period=5.0, # delay / wait in seconds
                actions=[
                    Node(
                        package='wayp_plan_tools',
                        executable='single_goal_pursuit',
                        namespace='sim1',
                        output='screen',
                        parameters=[
                                {"cmd_topic": "/model/vehicle_blue/cmd_vel"},
                                {"wheelbase": 1.0}, # from the /usr/share/ignition/ignition-gazebo6/worlds/ackermann_steering.sdf file wheel_base parameter
                                {"waypoint_topic": "targetpoints"},
                                {"max_angular_velocity": 1.0},
                        ],
                    ),
                ]   
            ),

            # TimerAction(
            #     period=5.0, # delay / wait in seconds
            #     actions=[
            #         IncludeLaunchDescription(
            #             PythonLaunchDescriptionSource([
            #                 FindPackageShare("sim_wayp_plan_tools"), '/launch/', 'stanley.launch.py'])
            #         ),
            #     ]
            # ), 
        ]
    )