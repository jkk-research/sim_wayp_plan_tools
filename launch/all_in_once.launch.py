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
            TimerAction(
                period=4.0, # delay / wait in seconds
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            FindPackageShare("sim_wayp_plan_tools"), '/launch/', 'waypoint_to_target.launch.py'])
                    ),
                ]
            ),    
            TimerAction(
                period=5.0, # delay / wait in seconds
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            FindPackageShare("sim_wayp_plan_tools"), '/launch/', 'single_goal_pursuit.launch.py'])
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