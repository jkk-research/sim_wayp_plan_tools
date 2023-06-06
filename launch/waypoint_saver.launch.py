from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    pkg_name = 'sim_wayp_plan_tools'
    pkg_dir = os.popen('/bin/bash -c "cd && source /usr/share/colcon_cd/function/colcon_cd.sh && colcon_cd %s && pwd"' % pkg_name).read().strip()
    #print(pkg_dir)

    return LaunchDescription([
        Node(
            package='wayp_plan_tools',
            executable='waypoint_saver',
            name='wayp_saver',
            output='screen',
            namespace='sim1',
            parameters=[
                {"file_dir": pkg_dir + "/csv"},
                {"file_name": "sim_waypoints2.csv"},
                {"topic_based_saving": False},
                {"tf_frame_id": "map"},
                {"tf_child_frame_id": "base_link"},
            ],
        )
    ])