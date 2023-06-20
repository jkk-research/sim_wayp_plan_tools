from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'sim_wayp_plan_tools'
    #pkg_dir = os.popen('/bin/bash -c "cd && source /usr/share/colcon_cd/function/colcon_cd.sh && colcon_cd %s && pwd"' % pkg_name).read().strip()
    pkg_dir = get_package_share_directory(pkg_name)
    #print(pkg_dir) # /home/he/ros2_ws/src/demo_jkk


    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            #name='rviz2',
            arguments=['-d', [os.path.join(pkg_dir, 'rviz', 'rviz1.rviz')]]
        )
    ])