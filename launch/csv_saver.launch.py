# https://github.com/jkk-research/sim_wayp_plan_tools/issues/1
# Benchmark CSV saving
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_name = 'sim_wayp_plan_tools'
    pkg_dir = get_package_share_directory(pkg_name)
    #print(pkg_dir)

    return LaunchDescription([
        Node(
            package='sim_wayp_plan_tools',
            executable='csv_saver',
            #name='wayp_saver',
            output='screen',
            namespace='sim1',
            parameters=[
                {"file_dir": pkg_dir + "/csv"},
                {"file_name": "tmp01"},
                {"topic_based_saving": False},
                {"tf_frame_id": "map"},
                {"tf_child_frame_id": "base_link"},
            ],
        )
    ])