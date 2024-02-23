# https://github.com/jkk-research/sim_wayp_plan_tools/issues/1
# Benchmark CSV saving
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_name = 'sim_wayp_plan_tools'
    pkg_dir = get_package_share_directory(pkg_name)
    file_prefix = "tmp"
    csv_dir = os.path.expanduser("~") + "/ros2_ws/src/sim_wayp_plan_tools/csv"
    i = 1
    # iterate until we find a file that does not exist
    while os.path.exists("%s/%s%02d_xypose.csv" % (csv_dir, file_prefix, i)):
        i += 1
    csv_file_name = "%s%02d" % (file_prefix, i)

    return LaunchDescription([
        Node(
            package='sim_wayp_plan_tools',
            executable='csv_saver',
            #name='wayp_saver',
            output='screen',
            namespace='sim1',
            parameters=[
                # {"file_dir": pkg_dir + "/csv"},
                # {"file_dir": "$HOME/ros2_ws/src/sim_wayp_plan_tools/csv"},
                {"file_dir": csv_dir},
                # {"file_name": "tmp01"},
                {"file_name": csv_file_name},
                {"topic_based_saving": False},
                {"tf_frame_id": "map"},
                {"tf_child_frame_id": "base_link"},
            ],
        )
    ])