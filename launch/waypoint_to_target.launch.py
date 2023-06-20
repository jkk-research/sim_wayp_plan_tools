from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    #pkg_name = 'wayp_plan_tools'
    #pkg_dir = os.popen('/bin/bash -c "cd && source /usr/share/colcon_cd/function/colcon_cd.sh && colcon_cd %s && pwd"' % pkg_name).read().strip()
    #print(pkg_dir)

    return LaunchDescription([
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
            ],
        ),
        Node(
            package='sim_wayp_plan_tools',
            executable='visuals',
            output='screen',
            namespace='sim1',
            parameters=[
                {"marker_topic": "marker_steering"},
                {"mod_limit": 40}, # modulo limit for path size (publish every n-th message)
                {"path_size": 800},
                {"pose_frame": "base_link"}, 
                {"publish_steer_marker": True},
            ],
        ),
    ])
