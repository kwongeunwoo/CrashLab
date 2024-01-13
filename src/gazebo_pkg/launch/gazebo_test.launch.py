import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'gazebo_pkg'
    world_filename = 'shop.world'
    packge_path = os.path.join(get_package_share_directory(package_name))
    world_path = os.path.join(packge_path, "worlds", world_filename)
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=["gazebo", world_path], output="screen"
        ),

        ExecuteProcess(
            cmd=["ros2", "run", "rviz2", "rviz2"], output="screen"
        ),
    ])