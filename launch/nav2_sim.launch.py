"""Ground-robot simulation bring-up: TurtleBot3 in Gazebo Classic + Nav2.

Run on the real rig (Ubuntu 22.04 / ROS 2 Humble):

    export TURTLEBOT3_MODEL=waffle
    ros2 launch launch/nav2_sim.launch.py headless:=False

Then, in another sourced terminal:

    python3 run_pipeline.py \
        --prompt "Drive the ground robot around a 10 metre square box three laps" \
        --backend nav2

This wraps nav2_bringup's canonical tb3_simulation_launch.py (Gazebo Classic
world + AMCL + Nav2 servers + RViz) so the pipeline needs no custom world.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    nav2_bringup = get_package_share_directory("nav2_bringup")
    headless = LaunchConfiguration("headless")

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="True",
                              description="False = show the Gazebo client GUI"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, "launch", "tb3_simulation_launch.py")),
            launch_arguments={
                "headless": headless,
                "use_sim_time": "True",
            }.items(),
        ),
    ])
