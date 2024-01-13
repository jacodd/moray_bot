
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    moray_bot_dir = get_package_share_directory("moray_bot")
    launch_dir = os.path.join(moray_bot_dir, 'launch')
    bringup_dir = get_package_share_directory('nav2_bringup')

    params_dir = os.path.join(moray_bot_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_gnss.yaml")
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # navigation2 launch
    ld.add_action(navigation2_cmd)

    return ld
