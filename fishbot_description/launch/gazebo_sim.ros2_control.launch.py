import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    # 通过 IncludeLaunchDescription 包含另外一个 launch 文件
    launch_gazebo_sim = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'fishbot_description'), '/launch', '/gazebo_sim.launch.py']),
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'fishbot_joint_state_broadcaster'],
        output='screen'
    )

    action_on_process_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=launch_gazebo_sim,
            on_exit=[load_joint_state_controller],
        )
    )

    return launch.LaunchDescription([
        launch_gazebo_sim,
        # action_on_process_exit
    ])
