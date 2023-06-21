from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_name_in_model = 'fishbot'
    urdf_name = "fishbot_gazebo.urdf"
    urdf_tutorial_path = get_package_share_path('fishbot_description')
    default_model_path = urdf_tutorial_path / 'urdf/fishbot.urdf.xacro'

    action_model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                             description='Absolute path to robot urdf file')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_ros_gz_sim, '/launch', '/gz_sim.launch.py']),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_entity_cmd = Node(package='ros_gz_sim', executable='create',
                            arguments=[
                                '-name', 'fishbot',
                                '-topic', '/robot_description'],
                            output='screen')

    return LaunchDescription([
        action_model_arg,
        robot_state_publisher_node,
        gazebo,
        spawn_entity_cmd
    ])
