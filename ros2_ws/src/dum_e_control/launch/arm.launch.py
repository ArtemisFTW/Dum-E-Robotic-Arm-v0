from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from dum_e_control.description import robot_description


def nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    config = str(Path(get_package_share_directory('dum_e_control')) / 'config' / 'control.yaml')
    model_path = value('robot_config')
    result = [
        Node(package='dum_e_control', executable='kinematics_node', name='kinematics',
             namespace='dum_e', output='screen', parameters=[config, {'robot_config': model_path}]),
        Node(package='dum_e_control', executable='serial_bridge', name='serial_bridge',
             namespace='dum_e', output='screen', parameters=[config, {
                 'dry_run': value('dry_run').lower() == 'true', 'port': value('port')}]),
    ]
    if value('gui').lower() == 'true':
        gui = Node(package='dum_e_control', executable='gui', name='arm_console',
                   namespace='dum_e', output='screen')
        result.extend([gui, RegisterEventHandler(OnProcessExit(target_action=gui,
                       on_exit=[EmitEvent(event=Shutdown(reason='Arm Console closed'))]))])
    if value('visualize').lower() == 'true':
        result.extend([
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': robot_description(model_path)}],
                 remappings=[('joint_states', '/dum_e/joint_targets')]),
            Node(package='rviz2', executable='rviz2', arguments=['-d', str(Path(config).with_name('arm.rviz'))]),
        ])
    return result


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('port', default_value='', description='Explicit /dev/serial/by-id/... port for hardware'),
        DeclareLaunchArgument('visualize', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('robot_config', default_value='', description='Optional absolute path to calibrated JSON model'),
        DeclareLaunchArgument('gui', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=nodes),
    ])
