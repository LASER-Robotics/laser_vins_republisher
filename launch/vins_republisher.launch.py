import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.events import matches_action
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value=os.getenv('UAV_NAME', "uav1"),
            description='Top-level namespace.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=PythonExpression(['"', os.getenv('REAL_UAV', "true"), '" == "false"']),
            description='Whether use the simulation time.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'vins_republisher_params_file',
            default_value=PathJoinSubstitution([FindPackageShare('laser_vins_republisher'),
                                                'params', 'vins_republisher.yaml']),
            description='Full path to the file with the parameters.'))

    # Initialize arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    vins_republisher_params_file = LaunchConfiguration('vins_republisher_params_file')

    # Declare nodes
    vins_republisher_lifecycle_node = LifecycleNode(
        package='laser_vins_republisher',
        executable='vins_republisher',
        name='vins_republisher',
        namespace=namespace,
        output='screen',
        parameters=[vins_republisher_params_file,
                    {'uav_name': namespace,
                    'use_sim_time': use_sim_time}],
        remappings=[('odometry_in', 'ov_msckf/odomimu'),
                    ('odometry_out', 'vins_republisher/odometry')])

    change_to_configure_state_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=vins_republisher_lifecycle_node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(vins_republisher_lifecycle_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))]))

    change_to_activate_state_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=vins_republisher_lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(vins_republisher_lifecycle_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))]))

    return LaunchDescription(declared_arguments + [vins_republisher_lifecycle_node,
                                                   change_to_configure_state_event_handler,
                                                   change_to_activate_state_event_handler])
