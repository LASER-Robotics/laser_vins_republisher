from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.events import matches_action
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():
    # === Declaração dos argumentos ===
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            # default_value='uav3_bags',
            default_value=EnvironmentVariable('UAV_NAME'),
            description='Top-level namespace.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'vins_republisher_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('laser_vins_republisher'),
                'params',
                'vins_republisher.yaml'
            ]),
            description='Full path to the file with the vins_republisher parameters.'
        )
    )

    # === Inicializa LaunchConfiguration ===
    namespace = LaunchConfiguration('namespace')
    vins_republisher_file = LaunchConfiguration('vins_republisher_file')

    # === Criação do LifecycleNode ===
    vins_republisher_node = LifecycleNode(
        package='laser_vins_republisher',
        executable='vins_republisher',
        name='vins_republisher',
        namespace=namespace,
        output='screen',
        parameters=[
            vins_republisher_file,
            {'UAV_NAME': namespace}
        ],
        remappings=[
            ('odometry_in', 'ov_msckf/odomimu'),
            ('odometry_out', 'vins_republisher/odom'),
        ]
    )

    # === Handlers para configurar e ativar automaticamente ===
    event_handlers = [
        RegisterEventHandler(
            OnProcessStart(
                target_action=vins_republisher_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(vins_republisher_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=vins_republisher_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(vins_republisher_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        )
    ]

    # === Composição do LaunchDescription ===
    ld = LaunchDescription()

    for arg in declared_arguments:
        ld.add_action(arg)

    ld.add_action(vins_republisher_node)

    for handler in event_handlers:
        ld.add_action(handler)

    return ld
