from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import RegisterEventHandler, EmitEvent
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
            'UAV_NAME',
            default_value=EnvironmentVariable('UAV_NAME'),
            description='Top-level namespace.'))
    
    # Argumento para o arquivo de parâmetros do vins_republisher
    declared_arguments.append(
        DeclareLaunchArgument(
            'vins_republisher_file',
            default_value=PathJoinSubstitution([FindPackageShare('laser_vins_republisher'),
                                                'params', 'vins_republisher.yaml']),
            description='Full path to the file with the vins_republisher loader parameters.'
        )
    )

    # Inicialização dos argumentos
    uav_name = LaunchConfiguration('UAV_NAME')
    vins_republisher_file = LaunchConfiguration('vins_republisher_file')

    # Definição do LifecycleNode
    vins_republisher_node = LifecycleNode(
        package='laser_vins_republisher',   # Nome do pacote
        executable='vins_republisher',      # Nome do executável
        name='vins_republisher',            # Nome do nó
        namespace=uav_name,  # Namespace do nó
        output='screen',  # Saída do log no terminal
        parameters=[
            vins_republisher_file,  # Passa o caminho do arquivo YAML
            {'UAV_NAME': uav_name}  # Passa o parâmetro adicional diretamente como um dicionário
        ],  # Carrega os parâmetros do arquivo YAML
        remappings=[
            # Remapeamentos de tópicos
            ('odometry_in',     'ov_msckf/odomimu'),  # Remapeia tópico de entrada
            ('odometry_out',    'vins_republisher/odom'),  # Remapeia tópico de entrada
        ]
    )

    # Lista de manipuladores de eventos
    event_handlers = []

    # Manipulador para realizar a transição para o estado 'configure' assim que o nó for iniciado
    event_handlers.append(
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
    )

    # Manipulador para realizar a transição para o estado 'activate' após o estado 'configuring'
    event_handlers.append(
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
        ),
    )

    # Criação do LaunchDescription
    ld = LaunchDescription()

    # Declaração dos argumentos
    for argument in declared_arguments:
        ld.add_action(argument)

    # Adiciona o nó do vins_republisher
    ld.add_action(vins_republisher_node)

    # Adiciona os manipuladores de eventos
    for event_handler in event_handlers:
        ld.add_action(event_handler)

    return ld
