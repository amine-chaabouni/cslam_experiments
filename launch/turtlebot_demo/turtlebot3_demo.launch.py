import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
NB_ROBOTS = int(os.environ['NB_ROBOTS'])

def launch_cslam(context, *args, **kwargs):
    launched_nodes = []
    for i in range(NB_ROBOTS):
        namespace = PythonExpression(expression=[LaunchConfiguration('namespace'), "+ '" ,f"{i}" + "'"])
        launched_nodes.append(Node(package='cslam',
                                executable='loop_closure_detection_node.py',
                                name=f'cslam_loop_closure_detection',
                                parameters=[
                                    LaunchConfiguration('config'), {
                                        "robot_id": i,
                                        "max_nb_robots":
                                        LaunchConfiguration('max_nb_robots'),
                                    }
                                ],
                                namespace=namespace))

        launched_nodes.append(Node(package='cslam',
                                executable='lidar_handler_node.py',
                                name=f'cslam_map_manager',
                                parameters=[
                                    LaunchConfiguration('config'), {
                                        "robot_id": i,
                                        "max_nb_robots":
                                        LaunchConfiguration('max_nb_robots'),
                                    }
                                ],
                                prefix=LaunchConfiguration('launch_prefix_cslam'),
                                namespace=namespace))

        launched_nodes.append(Node(package='cslam',
                                    executable='pose_graph_manager',
                                    name=f'cslam_pose_graph_manager',
                                    parameters=[
                                        LaunchConfiguration('config'), {
                                        "robot_id": i,
                                        "max_nb_robots":
                                        LaunchConfiguration('max_nb_robots'),
                                        "evaluation.enable_simulated_rendezvous": LaunchConfiguration('enable_simulated_rendezvous'),
                                        "evaluation.rendezvous_schedule_file": LaunchConfiguration('rendezvous_schedule_file'),
                                    }
                                ],
                                prefix=LaunchConfiguration('launch_prefix_cslam'),
                                namespace=namespace))

    return launched_nodes

#  ros2 run nav2_gazebo_spawner nav2_gazebo_spawner --robot_namespace turtlebot3_0 --robot_name waffle_0 --turtlebot_type waffle -x 2 -y 0.5

def launch_turtlebot(context, *args, **kwargs):
    launched_nodes = []

    for i in range(NB_ROBOTS):
        launched_nodes.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'gazebo_spawner_{i}',
            arguments=[
                '-entity', PythonExpression(expression=[LaunchConfiguration('robot_name'), "+ '" ,f"{i}" + "'"]),
                '-robot_namespace', PythonExpression(expression=[LaunchConfiguration('robot_namespace'), "+ '" ,f"{i}" + "'"]),
                '-file', LaunchConfiguration('turtlebot3_model_path'),
                '-x', LaunchConfiguration(f'x{i}'),
                '-y', LaunchConfiguration(f'y{i}'),
            ]
        ))

    return launched_nodes

def launch_remapper(context, *args, **kwargs):
    conf_cslam_robot_id = LaunchConfiguration('namespace')
    conf_real_robot_id = LaunchConfiguration('robot_namespace')
    launched_nodes = []
    
    for i in range(NB_ROBOTS):
        launched_nodes.append(Node(
            package='cslam_experiments',
            executable='remapper.py',
            name=f'remapper_for_r{i}',
            # parameters=[LaunchConfiguration('config')],
            remappings=[
                ("/odom_out", PythonExpression(expression=[conf_cslam_robot_id, "+ '" ,f"{i}" + "'+" ,"'/odom'"])),
                ("/pointcloud_out", PythonExpression(expression=[conf_cslam_robot_id, "+ '" ,f"{i}" + "'+","'/pointcloud'"])),
                ("/odom_in", PythonExpression(expression=["'/'", "+", conf_real_robot_id, "+ '" ,f"{i}" + "'+" ,"'/odom'", " if ", conf_real_robot_id, " != 'nan' ", " else ", "'/odom'"])),
                ("/scan_in", PythonExpression(expression=["'/'", "+", conf_real_robot_id, "+ '" ,f"{i}" + "'+" ,"'/scan'", " if ", conf_real_robot_id, " != 'nan' ", " else ", "'/scan'"])),
            ],
        ))

    return launched_nodes


def generate_launch_description():
    turtlebot_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL+ '/model.sdf'
    turtlebot_model = os.path.join(get_package_share_directory('cslam_experiments'),
                         'models', turtlebot_file_name)

    turtlebot_positions=[]
    # {-2, 0.5}, {-6, -0.5}
    for i in range(NB_ROBOTS):
        turtlebot_positions.append(DeclareLaunchArgument(f'x{i}', default_value='-2', description=''))
        turtlebot_positions.append(DeclareLaunchArgument(f'y{i}', default_value='0.5', description=''))

    return LaunchDescription([
        # cslam-specific
        DeclareLaunchArgument('namespace', default_value="'/r'", description=''),
        DeclareLaunchArgument('robot_id', default_value='0', description=''),
        DeclareLaunchArgument('max_nb_robots', default_value=f"{NB_ROBOTS}", description=''),
        DeclareLaunchArgument('config_path', default_value=os.path.join( get_package_share_directory('cslam_experiments'), 'config/'),
                              description=''),
        DeclareLaunchArgument('config_file', default_value='kitti_lidar.yaml', description=''),
        DeclareLaunchArgument('config',
                              default_value=[
                                  LaunchConfiguration('config_path'),
                                  LaunchConfiguration('config_file')
                              ],
                              description=''),
        DeclareLaunchArgument(
            'launch_prefix_cslam',
            default_value='',
            description=
            'For debugging purpose, it fills prefix tag of the nodes, e.g., "xterm -e gdb -ex run --args"'
        ),
        DeclareLaunchArgument('enable_simulated_rendezvous',
                              default_value='false',
                              description=''),
        DeclareLaunchArgument('rendezvous_schedule_file',
                              default_value='',
                              description=''),
        DeclareLaunchArgument('log_level',
                              default_value='fatal',
                              description=''),
        *turtlebot_positions,
        OpaqueFunction(function=launch_cslam),

        # turtlebot-specific
        DeclareLaunchArgument('robot_namespace', default_value="'turtlebot3_'", description=''),
        DeclareLaunchArgument('robot_name', default_value="'waffle_'", description=''),
        DeclareLaunchArgument('turtlebot3_model_path', default_value=f"{turtlebot_model}", description=''),
        

        OpaqueFunction(function=launch_turtlebot),

        # Remapper-specific
        OpaqueFunction(function=launch_remapper),

    ])
