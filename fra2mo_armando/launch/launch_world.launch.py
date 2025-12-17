from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
 
 
def generate_launch_description():
    fra2mo_dir = FindPackageShare('fra2mo_armando')

    # Dichiarazione degli argomenti del launch file
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Flag to enable GUI'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    controller_type_arg = DeclareLaunchArgument(
        name='controller_type',
        default_value='position_controller',
        description='Type of controller to use: position_controller or joint_trajectory_controller',
    )

    # Percorsi ai file di configurazione e del modello
    xacro_file_name = "fra2mo.urdf.xacro"
    rviz_config_file = os.path.join(get_package_share_directory('fra2mo_armando'), 'rviz_conf', 'fra2mo_conf.rviz')
    xacro_path = os.path.join(get_package_share_directory('fra2mo_armando'), "urdf", xacro_file_name)
    world_file = os.path.join(get_package_share_directory('fra2mo_armando'), "worlds", "office_small.sdf")
    models_path = os.path.join(get_package_share_directory('fra2mo_armando'), 'models')
    
    # Percorso unificato per tutti i controller (braccio + mobile base)
    controllers_config = os.path.join(get_package_share_directory('fra2mo_armando'), 'config', 'fra2mo_armando_controllers.yaml')

    # Configurazioni
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_type = LaunchConfiguration('controller_type')
    
    # Genera la descrizione del robot da file xacro
    robot_description_xacro = {"robot_description": ParameterValue(Command(['xacro ', xacro_path]),value_type=str)}
    
    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_xacro,
                    {"use_sim_time": use_sim_time}
            ]
    )
    
    # Nodo joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[robot_description_xacro,
                    {"use_sim_time": use_sim_time}
            ]
    )


    # Avvio di Gazebo
    gazebo_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]
        ),
        launch_arguments={'gz_args': world_file + ' -r'}.items()
    )

    # Spawn del robot in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'fra2mo',
                   '-allow_renaming', 'true',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.2']
    )

    # Bridge tra ROS 2 e Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'
        ],
        remappings=[('/lidar', '/scan')],
        output='screen'
    )

    # Bridge per la camera
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        remappings=[('/camera', '/videocamera'),
                    ('/camera_info', '/videocamera/camera_info')],
        output='screen'
    )

    # Nodo per pubblicare la trasformata odom -> base_link
    odom_tf = Node(
        package='fra2mo_armando',
        executable='dynamic_tf_publisher', # Assicurati che questo sia il nome corretto dell'eseguibile in CMakeLists.txt
        name='odom_tf',
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Spawner per i controller di ros2_control (UNIFIED per braccio + ruote + gripper)
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[controllers_config]
    )
    
    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[controller_type, '--controller-manager', '/controller_manager'],
        parameters=[controllers_config]
    )
    
    # diff_drive_controller commentato - Usiamo il plugin Gazebo nativo DiffDrive
    # spawn_diff_drive_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['diff_drive_controller', '--controller-manager', '/controller_manager']
    # )

    # Avvio dei controller dopo lo spawn del robot
    spawn_controllers_on_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[spawn_joint_state_broadcaster, spawn_arm_controller],
        )
    )

    # Nodo RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Configurazione del percorso dei modelli per Gazebo
    gz_resource = models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    amcl_params_file = LaunchConfiguration("amcl_params_file")
    map_file = LaunchConfiguration("map_file")
    rviz_config_file = os.path.join(get_package_share_directory('fra2mo_armando'), 'rviz_conf', 'amcl_view.rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value=PathJoinSubstitution([FindPackageShare("fra2mo_armando"), "maps", "mia_mappa.yaml"]),
        description="Full path to the yaml map file",
    )

    amcl_params_file_arg = DeclareLaunchArgument(
        "amcl_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("fra2mo_armando"), "config", "amcl.yaml"]
        ),
        description="Full path to the ROS2 parameters file to use for the amcl node",
    )

    # map_server, amcl e nav_manager sono lanciati tramite nav2_bringup_launch
    nav2_bringup_launch_file_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
    rviz_config_file = os.path.join(get_package_share_directory('fra2mo_armando'), 'rviz_conf', 'navigation.rviz')


    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([fra2mo_dir, 'maps', 'mia_mappa.yaml']),
        description='Full path to map yaml file to load',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([fra2mo_dir, 'config', 'navigation.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch_file_dir]),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    
    return LaunchDescription([
        # Variabili d'ambiente
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=gz_resource),

        # Argomenti
        gui_arg,
        use_sim_time_arg,
        controller_type_arg,

        # Nodi e azioni
        gazebo_ignition,
        gz_spawn_entity,

        robot_state_publisher_node,
        # joint_state_publisher_node rimosso - con Gazebo usa joint_state_broadcaster
        bridge,
        bridge_camera,
        #odom_tf, RIDONDANTE???

        spawn_controllers_on_spawn,


        #rviz_node
    ])
    