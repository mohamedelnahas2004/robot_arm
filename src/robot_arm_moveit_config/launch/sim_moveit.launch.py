import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='auto',
        description='Launch mode: auto (pick-and-place) or teleop (hand gesture servo)'
    )
    mode = LaunchConfiguration('mode')

    robot_desc_dir    = get_package_share_directory('robot_description')
    moveit_config_dir = get_package_share_directory('robot_arm_moveit_config')
    world_file        = os.path.join(robot_desc_dir, 'worlds', 'pick_and_place.sdf')

    moveit_config = MoveItConfigsBuilder(
        "robot_arm", package_name="robot_arm_moveit_config"
    ).to_moveit_configs()

    servo_yaml_path = os.path.join(moveit_config_dir, 'config', 'moveit_servo.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, {'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'robot_arm'],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
        output='screen',
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'teleop'"]))
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}]
    )
    vision_delayed = TimerAction(
        period=7.0,
        actions=[Node(
            package='arm_control',
            executable='vision_node',
            name='vision_node',
            output='screen',
        )],
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'teleop'"]))
    )
    pnp_node = Node(
        package="arm_control",
        executable="dynamic_pnp",
        name="dynamic_pnp",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
            {'start_state_max_bounds_error': 0.1},
            {'planning_pipelines': {'pipeline_names': ['ompl']}},
            {'plan_request_params': {
                'planning_attempts': 1,
                'planning_pipeline': 'ompl',
                'planner_id': 'RRTConnectkConfigDefault',
                'max_velocity_scaling_factor': 1.0,
                'max_acceleration_scaling_factor': 1.0,
                'planning_time': 5.0,
            }}
        ],
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'teleop'"]))
    )

    tracker = Node(
        package='arm_control',
        executable='tracker_node',
        output='screen',
        parameters=[moveit_config.robot_description, {'use_sim_time': True}]
    )
    
    translator = Node(
        package='arm_control',
        executable='translator_node',
        output='screen',
        parameters=[moveit_config.robot_description, {'use_sim_time': True}]
    )

    jsb_spawner     = Node(package="controller_manager", executable="spawner",
                           arguments=["joint_state_broadcaster"])
    arm_spawner     = Node(package="controller_manager", executable="spawner",
                           arguments=["arm_controller"])
    gripper_spawner = Node(package="controller_manager", executable="spawner",
                           arguments=["gripper_controller"])

    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[jsb_spawner, arm_spawner, gripper_spawner]
        )
    )

    pkg_install_dir   = get_package_prefix('robot_arm_moveit_config')
    workspace_root    = os.path.abspath(os.path.join(pkg_install_dir, '..', '..'))
    universal_src     = os.path.join(workspace_root, 'src')
    universal_install = os.path.join(workspace_root, 'install', 'share')

    mesh_path_fix = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{universal_src}:{universal_install}"
    )

    return LaunchDescription([
        mode_arg,           
        mesh_path_fix,
        gazebo,
        clock_bridge,       
        camera_bridge,      
        rsp,
        spawn_entity,
        load_controllers,
        move_group,
        vision_delayed,     
        pnp_node,  
        translator,         
        tracker,
    ])