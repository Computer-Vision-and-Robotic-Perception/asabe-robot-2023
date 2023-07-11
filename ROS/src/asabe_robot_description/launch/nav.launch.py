from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    urdf_tutorial_path = get_package_share_path('asabe_robot_description')
    default_model_path = str(urdf_tutorial_path / 'urdf/robot_description.urdf')
    default_rviz_config_path = str(urdf_tutorial_path / 'rviz/nav.rviz')
    controller_config = str(urdf_tutorial_path / 'config/control.yaml')
    ekf_config = str(urdf_tutorial_path / 'config/ekf.yaml')
    nav_params = str(urdf_tutorial_path / 'config/nav.yaml')
    map_yaml_file = str(urdf_tutorial_path / 'maps/map.yaml')

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    
    controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description}, controller_config],
            output="both",
        )
    
    joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="both",
        )
    
    base_trajectory_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["base_trajectory_controller", "-c", "/controller_manager"],
            output="both",
        )

    base_diff_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["base_diff_controller", "-c", "/controller_manager"],
            output="both",
        )
    
    arm_trajectory_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_trajectory_controller", "-c", "/controller_manager"],
            output="both",
        )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rplidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('rplidar_ros'), 
                                  'launch', 'rplidar_a1_launch.py'])
        ]), launch_arguments={
            'serial_port': '/dev/ttyAMA3',
            'frame_id': 'Link_lidar',
            'inverted': 'false',
        }.items()
        )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': False}],
        )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('asabe_robot_description'),
            'launch', 'nav2_bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'slam': 'False',            
            'use_sim_time': 'False',
            'use_composition': 'False',
            'use_namespace': 'False',
            'map': str(map_yaml_file),
            'params_file': nav_params,
            'log_level': 'info'
        }.items(),
        )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='both',
    #     arguments=['-d', LaunchConfiguration('rvizconfig')],
    #     )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        controller_manager,
        joint_state_broadcaster,
        # base_trajectory_controller,
        base_diff_controller,
        arm_trajectory_controller,
        robot_state_publisher_node,
        robot_localization_node,
        rplidar_node,
        nav2_launch,
        # rviz_node,
    ])
