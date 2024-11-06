from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction , DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file_path = '/home/aiia/madesh/src/LSS-ROS2-Humanoid/urdf/lss_humanoid-croccodyl.urdf'
    assert os.path.exists(urdf_file_path), f"URDF file does not exist: {urdf_file_path}"

    gazebo_launch_path = os.path.join(
        '/opt/ros/humble/share/gazebo_ros', 'launch', 'gazebo.launch.py'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )

    controller_config = os.path.join(
        get_package_share_directory('lss_humanoid'),
        'LSS-ROS2-HUMANOID'
        'config'
        'controller.yaml'
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'lss_humanoid',
            '-file', urdf_file_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',
            '-Y', '-1.0',
            '-R', '0.0',
            '-P', '0.06'
        ],
        output='screen'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[{
            'update_rate': 50,  # 50 Hz
        }],
        output='screen'
    )

    robot = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_file_path).read()
        }],
        output='screen'
    )

    # Adding delay before spawning controllers
    delay_time = 1.0

    joint_state_spawner = TimerAction(
        period=delay_time,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            )
        ]
    )

    position_controller_spawner = TimerAction(
        period=delay_time,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['position_controllers'],  # Ensure this matches your defined controllers
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        spawn,
        robot,
        controller_manager,
        joint_state_spawner,
        position_controller_spawner,

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['LHumerus_position_controller', '--controller-manager', '/controller_manager'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['LRadius_position_controller', '--controller-manager', '/controller_manager'],
        ),
    ])
