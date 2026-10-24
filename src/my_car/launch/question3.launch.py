import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from my_car.launch_env import mesa_environment
from launch.actions import TimerAction

def generate_launch_description():
    action_teleop = launch_ros.actions.Node(
    package="my_car",
    executable="teleop_node",
    output='screen',
    prefix=["gnome-terminal --"],
    )
    action_car = launch_ros.actions.Node(
        package="my_car",
        executable="car_node",
        output="screen",
    )
    action_bridge = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            #'/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            #'/initialpose@geometry_msgs/msg/PoseWithCovarianceStamped]ignition.msgs.PoseWithCovariance',
        ],
        output='screen',
    )

    urdf_package_path = get_package_share_directory("my_car")
    rviz_config = os.path.join(urdf_package_path, 'config', 'que3_slam.rviz')
    default_urdf_path = os.path.join(urdf_package_path, 'sdf', 'rebuild_robot.xacro')
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_urdf_path),
        description='加载的 urdf 文件路径'
    )
    command_result = launch.substitutions.Command(
        ['xacro ', launch.substitutions.LaunchConfiguration('model')]
    )
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        command_result, value_type=str
    )

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description' : robot_description_value },
                    {'use_sim_time': True}],
    )
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    action_slam = launch_ros.actions.Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('my_car'), 'config', 'mapper_params_online_async.yaml'), {'use_sim_time': True}],
    )
    action_rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    

# 启动 Gazebo（世界）
    default_world_path = os.path.join(urdf_package_path, 'sdf', 'world.sdf')
    action_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={ 'gz_args': ['-r -v 3 ', default_world_path],}.items(),
    )

# 把 URDF spawn 进 Gazebo
    action_spawn = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'Mecanum_car', '-topic', '/robot_description','-z', '0.7'],
        output='screen',
    )
    action_slam_rviz_delayed = TimerAction(
    period=5.0,
    actions=[action_slam, action_rviz],
)

    action_spawn_delayed = TimerAction(
        period=3.0,
        actions=[action_spawn],
    )

    actions_mesa_env = [
            launch.actions.SetEnvironmentVariable(name, value)
            for name, value in mesa_environment().items()
        ]


    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_teleop,
        action_bridge,
        action_car,
        action_robot_state_publisher,
        action_joint_state_publisher,
        *actions_mesa_env,
        action_gz_sim,
        action_spawn_delayed,
        action_slam_rviz_delayed,
    ])