import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

from my_car.launch_env import mesa_environment


# share/sdf 下两个可选世界（setup.py 会把它们一起装进来）
WORLD_CHOICES = ['Mecanum_car_world.sdf', 'diff_car_world.sdf']


def generate_launch_description():
    # 获取包 share 路径
    share = get_package_share_directory('my_car')

    # 声明 world 参数：默认全向轮世界，可选差速轮世界
    action_declare_arg_world = launch.actions.DeclareLaunchArgument(
        name='world',
        default_value=WORLD_CHOICES[0],
        choices=WORLD_CHOICES,
        description='share/my_car/sdf 下的世界文件：全向轮 / 差速轮'
    )

    # 拼接出完整的世界文件路径
    world_path = launch.substitutions.PathJoinSubstitution(
        [share, 'sdf', launch.substitutions.LaunchConfiguration('world')]
    )

    # Gazebo GUI 的 3D 视图需要新 Mesa 的 EGL，否则可能黑屏。
    # 必须在拉起 gazebo 之前设好。
    actions_mesa_env = [
        launch.actions.SetEnvironmentVariable(name, value)
        for name, value in mesa_environment().items()
    ]

    # 拉起 Gazebo 世界
    action_gz_sim = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 3 ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # 桥接语法：<话题>@<ROS类型>[<Gazebo类型>
    #   '[' = 单向 Gazebo -> ROS     ']' = 单向 ROS -> Gazebo
    # 全程用单向而非双向，是为了避免 ROS 侧回灌把 Gazebo 的话题搞乱。
    action_bridge = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        ],
        output='screen',
    )

    # 底盘中心：Gazebo 里 diff_car 的 model pose 是 z=0.10，chassis 就位于
    # model 原点，所以车体中心离地 0.10m。
    action_chassis_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'chassis',
                   '--z', '0.10'],
    )

    # 雷达：lidar_link 相对 chassis 是 +0.07，传感器在其上再 +0.02，
    # 而 /scan 的 frame_id 是 lidar_link，所以这里要用 0.09 才能让激光
    # 落在真实位置上（离地 0.10 + 0.09 = 0.19m）。
    action_lidar_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'chassis', '--child-frame-id', 'lidar_link',
                   '--z', '0.09'],
    )

    return launch.LaunchDescription(
        actions_mesa_env + [
            action_declare_arg_world,
            action_gz_sim,
            action_bridge,
            action_chassis_tf,
            action_lidar_tf,
        ]
    )