"""启动 Gazebo 世界、ROS 2 <-> Gazebo 桥接，以及机器人静态 TF。

只负责"仿真 + 传感器 + 坐标系"这一层，不启动 SLAM：
    ros2 launch my_car car_world.launch.py                          # 默认全向轮世界
    ros2 launch my_car car_world.launch.py world:=Mecanum_car_world.sdf
    ros2 launch my_car car_world.launch.py world:=diff_car_world.sdf
要建图请用 slam_mapping.launch.py（它会 include 本文件，world 参数同样可传）。

两个世界的几何是同一套（model pose z=0.10、lidar_link 相对 chassis +0.07），
所以下面的静态 TF 数值对两者都成立，只有底盘插件不同：
    Mecanum_car_world.sdf  全向轮（MecanumDrive），可横移
    diff_car_world.sdf     差速轮（DiffDrive），不能横移

=================== 为什么必须桥接 /clock /tf /odom ===================
Gazebo 和 ROS 2 是两个独立进程，话题即使同名也互不可见，必须靠
ros_gz_bridge 显式转发。少任何一个的后果：

  /clock    没有仿真时间 -> 开了 use_sim_time 的节点时钟永远停在 0，
            TF 按时间戳查不到 -> slam_toolbox 刷 "Failed to compute odom pose"
  /tf       没有 odom -> base_footprint，slam_toolbox 同样无从下手
  /odom     RViz 里看不到里程计轨迹
  /scan     没有激光数据，地图永远是空的

============================= TF 树 =============================
    map -> odom -> base_footprint -> chassis -> lidar_link
             |           |              |            |
             |           |              |            +-- 本文件静态发布 (z=0.09)
             |           |              +--------------- 本文件静态发布 (z=0.10)
             |           +------------------------------ 里程计插件直接发布
             +------------------------------------------ slam_toolbox 发布

（差速世界是 DiffDrive 插件发的，全向世界是 OdometryPublisher 发的，见下。）

里程计插件发布的位姿是纯 2D 里程计（只有 x/y/yaw，z 恒为 0），语义上
就是"车体在地面的投影点"，配合 SDF 里的 <child_frame_id>base_footprint</child_frame_id>
正好构成 SLAM / nav2 默认要找的标准 base_footprint。

注意 Mecanum_car_world.sdf 里发布 /odom 和 /tf 的是 OdometryPublisher 插件：
Fortress 6.18 的 MecanumDrive 会把话题广告出来但一条消息都不发（官方
mecanum_drive.sdf 也一样），SLAM 会卡在拿不到 odom->base_footprint。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from my_car.launch_env import mesa_environment

# share/sdf 下两个可选世界（setup.py 会把它们一起装进来）
WORLD_CHOICES = ['Mecanum_car_world.sdf', 'diff_car_world.sdf']


def generate_launch_description():
    share = get_package_share_directory('my_car')

    world_arg = DeclareLaunchArgument(
        'world', default_value=WORLD_CHOICES[0], choices=WORLD_CHOICES,
        description='share/my_car/sdf 下的世界文件：全向轮 / 差速轮')
    world = PathJoinSubstitution([share, 'sdf', LaunchConfiguration('world')])

    # Gazebo GUI 的 3D 视图需要新 Mesa 的 EGL，否则可能黑屏。
    # 必须在拉起 gazebo 之前设好。
    mesa_env = [
        SetEnvironmentVariable(name, value)
        for name, value in mesa_environment().items()
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -v 3 ', world],
            # 关掉 Gazebo（或它自己崩了）时把整条 launch 一起收掉，
            # 否则会留下 bridge / slam_toolbox / rviz 空转。
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # 桥接语法：<话题>@<ROS类型>[<Gazebo类型>
    #   '[' = 单向 Gazebo -> ROS     ']' = 单向 ROS -> Gazebo
    # 全程用单向而非双向，是为了避免 ROS 侧回灌把 Gazebo 的话题搞乱。
    bridge = Node(
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
    chassis_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'chassis',
                   '--z', '0.10'],
    )

    # 雷达：lidar_link 相对 chassis 是 +0.07，传感器在其上再 +0.02，
    # 而 /scan 的 frame_id 是 lidar_link，所以这里要用 0.09 才能让激光
    # 落在真实位置上（离地 0.10 + 0.09 = 0.19m）。
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'chassis', '--child-frame-id', 'lidar_link',
                   '--z', '0.09'],
    )

    return LaunchDescription(mesa_env + [world_arg, gz_sim, bridge,
                                         chassis_tf, lidar_tf])
