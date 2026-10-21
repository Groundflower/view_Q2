"""Nav2 导航：地图 + AMCL + 规划/控制 + RViz2（可选连 Gazebo 一起拉起来）。

    ros2 launch my_car navigation2.launch.py                        # 全向轮世界 + nav2
    ros2 launch my_car navigation2.launch.py sim:=false             # Gazebo 已经开着，只起 nav2
    ros2 launch my_car navigation2.launch.py map:=/path/to/x.yaml   # 换地图
    ros2 launch my_car navigation2.launch.py world:=diff_car_world.sdf
    ros2 launch my_car navigation2.launch.py initial_pose:="2.0 3.0 1.57"   # 车不在原点起步
    ros2 launch my_car navigation2.launch.py initial_pose:=none            # 不自动定位

=========================== 全向轮 / 差速轮两个世界 ===========================
config/nav2_params.yaml 是按全向轮（Mecanum）配的：DWB 有 max_vel_y、AMCL 用
OmniMotionModel。世界选 diff_car_world.sdf 时差速底盘吃不下 linear.y，横移指令
会被直接忽略，车只能靠前进 + 原地转向绕过去 —— 能跑，但路线会比全向别扭。要以
差速方式跑就把 nav2_params.yaml 里 FollowPath 的 max_vel_y / min_vel_y、
controller_server 的 min_y_velocity_threshold、velocity_smoother 的 y 分量都设 0。

=========================== 和 slam_mapping 的关系 ===========================
两者是**二选一**，不要同时跑：slam_mapping 用 slam_toolbox 发 map -> odom，
nav2 用 AMCL 发同一个变换，同时开会互相打架（TF 树重根、跳变）。标准流程：

    1) ros2 launch my_car slam_mapping.launch.py        # 建图，遥控跑一圈
    2) ros2 run nav2_map_server map_saver_cli -f ~/my_map   # 存图（~/my_map.pgm + .yaml）
    3) 把地图放进 src/my_car/maps/（或直接用 ~/my_map.yaml 传 map:=）
    4) ros2 launch my_car navigation2.launch.py         # 导航

=============================== 第一次导航 ===============================
AMCL 在拿到初始位姿之前不会发 map -> odom。global_costmap 激活时要查这个
变换，查不到就一直等，所以你会看到 planner_server / bt_navigator 停在 inactive、
日志刷 "Timed out waiting for transform from base_footprint to map"，
而且此时 /navigate_to_pose 这个 action 根本还没出现 —— 这是正常现象，不是坏了。

本 launch 默认会自动发一次初始位姿（initial_pose_publisher 节点，等 AMCL 订阅
/initialpose 之后发 "0 0 0"，也就是原点朝 +x，正好是仿真里车每次的起步位置），
所以上面那堆日志会自己消失、节点自动转 active。车不在原点起步时：

    ros2 launch my_car navigation2.launch.py initial_pose:="2.0 3.0 1.57"
    ros2 launch my_car navigation2.launch.py initial_pose:=none   # 关掉，改回手点

initial_pose 的格式是 "x y yaw"（米 / 米 / 弧度）或 "x,y,yaw"。不管你设没设对，
随时都能在 RViz 里再点一次【2D Pose Estimate】纠正（它和自动发的是同一个话题）。

Fixed Frame 用 map，导航过程中常看的话题：
    /map、/scan、/plan（全局路径）、/local_plan（DWB 轨迹）、/global_costmap/costmap
调试"车不动"时按这条链往下查：
    /cmd_vel 有没有数据 -> 桥接有没有把 /cmd_vel 送进 Gazebo -> 轮子转不转
    （ros2 topic echo /cmd_vel；Gazebo 里 MecanumDrive 收的是 /cmd_vel）
    car_node 只在遥控链路上（/teleop/cmd_vel -> /cmd_vel），导航时不需要它，
    别让它和 nav2 抢同一个 /cmd_vel。
=============================== 坐标系 ===============================
    map -> odom              AMCL 发布
    odom -> base_footprint   Gazebo OdometryPublisher 插件经 bridge 转发
    base_footprint -> chassis -> lidar_link    car_world.launch.py 里的静态 TF
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from my_car.launch_env import mesa_environment

WORLD_CHOICES = ['Mecanum_car_world.sdf', 'diff_car_world.sdf']


def generate_launch_description():
    share = get_package_share_directory('my_car')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(share, 'maps', 'my_map.yaml')
    default_params = os.path.join(share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    sim_arg = DeclareLaunchArgument(
        'sim', default_value='true', choices=['true', 'false'],
        description='是否连 Gazebo 世界 + 桥接一起拉起；Gazebo 已经在跑就传 false')
    world_arg = DeclareLaunchArgument(
        'world', default_value=WORLD_CHOICES[0], choices=WORLD_CHOICES,
        description='share/my_car/sdf 下的世界文件：全向轮 / 差速轮')
    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='地图 yaml 的绝对路径（默认包内 maps/my_map.yaml）')
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='nav2 参数文件（默认包内 config/nav2_params.yaml）')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Gazebo 仿真里必须是 true，否则节点用系统时钟去查仿真时间戳的 TF')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true', choices=['true', 'false'],
        description='是否启动 RViz2（用 nav2_bringup 的默认视图）')
    # 关掉自动定位传 initial_pose:=none（不能传空值，launch 命令行会报
    # "malformed launch argument ... expected format '<name>:=<value>'"）
    initial_pose_arg = DeclareLaunchArgument(
        'initial_pose', default_value='0 0 0',
        description='自动发给 AMCL 的初始位姿 "x y yaw"（米/米/弧度），'
                    '传 none 则关闭自动定位')

    # Gazebo GUI 和 RViz2 都要在启动前拿到新 Mesa 的 EGL，否则 3D 视图黑屏。
    mesa_env = [
        SetEnvironmentVariable(name, value)
        for name, value in mesa_environment().items()
    ]

    # Gazebo + bridge + 静态 TF，世界参数原样透传。
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            share, 'launch', 'car_world.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    # nav2_bringup 会拉起 localization（map_server + amcl）和 navigation
    # （controller/planner/behavior/bt_navigator/velocity_smoother），并统一由
    # lifecycle manager 配好。slam 固定 False：建图由 slam_mapping.launch.py 负责。
    #
    # 注意这里必须写大写的 'False'，不能写 'false'：bringup_launch.py 里是
    # PythonExpression(['not ', slam])，小写 false 在 Python 里是未定义变量，
    # 整个 launch 会以 "name 'false' is not defined" 直接退出。
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam': 'False',
            'autostart': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # 自动定位：这个节点会等 /initialpose 出现订阅者（AMCL 是在 on_activate 里
    # 才创建该订阅的，所以"有订阅者"就等于"AMCL 已激活"），然后发一次位姿并退出。
    # use_sim_time 必须跟着传：header.stamp 若用系统时间，AMCL 拿它去查
    # base_footprint -> odom 会 extrapolation 失败，位姿就等于白发了。
    initial_pose = Node(
        package='my_car',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen',
        parameters=[{
            'initial_pose': LaunchConfiguration('initial_pose'),
            'frame_id': 'map',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        # initial_pose:=none 时整个节点都不启动
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('initial_pose'),
             "'.strip().lower() not in ('', 'none', 'off')"])),
    )

    # 等 Gazebo 起来、/clock 和 /tf 稳定再起 nav2：AMCL 一上来就查
    # odom -> base_footprint，查不到会刷一屏 transform 超时，虽然能自愈但很吓人。
    return LaunchDescription(mesa_env + [
        sim_arg, world_arg, map_arg, params_arg, use_sim_time_arg, use_rviz_arg,
        initial_pose_arg,
        world_launch,
        TimerAction(period=5.0, actions=[nav2, initial_pose, rviz]),
    ])
