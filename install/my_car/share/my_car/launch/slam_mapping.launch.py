"""一键建图：Gazebo 世界 + 桥接 + slam_toolbox + RViz2。

    ros2 launch my_car slam_mapping.launch.py                        # 全向轮世界
    ros2 launch my_car slam_mapping.launch.py world:=diff_car_world.sdf

车不会自己动，地图要车跑起来才长得出来。另开一个终端：
    ros2 run my_car teleop_node --holonomic   # 键盘遥控（A/D 横移），发到 /teleop/cmd_vel
    ros2 run my_car car_node                  # 把 /teleop/cmd_vel 限速后转发到 /cmd_vel

全向车一定要加 --holonomic，否则 A/D 是原地转、没有横移；跑差速世界时
把 --holonomic 去掉即可（那时候 A/D 就是原地转）。

RViz2 的 Fixed Frame 是 map，正常情况下能看到：
    LaserScan (/scan)、Map (/map)、Odometry (/odom)、TF 四条坐标系
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from my_car.launch_env import mesa_environment

WORLD_CHOICES = ['Mecanum_car_world.sdf', 'diff_car_world.sdf']


def generate_launch_description():
    share = get_package_share_directory('my_car')
    params = os.path.join(share, 'config', 'mapper_params_online_async.yaml')
    rviz_config = os.path.join(share, 'config', 'slam_mapping.rviz')

    # 世界文件的选择透传给 car_world.launch.py（它自己声明了同名参数）
    world_arg = DeclareLaunchArgument(
        'world', default_value=WORLD_CHOICES[0], choices=WORLD_CHOICES,
        description='share/my_car/sdf 下的世界文件：全向轮 / 差速轮')

    # Gazebo GUI 和 RViz2 都要在启动前拿到新 Mesa 的 EGL，否则 3D 视图黑屏。
    mesa_env = [
        SetEnvironmentVariable(name, value)
        for name, value in mesa_environment().items()
    ]

    # 仿真 + 桥接 + 静态 TF 全部在 car_world.launch.py 里。
    # 它自己带 on_exit_shutdown，Gazebo 一退出这里的所有节点都会跟着收掉。
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            share, 'launch', 'car_world.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params, {'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    # 等 Gazebo 起来、桥接把 /clock 和 /tf 建好再启动 SLAM。
    # slam_toolbox 自己也会等 TF，但不等的话开机会刷一屏
    # "Failed to compute odom pose"，容易让人误以为又坏了。
    return LaunchDescription(mesa_env + [
        world_arg,
        world_launch,
        TimerAction(period=5.0, actions=[slam, rviz]),
    ])
