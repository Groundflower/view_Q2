"""
发布一次 AMCL 的初始位姿，然后自己退出.

AMCL 在收到 /initialpose 之前不会发布 map -> odom，于是 global_costmap 激活时
查不到 map 帧、planner_server / bt_navigator 一直 inactive、/navigate_to_pose
这个 action 也不出现。本节点就是替你去 RViz 点那一下【2D Pose Estimate】。

用法（一般由 navigation2.launch.py 拉起，不用手敲）：

    ros2 run my_car initial_pose_publisher --ros-args \
        -p initial_pose:="0 0 0" -p frame_id:=map

initial_pose 是 "x y yaw" 三个数（米 / 米 / 弧度），逗号或空格分隔都行。

为什么不用 ros2 topic pub --once：那玩意儿要自己拼一大串 YAML（还得手写
协方差），而且在 AMCL 还没激活、订阅者数量为 0 的时候会一直干等。这里改成
"盯着 /initialpose 的订阅者，出现了再发一次" —— AMCL 是在 on_activate 里
才创建这个订阅的，所以订阅者出现就等于"AMCL 已经激活"，比盲发稳。
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    """等 /initialpose 有订阅者后发一次位姿，发完退出."""

    def __init__(self):
        """读取参数，起一个轮询订阅者的定时器."""
        super().__init__('initial_pose_publisher')

        self.declare_parameter('initial_pose', '0 0 0')
        self.declare_parameter('frame_id', 'map')
        # 秒。用 wall clock 计时，仿真时钟还没走起来也能正常超时退出
        self.declare_parameter('timeout', 60.0)

        raw = self.get_parameter('initial_pose').value
        self.frame_id = self.get_parameter('frame_id').value
        self.timeout = float(self.get_parameter('timeout').value)
        self.x, self.y, self.yaw = self._parse(raw)

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.sent = False
        self._deadline = time.monotonic() + self.timeout
        # 0.2s 轮询：AMCL 订阅一出现基本立刻发出去
        self.timer = self.create_timer(0.2, self._tick)

        self.get_logger().info(
            f'等待 AMCL 订阅 /initialpose，随后发布 '
            f'{self.frame_id} 下的 ({self.x}, {self.y}, {self.yaw} rad)')

    def _parse(self, raw):
        """把 "x y yaw" 解析成三个 float，错了直接抛异常（比静默用 0 强）."""
        try:
            values = [float(v) for v in str(raw).replace(',', ' ').split()]
        except ValueError:
            values = []
        if len(values) != 3:
            raise ValueError(
                f'initial_pose 要写成 "x y yaw" 三个数，收到的是：{raw!r}')
        return values

    def _tick(self):
        if self.sent:
            # 上一步已经发出去，留一点时间让消息真正离开 DDS，再收工
            self.get_logger().info('初始位姿已发布，节点退出')
            self.timer.cancel()
            rclpy.shutdown()
            return

        if self.pub.get_subscription_count() < 1:
            if time.monotonic() > self._deadline:
                self.get_logger().warn(
                    f'{self.timeout:.0f}s 内没等到 AMCL 订阅 /initialpose，'
                    '放弃自动定位，请在 RViz 里手动点 2D Pose Estimate')
                self.timer.cancel()
                rclpy.shutdown()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        # 和 RViz 拖 2D Pose Estimate 的默认协方差同量级（x/y 0.5m、yaw 0.07rad），
        # AMCL 用它铺初始粒子云；全 0 会让粒子全挤在一个点上，反而更难收敛
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.07,
        ]
        self.pub.publish(msg)
        self.sent = True


def main(args=None):
    """入口：发完位姿节点自己结束，不用外部 kill."""
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
