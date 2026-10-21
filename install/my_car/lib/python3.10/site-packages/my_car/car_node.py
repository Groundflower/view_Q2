#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CarNode(Node):
    def __init__(self):
        super().__init__('car_node')
        # 安全限制参数
        self.MAX_LIN = 0.8   # m/s (前后/左右最大线速度)
        self.MAX_ANG = 2.0   # rad/s (最大角速度)
        self.TIMEOUT = 0.5   # s (超时刹车时间)

        self.last_msg_time = self.get_clock().now()
        self.cmd = Twist()

        self.sub = self.create_subscription(Twist, '/teleop/cmd_vel', self.on_teleop, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info('car_node up: relaying /teleop/cmd_vel -> /cmd_vel')

    def on_teleop(self, msg: Twist):
        # 1. 提取三个核心速度
        target_x = msg.linear.x
        target_y = msg.linear.y   # 【新增】全向轮横移
        target_z = msg.angular.z

        # 2. 防 NaN 处理
        if math.isnan(target_x): target_x = 0.0
        if math.isnan(target_y): target_y = 0.0
        if math.isnan(target_z): target_z = 0.0

        # 3. 限幅处理
        self.cmd.linear.x  = max(-self.MAX_LIN, min(self.MAX_LIN, target_x))
        self.cmd.linear.y  = max(-self.MAX_LIN, min(self.MAX_LIN, target_y))
        self.cmd.angular.z = max(-self.MAX_ANG, min(self.MAX_ANG, target_z))
        
        # 4. 更新心跳（非常重要）
        self.last_msg_time = self.get_clock().now()

    def on_timer(self):
        dt = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        
        if dt > self.TIMEOUT:
            # 【修改】超时判断：只要 x, y, z 任何一个不为 0，就全部强制清零
            if self.cmd.linear.x != 0.0 or self.cmd.linear.y != 0.0 or self.cmd.angular.z != 0.0:
                self.cmd = Twist()  # 全部归零
                self.get_logger().warn('Watchdog: 信号丢失，强制刹车！', throttle_duration_sec=1.0)
            
        self.pub.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist()) # 退出前发一次停止
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()