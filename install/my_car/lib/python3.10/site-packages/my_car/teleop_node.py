#!/usr/bin/env python3
import argparse
import select
import sys
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP_DIFF = """
------------- 差速轮遥控模式 (默认) -------------
  W : 前进        S : 后退
  A : 左转        D : 右转
  SPACE : 急停    Q : 退出
-------------------------------------------------
"""

HELP_HOLO = """
------------- 全向轮遥控模式 (全向) -------------
  W/S : 前后平移   A/D : 左右平移
  Q/E : 原地左/右转
  SPACE : 急停     CTRL+C : 退出
-------------------------------------------------
"""

class Teleop(Node):
    def __init__(self, holonomic_mode=False):
        super().__init__('tele_node')
        self.holonomic = holonomic_mode

        self.pub = self.create_publisher(Twist, '/teleop/cmd_vel', 10)

        self.speed = 0.8    # m/s
        self.turn = 1.0     # rad/s

        # 保存最后一次按键的时间，用来实现“松手刹车”
        self.last_key_time = time.time()
        self.key_timeout = 0.1 # 100毫秒没按键，就认为松手了

        # Save terminal settings
        self._old_settings = termios.tcgetattr(sys.stdin)

    def spin(self):
        tty.setcbreak(sys.stdin.fileno())
        print(HELP_HOLO if self.holonomic else HELP_DIFF)
        
        try:
            while rclpy.ok():
                x = 0.0
                y = 0.0
                yaw = 0.0

                # 检查是否有按键输入
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1).lower()
                    
                    if key == 'q' and not self.holonomic:
                        break
                    elif key == '\x03': # Ctrl+C
                        break
                    
                    self.last_key_time = time.time() # 刷新按键时间

                    # 根据按键设置当前这一帧的速度
                    if key == 'w': x = self.speed
                    elif key == 's': x = -self.speed
                    elif key == 'a':
                        if self.holonomic: y = self.speed
                        else: yaw = self.turn
                    elif key == 'd':
                        if self.holonomic: y = -self.speed
                        else: yaw = -self.turn
                    elif key == 'q' and self.holonomic: yaw = self.turn
                    elif key == 'e' and self.holonomic: yaw = -self.turn
                    elif key == ' ':
                        pass # 空格就是强制发 0 速度
                
                # 【修复核心】：判断距离上次按键过了多久
                # 终端在按住按键时会疯狂发送字符，所以只要一直按着，时间差就很短
                # 一旦松手，时间差超过 0.1 秒，速度就会自动归零
                if (time.time() - self.last_key_time) > self.key_timeout:
                    x = 0.0
                    y = 0.0
                    yaw = 0.0

                msg = Twist()
                msg.linear.x = x
                msg.linear.y = y
                msg.angular.z = yaw
                self.pub.publish(msg)
                
        finally:
            self.pub.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)

def main():
    parser = argparse.ArgumentParser(description='WASD Teleop Node')
    parser.add_argument('--alltoward', action='store_true', help='开启全向轮模式')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Teleop(holonomic_mode=args.alltoward)
    try:
        node.spin()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node._old_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()