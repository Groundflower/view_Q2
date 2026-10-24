#!/usr/bin/env python3
import select
import sys
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP_HOLO = """
  W/S : 前后平移   A/D : 左右平移
  Q/E : 原地左/右转
  SPACE : 急停     CTRL+C : 退出
"""

class Teleop(Node):
    def __init__(self):
        super().__init__('tele_node')
        self.pub = self.create_publisher(Twist, '/teleop/cmd_vel', 10)
        self.speed = 0.8
        self.turn = 1.0
        self.last_key_time = time.time()
        self.key_timeout = 0.1

        self._old_settings = termios.tcgetattr(sys.stdin)

    def spin_keyboard(self):
        tty.setcbreak(sys.stdin.fileno())
        print(HELP_HOLO)
        try:
            while rclpy.ok():
                x = y = yaw = 0.0

                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1).lower()
                    if key == '\x03':
                        break
                    self.last_key_time = time.time()

                    if key == 'w': x = self.speed
                    elif key == 's': x = -self.speed
                    elif key == 'a': y = self.speed
                    elif key == 'd': y = -self.speed
                    elif key == 'q': yaw = self.turn
                    elif key == 'e': yaw = -self.turn
                    elif key == ' ': pass

                elif (time.time() - self.last_key_time) > self.key_timeout:
                    x = y = yaw = 0.0

                msg = Twist()
                msg.linear.x = x
                msg.linear.y = y
                msg.angular.z = yaw
                self.pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            self.pub.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)


def main():
    rclpy.init()
    node = Teleop()
    try:
        node.spin_keyboard()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()