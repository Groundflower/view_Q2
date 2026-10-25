import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener


class GoalForwarder(Node):
    def __init__(self):
        super().__init__('goal_forwarder')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, qos)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.busy = False
        self.timer_wp = None
        self.timer_final = None

        # 矩形区域
        self.zone_min_x = 3.59
        self.zone_max_x = 4.52
        self.zone_min_y = 2.91
        self.zone_max_y = 4.17
        # 路点坐标
        self.wp_x = 5.12
        self.wp_y = 3.49

    def in_zone(self, x, y):
        return (self.zone_min_x <= x <= self.zone_max_x and
                self.zone_min_y <= y <= self.zone_max_y)

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'TF failed: {e}')
            return None

    def goal_cb(self, msg: PoseStamped):
        self.get_logger().info(
            f'收到目标: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        if self.busy:
            self.get_logger().warn('busy，丢弃')
            return

        gx, gy = msg.pose.position.x, msg.pose.position.y
        robot = self.get_robot_pose()
        if robot is None:
            self.get_logger().warn('TF 失败，透传原始目标')
            self.send_goal(msg)
            return

        rx, ry = robot
        need_forward = self.in_zone(gx, gy) or self.in_zone(rx, ry)
        self.get_logger().info(
            f'robot=({rx:.2f},{ry:.2f}) goal=({gx:.2f},{gy:.2f}) forward={need_forward}')

        if not need_forward:
            self.get_logger().info('不触发，透传原始目标')
            self.send_goal(msg)
            return

        self.busy = True
        self.get_logger().info('触发转发，1 秒后发路点')
        self.timer_wp = self.create_timer(1.0, lambda: self.send_waypoint(msg))

    def send_waypoint(self, original_msg):
        if self.timer_wp is not None:
            self.timer_wp.cancel()
            self.destroy_timer(self.timer_wp)
            self.timer_wp = None

        wp = PoseStamped()
        wp.header.frame_id = 'map'
        wp.header.stamp = self.get_clock().now().to_msg()
        wp.pose.position.x = self.wp_x
        wp.pose.position.y = self.wp_y
        wp.pose.orientation.w = 1.0
        self.get_logger().info(f'发送路点: ({self.wp_x:.2f}, {self.wp_y:.2f})')
        self.send_goal(wp)

        self.get_logger().info('30 秒后发原始目标')
        self.timer_final = self.create_timer(30.0, lambda: self.finish(original_msg))

    def finish(self, original_msg):
        if self.timer_final is not None:
            self.timer_final.cancel()
            self.destroy_timer(self.timer_final)
            self.timer_final = None

        self.get_logger().info(
            f'发送原始目标: ({original_msg.pose.position.x:.2f}, {original_msg.pose.position.y:.2f})')
        self.send_goal(original_msg)
        self.busy = False

    def send_goal(self, pose: PoseStamped):
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 action server not available')
            self.busy = False
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.client.send_goal_async(goal)


def main():
    rclpy.init()
    node = GoalForwarder()
    rclpy.spin(node)
    rclpy.shutdown()