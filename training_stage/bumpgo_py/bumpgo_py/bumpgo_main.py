import rclpy
import curio
import time
import async_btree as bt
import geometry_msgs.msg
from rclpy.node import Node
from turtlesim.msg import Pose


class Timer:

    def __init__(self):
        self.i_time = None
        self.c_time = None

    def init(self):
        self.i_time = time.time()
        self.c_time = time.time()

    def reset(self):
        self.init()

    def check(self, seconds):
        self.c_time = time.time()
        return (self.c_time - self.i_time) > seconds


class TreeNode(Node):
    
    def __init__(self):
        super().__init__("bump_and_go_node")
        loop_rate = 0.2
        self.timer = self.create_timer(loop_rate, self.timer_callback)
        self.subscription = self.create_subscription(
            Pose, 'turtle1/pose', self.pose_callback, 10)
        self.pubvel = self.create_publisher(geometry_msgs.msg.Twist, "/turtle1/cmd_vel", 10)

        # -- Behavior Tree Design --------
        self.alert = False
        self.b_tree = bt.sequence(children=[
            bt.decision(
                condition=self.check_alert,
                success_tree=bt.retry_until_success(
                    bt.action(target=self.move_backward, duration=2.5)),
                failure_tree=bt.always_failure(self.move_forward)
                ),
            bt.retry_until_success(bt.action(target=self.turn, duration=3))
        ])
        # ---------------------------------

        self.chron = Timer()
        self.chron.init()

    def check_alert(self):
        if self.alert:
            self.alert = False
            self.chron.reset()
            return bt.SUCCESS
        return bt.FAILURE

    def move_forward(self):
        vel = geometry_msgs.msg.Twist()
        vel.linear.x = 1.5
        vel.angular.z = 0.0
        self.pubvel.publish(vel)

    def turn(self, duration: float):
        vel = geometry_msgs.msg.Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.5
        self.pubvel.publish(vel)
        if self.chron.check(duration) is True:
            self.chron.reset()
            return bt.SUCCESS
        return bt.FAILURE

    def move_backward(self, duration: float):
        vel = geometry_msgs.msg.Twist()
        vel.linear.x = -0.5
        vel.angular.z = 0.0
        self.pubvel.publish(vel)
        if self.chron.check(duration) is True:
            self.chron.init()
            return bt.SUCCESS
        return bt.FAILURE

    def timer_callback(self):
        curio.run(self.b_tree)
        return

    def pose_callback(self, position_msg):
        if not (0.5 < position_msg.x < 10.0) or not (0.5 < position_msg.y < 10.0):
            self.alert = True


def main(args=None):
    rclpy.init(args=args)

    tree_node = TreeNode()
    rclpy.spin(tree_node)
    tree_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
