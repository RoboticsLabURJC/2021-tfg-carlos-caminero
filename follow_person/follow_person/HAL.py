import rclpy
import geometry_msgs.msg

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class VelocityPublisher(Node):

    def __init__(self):
        super().__init__("velocity_publisher_node")

        self.v = 0.0
        self.w = 0.0
        self.pub = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
    
    def getV(self):
        return self.v
    
    def getW(self):
        return self.w
    
    def setV(self, num):
        self.v = num

    def setW(self, num):
        self.w = num
    
    def timer_callback(self):
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = float(self.v)
        msg.angular.z = float(self.w)
        self.pub.publish(msg)



rclpy.init()
velocity_node = VelocityPublisher()

# -- HAL Functions --
#####################
def getW():
    return velocity_node.getW()

def getV():
    return velocity_node.getV()

def setV(num):
    velocity_node.setV(num)

def setW(num):
    velocity_node.setW(num)



def main(user_main, num_threads=4, args=None):

    executor = MultiThreadedExecutor(num_threads=num_threads)

    executor.add_node(velocity_node)
    try:
        while rclpy.ok():
            user_main()
            executor.spin_once()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()

    rclpy.shutdown()



