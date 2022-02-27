# Carlos Caminero
# My HAL. Only to test publishers and subscribers

import rclpy
import geometry_msgs.msg
import numpy as np
import cv2

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError


def imageMsg2Image(img, bridge):
    cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
    return cv_image


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


class DarknetRos(Node):
    def __init__(self):
        super().__init__("darknet_ros_node")
        
        self.bounding_boxes_susbscription = self.create_subscription(
            BoundingBoxes,
            'darknet_ros/bounding_boxes',
            self.boundingBoxesCallback,
            10)
        self.found_object_subscription = self.create_subscription(
            ObjectCount,
            'darknet_ros/found_object',
            self.foundObjectCallback,
            10
        )

        self.bounding_boxes_susbscription
        self.found_object_subscription

        self.bounding_boxes = []
        self.count_objects = 0


    def boundingBoxesCallback(self, msg):
        self.bounding_boxes = msg.bounding_boxes

    def getBoundingBoxList(self):
        return self.bounding_boxes
    
    def foundObjectCallback(self, msg):
        self.count_objects = msg.count
    
    def getNumObjects(self):
        return self.count_objects


class Laser(Node):
    
    def __init__(self):
        super().__init__("laser_node")
        
        self.laser_subscription = self.create_subscription(
            LaserScan,
            "scan",
            self.laserScanCallback,
            10)
        
        self.laser_subscription

        self.laser_data = None
    
    def laserScanCallback(self, msg):
        self.laser_data = msg
    
    def getLaserData(self):
        return self.laser_data


class Camera(Node):

    def __init__(self):
        super().__init__("camera_node")

        self.camera_subscription = self.create_subscription(
            Image,
            "/depth_camera/image_raw",
            self.cameraCallback,
            10)
        
        self.bridge = CvBridge()
        self.camera_subscription
        self.image = np.zeros((3, 3, 3), np.uint8)
    
    def cameraCallback(self, img):
        self.image = imageMsg2Image(img, self.bridge)
        #cv2.imshow("Imagen", imageMsg2Image(img, self.bridge))
    
    def getImage(self):
        return self.image


rclpy.init()
velocity_node = VelocityPublisher()
darknet_ros_node = DarknetRos()
laser_node = Laser()
camera_node = Camera()


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

def getBoundingBoxes():
    return darknet_ros_node.getBoundingBoxList()

def getNumObjects():
    return darknet_ros_node.getNumObjects()

def getLaserData():
    ldata = laser_node.getLaserData()
    if ldata != None:
        v1 = ldata.ranges[90:0:-1]
        v1.append(ldata.ranges[0])
        v2 = ldata.ranges[359:270:-1]
        return v1+v2
    return None

def getImage():
    return camera_node.getImage()


def main(user_main, num_threads=4, args=None):

    executor = MultiThreadedExecutor(num_threads=num_threads)

    executor.add_node(velocity_node)
    executor.add_node(darknet_ros_node)
    executor.add_node(laser_node)
    executor.add_node(camera_node)
    try:
        while rclpy.ok():
            user_main()
            executor.spin_once()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()

    rclpy.shutdown()



