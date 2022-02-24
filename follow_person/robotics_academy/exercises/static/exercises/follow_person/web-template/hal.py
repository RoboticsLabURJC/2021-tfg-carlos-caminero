from interfaces.motors import PublisherMotors
from interfaces.laser import ListenerLaser
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
import time
from datetime import datetime

def debug(cad):
    f = open("mydebug", "a")
    f.write(cad)
    f.close()
    
class HAL:

    def __init__(self):
    	# init node
    	rclpy.init()
    	
    	self.motors = PublisherMotors("cmd_vel", 4, 0.3)
    	self.laser = ListenerLaser("scan")
    	
    	self.listener_executor = MultiThreadedExecutor(num_threads=4)
    	self.listener_executor.add_node(self.laser)
    	
    	# Update thread
    	self.thread = ThreadHAL(self.listener_executor)
    
    def start_thread(self):
    	self.thread.start()
    	
    def setV(self, velocity):
    	self.motors.sendV(velocity)
    
    def setW(self, velocity):
    	self.motors.sendW(velocity)
    
    def getLaserData(self):
    	return self.laser.getLaserData()


class ThreadHAL(threading.Thread):
    def __init__(self, executor):
        super(ThreadHAL, self).__init__()
        self.executor = executor

    def run(self):
    	try:
    	    self.executor.spin()
    	finally:
    	    self.executor.shutdown()
