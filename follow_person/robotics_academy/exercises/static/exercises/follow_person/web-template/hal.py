from interfaces.motors import PublisherMotors
from interfaces.laser import ListenerLaser
import threading
import rclpy
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
        
    def setV(self, velocity):
    	self.motors.sendV(velocity)
    
    def setW(self, velocity):
    	self.motors.sendW(velocity)
    
    def getLaserData(self):
    	return self.laser.getLaserData()

