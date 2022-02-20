from interfaces.motors import PublisherMotors
import rclpy

def debug(cad):
    f = open("mydebug", "a")
    f.write(cad)
    f.close()
    
class HAL:

    def __init__(self):
    	# init node
    	rclpy.init()
    	
    	# creating publishers
    	self.motors = PublisherMotors("cmd_vel", 4, 0.3)
    	# creagin subscribers

    def setV(self, velocity):
    	debug("setV: " + str(velocity) + "\n")
    	self.motors.sendV(velocity)
    
    def setW(self, velocity):
    	debug("setW: " + str(velocity) + "\n")
    	self.motors.sendW(velocity)
