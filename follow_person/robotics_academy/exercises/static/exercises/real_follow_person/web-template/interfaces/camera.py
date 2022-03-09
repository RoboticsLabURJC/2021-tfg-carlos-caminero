import cv2

class Camera():
 
    def __init__(self, dev):
        self.cap = cv2.VideoCapture(dev)
        
    def stop(self):
        self.cap.release()
    
    def getImage(self):
    	ret, frame = self.cap.read()
    	return ret, frame
