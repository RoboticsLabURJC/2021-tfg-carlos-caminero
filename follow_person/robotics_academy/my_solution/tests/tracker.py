import cv2
import numpy as np
import math
from coco_labels import LABEL_MAP


FROZEN_GRAPH = "./trained_model/ssd_inception_v2_coco.pb"
PB_TXT = "./trained_model/ssd_inception_v2_coco.pbtxt"
SIZE = 300

wnd_name = "Camera Window"

# ROBOT STATES
SEARCH_PERSON = 0
FOLLOW_PERSON = 1

state = SEARCH_PERSON

vels = [0.3, 0.2, 0.1, 0.0, -0.1, -0.2, -0.3]


class BoundingBox:

	def __init__(self, identifier, class_id, score, xmin, ymin, xmax, ymax):
		self.id = identifier
		self.class_id = class_id
		self.score = score
		self.xmin = xmin
		self.ymin = ymin
		self.xmax = xmax
		self.ymax = ymax
	

	def __str__(self):
		s = "[id:{}\nclass:{}\nscore:{}\nxmin:{}\nymin:{}\nxmax:{}\nymax:{}\n".format(
			self.id, self.class_id, self.score, self.xmin, self.ymin, self.xmax, self.ymax)
		return s



class NeuralNetwork:

	def __init__(self):
		self.net = cv2.dnn.readNetFromTensorflow(FROZEN_GRAPH, PB_TXT)
		

	def detect(self, img):
		rows = img.shape[0]
		cols = img.shape[1]
		self.net.setInput(cv2.dnn.blobFromImage(img, 1.0/127.5, (SIZE, SIZE), (127.5, 127.5, 127.5), swapRB=True, crop=False))
		cvOut = self.net.forward()
		
		return cvOut[0, 0, :, :]



class BoundingBoxObject:

    def __init__(self, bounding_box):
        self.bounding_box = bounding_box
        self.centroid = centroid_bounding_box(bounding_box)
        self.area = area_bounding_box(bounding_box)
    


class Tracker:

    def __init__(self, obj=None):
        self.obj = None
        self.limit = 50

        self.memory_activated = True



    def setObjective(self, obj):
        self.obj = obj
        self.memory_activated = True
    

    def getObjective(self):
        return self.obj
    

    def getObjectiveFromSet(self, objlist):

        closer_obj_i = 0
        min_distance = 9999

        for index in range(len(objlist)):
            dist = euclidean_distance(self.obj.centroid, objlist[index].centroid)
            if dist < min_distance:
                min_distance = dist
                closer_obj_i = index
        
        if min_distance > self.limit:
            return (self.obj, -1)
        
        if abs(objlist[closer_obj_i].area - self.obj.area) > 30000:
            return (self.obj, -1)

        self.obj = objlist[closer_obj_i]

        return (self.obj, closer_obj_i)


def bounding_box_properties(bbox):
    name = bbox.class_id
    xmin = bbox.xmin
    ymin = bbox.ymin
    xmax = bbox.xmax
    ymax = bbox.ymax
    return name, xmin, ymin, xmax, ymax


def draw_bounding_box(img, bbox, color=(23, 230, 210), thickness=2):
    _, xmin, ymin, xmax, ymax = bounding_box_properties(bbox)
    cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), color, thickness)

def bounding_boxes_by_name(bounding_boxes, name):
    selected = []
    for bounding_box in bounding_boxes:
        if bounding_box.class_id == name:
            selected.append(bounding_box)
    return selected


def bounding_boxes_by_score(bounding_boxes, score_limit):
    selected = []
    for bounding_box in bounding_boxes:
        if bounding_box.score > score_limit:
            selected.append(bounding_box)
    return selected


def get_bounding_boxes(img):
    detections = net.detect(img)

    rows = img.shape[0]
    cols = img.shape[1]

    bounding_boxes = []
    for detection in detections:
        bounding_box = BoundingBox(
            int(detection[1]),
            LABEL_MAP[int(detection[1])],
            float(detection[2]),
            detection[3]*cols,
            detection[4]*rows,
            detection[5]*cols,
            detection[6]*rows)
        bounding_boxes.append(bounding_box)
    
    return bounding_boxes


def max_bounding_box(bounding_boxes):
    max_area = 0
    max_index = -1
    for i in range(len(bounding_boxes)):
        area = area_bounding_box(bounding_boxes[i])
        if area > max_area:
            max_area = area
            max_index = i
    
    if max_index == -1:
        return None
    return bounding_boxes[max_index]


def area_bounding_box(bbox):
    _, xmin, ymin, xmax, ymax = bounding_box_properties(bbox)
    area = (xmax - xmin) * (ymax - ymin)
    return area


def euclidean_distance(p1, p2):
    return math.sqrt(math.pow(abs(p2[0]-p1[0]), 2) + math.pow(abs(p2[1]-p1[1]), 2))



def centroid_bounding_box(bbox):
    cx = int((bbox.xmin + bbox.xmax)/2)
    cy = int((bbox.ymin + bbox.ymax)/2)
    return (cx, cy)




# Main program
net = NeuralNetwork()

cap = cv2.VideoCapture(4)

tracker = Tracker()

tracking_failure_cont = 0

while True:

    ret, img = cap.read()

    if ret:

        bounding_boxes = get_bounding_boxes(img)

        filter1 = bounding_boxes_by_score(bounding_boxes, 0.5)
        filter2 = bounding_boxes_by_name(filter1, "person")
        
        if state == SEARCH_PERSON:
            if len(filter2) > 0:
                
                for bbox in filter2:
                    draw_bounding_box(img, bbox, color=(0, 255, 0), thickness=3)
                max_bbox = max_bounding_box(filter2)

                candidate2follow = BoundingBoxObject(max_bbox)
                
                if candidate2follow.centroid[0] > img.shape[1]/3 and candidate2follow.centroid[1] < img.shape[1]*2/3:
                    tracker.setObjective(candidate2follow)
                    state = FOLLOW_PERSON
                    print("Person found")


        elif state == FOLLOW_PERSON:
            if len(filter2) > 0:
                centroids = []
                objects = []
                
                for bbox in filter2:
                    objects.append(BoundingBoxObject(bbox))
                    draw_bounding_box(img, bbox, color=(0, 255, 0), thickness=3)

                # Setting angular velocity according to the centroid of the person to follow
                width = img.shape[1]
                step = width/len(vels)

                object2follow, index = tracker.getObjectiveFromSet(objects)
                if index >= 0:
                    tracking_failure_cont = 0
                    draw_bounding_box(img, filter2[index], color=(0, 0, 255), thickness=3)
                    
                    angular_vel = vels[int(objects[index].centroid[0]/step)]
                    if angular_vel > 0:
                        print("Angular velocity {} to Left".format(angular_vel))
                    elif angular_vel < 0:
                        print("Angular velocity {} to Right".format(angular_vel))
                    else:
                        print("Angular velocity {} to Center".format(angular_vel))
                else:
                    tracking_failure_cont += 1
            
            else:
                tracking_failure_cont += 1
            if tracking_failure_cont > 50:
                state = SEARCH_PERSON
                print("Person lost")

        cv2.imshow(wnd_name, img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_thread = True
            break

cap.release()
cv2.destroyAllWindows()
    