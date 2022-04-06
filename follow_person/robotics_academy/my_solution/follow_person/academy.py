from GUI import GUI
from HAL import HAL
import cv2
import math

SEARCH_PERSON = 0
FOLLOW_PERSON = 1

state = SEARCH_PERSON

vels = [0.3, 0.2, 0.1, 0.0, -0.1, -0.2, -0.3]


class BoundingBoxObject:

    def __init__(self, bounding_box):
        self.bounding_box = bounding_box
        self.centroid = centroid_bounding_box(bounding_box)
        self.area = area_bounding_box(bounding_box)


class Tracker:

    def __init__(self, obj=None):
        self.obj = None
        self.limit = 50


    def setObjective(self, obj):
        self.obj = obj


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


def bounding_boxes_by_area(bounding_boxes, min_area):
    selected = []
    for bounding_box in bounding_boxes:
        if area_bounding_box(bounding_box) > min_area:
            selected.append(bounding_box)
    return selected


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


def parse_laser_data(laser_data):
    values = laser_data.values
    return values[90:0:-1] + values[0:1] + values[360:270:-1]

# Main program
tracker = Tracker()

tracking_failure_cont = 0

while True:
    
    img = HAL.getImage()
    
    bounding_boxes = HAL.getBoundingBoxes(img)

    filter1 = bounding_boxes_by_score(bounding_boxes, 0.3)
    filter2 = bounding_boxes_by_name(filter1, "person")
    filter3 = bounding_boxes_by_area(filter2, 5000)
    
    if state == SEARCH_PERSON:
        HAL.setV(0)
        HAL.setW(0.1)
        if len(filter3) > 0:
            
            for bbox in filter3:
                draw_bounding_box(img, bbox, color=(0, 255, 0), thickness=3)
                print(area_bounding_box(bbox))
            max_bbox = max_bounding_box(filter3)

            candidate2follow = BoundingBoxObject(max_bbox)
            
            if candidate2follow.centroid[0] > img.shape[1]/3 and candidate2follow.centroid[1] < img.shape[1]*2/3:
                tracker.setObjective(candidate2follow)
                state = FOLLOW_PERSON
                print("Person found")


    elif state == FOLLOW_PERSON:
        if len(filter3) > 0:
            centroids = []
            objects = []
            
            for bbox in filter3:
                objects.append(BoundingBoxObject(bbox))
                draw_bounding_box(img, bbox, color=(0, 255, 0), thickness=3)

            # Setting angular velocity according to the centroid of the person to follow
            width = img.shape[1]
            step = width/len(vels)

            object2follow, index = tracker.getObjectiveFromSet(objects)
            if index >= 0:
                tracking_failure_cont = 0
                draw_bounding_box(img, filter3[index], color=(0, 0, 255), thickness=3)
            else:
                tracking_failure_cont += 1
            
            angular_vel = vels[int(object2follow.centroid[0]/step)]
            HAL.setW(angular_vel)
            HAL.setV(0.1)
        
        else:
            tracking_failure_cont += 1
        if tracking_failure_cont > 8:
            state = SEARCH_PERSON
            print("Person lost")

    GUI.showImage(img)

cv2.destroyAllWindows()