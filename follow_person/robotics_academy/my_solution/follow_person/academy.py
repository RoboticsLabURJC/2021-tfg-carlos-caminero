##########################
# Carlos Caminero
# Follow Person Solution
# Robotics Software
# TFG 2021/2022
##########################

from GUI import GUI
from HAL import HAL
import cv2
import math


class PIDController:
    
    def __init__(self, Kp=None, Kd=None, Ki=None):
        self.kp = Kp
        self.kd = Kd
        self.ki = Ki

    
    def run(self, current_value, objective):
        error = (objective - current_value)
        P = D = I = 0
        if self.kp != None:
            P = - self.kp * error
        return P + D + I


def parse_laser_data(laser_data):
    values = laser_data.values
    return values[90:0:-1] + values[0:1] + values[360:269:-1]


def euclidean_distance(p1, p2):
    return math.sqrt(math.pow(abs(p2[0]-p1[0]), 2) + math.pow(abs(p2[1]-p1[1]), 2))
    
    
def bounding_box_properties(bbox):
    name = bbox.class_id
    xmin = bbox.xmin
    ymin = bbox.ymin
    xmax = bbox.xmax
    ymax = bbox.ymax
    return name, xmin, ymin, xmax, ymax


def area_bounding_box(bbox):
    _, xmin, xmax, ymin, ymax = bounding_box_properties(bbox)
    area = (xmax - xmin) * (ymax - ymin)
    return area


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


def centroid_bounding_box(bbox):
    cx = int((bbox.xmin + bbox.xmax)/2)
    cy = int((bbox.ymin + bbox.ymax)/2)
    return (cx, cy)


def nearest_bounding_box_from_centroid(bounding_boxes, centroid):
    min_distance = 3000
    nearest_index = -1
    for i in range(len(bounding_boxes)):
        actual_centroid = centroid_bounding_box(bounding_boxes[i])
        distance = euclidean_distance(actual_centroid, centroid)
        if distance < min_distance:
            min_distance = distance
            nearest_index = 0
    
    if nearest_index == -1:
        return None
    return bounding_boxes[nearest_index]
    

def draw_bounding_box(img, bbox, color=(23, 230, 210), thickness=2):
    _, xmin, ymin, xmax, ymax = bounding_box_properties(bbox)
    cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), color, thickness)


def draw_point(img, point, color=(0, 0, 255), radius=3):
    cv2.circle(img, point, radius, color, -1)



controller = PIDController(Kp=0.002)

img = HAL.getImage()
rows = img.shape[0]
cols = img.shape[1]
middle = int(cols/2)

# -- First, detect the person to follow
bounding_boxes = HAL.getBoundingBoxes(img)
filter1 = bounding_boxes_by_name(bounding_boxes, "person")
filter2 = bounding_boxes_by_score(filter1, 0.3)
bbox = max_bounding_box(filter2)
centroid = centroid_bounding_box(bbox)


print(bbox)
print(centroid)

HAL.setV(0)
HAL.setW(0)

while True:
    img = HAL.getImage()
    bounding_boxes = HAL.getBoundingBoxes(img)
    
    # Get Person Objects
    filter1 = bounding_boxes_by_name(bounding_boxes, "person")
    filter2 = bounding_boxes_by_score(filter1, 0.3)
    
    person = nearest_bounding_box_from_centroid(filter2, centroid)
    if person != None:
        # Update centroid
        centroid = centroid_bounding_box(person)
        draw_bounding_box(img, person)
        draw_point(img, centroid)
        aw = -controller.run(centroid[0], middle)
        HAL.setW(aw)
    
    GUI.showImage(img)
