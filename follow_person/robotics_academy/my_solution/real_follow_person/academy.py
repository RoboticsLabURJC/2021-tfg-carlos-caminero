from GUI import GUI
from HAL import HAL
import cv2
import math
# Enter sequential code!

def bounding_box_properties(bbox):
    name = bbox.class_id
    xmin = bbox.xmin
    ymin = bbox.ymin
    xmax = bbox.xmax
    ymax = bbox.ymax
    return name, xmin, ymin, xmax, ymax

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


def draw_bounding_box(img, bbox, color=(23, 230, 210), thickness=2):
    _, xmin, ymin, xmax, ymax = bounding_box_properties(bbox)
    cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), color, thickness)


def area_bounding_box(bbox):
    _, xmin, ymin, xmax, ymax = bounding_box_properties(bbox)
    area = (xmax - xmin) * (ymax - ymin)
    return area


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


def euclidean_distance(p1, p2):
    return math.sqrt(math.pow(abs(p2[0]-p1[0]), 2) + math.pow(abs(p2[1]-p1[1]), 2))



def centroid_bounding_box(bbox):
    cx = int((bbox.xmin + bbox.xmax)/2)
    cy = int((bbox.ymin + bbox.ymax)/2)
    return (cx, cy)



SEARCH_PERSON = 0
FOLLOW_PERSON = 1

state = SEARCH_PERSON
current_centroid = None

vels = [0.3, 0.2, 0.1, 0.0, -0.1, -0.2, -0.3]

while True:
    img = HAL.getImage()
    
    bboxes = HAL.getBoundingBoxes(img)
    filter1 = bounding_boxes_by_name(bboxes, "person")
    filter2 = bounding_boxes_by_score(filter1, 0.5)
    
    
    if state == SEARCH_PERSON:
        if len(filter2) > 0:
            for bbox in filter2:
                draw_bounding_box(img, bbox, color=(0, 255, 0), thickness=3)
            max_bbox = max_bounding_box(filter2)
            current_centroid = centroid_bounding_box(max_bbox)
            if current_centroid[0] > img.shape[1]/3 and current_centroid[0] < img.shape[1]*2/3:
                print(current_centroid)
                cv2.circle(img, (current_centroid), 3, (0, 0, 255), -1)
                state = FOLLOW_PERSON
    
    else:
        if len(filter2) > 0:
            centroids = []
            for bbox in filter2:
                centroids.append(centroid_bounding_box(bbox))
            
            closer_centroid_id = 0
            min_distance = 999
            i = 0
            for centroid in centroids:
                dist = euclidean_distance(centroid, current_centroid)
                if dist < min_distance:
                    min_distance = dist
                    closer_centroid_id = i
                i += 1
            current_centroid = centroids[closer_centroid_id]
            cv2.circle(img, (current_centroid), 3, (0, 0, 255), -1)
        
            width = img.shape[1]
            step = width/len(vels)
            HAL.setW(vels[int(current_centroid[0]/step)])
            HAL.setV(0.1)
            
        else:
            HAL.setV(0)
            HAL.setW(0)
                
        
            
            
    #print(bbox)
    GUI.showImage(img)