from GUI import GUI
from HAL import HAL
import cv2
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


while True:
    ret, img = HAL.getImage()
    print(ret)
    bboxes = HAL.getBoundingBoxes(img)
    filter1 = bounding_boxes_by_name(bboxes, "person")
    filter2 = bounding_boxes_by_score(filter1, 0.5)
    
    for bbox in filter2:
        draw_bounding_box(img, bbox, color=(0, 255, 0), thickness=3)
    #print(bbox)
    GUI.showImage(img)