from GUI import GUI
from HAL import HAL
import cv2
# Enter sequential code!

def getBoundingBoxes(bboxes, class_id):
    selected_bounding_boxes = []
    for bbox in bboxes:
        if bbox.class_id == class_id:
            selected_bounding_boxes.append(bbox)
    return selected_bounding_boxes

def getCentroid(bounding_box):
    xmin = bounding_box.xmin
    ymin = bounding_box.ymin
    xmax = bounding_box.xmax
    ymax = bounding_box.ymax
    return (int((xmin + xmax)/2), int((ymin + ymax)/2))

def drawBoundingBox(img, bounding_box, color=(0, 0, 255)):
    start_point = (bounding_box.xmin, bounding_box.ymin)
    end_point = (bounding_box.xmax, bounding_box.ymax)
    cv2.rectangle(img, start_point, end_point, color, 2)


# -- Init velocity
HAL.setV(0)
HAL.setW(0)

while True:
    #-- calling HAL functions
    img = HAL.getImage()
    bboxes = HAL.getBoundingBoxes().bounding_boxes
    
    # -- Current init parameters
    width_image = img.shape[1]
    person_bboxes = getBoundingBoxes(bboxes, "person")
    
    # -- Processing inputs
    for person_bbox in person_bboxes:
        drawBoundingBox(img, person_bbox)
        centroid = getCentroid(person_bbox)
        cv2.circle(img, centroid, 5, (0, 255, 0), -1)
        if centroid[0] < width_image/2:
            HAL.setW(0.1)
        else:
            HAL.setW(-0.1)
    
    # -- Drawing Results on image
    GUI.showImage(img)