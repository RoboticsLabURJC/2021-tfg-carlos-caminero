##########################
# Carlos Caminero
# Real Follow Person
# 2021/2022
##########################
from GUI import GUI
from HAL import HAL
import cv2
import math

SEARCH_PERSON = 0
FOLLOW_PERSON = 1

state = SEARCH_PERSON

class BoundingBoxObject:

    def __init__(self, bounding_box):
        self.bounding_box = bounding_box
        self.centroid = centroid_bounding_box(bounding_box)
        self.area = area_bounding_box(bounding_box)


class Tracker:

    def __init__(self, obj=None):
        self.obj = None
        self.limit = 60


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


def atraction_vector(centroid_x, width_img):
    
    dist = 2
    horizontal_fov = 1.05
    
    # angle of the object to follow with the camera
    angle = horizontal_fov*centroid_x/width_img - horizontal_fov/2
    
    # attraction vector
    tx = dist*math.sin(angle)
    ty = dist*math.cos(angle)
    return (tx, ty)


def repulsion_vector(laser_data):
	""" It returns a vector (x, y), being the sum of
		all repulsions vectors of each laser and
		minimized by a scale factor """
	x = y = 0
	scale_laser = 0.2
	
	for dist_rep, ang in laser_data:
		# Scale factor -> Scale laser
		dist_rep = scale_laser/dist_rep

		# repulsion distance is raised to 2 because we have control
		# better the laser and we can reduce the locals minimums
		x += (dist_rep**2) * math.cos(ang)
		y += -(dist_rep**2) * math.sin(ang)

	return (x, y)



def parse_laser_data(laser_data):
    values = []
    for i in range(len(laser_data)):
        dist = laser_data[i]
        if dist == float("inf"):
            continue
        if dist < 0.35:
            continue
        angle = math.radians(i)
        values += [(dist, angle)]
    return values
    

# Main program
tracker = Tracker()

tracking_failure_cont = 0

# VFF variables
alfa = 1.5
beta = 0.5

last_centroid = None

state = SEARCH_PERSON
while True:
    
    img = HAL.getImage()
    
    bounding_boxes = HAL.getBoundingBoxes(img)

    filter1 = bounding_boxes_by_score(bounding_boxes, 0.3)
    filter2 = bounding_boxes_by_name(filter1, "person")
    filter3 = bounding_boxes_by_area(filter2, 5000)
    
    width = img.shape[1]
    
    if state == SEARCH_PERSON:
        HAL.setV(0)
        if last_centroid != None:
            if last_centroid[0] > (width/2):
                HAL.setW(-0.05)
        else:
            HAL.setW(0.05)
            
        if len(filter3) > 0:
            
            for bbox in filter3:
                draw_bounding_box(img, bbox, color=(0, 255, 0), thickness=3)
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

            object2follow, index = tracker.getObjectiveFromSet(objects)
            if index >= 0:
                tracking_failure_cont = 0
                centroid = object2follow.centroid
                last_centroid = centroid
                
                # VFF Algorithm
                tx, ty = atraction_vector(centroid[0], width)
                rx, ry = repulsion_vector(parse_laser_data(HAL.getLaserData()))
                fx = alfa * tx + beta * rx # final vector (fx)
                fy = alfa * ty + beta * ry # final vector (fy)
                
                #print("({},{})".format(round(tx, 3), round(ty, 3)), end=' ')
                #print("({},{})".format(round(rx, 3), round(ry, 3)), end=' ')
                print("({},{})".format(round(fx, 3), round(fy, 3)))
                
                print(object2follow.area)
                if object2follow.area < 30000:
                    linear_vel = 0.3
                elif object2follow.area < 95000:
                    linear_vel = 0.2
                #else:
                    #linear_vel = 0.0
                draw_bounding_box(img, filter3[index], color=(0, 0, 255), thickness=3)
                HAL.setW(-fx)
                HAL.setV(linear_vel)
            else:
                tracking_failure_cont += 1
        
        else:
            tracking_failure_cont += 1
        if tracking_failure_cont > 8:
            state = SEARCH_PERSON
            print("Person lost")
    else:
        print(HAL.getLaserData().values[270])

    GUI.showImage(img)