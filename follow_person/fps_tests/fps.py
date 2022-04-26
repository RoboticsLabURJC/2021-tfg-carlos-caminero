import cv2
import time
from coco_labels import LABEL_MAP

FROZEN_GRAPH = "interfaces/pretrained_model/ssd_inception_v2_coco.pb"
PB_TXT = "interfaces/pretrained_model/ssd_inception_v2_coco.pbtxt"
SIZE = 300

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


cap = cv2.VideoCapture(4)

net = NeuralNetwork()

start_time = time.time()
rate = 1
counter = 0

while True:
	
	ret, image = cap.read()
	
	
	if ret:
		
		detections = net.detect(image)
		
		rows = image.shape[0]
		cols = image.shape[1]

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
		
		
		filter1 = bounding_boxes_by_score(bounding_boxes, 0.3)
		filter2 = bounding_boxes_by_name(filter1, "person")
		
		for bbox in filter2:
			draw_bounding_box(image, bbox, color=(0, 255, 0), thickness=3)
		
		cv2.imshow("Imagen", image)

		counter += 1
		if (time.time() - start_time) > rate:
			print("FPS: ", counter / (time.time() - start_time))
			counter = 0
			start_time = time.time()
	
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()