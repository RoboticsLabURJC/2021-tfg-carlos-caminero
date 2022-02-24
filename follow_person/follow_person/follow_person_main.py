import follow_person.HAL as HAL
import cv2


# ----- USER PROGRAM -----

v = 0.3
HAL.setV(0)
HAL.setW(0)

def getCentroid(bounding_box):
    xmin = bounding_box.xmin
    xmax = bounding_box.xmax
    ymin = bounding_box.ymin
    ymax = bounding_box.ymax
    return ((xmin + xmax)/2.0, (ymin + ymax)/2.0)


def getArea(bounding_box):
    xmin = bounding_box.xmin
    xmax = bounding_box.xmax
    ymin = bounding_box.ymin
    ymax = bounding_box.ymax
    return ((xmax - xmin) * (ymax - ymin))


def maxAreaBoundingBox(bounding_boxes, class_id):
    imax = 0
    detection = False
    for i in range(len(bounding_boxes)):
        if bounding_boxes[i].class_id == class_id:
            detection = True
            area = getArea(bounding_boxes[i])
            if area > getArea(bounding_boxes[imax]):
                imax = i
    if not detection:
        return None
    return bounding_boxes[imax]


def user_main(args=None):
    # bounding_boxes = HAL.getBoundingBoxes()

    # if HAL.getNumObjects() != 0:
    #     for bounding_box in bounding_boxes:
    #         if bounding_box.class_id == "person":
    #             cx, cy = getCentroid(bounding_box)
    #             print("persona detectada en ({},{}) y area {}".format(cx, cy, getArea(bounding_box)))
    #     max_bounding_box = maxAreaBoundingBox(bounding_boxes, 'person')
    #     if max_bounding_box != None:
    #         print("maximo area: ", getArea(max_bounding_box))

    # laser_data = HAL.getLaserData()
    # if laser_data != None:
    #     for i in range(len(laser_data)):
    #         print("["+str(i)+"]:", laser_data[i], end=' ')
    #     print("\n\n")

    image = HAL.getImage()
    cv2.imshow("Imagen", image)
    cv2.waitKey(1)

##########################


# ----- DO NOT TOUCH -----
def main():
    HAL.main(user_main)

if __name__ == '__main__':
    main()