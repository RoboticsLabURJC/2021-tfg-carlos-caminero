import follow_person.HAL as HAL
import numpy as np
import cv2


# ----- USER PROGRAM -----

v = 0.3
HAL.setV(0)
HAL.setW(0)

HMIN = 40
HMAX = 70
SMIN = 50
SMAX = 255
VMIN = 50
VMAX = 255

low_color = np.array([HMIN, SMIN, VMIN])
high_color = np.array([HMAX, SMAX, VMAX])

def create_mask(hsv_image, low_color, high_color):
    """ It returns the mask giving the hsv image as input"""

    # it filters the road with the established color range:
    mask = cv2.inRange(hsv_image, low_color, high_color)

    # it deletes the noise:
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask


def get_filters(image, low_color, high_color):
    """ It returns values: [hsv image, mask image] giving image as imput """

    # we convert the image from rgb to hsv
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # we obtian mask image where colors are black and white (area of interest).
    mask_image = create_mask(hsv_image, low_color, high_color)

    return hsv_image, mask_image

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


def nothing(x):
    pass

# cv2.namedWindow("Parametros")
# cv2.createTrackbar('HMIN', 'Parametros', 0, 180, nothing)
# cv2.createTrackbar('HMAX', 'Parametros', 0, 180, nothing)
# cv2.createTrackbar('SMIN', 'Parametros', 0, 255, nothing)
# cv2.createTrackbar('SMAX', 'Parametros', 0, 255, nothing)
# cv2.createTrackbar('VMIN', 'Parametros', 0, 255, nothing)
# cv2.createTrackbar('VMAX', 'Parametros', 0, 255, nothing)

# cv2.setTrackbarPos('HMIN', 'Parametros', 0)
# cv2.setTrackbarPos('HMAX', 'Parametros', 0)
# cv2.setTrackbarPos('SMIN', 'Parametros', 0)
# cv2.setTrackbarPos('SMAX', 'Parametros', 0)
# cv2.setTrackbarPos('VMIN', 'Parametros', 0)
# cv2.setTrackbarPos('VMAX', 'Parametros', 0)


def user_main(args=None):
    # global HMIN, HMAX, SMIN, SMAX, VMIN, VMAX

    # HMIN = cv2.getTrackbarPos('HMIN', 'Parametros')
    # HMAX = cv2.getTrackbarPos('HMAX', 'Parametros')
    # SMIN = cv2.getTrackbarPos('SMIN', 'Parametros')
    # SMAX = cv2.getTrackbarPos('SMAX', 'Parametros')
    # VMIN = cv2.getTrackbarPos('VMIN', 'Parametros')
    # VMAX = cv2.getTrackbarPos('VMAX', 'Parametros')

    image = HAL.getImage()
    #print(image.shape[0])
    
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    hsv, mask = get_filters(gray_image, low_color, high_color)
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    #cx, cy = get_meddium_point(contours)
    cv2.drawContours(image, contours, -1, (0, 255, 0), 1)
    if len(contours) > 0:
        cnt = max(contours, key=cv2.contourArea)
        x,y,w,h = cv2.boundingRect(cnt)
        
        shift = 10
        x -= shift
        y -= shift
        w += 2*shift
        h += 2*shift
        cx = int(x + w/2)
        cy = int(y + h/2)
        
        cv2.rectangle(image, (x,y), (x+w, y+h), (255,255,0), 2)
        cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)
    
    cv2.imshow("Imagen", image)
    cv2.waitKey(1)

##########################


# ----- DO NOT TOUCH -----
def main():
    HAL.main(user_main)

if __name__ == '__main__':
    main()