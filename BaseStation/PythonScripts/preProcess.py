import cv2
import numpy as np
import math
from scipy import ndimage

def periodic_corr_np(x, y):
    """Periodic correlation, implemented using np.correlate.

    x and y must be real sequences with the same length.
    """
    return np.correlate(x, np.hstack((y[1:], y)), mode='valid')


def make2houghP(img):
    #gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(img,100,200,True)

    return cv2.HoughLinesP(edges,1,np.pi/180,30, maxLineGap=15,minLineLength=20)

def make2hough(img):
    #gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(img,800,1200,True)
    cv2.imshow('edges', edges)
    cv2.waitKey(0)
    return cv2.HoughLines(edges,1,np.pi/90,70)


def vizHoughP(hough, img, saveFile):
    for lns in hough:
        for x1,y1,x2,y2 in lns:
            cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)

    cv2.imwrite(saveFile,img)

def vizHough(hough, img, saveFile):
    for lns in hough:
        for rho,theta in lns:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))

            cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

    cv2.imwrite(saveFile,img)

def rotateP(hough, img, saveFile):
    largestId = 0 
    largestLen = 0
    i = 0
    for lns in hough:
        for x1,y1,x2,y2 in lns:
            len = math.sqrt(math.pow(x1-x2,2)+math.pow(y1-y2,2)) 
            if len > largestLen:
                largestId = i
                largestLen = len
        i = i+1
    print largestId
    print largestLen

    angles = []

    for x1, y1, x2, y2 in hough[largestId]:
        cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 3)
        angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
        angles.append(angle)

    median_angle = np.median(angles)
    img_rotated = ndimage.rotate(img, median_angle)

    print "Angle is {}".format(median_angle)
    cv2.imwrite(saveFile, img_rotated) 
 

img = cv2.imread('../Images/map.pgm')
lines = make2hough(img)
vizHough(lines, img, '../Images/hough.jpg')
#rotateP(lines, img, 'hough_rotate.jpg')

img2 = cv2.imread('../Images/map2.pgm')
lines2 = make2hough(img2)
vizHough(lines2, img2, '../Images/hough2.jpg')
#rotateP(lines2, img2, 'hough2_rotate.jpg')
