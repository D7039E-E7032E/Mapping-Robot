import cv2
import numpy as np
import math
from scipy import ndimage
from PIL import Image

def periodic_corr_np(x, y):
    """Periodic correlation, implemented using np.correlate.

    x and y must be real sequences with the same length.
    """
    return np.correlate(x, np.hstack((y[1:], y)), mode='valid')


def make2houghP(img):
    """
    Inputs an opened image and returns the result of a probabilistic
    hough transform converted back to the xy-plane
    """
    #gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(img,100,200,True)

    return cv2.HoughLinesP(edges,1,np.pi/180,30, maxLineGap=15,minLineLength=20)

def make2hough(img):
    """
    Inputs an opened image and returns the result of a hough transform
    in the (tho,theta)-plane. The points in the output array are the 
    points of interest derived from the hough space
    """
    #gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(img,800,1200,True)
    #cv2.imshow('edges', edges) #Shows the detected edges
    #cv2.waitKey(0)
    return cv2.HoughLines(edges,1,np.pi/90,70)



def vizHoughP(hough, img, saveFile):
    """
    Vizualizes  a Probablistic hough transform on top of an image and
    is saved
    """
    for lns in hough:
        for x1,y1,x2,y2 in lns:
            cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)

    cv2.imwrite(saveFile,img)

def vizHough(hough, img, saveFile):
	"""
	Vizualizes a hough transform on top of an image and is saved
	"""
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
	"""
	Rotates a probabilistic hough transform by finding the longest vector and
	turns it to be horizontal
	"""
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

def extendImage(img, larger):
	size = larger.size
	layer = Image.new('L', size, 205)
	layer.paste(img, tuple(map(lambda x:(x[0]-x[1])/2, zip(size, img.size))))
	layer.save('../Images/extended.pgm')


img = Image.open('../Images/map.pgm')
img2 = Image.open('../Images/map2.pgm')

if img.size > img2.size:
	extendImage(img2, img)
	img = cv2.imread('../Images/map.pgm')
	img2 = cv2.imread('../Images/extended.pgm')
elif img.size < img2.size:
	extendImage(img, img2)
	img = cv2.imread('../Images/extended.pgm')
	img2 = cv2.imread('../Images/map2.pgm')
else:
	img = cv2.imread('../Images/map.pgm')
	img2 = cv2.imread('../Images/map2.pgm')

lines = make2hough(img)
vizHough(lines, img, '../Images/hough.jpg')
#rotateP(lines, img, 'hough_rotate.jpg')

lines2 = make2hough(img2)
print 'Lines'
for i in lines:
	print i

print '\nLines2'
for i in lines2:
	print i
vizHough(lines2, img2, '../Images/hough2.jpg')
#rotateP(lines2, img2, 'hough2_rotate.jpg')
