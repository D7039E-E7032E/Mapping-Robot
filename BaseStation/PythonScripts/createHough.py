from math import hypot, pi, cos, sin
import numpy as np
from PIL import Image
import cv2

def hough(im, ntx=1280, mry=720):
    """
    Calculate Hough transform.

    ntx and mry is the resolution for the image.
    More complex images require higher resolution
	"""
    pim = im.load()
    nimx, mimy = im.size
    mry = int(mry/2)*2          #Make sure that this is even
    him = Image.new("L", (ntx, mry), 255)
    phim = him.load()

    rmax = hypot(nimx, mimy)
    dr = rmax / (mry/2)
    dth = pi / ntx

    for jx in xrange(nimx):
        for iy in xrange(mimy):
            col = pim[jx, iy]
            if col == 0: continue
            for jtx in xrange(ntx):
                th = dth * jtx
                th = th - (pi/2)
                r = jx*cos(th) + iy*sin(th)
                iry = mry/2 + int(r/dr+0.5)
                phim[jtx, iry] -= 1
    return him

def detectEdges(img, savePath):
	edges = cv2.Canny(img,100,200,True)
	cv2.imwrite(savePath, edges)

def avg(im):
	pim = im.load()
	imx, imy = im.size
	nim = Image.new("L", (imx, 256), 255)
	pnim = nim.load()
	for x in range(imx):
		i = 0
		tot = 0
		for y in range(imy):
			if pim[x, y] != 255:
				i += 1
				#print x
				#print y
				#print "\n"
				tot -= pim[x,y]
		pnim[x, -tot/i] = 0
	return nim

def tIFmax(im):
	pim = im.load()
	imx, imy = im.size
	nim = Image.new("L", (imx, 256), 255)
	pnim = nim.load()
	for x in range(imx):
	    max = 255
	    for y in range(imy):
	    	if pim[x, y] != 255 and max > pim[x,y]:
	    		max = pim[x,y]
	    pnim[x, max] = 0
	return nim

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

detectEdges(img, '../Images/edgesH.png')
detectEdges(img2, '../Images/edges2H.png')

im = Image.open('../Images/edgesH.png').convert('L') #Loads the image and makes it grayscale
him = hough(im)
tIF = avg(him)
tIF.save('../Images/tIF.bmp')
tIFma = tIFmax(him)
tIFma.save('../Images/tIFmax.bmp')
him.save('../Images/ho.bmp')
im.close()

im2 = Image.open('../Images/edges2H.png').convert('L') #Loads the image and makes it grayscale
him2 = hough(im2)
tIF = avg(him2)
tIF.save('../Images/tIF2.bmp')
tIFmax2 = tIFmax(him2)
tIFmax2.save('../Images/tIFmax2.bmp')
him2.save('../Images/ho2.bmp')
im2.close()
