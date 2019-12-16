from math import hypot, pi, cos, sin, tan
import numpy as np
from PIL import Image, ImageDraw
import cv2
from numpy.fft import fft, ifft
from scipy import ndimage

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
	arr = []
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
		arr.append(-tot/i)
	return nim, arr

def tIFmax(im):
	pim = im.load()
	imx, imy = im.size
	nim = Image.new("L", (imx, 256), 255)
	#pnim = nim.load()
	arr = []
	drim = ImageDraw.Draw(nim)
	prev = 0
	for x in range(imx):
		max = 255
		for y in range(imy):
			if pim[x, y] != 255 and max > pim[x,y]:
				max = pim[x,y]
	    #pnim[x, max] = 0
		arr.append(max)
		if prev == 0:
			prev = (x, max)
			#print prev
		else:
			drim.line([prev, (x, max)], fill="black", width = 0)
			prev = (x, max)
	return nim, arr


def extendImage(img, larger):
    size = larger.size
    layer = Image.new('L', size, 205)
    layer.paste(img, tuple(map(lambda x:(x[0]-x[1])/2, zip(size, img.size))))
    layer.save('../Images/extended.pgm')
def periodic_corr(x, y):
	"""Periodic correlation, implemented using the FFT.

	x and y must be real sequences with the same length.
	"""
	return ifft(fft(x) * fft(y).conj()).real
def periodic_corr_np(x, y):
	"""Periodic correlation, implemented using np.correlate.

	x and y must be real sequences with the same length.
	"""
	return np.correlate(x, np.hstack((y[1:], y)), mode='valid')
def corr2Img(arr, saveFile):
	"""
	Takes a one dimentional array and saves it to an image where the value
	in the array is where it puts the black dot in the y-axis
	"""
	imx = arr.size
	nim = Image.new("L", (imx, 256), 255)
	pnim = nim.load()
	arr = (arr / float(max(arr)))*128
	#drim = ImageDraw.Draw(nim)
	for x in range(imx):
		pnim[x, int(arr[x])] = 0
	nim.save(saveFile)

def find_peaks(arr):
	"""
	Finds the peaks with a minimum distance of 200 between them
	Outputs a list of the location of the peaks
	"""
	size = arr.size
	peaks = []
	last = 0
	for i in range(size):
		if not peaks:
			peaks.append(0)
		elif arr[i] > arr[last] and i-last < 200:
			peaks[len(peaks)-1] = i
			last = i
		elif i-last >= 200 and arr[i] != arr[last]:
			peaks.append(i)
			last = i
	return peaks
def find_local_peaks(lst, hImg, gate):
	pImg = hImg.load()
	size = len(lst)
	imx, imy = hImg.size
	peaks = []
	P = []
	last = 0
	for i in range(size):
		if not peaks:
			peaks.append((0, lst[0]))
		elif lst[i] < lst[last] and i-last < gate:
			peaks[len(peaks)-1] = (i, lst[i])
			last = i
		elif i-last >= gate and lst[i] != lst[last]:
			peaks.append((i, lst[i]))
			last = i
	for th,v in peaks: #Theta, value
		for p in range(imy): # Rho
			if pImg[th,p] == v:
				P.append((th, p, v))
				break
			elif p == imy-1:
				print "NOT FOUND AT %d" % th
	return P

def mhh(P1, P2, vgate):
	C = []
	phi = 0
	for i in range(len(P1)):
		argmin = (1000, -1)
		for j in range(len(P2)):
			if abs(P1[i][2]-P2[j][2]) < argmin[0]:
				argmin = (abs(P1[i][2]-P2[j][2]), j)
		if argmin[0] < vgate:
			C.append((i, argmin[1]))
	for i,j in C:
		phi = phi - ((1/tan(P1[i][0]))-(1/tan(P1[j][0])))
	return phi/len(C)

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
tIF, arrA = avg(him)
tIF.save('../Images/tIF.bmp')
tIFma, arrM = tIFmax(him)
tIFma.save('../Images/tIFmax.bmp')
him.save('../Images/ho.bmp')

im2 = Image.open('../Images/edges2H.png').convert('L') #Loads the image and makes it grayscale
him2 = hough(im2)
tIF, arrA2 = avg(him2)
tIF.save('../Images/tIF2.bmp')
tIFmax2, arrM2 = tIFmax(him2)
tIFmax2.save('../Images/tIFmax2.bmp')
him2.save('../Images/ho2.bmp')


corrM = periodic_corr_np(arrM, arrM2)
corrA = periodic_corr_np(arrA, arrA2)

#indM = np.argmax(corrM)
#indM = ((indM/1280.0)*180)-90


arrFP = find_peaks(corrM)
for x in range(len(arrFP)):
	arrFP[x] = ((arrFP[x]/1280.0)*180)-90
print arrFP

#indA = np.argmax(corrA)
#indA = ((indA/1280.0)*180)-90

#img = Image.open('../Images/map.pgm')
#img.rotate(indM).show()
#for x in arrFP:
#	img.rotate(x).show()
#img.rotate(indA).show()
#img.show()

corr2Img(corrM, '../Images/maxCorr.bmp')
corr2Img(corrA, '../Images/avgCorr.bmp')

img = cv2.imread('../Images/map2.pgm')


lp = find_local_peaks(arrM, him, 100)
lp2 = find_local_peaks(arrM2, him2, 100)

phiH = mhh(lp, lp2, 15)
phi = 360

for x in arrFP:
	rotated = ndimage.rotate(img, -x, cval = 205)
	cv2.imwrite('../Images/rotated_%d_degrees.bmp' % x, rotated)

	if abs(x - phiH) < phi:
		phi = x

rotated = ndimage.rotate(img, -phi, cval = 205)
cv2.imwrite('../Images/rotated.bmp', rotated)

im.close()
im2.close()
