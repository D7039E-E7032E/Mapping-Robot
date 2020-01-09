from math import hypot, pi, cos, sin, tan
import numpy as np
from PIL import Image, ImageDraw
import cv2
from numpy.fft import fft, ifft
from scipy import ndimage
from time import gmtime
from os import remove
from glob import glob
from sys import argv, exit

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
                #th = th - (pi/2)
                r = jx*cos(th) + iy*sin(th)
                iry = mry/2 + int(r/dr+0.5)
                phim[jtx, iry] -= 1
    return him

def detectEdges(img, savePath):
	edges = cv2.Canny(img,500,800,True)
	cv2.imwrite(savePath, edges)

def avg(im):
	"""
	Calculates the average intensity over each angle in a hough image

	Returns an image and array of the intensities
	"""
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
	"""
	Finds the maximum intensity over each angle in a hough image

	Returns an image and array of the intensities
	"""
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


def extendImage(img):
	"""
	Extends the given image to make the new sides the length of the hypotenuse
	witch the previous image placed in the center

	Returns the extended image and the hypotenuse
	"""
	nimx, mimy = img.size
	rmax = int(hypot(nimx, mimy))
	size = (rmax,rmax)

	layer = Image.new('L', size, 205)
	layer.paste(img, tuple(map(lambda x:(x[0]-x[1])/2, zip(size, img.size))))
	return layer, rmax

def extendImage2(img, rmax):
    """
    Extends an image to the given size rmax and places the image in the center

    Returns the extended image
    """
    nimx, mimy = img.size
    size = (rmax,rmax)

    layer = Image.new('L', size, 205)
    layer.paste(img, tuple(map(lambda x:(x[0]-x[1])/2, zip(size, img.size))))
    return layer

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

def find_peaks(arr, gate):
	"""
	Finds the peaks in an array with a minimum distance of the gate between them
	Returns a list of the location of the peaks
	"""
	size = arr.size
	peaks = []
	last = 0
	for i in range(size):
		if not peaks:
			peaks.append(0)
		elif arr[i] > arr[last] and i-last < gate:
			peaks[len(peaks)-1] = i
			last = i
		elif i-last >= gate and arr[i] != arr[last]:
			peaks.append(i)
			last = i
	#print peaks
	return peaks
def find_local_peaks(lst, hImg, gate):
	"""
	Finds the local peaks of an array with a minimum distance of the gate
	It then finds these peaks in the given hough image and locates the theta,
	rho and the intensity

	Returns a list of all the thetas, rhos and intensities at the local peaks
	"""
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
	"""
	Multiple hypothesis handling
	Inputs a list of peaks for each hough image containing the theta, rho and intensity
	Finds the average rotation needed to line up peaks that could correspond to each other
	which is if the intensity is within a range of vgate

	Returns a theoretical rotation value
	"""
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
		th1 = ((P1[i][0]/1280.0)*180)-90
		th2 = ((P2[j][0]/1280.0)*180)-90
		if th1 == 0:
			th1 = 1
		if th2 == 0:
			th2 = 1
		phi = phi - ((1/tan(th1))-(1/tan(th2)))
		#print phi
	return phi/len(C)

def transMP(P1, P2, dgate, vgate):
	"""
	Translation by Matching Peaks of hough images
	Inputs a list of peaks for each hough image containing the theta, rho and intensity
	vgate is the minimum difference between intensity peaks where they can be associated
	dgate is the minimum difference between the rho of the intensity peaks where they can still be associated

	Returns a transformation matrix
	"""
	C = []
	d = int((1280/180)*5)	#d is the range in degrees in will check for matching peaks

	for i in range(len(P1)):
		min = None
		minDif = None
		for j in range(P1[i][0]-d, P1[i][0]+d):
			if j < 0:
				continue
			else:
				for g in range(len(P2)):
					if P2[g][0] == j and min == None:
						min = g
						minDif = abs(P2[g][2]-P1[i][2])+abs(P2[g][1]-P1[i][1])
					elif P2[g][0] == j:
						dif = abs(P2[g][2]-P1[i][2])+abs(P2[g][1]-P1[i][1])

						if dif < minDif:
							minDif = dif
							min = g
		if min != None:
			if abs(P2[min][2]-P1[i][2]) > vgate or abs(P2[min][1]-P1[i][1]) > dgate:
				continue
			else:
				C.append((i,min))
	return C

files = glob('../Images/rotated_*')
for f in files:
	remove(f) #Removes all previous rotation hypothesies from earlier runs

st = gmtime().tm_sec
if len(argv) == 3:
	img = Image.open('../Images/%s.pgm' % argv[1])
	img2 = Image.open('../Images/%s.pgm' % argv[2])
else:
	print 'Please pass in the name of the two maps to be merged without the extention'
	exit()

img, rmax = extendImage(img)
img2, rmax2 = extendImage(img2)

img.save('../Images/extend.png')
img2.save('../Images/extend2.png')

img = cv2.imread('../Images/extend.png')
img2 = cv2.imread('../Images/extend2.png')

detectEdges(img, '../Images/edgesH.png')
detectEdges(img2, '../Images/edges2H.png')

im = Image.open('../Images/edgesH.png').convert('L') #Loads the image and makes it grayscale
him = hough(im)
tIFma, arrM = tIFmax(him)
tIFma.save('../Images/tIFmax.png')
him.save('../Images/ho.png')

im2 = Image.open('../Images/edges2H.png').convert('L') #Loads the image and makes it grayscale
him2 = hough(im2)

tIFmax2, arrM2 = tIFmax(him2)
tIFmax2.save('../Images/tIFmax2.png')
him2.save('../Images/ho2.png')


corrM = periodic_corr_np(arrM, arrM2)


arrFP = find_peaks(corrM, 250)
for x in range(len(arrFP)):
	arrFP[x] = ((arrFP[x]/1280.0)*180)-90 #Converts the peaks to degrees instead of pixels
#print arrFP


corr2Img(corrM, '../Images/maxCorr.png')

img = cv2.imread('../Images/%s.pgm' % argv[2])


lp = find_local_peaks(arrM, him, 150)
lp2 = find_local_peaks(arrM2, him2, 150)
#print lp

phiH = mhh(lp, lp2, 15)
#print phiH
ab = 360
phi = 360

for x in arrFP: # Saves the different rotation hypothesies and finds the one closest to the hypothetical rotation
	rotated = ndimage.rotate(img, -x, cval = 205)
	cv2.imwrite('../Images/rotated_%d_degrees.png' % x, rotated)

	if abs(x - phiH) < ab:
		ab = abs(x - phiH)
		phi = x

rotated = ndimage.rotate(img, -phi, cval = 205)
cv2.imwrite('../Images/rotated.png', rotated)
imgR = Image.open('../Images/rotated.png')
imgR = extendImage2(imgR, rmax2)
imgR.save('../Images/rotated_extended.png')
imgR.close()
rotated = cv2.imread('../Images/rotated_extended.png')
rotated = cv2.cvtColor(rotated,cv2.COLOR_BGR2GRAY)

im.close()
im2.close()

detectEdges(rotated, '../Images/edgesR.png')
imR = Image.open('../Images/edgesR.png').convert('L')
himR = hough(imR)
himR.save('../Images/hoR.png')

tIFma2, arrM2 = tIFmax(himR)
tIFma2.save('../Images/tIFmaxR.png')

lp2 = find_local_peaks(arrM2, himR, 125)


C = transMP(lp, lp2, 100, 100)

Al = []
Bl = []

for i,j in C:
	Bl.append([lp[i][1]-lp2[j][1]])
	a = cos((lp2[j][0]/1280.0)*pi)
	b = sin((lp2[j][0]/1280.0)*pi)
	Al.append([a, b])

A = np.asarray(Al)
B = np.asarray(Bl)

At = np.transpose(A)

T = np.dot(np.linalg.inv(np.dot(At,A)), np.dot(At,B))

img = Image.open('../Images/extend.png').convert('L')
imgR = Image.open('../Images/rotated_extended.png').convert('L')

imx, imy = img.size
imxR, imyR = imgR.size

x = int((imx/2) - (imxR/2) + T[0][0])
y = int((imy/2) - (imyR/2) + T[1][0])

imgT = Image.new("L", (imx, imy), 205)
imgT.paste(imgR, (x,y))
merged = Image.blend(img, imgT, 0.5)
merged.save('../Images/merged.png')

ct = gmtime().tm_sec
dt = ct-st
if dt < 0:
	dt += 60
print "%d seconds to complete" % dt
