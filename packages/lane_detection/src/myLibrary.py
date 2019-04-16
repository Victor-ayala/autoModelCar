#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  2 11:58:31 2019

@author: Victor
"""

import cv2
import numpy as np
import operator


def obtainCentroid(contours, minArea = 0, maxArea = 1000000):
    
    x = []
    y = []
    centroids = []
    for i in contours:

        area = cv2.contourArea(i)
        if area > minArea and area < maxArea:
            M = cv2.moments(i)
            cx = int(M['m10']/M['m00'])
            x.append(cx)
            cy = int(M['m01']/M['m00'])
            y.append(cy)
            centroids.append([cx, cy])
    return x, y, centroids
#blob

def linearRegression(x, y):
    x = np.array(x, dtype=np.float64)
    y = np.array(y, dtype=np.float64)
    
    n = len(x)              # Total elements
    x_s = sum(x)          # X sum
    x2_s = sum(x**2)      # X^2 sum
    y_s = sum(y)          # Y sum
    xy_s = sum(x*y)       # Y^2 sum

    # Linear regression
    m = (n*xy_s - (x_s * y_s)) / ((n * x2_s) - x_s**2)
    b = (y_s/n) - m*(x_s/n)
    
    return m, b

def getDashedLine(m, b, h):
    # Find endpoint values
    y_t = [0, h]
#    y_t = np.array(y_t, dtype=np.float64)
    x_t = (y_t - b)/m
    p0 = (int(x_t[0]), int(y_t[0]))
    p1 = (int(x_t[1]), int(y_t[1]))
    
    return p0, p1

def obtainLaneCenter(p0, p1):
    
    # Desired center point of car
    x_l = int((p1[0] + p0[0])/2)                      
    y_l = int((p1[1] + p0[1])/2)

    return x_l, y_l

def checkSlope(centroids, kpts, rangem):
    centroids = sorted(centroids, key = operator.itemgetter(1), reverse = True)
    x = []
    y = []
    for i in centroids:
        x.append(i[0])
        y.append(i[1])
        
    m = []
    for i in range(1, len(x)):
        slope = float(float(y[i] - y[i - 1])/float(x[i] - x[i - 1]))
        m.append(slope)
    # print len(centroids)
    poplist = []
    for i in reversed(range(1, len(centroids) - 1)):
        # print i
        # print abs(m[i] - m[0])
        if abs(m[i] - m[0]) > rangem :
#            print "pop"
            poplist.append(i + 1)
    
    ## Pop blacklisted centroids
    for i in poplist:
        centroids.pop(i)
        x.pop(i)
        y.pop(i)
        kpts.pop(i)

    return x, y, centroids, kpts



### Bilateral filter
#blurred_image = cv2.bilateralFilter(cropped_image, 15, 55, 55)
##cv2.imshow("blurred image", blurred_image)
#
### Threshold
##th, im_th = cv2.threshold(blurred_image, 250, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
#th, im_th = cv2.threshold(cropped_image, 240, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
#
#cv2.imshow("Cropped image", cropped_image)
#
### Morphological transformantion
### closing
#kernel = np.ones((3, 3),np.uint8)
#im_th = cv2.morphologyEx(im_th, cv2.MORPH_CLOSE, kernel, iterations = 1)
#cv2.imshow("Not image binary", im_th)