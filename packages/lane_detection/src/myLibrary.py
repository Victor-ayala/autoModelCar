#!/usr/bin/env python

import cv2
import numpy as np

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

def linearRegression(x, y, h):
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

    # Find endpoint values
    y_t = [0, h]
    y_t = np.array(y_t, dtype=np.float64)
    x_t = (y_t - b)/m

    y_d = int(sum(y_t)/len(y_t))
    x_d = int((y_d - b)/m)

    return m, b, x_d, y_d, x_t, y_t

def obtainCarCenter(x_d, y_d, x_c, y_c):

    # Desired center point of car
    x_car = int((x_c + x_d)/2)
    y_car = int((y_c + y_d)/2)

    return x_car, y_car

def checkSlope(x, y, centroids):
    m = []
    for i in range(1, len(x)):
        slope = (y[i] - y[i - 1])/(x[i] - x[i - 1])
        m.append(slope)

    mode = max(set(m), key=m.count)

    k = len(centroids)

    for i in range(1, k):
        if m[i - 1] != mode:
            print "pop"
            centroids.pop(i)
            x.pop(i)
            y.pop(i)

#    for i in range(1, len(x)):
##      print(i)
#        slope = (y[i] - y[i - 1])/(x[i] - x[i - 1])
#        m_array.append(slope)
#
#        mode = max(set(m_array), key=m_array.count)
#
#    for i in range(1, len(centroids_sorted)):
#        if m_array[i - 1] != mode:
#            print "pop"
#            centroids_sorted.pop(i)
#            x.pop(i)
#            y.pop(i)
#        else:
#            cv2.circle(img, (x[i], y[i]) , 10, (0, 200, 0), 4)
    return x, y, centroids
