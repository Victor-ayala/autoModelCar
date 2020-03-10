#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import operator
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int16



class laneDetector :
    def __init__(self):
        rospy.Subscriber("/app/camera/rgb/image_raw/compressed", CompressedImage, self.imageCallback)
        self.steer_pub = rospy.Publisher('/manual_control/desired_angle', Int16, queue_size=1)
        self.speed_pub = rospy.Publisher('/manual_control/desired_speed', Int16, queue_size=1)
        self.steer_16 = Int16()
        self.speed_16 = Int16()

        ## Parameters (To be included in a launch file)
        self.speed = 0
        self.flip = False
        self.crop = True
        self.active = True
        self.crosscont = 0
        self.pcross = False
        self.show = True
        self.reverse = True         # True -> Black lines; False -> White lines
        self.draw = True
        self.thresh = 160

    def imageCallback(self, msg):
        ## Decompress image
        np_arr = np.fromstring(msg.data, np.uint8)
        imgc = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.imgc = np.copy(imgc)
        self.image = cv2.cvtColor(imgc, cv2.COLOR_RGB2GRAY)


        ## Lane detector
        self.laneDetection()
        cv2.imshow("Color lane", self.imgc)
        cv2.imshow("Lane", self.img)
        cv2.waitKey(1)
        #print("steer = " + str(self.steer_16.data))
        #print("angle = " + str(angle))

## Wait for obstacle detection
    def obstCallback(self, msg):
        flag = 0
        flag = msg.data
        if flag == 1:                       # Obstacle detected navigation
            self.flip = True
            self.active = True
        if flag == 0:                       # Return to the right
            self.flip == False
            self.active == True
        if flag == -1:
            self.active == False

    def imgPreProcessing(self):
        show = True
        w = self.image.shape[1]
        h = self.image.shape[0]
        mn_th = 160         # Min threshold value
        fraction = 0.7      # Fraction of the image to be used bottom->top

    ## Switch white/black image
        if self.reverse:
            self.image = cv2.bitwise_not(self.image)

    ## Crop image
        if self.crop:
            crop_y = int(self.image.shape[0]*(1-fraction))
            self.img = self.image[crop_y:h,0:w]
            self.imgc = self.imgc[crop_y:h,0:w]
            h = int(self.img.shape[0])
        else:
            self.img = self.image.copy()

        if self.flip:
            self.img = cv2.flip(self.img, 1)
            self.imgc = cv2.flip(self.imgc, 1 )

        if show:
            #cv2.imshow("Cropped image", self.img)
            cv2.imshow("nonfilter", self.img)

    ## Normalize image
        n_img = cv2.normalize(self.img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        if show:
            cv2.imshow("norm", n_img)

    ## Filter image (less to more complex filter)
        # Kernel size
        k_size = 3
        # No Filter
        #m_img = n_img

        # Simple blur filter
        #m_img = cv2.blur(n_img, (k_size, k_size))

        # Salt & Pepper filter
        m_img = cv2.medianBlur(n_img, k_size)


        # Better noise removement
        # m_img = cv2.GaussianBlur(n_img, (k_size, k_size), 0, 0)

        # Blur preserving edges
        # m_img = cv2.bilateralFilter(n_img, k_size, 55, 55)

        if show:
            cv2.imshow("medianBlur", m_img)

        _, t_img = cv2.threshold(m_img, mn_th, 255, cv2.THRESH_BINARY) #cv2.THRESH_BINARY, cv2.THRESH_TOZERO, cv2.THRESH_OTSU
        #cv2.THRESH_BINARY+cv2.THRESH_OTSU Needs GaussianBlur
        #self.img = cv2.adaptiveThreshold(self.img, 250, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
        self.img = t_img

    ## Morphological operations
        k_size = 4
        kernel = np.ones((k_size, k_size), np.uint8)
        # Closing
        self.img = cv2.morphologyEx(self.img, cv2.MORPH_CLOSE, kernel, iterations = 1)

        self.left_img = self.img.copy()
        self.right_img = self.img.copy()
        #curv_img = self.img.copy()
        wo2 = w/2

    ## Separate image (left-right)
        for i in range(w/2):
            self.right_img[:, i] = 0
            #curv_img[:, i] = 0
            self.left_img[:, wo2 + i] = 0
        if show:
            cv2.imshow("left", self.left_img)
            cv2.imshow("right", self.right_img)

    def laneDetection(self):
        self.imgPreProcessing()
        width = self.img.shape[1]
        height = self.img.shape[0]
        ## Dashed line
        _, contours, _ = cv2.findContours(self.left_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #rectd = []
        contoursd = []
        for x in contours:
            rect = cv2.minAreaRect(x)
        #    print(rect[2])
            if abs(rect[2]) > 35 and abs(rect[2]) < 45:
                contoursd.append(x)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                if self.draw:
                    cv2.drawContours(self.img, [box], 0, (255, 100, 0), 5)
        xd, yd, centroids, contoursd = self.obtainCentroid(contoursd, minArea = 20, maxArea = 6500)
        l_dashedline = len(centroids)


        ## Continuous line
        _, contoursc, _ = cv2.findContours(self.right_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        xc, yc, cont, contoursc = self.obtainCentroid(contoursc, minArea = 200, maxArea = 90000)

        curve = False
        for x in contoursc:
            rect = cv2.minAreaRect(x)
        #    rectc.append(rect)
#            print(rect[2])
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            if self.draw:
                cv2.drawContours(self.imgc, [box], 0, (0, 100, 255), 5)
#            if abs(rect[2]) > 45 and l_dashedline < 1:
#                curve = True
        l_contline = len(cont)

        ## Horizontal line
        _, contoursh, _ = cv2.findContours(self.img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        xd, yd, horiz, contoursh = self.obtainCentroid(contoursh, minArea = 1000, maxArea = 90000)
        cross = False

        #recth = []

        for x in contoursh:
            rect = cv2.minAreaRect(x)
            print(cv2.contourArea(x))
            print(rect[2])
            if abs(rect[2]) < 16 or abs(rect[2]) > 74:
                cross = True
                print("cross")
        #        recth.append(rect)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                if self.draw:
                    cv2.drawContours(self.imgc, [box], 0, (100, 255, 100), 5)
#        l_horizline = len(horiz)

        #---------------------------------------

        pcar = (int(width/2), int(height/2))

        steer = 90
        angle = 0
        self.speed = 0
        speed_ = -250
        speedc_ = -150
        speedcr_ = -130

        ## Controller parameters
        theta_m = 30                                # deg   min/max value
        w_lane = 40.0                               # cm    lane width

        kv = 1/8                                   # Conversion from pixels to cm
        d_offset = 23                               # cm    from car to bottom image

        ## ------- ##
        ##Check if there's a continuous line
        if l_contline > 0:
            print("Continuous line")
            cont = sorted(cont, key = operator.itemgetter(1), reverse = True)
            pc = cont[0]
            for x in contoursc:
                M = cv2.moments(x)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if cx == pc[0] and cy == pc[1]:
                    rect = cv2.minAreaRect(x)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    if self.draw:
#                        print(rect[2])
                        cv2.drawContours(self.imgc, [box], 0, (0, 00, 200), 5)
                    if abs(rect[2]) > 50 or abs(rect[2]) < 36:
                        curve = True

            ycenter = pc[1]
            xcenter = pc[0] - 225
            center = (int(xcenter), int(ycenter))

            ## Controller
            k = float(w_lane/550)                       # Conversion from pixels to cm
            d = kv*(height - center[1])                 # cm    from bottom to calculated point

            error = k*(center[0] - pcar[0])
            angle = np.arctan2(error , d + d_offset)*(180/np.pi)
            if angle > theta_m:
                angle = theta_m
            if angle < -theta_m:
                angle = -theta_m
            if flip:
                angle = -angle

            ## Conversion to int steering value
            steer_range = 170                   # 5 - 175
            theta_range = 60                    # -30 - 30
            steer = int((steer_range/theta_range * angle + 90))
            if self.draw:
                cv2.circle(self.imgc, (center[0], center[1]), 10, (200, 200, 0), 4)

            self.speed = speed_
        else:
            ## If there's only one blob
            if l_dashedline == 1:
                print("One blob")
                pd = (centroids[0][0], centroids[0][1])

                center = (pd[0] + 310, pd[1])
                if self.draw:
                    for i in centroids:
                        if not flip:
                            cv2.circle(self.imgc, (i[0], i[1]), 8, (0, 255, 189), 3)
                        else:
                            cv2.circle(self.imgc, (width - i[0], i[1]), 8, (0, 255, 189), 3)
                    if not flip:
                        cv2.circle(self.imgc, pd, 10, (200, 200, 0), 4)
                        cv2.circle(self.imgc, center, 10, (200, 200, 0), 4)
                    ## !
                    else:
                        cv2.circle(self.imgc, (width - pd[0], pd[1]), 10, (200, 200, 0), 4)
                        cv2.circle(self.imgc, (width - center[0], center[1]), 10, (200, 200, 0), 4)

                ## Controller

                k = float(w_lane/550)      # Conversion from pixels to cm
                d = kv*(height - center[1])                 # cm    from bottom to calculated point

                error = k*(center[0] - pcar[0])
                angle = np.arctan2(error, d_offset + d)*(180/np.pi)
                if angle > theta_m:
                    angle = theta_m
                if angle < -theta_m:
                    angle = -theta_m
                if flip:
                    angle = -angle

                ## Conversion to int steering value
                steer_range = 170                   # 5 - 175
                theta_range = 60                    # -30 - 30
                steer = int((steer_range/theta_range * angle + 90))

                self.speed = speed_

            ## If there's 2+ blobs of dashed line
            elif l_dashedline > 1:
                print("Dashed line")
                rangem = 0.2
                x, y, centroids = self.checkSlope(centroids, rangem)

                m, b = self.linearRegression(x, y)
                pi, pf = self.getDashedLine(m, b, h_img)

                pd = (int((pi[0] + pf[0])/2), int((pi[1] + pf[1])/2))

                center = (pd[0] + 320, pd[1])

                if self.draw:
                     #Draw points and line found
                    for i in centroids:
                        if not flip:
                            cv2.circle(self.imgc, (i[0], i[1]), 8, (0, 255, 189), 3)
                        else:
                            cv2.circle(self.imgc, (width - i[0], i[1]), 8, (0, 255, 189), 3)
                    if not flip:
                        cv2.line(self.imgc, (pi[0], pi[1]), (pf[0], pf[1]), (0, 200, 200), thickness=4)
                        cv2.circle(self.imgc, (pd[0], pd[1]), 10, (250, 100, 0), 4)
                        cv2.circle(self.imgc, center, 10, (200, 100, 100), 4)
                    ## !
                    else:
                        cv2.line(self.imgc, (width - pi[0], pi[1]), (width - pf[0], pf[1]), (0, 200, 200), thickness=4)
                        cv2.circle(self.imgc, (width - pd[0], pd[1]), 10, (200, 200, 0), 4)
                        cv2.circle(self.imgc, (width - center[0], center[1]), 10, (200, 100, 100), 4)

                ## Controller
                k = float(w_lane/550)      # Conversion from pixels to cm
                d = kv*(height - center[1])                 # cm    from bottom to calculated point

                error = k*(center[0] - pcar[0])
                angle = np.arctan2(error, d_offset)*(180/np.pi)
                if angle > theta_m:
                    angle = theta_m
                if angle < -theta_m:
                    angle = -theta_m
                if flip:
                    angle = -angle

                ## Conversion to int steering value
                steer_range = 170                   # 5 - 175
                theta_range = 60                    # -30 - 30
                steer = int((steer_range/theta_range * angle + 90))

                self.speed = speed_

        if curve and not cross:
            print("Curve")
            if not flip:
                steer = 55
            else:
                steer = 125

            self.speed = speedc_

        if cross:
            self.speed = speedcr_
            steer = 92

        if self.draw:
            cv2.circle(self.imgc, (pcar[0], pcar[1]), 10, (250, 100, 0), 4)

    def obtainCentroid(contours, minArea=0, maxArea = 1000000):

        x = []
        y = []
        centroids = []
        poplist = []
        cont = 0
        for i in contours:

            area = cv2.contourArea(i)
            if area > minArea and area < maxArea:
                M = cv2.moments(i)
                cx = int(M['m10']/M['m00'])
                x.append(cx)
                cy = int(M['m01']/M['m00'])
                y.append(cy)
                centroids.append([cx, cy])
            else:
                poplist.append(cont)
            cont += 1

        for pop in reversed(poplist):
            contours.pop(pop)

        return x, y, centroids, contours

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

    def checkSlope(centroids, rangem):
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
            #kpts.pop(i)

        return x, y, centroids



if __name__ == '__main__':
    rospy.init_node('lanedetection')
    rospy.loginfo("Init Lane detection")
    ld = laneDetector()
    r = rospy.Rate(10)
    try:
        rospy.spin()
        r.sleep()
    except rospy.ROSInterruptException:
        pass


##-----------------------------
