#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import operator
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int16
import myLibrary as my



class laneDetector :
    def __init__(self):
#        rospy.Subscriber("/manual_control/obstacle", Int16, self.obstCallback)
        rospy.Subscriber("/app/camera/color/image_raw/compressed", CompressedImage, self.imageCallback)
        self.steer_pub = rospy.Publisher('/manual_control/steering', Int16, queue_size=1)
        #self.steer_pub = rospy.Publisher('/manual_control/desired_angle', Int16, queue_size=1)
        self.speed_pub = rospy.Publisher('/manual_control/speed', Int16, queue_size=1)
#        self.speed_pub = rospy.Publisher('/manual_control/desired_speed', Int16, queue_size=1)
        self.steer_16 = Int16()
        self.speed_16 = Int16()
        self.speed_16.data = 0
        self.speed = 0
        self.Flip = False
        self.active = True
        self.crosscont = 0
        self.pcross = False

    def imageCallback(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        lane_img, self.steer_16.data, angle = self.laneDetection(self.cv2_img, reverse = True, flip = self.Flip, active = self.active, Draw = False)
        #lane_img, _, _ = self.laneDetection(self.cv2_img, reverse = True, flip = False)
        #lane_img = self.cv2_img
        self.speed_16.data = self.speed
        self.steer_pub.publish(self.steer_16)
        self.speed_pub.publish(self.speed_16)
        #cv2.imshow("Lane", self.cv2_img)
        #cv2.waitKey(1)
        print("steer = " + str(self.steer_16.data))
        print("angle = " + str(angle))
        # print self.Flip

    def obstCallback(self, msg):
        flag = 0
        flag = msg.data
        if flag == 1:                       # Obstacle detected navigation
            self.Flip = True
            self.active = True
        if flag == 0:                       # Return to the right
            self.Flip == False
            self.active == True
        if flag == -1:
            self.active == False

    def laneDetection(self, image, reverse = False, flip = False, crop = True, Draw = False, fraction = 0.6, active = True):
        ##-----------------------------

        im_in = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        width = int(im_in.shape[1])
        height = int(im_in.shape[0])
        
        steer = 90
        angle = 0

        if reverse:
            im_in = cv2.bitwise_not(im_in)
        #    cv2.imshow("Not image0", im_in)

        #im_in = cv2.bilateralFilter(im_in, 15, 55, 55)
#        im_in = cv2.GaussianBlur(im_in, (7, 7), 0)
        _, im_in = cv2.threshold(im_in, 200, 255, cv2.THRESH_TOZERO)
        kernel = np.ones((6, 6), np.uint8)
        im_in = cv2. morphologyEx(im_in, cv2.MORPH_CLOSE, kernel, iterations = 1)

        #im_in = cv2.adaptiveThreshold(im_in, 250, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

        #cv2.imshow("im_in", im_in)

        # cropping image
        if crop:
            crop_y = int(im_in.shape[0]*(1-fraction))
            cropped_image = im_in[crop_y:height,0:width]
            img = image[crop_y:height,0:width]
            width = int(cropped_image.shape[1])
            height = int(cropped_image.shape[0])
        else:
            cropped_image = im_in.copy()
            img = image.copy()

        if flip:
            cropped_image = cv2.flip(cropped_image, 1 )
            img = cv2.flip(img, 1 )

        left_img = cropped_image.copy()
        right_img = cropped_image.copy()
        curv_img = cropped_image.copy()
        wo2 = width/2

        for i in range(width/2):
            right_img[:, i] = 0
            curv_img[:, i] = 0
            left_img[:, wo2 + i] = 0

        #cv2.imshow("Cropped image", cropped_image)
        h_img = int(img.shape[0])
	

    	## Dashed line
        _, contours, _ = cv2.findContours(left_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        #rectd = []
        contoursd = []
        for x in contours:
            rect = cv2.minAreaRect(x)
        #    print(rect[2])
            if abs(rect[2]) > 35 and abs(rect[2]) < 45:
                contoursd.append(x)
        #        rectd.append(rect)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                if Draw:
                    cv2.drawContours(img, [box], 0, (255, 100, 0), 5)
        xd, yd, centroids, contoursd = my.obtainCentroid(contoursd, minArea = 20, maxArea = 6500)
        l_dashedline = len(centroids)
        
        
        ## Continuous line
        _, contoursc, _ = cv2.findContours(right_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        xc, yc, cont, contoursc = my.obtainCentroid(contoursc, minArea = 200, maxArea = 90000)
        
        curve = False
        #rectc = []
        for x in contoursc:
            rect = cv2.minAreaRect(x)
        #    rectc.append(rect)
#            print(rect[2])
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            if Draw:
                cv2.drawContours(img, [box], 0, (0, 100, 255), 5)
#            if abs(rect[2]) > 45 and l_dashedline < 1:
#                curve = True
        l_contline = len(cont)
        
        ## Horizontal line
        _, contoursh, _ = cv2.findContours(cropped_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        xd, yd, horiz, contoursh = my.obtainCentroid(contoursh, minArea = 1000, maxArea = 90000)
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
                if Draw:
                    cv2.drawContours(img, [box], 0, (100, 255, 100), 5)
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
                    if Draw:
#                        print(rect[2])
                        cv2.drawContours(img, [box], 0, (0, 00, 200), 5)
                    if abs(rect[2]) > 50 or abs(rect[2]) < 36:
                        curve = True
        
            ycenter = pc[1]
            xcenter = pc[0] - 230
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
            if Draw:
                cv2.circle(img, (center[0], center[1]), 10, (200, 200, 0), 4)

            self.speed = speed_
        else:
            ## If there's only one blob
            if l_dashedline == 1:
                print("One blob")
                pd = (centroids[0][0], centroids[0][1])
        
                center = (pd[0] + 310, pd[1])
                if Draw:
                    for i in centroids:
                        if not flip:
                            cv2.circle(img, (i[0], i[1]), 8, (0, 255, 189), 3)
                        else:
                            cv2.circle(img, (width - i[0], i[1]), 8, (0, 255, 189), 3)
                    if not flip:
                        cv2.circle(img, pd, 10, (200, 200, 0), 4)
                        cv2.circle(img, center, 10, (200, 200, 0), 4)
                    ## !
                    else:
                        cv2.circle(img, (width - pd[0], pd[1]), 10, (200, 200, 0), 4)
                        cv2.circle(img, (width - center[0], center[1]), 10, (200, 200, 0), 4)
        
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
                x, y, centroids = my.checkSlope(centroids, rangem)
        
                m, b = my.linearRegression(x, y)
                pi, pf = my.getDashedLine(m, b, h_img)
        
                pd = (int((pi[0] + pf[0])/2), int((pi[1] + pf[1])/2))
        
                center = (pd[0] + 320, pd[1])
        
                if Draw:
                     #Draw points and line found
                    for i in centroids:
                        if not flip:
                            cv2.circle(img, (i[0], i[1]), 8, (0, 255, 189), 3)
                        else:
                            cv2.circle(img, (width - i[0], i[1]), 8, (0, 255, 189), 3)
                    if not flip:
                        cv2.line(img, (pi[0], pi[1]), (pf[0], pf[1]), (0, 200, 200), thickness=4)
                        cv2.circle(img, (pd[0], pd[1]), 10, (250, 100, 0), 4)
                        cv2.circle(img, center, 10, (200, 100, 100), 4)
                    ## !
                    else:
                        cv2.line(img, (width - pi[0], pi[1]), (width - pf[0], pf[1]), (0, 200, 200), thickness=4)
                        cv2.circle(img, (width - pd[0], pd[1]), 10, (200, 200, 0), 4)
                        cv2.circle(img, (width - center[0], center[1]), 10, (200, 100, 100), 4)
        
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
#        if cross:
#            print("Cross")
#            steer = 90
#            self.speed = speedc_
        
            #--------#


#        ## If we are in a crosslane
#        if cross != self.pcross:
#            self.crosscont += 1
#            self.crosscont = self.crosscont%2
##            self.speed = speedc_
#
#        # print cross
#        if self.crosscont:
#            if len(cont) > 0:
#                 cont = sorted(cont, key = operator.itemgetter(1), reverse = True)
#                 pc = cont[0]
#                 ycenter = pc[1]
#                 xcenter = pc[0] - 300
#                 center = (int(xcenter), int(ycenter))
#        
#                 ## Controller
#                 k = float(w_lane/550)                       # Conversion from pixels to cm
#                 d = kv*(height - center[1])                 # cm    from bottom to calculated point
#        
#                 error = k*(center[0] - pcar[0])
#                 angle = np.arctan2(error , d + d_offset)*(180/np.pi)
#                 if angle > theta_m:
#                     angle = theta_m
#                 if angle < -theta_m:
#                     angle = -theta_m
#                 if flip:
#                     angle = -angle
#        
#                 ## Conversion to int steering value
#                 steer_range = 170                   # 5 - 175
#                 theta_range = 60                    # -30 - 30
#                 steer = int((steer_range/theta_range * angle + 90))
#                 if Draw:
#                    cv2.circle(img, (center[0], center[1]), 10, (200, 200, 0), 4)
#        
#            self.pcross = cross
#            self.speed = speedc_
        
        if Draw:
            cv2.circle(img, (pcar[0], pcar[1]), 10, (250, 100, 0), 4)
            #cv2.imshow("L image", left_img)
            #cv2.imshow("R image", right_img)
            #cv2.imshow("Curv image", curv_img)
#            cv2.imshow("H image", cropped_image)

        return img, steer, angle




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
