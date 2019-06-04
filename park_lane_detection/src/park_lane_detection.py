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
        rospy.Subscriber("/manual_control/obstacle", Int16, self.obstCallback)
        rospy.Subscriber("/app/camera/color/image_raw/compressed", CompressedImage, self.imageCallback)
        self.steer_pub = rospy.Publisher('/manual_control/desired_angle', Int16, queue_size=1)
        self.speed_pub = rospy.Publisher('/manual_control/speed', Int16, queue_size=1)
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
        lane_img, self.steer_16.data, angle = self.laneDetection(self.cv2_img, reverse = False, flip = self.Flip, active = self.active, Draw = False)
        self.speed_16.data = self.speed
        self.steer_pub.publish(self.steer_16)
        #self.speed_pub.publish(self.speed_16)
        cv2.imshow("Lane", self.cv2_img)
        cv2.waitKey(1)
        print("steer = " + str(self.steer_16.data))
        print("angle = " + str(angle))

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
        #steer = 90
        #angle = 0

        angle = 0

        if reverse:
            im_in = cv2.bitwise_not(im_in)
        #    cv2.imshow("Not image0", im_in)

        #im_in = cv2.bilateralFilter(im_in, 15, 55, 55)
        im_in = cv2.GaussianBlur(im_in, (7, 7), 0)
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
            #print(rect[2])
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
        xc, yc, cont, constoursc = my.obtainCentroid(contoursc, minArea = 200, maxArea = 90000)

        curve = False
        #rectc = []
        for x in contoursc:
            rect = cv2.minAreaRect(x)
        #    rectc.append(rect)
            #print(rect[2])
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            if Draw:
                cv2.drawContours(img, [box], 0, (0, 100, 255), 5)
            if abs(rect[2]) > 45 and l_dashedline < 1:
                #curve = True
                curve = False
        l_contline = len(cont)

        ## Horizontal line
#        _, contoursh, _ = cv2.findContours(cropped_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#        xd, yd, horiz, contoursh = my.obtainCentroid(contoursh, minArea = 8000, maxArea = 15000)
#        cross = False
#
#        #recth = []
#        for x in contoursh:
#            rect = cv2.minAreaRect(x)
#        #    print(rect[2])
#            if abs(rect[2] % 90) < 5 and abs(rect[2] % 90) > 1:
#                cross = True
#        #        recth.append(rect)
#                box = cv2.boxPoints(rect)
#                box = np.int0(box)
#                if Draw:
#                    cv2.drawContours(img, [box], 0, (100, 255, 100), 5)
#        l_horizline = len(horiz)

        #---------------------------------------

        pcar = (int(width/2), int(height/2))

        steer = 90
        angle = 0
        self.speed = 0
        speed_ = -150
        speedc_ = -120

        ## Controller parameters
        theta_m = 30                                # deg   min/max value
        w_lane = 40.0                               # cm    lane width

        kv = 1/8                                   # Conversion from pixels to cm
        d_offset = 23                               # cm    from car to bottom image

        ## Check if there are lines to detect
        if l_dashedline > 1 and l_contline > 0:                                 # Both continuous and dashed line complete
            ## Sort right blobs by Y position
            # cont = []
            # for x in contline:
            #     cont.append((int(x.pt[0]), int(x.pt[1])))
            #     #print x.pt[0], x.pt[1]
            cont = sorted(cont, key = operator.itemgetter(1), reverse = True)
            pc = cont[0]

            # centroids = []
            # for x in dashedline:
            #     centroids.append((int(x.pt[0]), int(x.pt[1])))

            rangem = 0.2
            x, y, centroids = my.checkSlope(centroids, rangem)

            m, b = my.linearRegression(x, y)
            pi, pf = my.getDashedLine(m, b, h_img)


            yd = pc[1]
            xd = int((yd - b)/m)
            pd = (xd, yd)

            center = my.obtainLaneCenter(pd, pc)
            if Draw:
                #Draw points and line found
                for i in centroids:
                    if not flip:
                        cv2.circle(img, (i[0], i[1]), 8, (0, 255, 189), 3)
                    else:
                        cv2.circle(img, (width - i[0], i[1]), 8, (0, 255, 189), 3)
                if not flip:
                    cv2.line(img, (pi[0], pi[1]), (pf[0], pf[1]), (0, 200, 200), thickness=4)
                    cv2.line(img, (pd[0], pd[1]), (pc[0], pc[1]), (250, 200, 0), thickness=4)
                    cv2.circle(img, (pc[0], pc[1]), 10, (250, 100, 0), 4)
                    cv2.circle(img, (pd[0], pd[1]), 10, (250, 100, 0), 4)
                    cv2.circle(img, center, 10, (200, 200, 0), 4)
                else:
                    cv2.line(img, (width - pi[0], pi[1]), (width - pf[0], pf[1]), (0, 200, 200), thickness=4)
                    cv2.line(img, (width - pd[0], pd[1]), (width - pc[0], pc[1]), (250, 200, 0), thickness=4)
                    cv2.circle(img, (width - pc[0], pc[1]), 10, (250, 100, 0), 4)
                    cv2.circle(img, (width - pd[0], pd[1]), 10, (250, 100, 0), 4)
                    cv2.circle(img, (width - center[0], center[1]), 10, (200, 200, 0), 4)

            ## Controller
            k = float(w_lane/float(pc[0] - pd[0]))      # Conversion from pixels to cm
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

            #cv2.circle(img, (pcar[0], pcar[1]), 10, (250, 100, 0), 4)

        elif l_dashedline == 1 and l_contline > 0:                          ## Continuous line and one blob from dashed line
            # centroids = []
            # for x in dashedline:
            #     centroids.append((int(x.pt[0]), int(x.pt[1])))
            # pd = (centroids[0][0], centroids[0][1])
            #
            # ## Sort right blobs by Y position
            # # cont = []
            # # for x in contline:
            # #     cont.append((int(x.pt[0]), int(x.pt[1])))
            # #     #print x.pt[0], x.pt[1]
            # cont = sorted(cont, key = operator.itemgetter(1), reverse = True)
            # pc = cont[0]
            #
            # center = my.obtainLaneCenter(pd, pc)
            #
            # ## Controller
            # k = float(w_lane/550)                       # Conversion from pixels to cm
            # d = kv*(height - center[1])                 # cm    from bottom to calculated point
            #
            # error = k*(center[0] - pcar[0])
            # angle = np.arctan2(error , d + d_offset)*(180/np.pi)
            # if angle > theta_m:
            #     angle = theta_m
            # if angle < -theta_m:
            #     angle = -theta_m
            # if flip:
            #     angle = -angle
            #
            # ## Conversion to int steering value
            # steer_range = 170                   # 5 - 175
            # theta_range = 60                    # -30 - 30
            # steer = int((steer_range/theta_range * angle + 90))
            #
            # if Draw:
            #     #Draw points and line found
            #     if not flip:
            #         cv2.line(img, (pd[0], pd[1]), (pc[0], pc[1]), (250, 200, 0), thickness=4)
            #         cv2.circle(img, (pc[0], pc[1]), 10, (250, 100, 0), 4)
            #         cv2.circle(img, (pd[0], pd[1]), 10, (250, 100, 0), 4)
            #         cv2.circle(img, center, 10, (200, 200, 0), 4)
            #     ## !
            #     else:
            #         cv2.line(img, (width - pd[0], pd[1]), (width - pc[0], pc[1]), (250, 200, 0), thickness=4)
            #         cv2.circle(img, (width - pc[0], pc[1]), 10, (250, 100, 0), 4)
            #         cv2.circle(img, (width - pd[0], pd[1]), 10, (250, 100, 0), 4)
            #         cv2.circle(img, (width - center[0], center[1]), 10, (200, 200, 0), 4)
            ## Act as we only have continuous line
            if len(cont) > 0:
                cont = sorted(cont, key = operator.itemgetter(1), reverse = True)
                pc = cont[0]
                ycenter = pc[1]
                xcenter = pc[0] - 275
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


        elif l_dashedline < 1 and l_contline > 0:                                       # Just continuous line

            # cont = []
            # for x in contline:
            #     cont.append((int(x.pt[0]), int(x.pt[1])))

            if len(cont) > 0:
                cont = sorted(cont, key = operator.itemgetter(1), reverse = True)
                pc = cont[0]
                ycenter = pc[1]
                xcenter = pc[0] - 275
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

            # if not flip:
            #     steer = 50
            # else:
            #     steer = 130

            self.speed = speed_

        elif l_contline < 1 and l_dashedline > 1:                                       ## Complete dashed line
            # centroids = []
            # for x in dashedline:
            #     centroids.append((int(x.pt[0]), int(x.pt[1])))

            rangem = 0.2
            x, y, centroids = my.checkSlope(centroids, rangem)

            m, b = my.linearRegression(x, y)
            pi, pf = my.getDashedLine(m, b, h_img)

            pd = (int((pi[0] + pf[0])/2), int((pi[1] + pf[1])/2))

            center = (pd[0] + 250, pd[1])

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
                    cv2.circle(img, center, 10, (200, 200, 0), 4)
                ## !
                else:
                    cv2.line(img, (width - pi[0], pi[1]), (width - pf[0], pf[1]), (0, 200, 200), thickness=4)
                    cv2.circle(img, (width - pd[0], pd[1]), 10, (200, 200, 0), 4)
                    cv2.circle(img, (width - center[0], center[1]), 10, (200, 200, 0), 4)

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

            #cv2.circle(img, (pcar[0], pcar[1]), 10, (250, 100, 0), 4)

        elif l_contline < 1 and l_dashedline == 1:                                              # Just one blob of dashed line
            # centroids = []
            # for x in dashedline:
            #     centroids.append((int(x.pt[0]), int(x.pt[1])))
            pd = (centroids[0][0], centroids[0][1])

            center = (pd[0] + 250, pd[1])
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

        if curve:
            if not flip:
                steer = 55
            else:
                steer = 125
            self.speed = speedc_

        ## If we are in a crosslane
        # if cross != self.pcross:
        #     self.crosscont += 1
        #     self.crosscont = self.crosscont%2
        #     self.speed = speed_

        # print cross
        # if self.crosscont:
        #     if len(cont) > 0:
        #         cont = sorted(cont, key = operator.itemgetter(1), reverse = True)
        #         pc = cont[0]
        #         ycenter = pc[1]
        #         xcenter = pc[0] - 300
        #         center = (int(xcenter), int(ycenter))
        #
        #         ## Controller
        #         k = float(w_lane/550)                       # Conversion from pixels to cm
        #         d = kv*(height - center[1])                 # cm    from bottom to calculated point
        #
        #         error = k*(center[0] - pcar[0])
        #         angle = np.arctan2(error , d + d_offset)*(180/np.pi)
        #         if angle > theta_m:
        #             angle = theta_m
        #         if angle < -theta_m:
        #             angle = -theta_m
        #         if flip:
        #             angle = -angle
        #
        #         ## Conversion to int steering value
        #         steer_range = 170                   # 5 - 175
        #         theta_range = 60                    # -30 - 30
        #         steer = int((steer_range/theta_range * angle + 90))
        #         if Draw:
                    #cv2.circle(img, (center[0], center[1]), 10, (200, 200, 0), 4)
        #
        #     self.pcross = cross
        #     self.speed = speed_

        if Draw:
            cv2.circle(img, (pcar[0], pcar[1]), 10, (250, 100, 0), 4)
            #cv2.imshow("L image", left_img)
            #cv2.imshow("R image", right_img)
            #cv2.imshow("Curv image", curv_img)
            #cv2.imshow("H image", cropped_image)

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
