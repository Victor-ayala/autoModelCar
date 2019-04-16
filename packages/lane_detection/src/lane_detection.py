#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import operator
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError # ROS Image message -> OpenCV2 image converter
from std_msgs.msg import Int16
import myLibrary as my

# Instantiate CvBridge
bridge = CvBridge()

class laneDetector :
    def __init__(self):
        rospy.Subscriber("/app/camera/color/image_raw/compressed", CompressedImage, self.image_callback)
        # rospy.Subscriber("/manual_control/obstacle", Int16, self.obstcallback)
        self.steer_pub = rospy.Publisher('/manual_control/desired_angle', Int16, queue_size=1)
        self.steer_16 = Int16()

    def image_callback(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        lane_img, self.steer_16.data, angle = self.laneDetection(self.cv2_img, reverse = True, flip = False)
        #lane_img, _, _ = self.laneDetection(self.cv2_img, reverse = True, flip = False)
        #lane_img = self.cv2_img
        self.steer_pub.publish(self.steer_16)
        #cv2.imshow("Lane", self.cv2_img)
        cv2.waitKey(1)
        print("steer = " + str(self.steer_16.data))
        print("angle = " + str(angle))

    def obstCallback(msg):
        flag = msg.data
        if flag == 10:                       # Evading an obstacle
            Flip = True
        if flag == 30:                       # Return to the right 
            Flip == False

    def laneDetection(self, image, reverse = False, flip = False, crop = True, fraction = 0.5):
        ##-----------------------------
        im_in = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        width = int(im_in.shape[1])
        height = int(im_in.shape[0])

        angle = 0

        if reverse:
            im_in = cv2.bitwise_not(im_in)
        #    cv2.imshow("Not image0", im_in)

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
            #img = cv2.flip(img, 1 )
            left_img = cv2.flip(img, 1 )

        left_img = cropped_image.copy()
        right_img = cropped_image.copy()
        wo2 = width/2
   
        for i in range(width/2):
            right_img[:, i] = 1
            left_img[:, wo2 + i] = 1

        
        #cv2.imshow("Cropped image", cropped_image)

        pixels = int(img.shape[1]*img.shape[0])
        h_img = int(img.shape[0])

        ##-----------------------------


        ## Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
 
        maxArea = 1*pixels/100                  # 1% of cells
        minArea = 0.05*pixels/100               # 0.02% of cells
        #maxArea = 5*pixels/100                  # 5% of cells
        #minArea = 0.5*pixels/100               # 0.5% of cells

        ## Change thresholds
        params.filterByColor = True
        params.blobColor = 255
        params.minThreshold = 180
        params.maxThreshold = 255
 
        ## Filter by Area.
        params.filterByArea = True
        params.minArea = minArea
        params.maxArea = maxArea
 
        ## Filter by Circularity
        params.filterByCircularity = False
        #params.minCircularity = 0.1
 
        ## Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.7
        params.maxConvexity = 1
 
        ## Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.0
        params.maxInertiaRatio = 0.2 

        ## Create a detector with the parameters
        detector = cv2.SimpleBlobDetector_create(params)

        ## Detect blobs.
        dashedline = detector.detect(left_img)

        # ## Setup new detection parameters
        # params2 = cv2.SimpleBlobDetector_Params()
 
        # maxArea = 4000            
        # minArea = 400               

        # ## Change thresholds
        # params2.filterByColor = True
        # params2.blobColor = 255
        # params2.minThreshold = 100
        # params2.maxThreshold = 255
 
        # ## Filter by Area.
        # params2.filterByArea = True
        # params2.minArea = minArea
        # params2.maxArea = maxArea
 
        # ## Filter by Circularity
        # params2.filterByCircularity = False
        # #params.minCircularity = 0.1
 
        # ## Filter by Convexity
        # params2.filterByConvexity = True
        # params2.minConvexity = 0.7
 
        # ## Filter by Inertia
        # params2.filterByInertia = True
        # params2.minInertiaRatio = 0.004
        # #params2.maxInertiaRatio = 0.2 

        # detector2 = cv2.SimpleBlobDetector_create(params2)
        # contline = detector2.detect(right_img)


        ## Setup new detection parameters
        params2 = cv2.SimpleBlobDetector_Params()
 
        maxArea = 4000            
        minArea = 400               

        ## Change thresholds
        params2.filterByColor = True
        params2.blobColor = 255
        params2.minThreshold = 100
        params2.maxThreshold = 255
 
        ## Filter by Area.
        params2.filterByArea = True
        params2.minArea = minArea
        params2.maxArea = maxArea
 
        ## Filter by Circularity
        params2.filterByCircularity = False
        #params.minCircularity = 0.1
 
        ## Filter by Convexity
        params2.filterByConvexity = True
        params2.minConvexity = 0.7
 
        ## Filter by Inertia
        params2.filterByInertia = False
        params2.minInertiaRatio = 0.04
        params2.maxInertiaRatio = 0.06

        detector2 = cv2.SimpleBlobDetector_create(params2)
        contline = detector2.detect(right_img)

        #---------------------------------------

        ## Check if there are lines to detect
        if len(dashedline) > 2 and len(contline) > 0:
            
            ## Sort right blobs by Y position
            cont = []
            for x in contline:
                cont.append((int(x.pt[0]), int(x.pt[1])))
            cont = sorted(cont, key = operator.itemgetter(1), reverse = True)
    
            pc = cont[0]
    
            centroids = []
            for x in dashedline:
                centroids.append((int(x.pt[0]), int(x.pt[1])))
    
            rangem = 0.2
            x, y, centroids, dashedline = my.checkSlope(centroids, dashedline, rangem)
    
            m, b = my.linearRegression(x, y)
            pi, pf = my.getDashedLine(m, b, h_img)
    

            yd = pc[1]
            xd = int((yd - b)/m)
            pd = (xd, yd)
    
            center = my.obtainLaneCenter(pd, pc)
    
            #Draw points and line found
            for i in centroids:
                if not flip:
                    cv2.circle(img, i, 8, (0, 255, 189), 3)
                else:
                    cv2.circle(img, (width - i[0], i[1]), 8, (0, 255, 189), 3)  
            if not flip:
                cv2.line(img, (pi[0], pi[1]), (pf[0], pf[1]), (0, 200, 200), thickness=4)
                cv2.line(img, (pd[0], pd[1]), (pc[0], pc[1]), (250, 200, 0), thickness=4)
                cv2.circle(img, pc, 10, (250, 100, 0), 4)
                cv2.circle(img, pd, 10, (250, 100, 0), 4)
                cv2.circle(img, center, 10, (200, 200, 0), 4)
            ## !
            else:
                cv2.line(img, (width - pi[0], pi[1]), (width - pf[0], pf[1]), (0, 200, 200), thickness=4)
                cv2.line(img, (width - pd[0], pd[1]), (width - pc[0], pc[1]), (250, 200, 0), thickness=4)
                cv2.circle(img, (width - pc[0], pc[1]), 10, (250, 100, 0), 4)
                cv2.circle(img, (width - pd[0], pd[1]), 10, (250, 100, 0), 4)
                cv2.circle(img, (width - center[0], center[1]), 10, (200, 200, 0), 4)
    
            ## Controller 
            theta_m = 30                                # deg   min/max value 
            w_lane = 40.0                               # cm    lane width
            k = float(w_lane/float(pc[0] - pd[0]))      # Conversion from pixels to cm
            kv = 0.01
            d_offset = 42                               # cm    from car to bottom image
            d = kv*(height - center[1])                 # cm    from bottom to calculated point
    
            pcar = (width/2, int(height/2))
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

        else:
            if len(dashedline) < 3:
                if not flip:
                    steer = 40
                else:
                    steer = 140
            if len(contline) < 1:
                if not flip:
                    steer = 140
                else:
                    steer = 40
            # print "There are no blobs to detect"

        # print("steer = " + str(steer))
        # print angle



        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        right_img = cv2.drawKeypoints(right_img, contline, np.array([]), (255, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        left_img = cv2.drawKeypoints(left_img, dashedline, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        cv2.imshow("L image", left_img)
        cv2.imshow("R image", right_img)

        # Show keypoints
        # cv2.imshow("Lane detection", img)
        # cv2.waitKey(0)

        return img, steer, angle

    # def showImage(self):



if __name__ == '__main__':
    rospy.init_node('lanedetection')
    rospy.loginfo("Init Lane detection")    
    ld = laneDetector()
    try:
        rospy.spin()
        #cv2.waitKey(0)
    except rospy.ROSInterruptException:
        pass
