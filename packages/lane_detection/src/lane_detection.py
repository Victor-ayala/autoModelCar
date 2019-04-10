#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import operator
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # ROS Image message -> OpenCV2 image converter
import myLibrary as my

# Instantiate CvBridge
bridge = CvBridge()

class laneDetector :
    def __init__(self):
        rospy.Subscriber("/bebop/image_raw", Image, self.image_callback)

    def image_callback(self, msg):
        try:
            self.cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            success, lane_img = self.laneDetection(self.cv2_img)
            # lane_img = self.cv2_img
            cv2.imshow("Lane", lane_img)
            cv2.waitKey(1)

    def laneDetection(self, image):
        ##-----------------------------
        reverse = True
        crop = True
        fraction = 0.8
        
        im_in = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        width = int(im_in.shape[1])
        height = int(im_in.shape[0])
        
        if reverse:
            im_in = cv2.bitwise_not(im_in)
            #cv2.imshow("Not image", im_in)
        #    cv2.imshow("Not image0", im_in)
        
        # cropping image
        if crop:
            crop_y = int(im_in.shape[0]*(1-fraction))
            cropped_image = im_in[crop_y:height,0:width]
            img = image[crop_y:height,0:width]
        else:
            cropped_image = im_in
            img = image
        #cv2.imshow("Cropped image", img)
            
        pixels = width*height
        
        pixels = int(img.shape[1]*img.shape[0])
        maxArea = pixels*10/100                     # 10% of pixels
        minArea = 20             # 0.01% of pixels
        
        h_img = int(img.shape[0])
        
        ## Bilateral filter
#        blurred_image = cv2.bilateralFilter(cropped_image, 15, 55, 55)
        #cv2.imshow("blurred image", blurred_image)
        
        #x_image= cv2.adaptiveThreshold(cropped_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
        #                               cv2.THRESH_BINARY, 11, 2)
        #cv2.imshow("blurred image", x_image)
        
        ## Threshold
        #th, im_th = cv2.threshold(blurred_image, 250, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        th, im_th = cv2.threshold(cropped_image, 250, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        #th, im_th = cv2.threshold(cropped_image, 250, 255, cv2.THRESH_BINARY)
#        cv2.imshow("Not image binary", im_th)
        
        ## Morphological transformantion
        ## closing
        kernel = np.ones((3, 3),np.uint8)
        im = cv2.morphologyEx(im_th, cv2.MORPH_CLOSE, kernel, iterations = 1)
        #cv2.imshow("Not image", im)
        ##-----------------------------
        
        ## Find contours
        im2, contours, hierarchy = cv2.findContours(im, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        for i in contours:
            cv2.drawContours(img, i, -1, (0, 0, 255), 3)
        #cv2.imshow("Cropped image", img)
        
        ## Ordered contours
        contours = sorted(contours, key = cv2.contourArea, reverse = True)
        
        contours_ord = []
#        bl_contours = []
        big_blob = []
        # Draw centroids
        for i in contours:
            area = cv2.contourArea(i)
            if area > minArea and area < maxArea:
                contours_ord.append(i)
                cv2.drawContours(img, i, -1, (0, 0, 255), 3)
                
        if len(contours_ord) > 0:   
            # Obtain biggest blob (continuous line)
            big_blob.append(contours_ord[0])
            _, _, big_blob_centroid = my.obtainCentroid(big_blob, minArea, maxArea)
            cv2.drawContours(img, big_blob, -1, (0, 200 ,200), 3)
            
            cx_array, cy_array, centroids = my.obtainCentroid(contours_ord, minArea, maxArea)
            
            ## Sorting centroids
            centroids = sorted(centroids, key = operator.itemgetter(0), reverse=True)
            
            centroids_srt = []
            condition = False
            
            ## Pop blobs on the right of continuous line
            for i in centroids:
                if i == big_blob_centroid[0]:
                    condition = True
                if condition:
                    centroids_srt.append(i)
        
            if len(centroids_srt) > 2:
            
                ## Center point of continuous line
                x_c = centroids_srt[0][0]                    
                y_c = centroids_srt[0][1]
                ## Exclude most right centroid (continuous line)
                centroids_srt.pop(0)
                
                ## Re-crop image
                max_y = max(centroids, key = operator.itemgetter(1))
                new_cropped_image = img[0:max_y[1], 0:width]
                #cv2.imshow("New cropped image", new_cropped_image)
            
                centroids_srt = sorted(centroids_srt, key = operator.itemgetter(1), reverse=True)
            
                ## Preparing centroids for linear regresion
                x = []
                y = []
                for i in centroids_srt:
                    x.append(i[0])
                    y.append(i[1])
                #print(len(x))
                ## Check slope to discard false blob in dashed line
                #a, b, centroids = my.checkSlope(x, y, centroids)
            
                ## Obtain slopes to compare between them
                m_array = []
                for i in range(1, len(x)):
                    slope = (y[i] - y[i - 1])/(x[i] - x[i - 1])
                    m_array.append(slope)
            
                ## Search the  slope mode
                mode = max(set(m_array), key=m_array.count)
                xpop = []
                for i in range(1, len(centroids_srt)):
                    if m_array[i - 1] != mode:
                        xpop.append(i)
                    else: 
                        cv2.circle(img, (x[i], y[i]) , 10, (0, 200, 0), 4)
                
                ## Pop blacklisted centroids
                for i in reversed(xpop):
                    centroids_srt.pop(i)
                    x.pop(i)
                    y.pop(i)
            
                ## Linear regresion procedure
                m, b, x_d, y_d, x_m, y_m = my.linearRegression(x, y, h_img)
                #cv2.circle(img, (0, int(b)) , 10, (200, 200, 0), 4)
                
                y_t = [b, 0]
                y_t = np.array(y_t, dtype=np.float64)
#                x_t = (y_t - b)/m
                    
                y_d = int(y_c)                    
                x_d = int((y_d - b)/m)
                
                # Obtain car center
                x_car, y_car = my.obtainCarCenter(x_d, y_d, x_c, y_c)
                
                # Draw obtained lines and point
                cv2.line(img, (int(x_m[0]), int(y_m[0])), (int(x_m[-1]), int(y_m[-1])), (255, 0, 0), thickness = 4)
                cv2.line(img, (x_d, y_d), (x_c, y_c), (0, 200, 200), thickness=4)
                cv2.circle(img, (x_car, y_car), 8, (0, 255, 189), 3)
                #cv2.circle(img,(x_c,y_c), 10, (0, 200, 0), 4)
                #cv2.circle(img,(x_d, y_d), 10, (0, 200, 0), 4)
          
            else:
                print "Not enough points on dashed line"
                Success = False
        else:
            print "There's no blobs to detect"
            Success = False

        return Success, img

    # def showImage(self):



if __name__ == '__main__':
    rospy.init_node('lanedetection')
    rospy.loginfo("Init Lane detection")
    ld = laneDetector()
    try:

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
