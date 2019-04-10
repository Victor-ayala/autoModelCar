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
            lane_img = self.laneDetection(self.cv2_img)
            # lane_img = self.cv2_img
            cv2.imshow("Lane", lane_img)
            cv2.waitKey(1)

    def laneDetection(self, image):
        reverse = True
        crop = False
        img_fraction = 2

        im_in = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        if reverse:
            im_in = cv2.bitwise_not(im_in)
            #cv2.imshow("Not image", im_in)

        # cropping image
        crop_y = int(im_in.shape[0]/img_fraction)
        width = int(im_in.shape[1])
        height = int(im_in.shape[0])
        pixels = width*height

        if crop:
            cropped_image = im_in[crop_y:height,0:width]  # Image to be filtered
            img = image[crop_y:height,0:width]            # Image to draw on it
        else:
            cropped_image = im_in   # Image to be filtered
            img = image             # Image to draw on it


        pixels = int(img.shape[1]*img.shape[0])
        maxArea = 10*pixels/100                     # 10% of cells
        minArea = 20

        h_img = int(img.shape[0])                   # Height of the new image

        # Gaussian blur
        blurred_image_1 = cv2.GaussianBlur(cropped_image, (15, 15), 0)

        # Bilateral filter
        blurred_image_2 = cv2.bilateralFilter(cropped_image,15,55,55)

        # Threshold
        #th, im_th = cv2.threshold(blurred_image_2, 200, 255, cv2.THRESH_BINARY)
        th, im_th = cv2.threshold(blurred_image_1, 200, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        #cv2.imshow("Not image binary", im_th)

        # Morphological transformantion
        # closing
        kernel = np.ones((9,9),np.uint8)
        closing = cv2.morphologyEx(im_th, cv2.MORPH_CLOSE, kernel, iterations = 1)
        #cv2.imshow("closed image", closing)

        # Ffind contours
        im2, contours, hierarchy = cv2.findContours(closing,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

        #ordered contours
        sorted_contours = sorted(contours, key = cv2.contourArea, reverse = True)

        ordered_contours = []
        big_blob = []

        # Draw centroids
        for i in sorted_contours:
            area = cv2.contourArea(i)
            if area > minArea and area < maxArea:
                ordered_contours.append(i)
                cv2.drawContours(img, i, -1, (0, 0, 255), 3)

        big_blob.append(ordered_contours[0])
        _, _, big_blob_centroid = my.obtainCentroid(big_blob, minArea, maxArea)

        cx_array, cy_array, centroids = my.obtainCentroid(ordered_contours, minArea, maxArea)

        # Sorting centroids
        centroids = sorted(centroids, key = operator.itemgetter(0), reverse=True)

        centroids_sorted = []
        condition = False

        for i in centroids:
            if i == big_blob_centroid[0]:
                condition = True
            if condition:
                centroids_sorted.append(i)
        #        cv2.circle(img, (i[0], i[1]) , 10, (0, 200, 0), 4)

        cv2.drawContours(img, big_blob, -1, (0, 200 ,200), 3)
        #cv2.imshow("contours image", img)


        ## Center point of continuous line
        x_c = centroids_sorted[0][0]
        y_c = centroids_sorted[0][1]
        ## Exclude most right centroid (continuous line)
        centroids_sorted.pop(0)

        max_y = max(centroids, key = operator.itemgetter(1))
        new_cropped_image = img[0:max_y[1], 0:width]
        cv2.imshow("New cropped image", new_cropped_image)

        ## Preparing centroids for linear regresion
        x = []
        y = []
        for i in centroids_sorted:
            x.append(i[0])
            y.append(i[1])
        # print(len(x))
        ## Check slope to discard false blob in dashed line
        #a, b, centroids = my.checkSlope(x, y, centroids)

        m_array = []
        for i in range(1, len(x)):
            slope = (y[i] - y[i - 1])/(x[i] - x[i - 1])
            m_array.append(slope)

        mode = max(set(m_array), key=m_array.count)

        for i in range(1, len(centroids_sorted)):
            if m_array[i - 1] != mode:
                print "pop"
                centroids_sorted.pop(i)
                x.pop(i)
                y.pop(i)
            else:
                cv2.circle(img, (x[i], y[i]) , 10, (0, 200, 0), 4)
        # print(len(x))

        ## Linear regresion procedure
        m, b, x_d, y_d, x_m, y_m = my.linearRegression(x, y, h_img)
        cv2.circle(img, (0, int(b)) , 10, (200, 200, 0), 4)

        y_t = [b, 0]
        y_t = np.array(y_t, dtype=np.float64)
        x_t = (y_t - b)/m

        y_d = int(sum(y_t)/len(y_t))
        x_d = int((y_d - b)/m)

        # Obtain car center
        x_car, y_car = my.obtainCarCenter(x_d, y_d, x_c, y_c)

        # Draw obtained lines and point
        cv2.line(img, (int(x_m[0]), int(y_m[0])), (int(x_m[-1]), int(y_m[-1])), (255, 0, 0), thickness = 4)
        cv2.line(img, (x_d, y_d), (x_c, y_c), (0, 200, 200), thickness=4)
        cv2.circle(img, (x_car, y_car), 8, (0, 255, 189), 3)
        #cv2.circle(img,(x_c,y_c), 10, (0, 200, 0), 4)
        #cv2.circle(img,(x_d, y_d), 10, (0, 200, 0), 4)

        # Final image with blobs and centroids
        # cv2.imshow("lines", img)
        return img

    # def showImage(self):



if __name__ == '__main__':
    rospy.init_node('lanedetection')
    rospy.loginfo("Init Lane detection")
    ld = laneDetector()
    try:

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
