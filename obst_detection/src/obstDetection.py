#!/usr/bin/env python

#
# Obstacle detection for autoModelCar
#
# coded by: Victor Ayala Alfaro
#
# Vision, Robotics and Artificial Intelligence Laboratory 
# (LaViRIA), Guanajuato, Mexico
#

import rospy
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

MIN_FRONT = 0
MIN_LEFT = 0
MIN_REAR = 0
MIN_RIGHT = 0
ANGLE_AP = 0

class obstacleDetector :
    def __init__(self):

        self._distance = np.zeros((360, 1))
        self._cmd_speed = 0
        self._cmd_steer = 90

        ## Obstacle detection in four sides 
        self.obstFront = Bool()
        self.obstLeft = Bool()
        self.obstRear = Bool()
        self.obstRight = Bool()
        
        self.obstFront.data = False
        self.obstLeft.data = False
        self.obstRear.data = False
        self.obstRight.data = False
        self.minFront = 0
        self.minLeft = 0
        self.minRear = 0
        self.minRight = 0

        self.minDist = 0
        self.frontDist = 0

        self.getDetectionParams()

        ## Subscribers
        rospy.Subscriber("/scan", LaserScan, self.laserCallback)

        ## Publishers
        self.fDist_pub = rospy.Publisher('/obst_detection/front_distance', Float32, queue_size=1)
        self.rDist_pub = rospy.Publisher('/obst_detection/right_distance', Float32, queue_size=1)
        self.front_pub = rospy.Publisher('/obst_detection/front', Bool, queue_size=1)
        self.left_pub = rospy.Publisher('/obst_detection/left', Bool, queue_size=1)
        self.rear_pub = rospy.Publisher('/obst_detection/rear', Bool, queue_size=1)
        self.right_pub = rospy.Publisher('/obst_detection/right', Bool, queue_size=1)

    def getDetectionParams(self):
        ## Get angle ranges for the four sides
        global MIN_FRONT
        global MIN_LEFT
        global MIN_REAR
        global MIN_RIGHT
        global ANGLE_AP
        if rospy.has_param('/autoModelCar/obstDetector/MINA_FRONT'):
            MIN_FRONT = rospy.get_param('/autoModelCar/obstDetector/MINA_FRONT')
        if rospy.has_param('/autoModelCar/obstDetector/MINA_LEFT'):
            MIN_LEFT = rospy.get_param('/autoModelCar/obstDetector/MINA_LEFT')
        if rospy.has_param('/autoModelCar/obstDetector/MINA_REAR'):
            MIN_REAR = rospy.get_param('/autoModelCar/obstDetector/MINA_REAR')
        if rospy.has_param('/autoModelCar/obstDetector/MINA_RIGHT'):
            MIN_RIGHT = rospy.get_param('/autoModelCar/obstDetector/MINA_RIGHT')
        if rospy.has_param('/autoModelCar/obstDetector/ANGLE_AP'):
            ANGLE_AP = rospy.get_param('/autoModelCar/obstDetector/ANGLE_AP')
        ## Get minima distances for object detection
        if rospy.has_param('/autoModelCar/obstDetector/MIND_FRONT'):
            self.minFront = rospy.get_param('/autoModelCar/obstDetector/MIND_FRONT')
        if rospy.has_param('/autoModelCar/obstDetector/MIND_LEFT'):
            self.minLeft = rospy.get_param('/autoModelCar/obstDetector/MIND_LEFT')
        if rospy.has_param('/autoModelCar/obstDetector/MIND_REAR'):
            self.minRear = rospy.get_param('/autoModelCar/obstDetector/MIND_REAR')
        if rospy.has_param('/autoModelCar/obstDetector/MIND_RIGHT'):
            self.minRight = rospy.get_param('/autoModelCar/obstDetector/MIND_RIGHT')
        #print(self.minFront, self.minLeft, self.minRear, self.minRight, ANGLE_AP)

    def laserCallback(self, msg):
        ## Store distance values
        for i in range(0, 360):
            if not np.isinf(float(msg.ranges[i])) and not np.isnan(float(msg.ranges[i])) and msg.ranges[i] > 0.15 :
                self._distance[i] = msg.ranges[i]
            else:
                self._distance[i] = -1
        #print(self._distance[270])
        #print(self._distance)
        #print("..............")

    def checkSideObstacle(self, side, minDist = 0.30):
        self.minDist = minDist
        methodName = str(side) + 'Detection'
        method = getattr(self, methodName, lambda: "Invalid side")
        return method()

    def frontDetection(self):
        self.obstFront.data = False
        angles = []
        for d in range(ANGLE_AP):
            angles.append((d + MIN_FRONT) % 360)
        for a in angles:
            if self._distance[a] < self.minDist and self._distance[a] > 0:
                self.obstFront.data = True
                break
        self.front_pub.publish(self.obstFront)
        return self.obstFront.data 

    def leftDetection(self):
        self.obstLeft.data = False
        angles = []
        for d in range(ANGLE_AP):
            angles.append((d + MIN_LEFT) % 360)
        for a in angles:
            if self._distance[a] < self.minDist and self._distance[a] > 0:
                self.obstLeft.data = True
                break
        self.left_pub.publish(self.obstLeft)
        return self.obstLeft.data

    def rearDetection(self):
        self.obstRear.data = False
        angles = []
        for d in range(ANGLE_AP):
            angles.append((d + MIN_REAR) % 360)
        for a in angles:
            if self._distance[a] < self.minDist and self._distance[a] > 0:
                self.obstRear.data = True
                break
        self.rear_pub.publish(self.obstRear)
        return self.obstRear.data

    def rightDetection(self):
        self.obstRight.data = False
        angles = []
        for d in range(ANGLE_AP):
            angles.append((d + MIN_RIGHT) % 360)
        for a in angles:
            if self._distance[a] < self.minDist and self._distance[a] > 0:
                self.obstRight.data = True
                break
        self.right_pub.publish(self.obstRight)
        return self.obstRight.data

    def getDistance(self, degree, op=5):
        ## Get distance for a specific degree
        angles = []
        mind = (degree - op + 360) % 360
        maxd = (degree + op + 360) % 360
        for a in range(-op, op + 1):
            angles.append((a + degree + 360)% 360)
        cnt = 0
        distance = 0
        for a in angles:
            if not self._distance[a] == -1:
                distance += self._distance[a]
                cnt += 1
        if cnt > 0:
            return float(distance/cnt)
        else:
            return float(-1)

    def getFrontDistance(self):
        d = Float32()
        d.data = self.getDistance(degree=0, op = 10)
        self.fDist_pub.publish(d) 
        return d.data

    def getRightDistance(self):
        d = Float32()
        d.data = self.getDistance(degree=270, op = 5)
        self.rDist_pub.publish(d) 
        return d.data        

if __name__ == '__main__':
    rospy.init_node('obstacle_detector')
    rospy.loginfo("Init obstacle detector")
    od = obstacleDetector()
    r = rospy.Rate(12)
    while not rospy.is_shutdown():
        if od.checkSideObstacle(side='front', minDist=od.minFront):
            print("Front obstacle")
        if od.checkSideObstacle(side='left', minDist=od.minLeft):
            print("Left obstacle")
        if od.checkSideObstacle(side='rear', minDist=od.minRear) :
            print("Rear obstacle")
        if od.checkSideObstacle(side='right', minDist=od.minRight):
            print("Right obstacle")
        od.getFrontDistance()
        od.getRightDistance()
        r.sleep()     
