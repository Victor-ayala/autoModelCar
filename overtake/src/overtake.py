#!/usr/bin/env python

#
# Overtake for autoModelCar
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

FRONT_STEER = 90
TIMEOUT_1 = 1
TIMEOUT_2 = 0.5
TIMEOUT_3 = 3
TIMEOUT_4 = 0.5
TIMEOUT_5 = 0.5
FRONTAL_DISTANCE = 1
STEER_1 = 30
SPEED_1 = -250
STEER_2 = 150
SPEED_2 = -250
k_p = 350
DISTANCE_R = 0.22
SPEED_3 = -250
STEER_4 = 150
SPEED_4 = -250
STEER_5 = 20
SPEED_5 = -250

class overtakeController :
    def __init__(self):

        ## Subscribers
        rospy.Subscriber('/obst_detection/front', Bool, self.frontCallback)
        rospy.Subscriber('/obst_detection/left', Bool, self.leftCallback)
        rospy.Subscriber('/obst_detection/rear', Bool, self.rearCallback)
        rospy.Subscriber('/obst_detection/right', Bool, self.rightCallback)
        rospy.Subscriber('/obst_detection/front_distance', Float32, self.fdistanceCallback)
        rospy.Subscriber('/obst_detection/right_distance', Float32, self.rdistanceCallback)

        ## Publishers
        self.speed_pub = rospy.Publisher('/overtake/cmd_speed', Int16, queue_size=1)    # '/overtake/cmd_speed'
        self.steer_pub = rospy.Publisher('/overtake/cmd_steer', Int16, queue_size=1)
        self.overtake_pub = rospy.Publisher('/overtake/available', Bool, queue_size=1)
        
        ## Command variables
        self.cmd_speed = Int16()
        self.cmd_steer = Int16()
        self._overtake = Bool()

        ## Obstacle detection in four sides 
        self._obstFront = False
        self._obstLeft = False
        self._obstRear = False
        self._obstRight = False

        ## Distance in front and right side
        self._frontDist = 10
        self._rightDist = 10

        self._state = 0 # 0 -> normal 1 -> detected obstacle (idle) 2 -> overtake 3 -> run parallel to car  4 -> return to main lane 
        self._pstate = -1 

        self.tStart = False
        self.ctime = rospy.Time.now()
        self.ptime = rospy.Time.now()

    def getOvertakeParams(self):

        if rospy.has_param('/autoModelCar/overtake/a'):
            self.minFront = rospy.get_param('/autoModelCar/overtake/a')

    def frontCallback(self, msg):
        self._obstFront = msg.data
    def leftCallback(self, msg):
        self._obstLeft = msg.data
    def rearCallback(self, msg):
        self._obstRear = msg.data
    def rightCallback(self, msg):
        self._obstRight = msg.data
    def fdistanceCallback(self, msg):
        self._frontDist = msg.data
    def rdistanceCallback(self, msg):
        self._rightDist = msg.data

    def overtake(self):
        methodName = 'state_' + str(self._state)
        method = getattr(self, methodName, lambda: "Invalid state")
        if not self._state == self._pstate:
        	print("State = " + str(self._state))
        	self._pstate = self._state
        if not self._state == 0:
            self._overtake.data = True
        else:
            self._overtake.data = False
        self.overtake_pub.publish(self._overtake)
        return method()

    def state_0(self):
    	## Check front distance
        if self._frontDist < FRONTAL_DISTANCE and self._frontDist > 0:
            self._state = 1
        

    def state_1(self):
        self.cmd_steer.data = STEER_1
        self.cmd_speed.data = SPEED_1
        # if not self._obstFront:
        # 	self._state = 2
        t = self.runTimer()
        if t > TIMEOUT_1:
            self._state = 2
            self.tStart = False

    def state_2(self):
        self.cmd_steer.data = STEER_2
        self.cmd_speed.data = SPEED_2
        # if not self._obstFront:
        #     self._state = 2
        t = self.runTimer()
        if t > TIMEOUT_2:
            self._state = 3
            self.tStart = False

    def state_3(self):
        ## Keep the car parallel to the obstacle
        self.errx = DISTANCE_R - self._rightDist
        print(k_p*self.errx)
        self.cmd_steer.data = FRONT_STEER - k_p*self.errx 
        self.cmd_speed.data = SPEED_3

        if not self._obstRight or abs(k_p*self.errx) > 200:
            self._state = 4

        # t = self.runTimer()
        # if t > TIMEOUT_3:
        # 	self._state = 3
        # 	self.tStart = False

    def state_4(self):
        self.cmd_steer.data = STEER_4
        self.cmd_speed.data = SPEED_4

        # if not self._obstRear and not self._obstRight:
        #     self._state = 0

        t = self.runTimer()
        if t > TIMEOUT_4:
        	self._state = 5
        	self.tStart = False

    def state_5(self):
        ## Reincorporate to main lane
        self.cmd_steer.data = STEER_5
        self.cmd_speed.data = SPEED_5

        # if not self._obstRear and not self._obstRight:
        #     self._state = 0

        t = self.runTimer()
        if t > TIMEOUT_5:
            self._state = 0
            self.tStart = False

    def state_11(self):
        # Wait to see if the obstacle is dynamic
        t = self.runTimer()
        if t > TIMEOUT_4:
            self._state = 2
            self.tStart = False

    def runTimer(self):
    	if not self.tStart:
    		## Intialize timers
    		self.ctime = rospy.Time.now()
    		self.ptime = rospy.Time.now()	
    		self.tStart = True

    	## Get current time elapsed	
    	self.ctime = rospy.Time.now()
        d = self.ctime - self.ptime
        return d.to_sec()

if __name__ == '__main__':
    rospy.init_node('overtake')
    rospy.loginfo("Init overtake capabilities")
    ov = overtakeController()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        ov.overtake()
        if ov._state > 0:
            ov.speed_pub.publish(ov.cmd_speed)
            ov.steer_pub.publish(ov.cmd_steer)
        r.sleep()

    ov._overtake.data = False
    ov.overtake_pub.publish(ov._overtake) 

