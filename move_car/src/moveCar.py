#!/usr/bin/env python

#
# Auto-movement for autoModelCar
#
# coded by: Victor Ayala Alfaro
#
# Vision, Robotics and Artificial Intelligence Laboratory 
# (LaViRIA), Guanajuato, Mexico
#

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

class autoModelCar:
    def __init__(self):

        ## Frequency rate 
        self.MOVEMENT_RATE = 50 #Hz
        self.FRONT_STEER = 90

        self._speed = 0
        self._steer = self.FRONT_STEER
        self._ospeed = 0
        self._osteer = self.FRONT_STEER
        self._overtake = False

        self.speed = Int16()
        self.speed.data = self._speed
        self.steer = Int16()
        self.steer.data = self._steer
        

        self.psteer = self.steer.data
        self.pspeed = self.speed.data
        self.emergency_stop = False

        ## Subscribers
        rospy.Subscriber("/manual_control/cmd_steer", Int16, self.steerCallback)
        rospy.Subscriber("/manual_control/cmd_speed", Int16, self.speedCallback)
        rospy.Subscriber("/manual_control/cmd_estop", Bool, self.emergencyCallback)

        rospy.Subscriber("/overtake/cmd_steer", Int16, self.overSteerCallback)
        rospy.Subscriber("/overtake/cmd_speed", Int16, self.overSpeedCallback)
        rospy.Subscriber("/overtake/available", Bool, self.overtakeCallback)

        ## Publishers
        self.speed_pub = rospy.Publisher('/manual_control/speed', Int16, queue_size=1)
        self.steer_pub = rospy.Publisher('/manual_control/steering', Int16, queue_size=1)

    def drive(self):
        self.speed.data = self._speed
        self.steer.data = self._steer

        ## accelerate
        self.speed_pub.publish(self.speed)      
        ## steer
        if not self.steer.data == self.psteer:
            self.steer_pub.publish(self.steer)

        # print(self.speed.data, self.steer.data)
        self.pspeed = self.speed.data
        self.psteer = self.steer.data

    def overtake(self):
        self.speed.data = self._ospeed
        self.steer.data = self._osteer
        ## Publish movement commands
        self.speed_pub.publish(self.speed)
        if not self.steer.data == self.psteer:
            self.steer_pub.publish(self.steer)

        self.pspeed = self.speed.data
        self.psteer = self.steer.data

    def ebreak(self):
        ## Set speed to zero
        self.speed.data = 0
        self.speed_pub.publish(self.speed)

        ## Set direction to front (no-steer)
        if not self.steer.data == self.FRONT_STEER:
            self.steer.data = self.FRONT_STEER
            self.steer_pub.publish(self.steer)


    def steerCallback(self, msg):
        self._steer = msg.data

    def speedCallback(self, msg):
        self._speed = msg.data

    def emergencyCallback(self, msg):
        self.emergency_stop = msg.data

    def overtakeCallback(self, msg):
        self._overtake = msg.data

    def overSteerCallback(self, msg):
        self._osteer = msg.data

    def overSpeedCallback(self, msg):
        self._ospeed = msg.data

if __name__ == '__main__':
    rospy.init_node('car_motor')
    rospy.loginfo("Init car movement")
    car = autoModelCar()
    r = rospy.Rate(car.MOVEMENT_RATE)

    while not rospy.is_shutdown():
        # car.overtake = False
        if not car.emergency_stop:
            if not car._overtake:
                car.drive()
            else:
                car.overtake()
        else:
            car.ebreak()
        #print(car.overtake)
        r.sleep()
    
