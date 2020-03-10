#!/usr/bin/env python

#
# Manual control for autoModelCar
#
# coded by: Victor Ayala Alfaro
#
# Vision, Robotics and Artificial Intelligence Laboratory 
# (LaViRIA), Guanajuato, Mexico
#

import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from std_msgs.msg import String
from std_msgs.msg import Bool

FRONT_STEER = 90
CRUISE_SPEED = 250
rate = rospy.get_param("/autoModelCar/joy2car/rate")
manual = rospy.get_param("/autoModelCar/joy2car/manual_control")

class joystick2car:
	def __init__(self):
		## Variables
		self.speed = Int16()
		self.steer = Int16()
		self.light = String()
		self.estop = Bool()
		self._speed = 0
		self._steer = 90
		self._light = False
		self._cruise = False
		
		## Publishers
		if manual:
			self.cmd_pub = rospy.Publisher('/manual_control/speed', Int16, queue_size=10)
			self.steer_pub = rospy.Publisher('/manual_control/steering', Int16, queue_size=10)
		else:
			self.cmd_pub = rospy.Publisher('/manual_control/cmd_speed', Int16, queue_size=10)
			self.steer_pub = rospy.Publisher('/manual_control/cmd_steer', Int16, queue_size=10)
		self.led_pub = rospy.Publisher('/manual_control/lights', String, queue_size=10)
		self.estop_pub = rospy.Publisher('/manual_control/cmd_estop', Bool, queue_size=1)
		
		## Subscribers
		rospy.Subscriber("joy", Joy, self.callback)

		self.getJoyParams()
	
	def getJoyParams(self):
		self.kspeed = rospy.get_param("/autoModelCar/joy2car/kspeed")
		self.kspeedrv = rospy.get_param("/autoModelCar/joy2car/kspeedrv")
		self.kbreak = rospy.get_param("/autoModelCar/joy2car/kbreak")
		self.deceleration = rospy.get_param("/autoModelCar/joy2car/kidle")

		self.angle_max = rospy.get_param("/autoModelCar/joy2car/max_angle")
		self.speed_max = rospy.get_param("/autoModelCar/joy2car/max_speed")
		self.speed_maxrv = rospy.get_param("/autoModelCar/joy2car/max_speed_rv")
		self.speed_offset = rospy.get_param("/autoModelCar/joy2car/speed_offset")
		self.rv_offset = rospy.get_param("/autoModelCar/joy2car/reverse_offset")

	def callback(self, msg):
		
		## Emergency stop
		if msg.buttons[2]:
			if not self.estop.data:
				rospy.loginfo("Emergency break") 
			self.estop.data = True
			self.estop_pub.publish(self.estop)
			self._speed = 0
			
		if msg.buttons[3]:
			if self.estop.data:
				rospy.loginfo("Restoring driving capabilities")
			self.estop.data = False
			self.estop_pub.publish(self.estop)

		if msg.buttons[1]:
			self._speed = CRUISE_SPEED
			self._steer = FRONT_STEER
			self._cruise = not self._cruise

		if not self.estop.data:
			if not self._cruise:
				## Steer value
				self._steer = FRONT_STEER - self.angle_max*msg.axes[0]
			
				## Accelerate
				if msg.buttons[7] > 0 and msg.buttons[6] == 0:
					# Start at offset speed value
					if self._speed >= 0 and self._speed < self.speed_offset:
						self._speed = self.speed_offset
					if self._speed >= self.speed_offset:
						self._speed += self.kspeed
					if self._speed < 0:
						self._speed += self.kbreak
						
				## Break & reverse
				if msg.buttons[6] > 0:
					if self._speed > 0:
						self._speed -= self.kbreak
					if self._speed > self.rv_offset and self._speed <= 0:
						self._speed = self.rv_offset
					if self._speed <= self.rv_offset:
						self._speed -= self.kspeedrv
				
				## If idle
				if msg.buttons[6] == 0 and msg.buttons[7] == 0:
					if self._speed > 0:
						self._speed -= self.deceleration
					if self._speed < 0:
						self._speed += self.deceleration
					self.light.data = "diL"

				self.checkSpeedLimits()

				## Lights commands
				if msg.buttons[7] == 1:
					if self._speed > self.speed_offset:
						rospy.loginfo("forward")
				if msg.buttons[6] == 1:
					if self._speed < 0:
						rospy.loginfo("backward")
						if self._light:
							self.light.data = "re"
					else:
						rospy.loginfo("break")
						if self._light:
							self.light.data = "pa"
						else:
							self.light.data= "stop"
				if msg.buttons[9] == 1:
					self._light = not self._light
					if self._light:
						self.light.data = "fr"
					else:
						self.light.data = "diL"
			
			## Command are inversed for autoModelCar (negative -> forward, positive -> backward)
			self.speed.data = -self._speed
			self.steer.data = self._steer
			
			print("speed: " + str(self.speed.data) + ", steer: " + str(self.steer.data))
				
			self.cmd_pub.publish(self.speed)
			self.steer_pub.publish(self.steer)
			self.led_pub.publish(self.light)

	def checkSpeedLimits(self):
		## Check for speed limits
		if self._speed > self.speed_max:
			self._speed = self.speed_max
		if self._speed < self.speed_maxrv:
			self._speed = self.speed_maxrv

	def ebreak(self):
	    ## Set speed to zero
	    self.speed.data = 0
	    self.cmd_pub.publish(self.speed)

	    ## Set direction to front (no-steer)
	    self.steer.data = FRONT_STEER
	    self.steer_pub.publish(self.steer)


if __name__ == '__main__':
	
	rospy.init_node('joy2car')
	rospy.loginfo("Init joystick to autoModelCar")
	j = joystick2car()
	r = rospy.Rate(rate)
	try:

		if j.estop.data:
			j.ebreak()
		rospy.spin()
		r.sleep()
	except rospy.ROSInterruptException:
		pass
