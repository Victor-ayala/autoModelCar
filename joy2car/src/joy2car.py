#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

angle_max = rospy.get_param("/autoModelCar/max_angle")
speed_max = rospy.get_param("/autoModelCar/max_speed")
rv_offset = rospy.get_param("/autoModelCar/reverse_offset")
rv_speed = rospy.get_param("/autoModelCar/reverse_speed")

speed = 0

class joystick:
	def __init__(self):
		self.speed = Int16()
		self.steer = Int16()
		self.light = ""
		self.speed.data = 0
		self.steer.data = 0
		
		self.kspeed = rospy.get_param("/autoModelCar/kspeed")
		self.kbreak = rospy.get_param("/autoModelCar/kbreak")
		self.deceleration = rospy.get_param("/autoModelCar/kidle")
		
		self.cmd_pub = rospy.Publisher('/manual_control/speed', Int16, queue_size=10)
		self.steer_pub = rospy.Publisher('/manual_control/steering', Int16, queue_size=10)
		rospy.Subscriber("joy", Joy, self.callback)
	
	def callback(self, msg):
		global speed_max
		global angle_max
		global speed
		global rv_offset
		global rv_speed
		
		# Angle value
		self.steer.data = 90 - angle_max*msg.axes[0]
		
		# Assign speed and steer values from joystick
		if msg.buttons[7] > 0:
			if msg.buttons[6] == 0 and speed >= 0:
				speed += self.kspeed
			else:
				speed = 0
		
		if msg.buttons[6] > 0:
			speed -= self.kbreak
		
		if msg.buttons[6] == 0 and msg.buttons[7] == 0:
			if speed > 0:
				speed -= self.deceleration
			if speed < 0:
				speed += self.deceleration
		
		# Check for speed limits and break time
		if speed > speed_max:
			speed = speed_max

		
		# Command are inversed for autoModelCar (negative -> forward, positive -> backward)
		if speed < 0 and speed > rv_offset:
			self.speed.data = 0
		elif speed < rv_offset:
			self.speed.data = -rv_speed
			speed = rv_offset
		else:
			self.speed.data = -speed
		
		if msg.buttons[7] == 1:
			rospy.loginfo("forward")
		if msg.buttons[6] == 1:
			if speed < 0:
				rospy.loginfo("backward")
			else:
				rospy.loginfo("break")
		
		print("speed: " + str(self.speed.data))
		print("steer: " + str(self.steer.data))
			
		self.cmd_pub.publish(self.speed)
		self.steer_pub.publish(self.steer)


if __name__ == '__main__':
	rospy.init_node('joy2car')
	rospy.loginfo("Init joystick to autoModelCar")
	j = joystick()
	r = rospy.Rate(10)
	try:
		rospy.spin()
		r.sleep()
	except rospy.ROSInterruptException:
		pass
