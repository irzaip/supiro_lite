#!/usr/bin/env python
import rospy
from supiro_lite.msg import motorpower
from supiro_lite.msg import encoder


def callback(data):
	print("leftenc:", data.leftenc, " rightenc:", data.rightenc)

def sendpower():
	pub = rospy.Publisher("motorpwr", motorpower)
	rospy.init_node("pwr", anonymous=True)
	rate = rospy.Rate(2)

	rospy.Subscriber('encoder', encoder, callback)

	mtrpwr = motorpower()
	mtrpwr.leftpwr = 1000
	mtrpwr.rightpwr = 1000
	mtrpwr.leftdir = 0
	mtrpwr.rightdir = 0

	while not rospy.is_shutdown():
		rospy.loginfo("Send power")
		pub.publish(mtrpwr)
		rate.sleep()


if __name__ == '__main__':
	try:
		print("Hai")
		sendpower()
	except rospy.ROSInterruptException:
		pass


