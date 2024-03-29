#!/usr/bin/env python
import rospy
import time
from supiro_lite.msg import motorpower
from supiro_lite.msg import encoder

l_leftpwr = 0
l_rightpwr = 0
l_leftdir = 0
l_rightdir = 0
l_leftenc = 0
l_rightenc = 0


def callback(data):
	rospy.loginfo("leftenc: {}, rightenc: {}".format(data.leftenc, data.rightenc))
	l_leftenc = data.leftenc
	l_rightenc = data.rightenc

def sendpwr(leftpwr, rightpwr, leftdir, rightdir):
	global pub
	mtrpwr = motorpower()
	mtrpwr.leftpwr = leftpwr
	mtrpwr.rightpwr = rightpwr
	mtrpwr.leftdir = leftdir
	mtrpwr.rightdir = rightdir
	rospy.loginfo("Send power: {}, {}, {}, {}".format(leftpwr, rightpwr, leftdir, rightdir))
	pub.publish(mtrpwr)
	l_leftpwr = leftpwr
	l_rightpwr = rightpwr
	l_leftdir = leftdir
	l_rightdir = rightdir


def main():
	global pub
	pub = rospy.Publisher("motorpwr", motorpower)
	rospy.init_node("pwr", anonymous=True)
	rate = rospy.Rate(10)

	rospy.Subscriber('encoder', encoder, callback)


	while not rospy.is_shutdown():
		lp, rp, ld, rd = input("input values:")
		t_end = time.time() + 60 * 15
		while time.time() < t_end
			sendpwr(lp,rp,ld,rd)
		rate.sleep()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass


