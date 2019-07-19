#!/usr/bin/env python
import rospy
import time
from supiro_lite.msg import motorpower
from supiro_lite.msg import encoder
import pandas as pd


l_leftpwr = 0
l_rightpwr = 0
l_leftdir = 0
l_rightdir = 0
l_leftenc = 0
l_rightenc = 0

df = []

def callback(data):
	global l_leftpwr, l_rightpwr,l_leftdir,l_rightdir,l_leftenc,l_rightenc
	rospy.loginfo("leftenc: {}, rightenc: {}".format(data.leftenc, data.rightenc))
	l_leftenc = data.leftenc
	l_rightenc = data.rightenc

def sendpwr(leftpwr, rightpwr, leftdir, rightdir):
	global pub, l_leftpwr, l_rightpwr,l_leftdir,l_rightdir,l_leftenc,l_rightenc
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
	time.sleep(3)

def main():
	global pub, df, l_leftpwr, l_rightpwr,l_leftdir,l_rightdir,l_leftenc,l_rightenc
	pub = rospy.Publisher("motorpwr", motorpower)
	rospy.init_node("pwr", anonymous=True)
	rate = rospy.Rate(0.002)

	rospy.Subscriber('encoder', encoder, callback)

	lp, rp, ld, rd = 323,323,0,0
	while not rospy.is_shutdown():
		
		while lp <= 1023:
			t_end = time.time() + 35 * 1     #15 detik nih
			while time.time() < t_end:
				sendpwr(lp,rp,ld,rd)
				df.append([l_leftpwr, l_rightpwr,l_leftdir,l_rightdir,l_leftenc,l_rightenc])

			lp += 50
			rp += 50

		df = pd.DataFrame(data=df)
		df.to_csv("/home/irq/mycsv.csv")

		break			
		rate.sleep()
			

		

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass


