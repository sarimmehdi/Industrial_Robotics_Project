#!/usr/bin/env python
import rospy
import tf
import random
import copy
import math
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np

from Robot import *
from utils import *

def main():
	rospy.init_node('mapping', anonymous=True)
	myRobot = Robot()
	rospy.Subscriber('/scan', LaserScan, myRobot.laserScannerCallback)
	rospy.Subscriber("/cmd_vel", Twist, myRobot.getRobotSpeed)
	rospy.Subscriber('/odom', Odometry, myRobot.updateRobotPos)
	vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	viz_publisher = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

	rospy.loginfo("WAITING FOR ROBOT POSITION...")
	while not myRobot.gotRobotPos:
		rospy.sleep(0.01)
	rospy.loginfo("GOT ROBOT POSITION!")

	vel_msg = Twist()
	gotviaPt = False
	recentPos = myRobot.curPos
	reverse = False

	#make one or both of these true if you are trying to observe other robot behaviors
	#useful if you want to see whether the robot successfully turns towards a goal or not 
	dontMove = False
	dontRot = False
	while not rospy.is_shutdown():
		markerArray = MarkerArray()

		#erase previous markers to draw new ones
		if markerArray.markers:
			for marker in markerArray.markers:
				marker.action = marker.DELETE	
			viz_publisher.publish(markerArray)
			markerArray.markers[:] = []

		#draw points
		#markerArray.markers.append(createLine(myRobot.curPos, myRobot.farthestObsInFront, 0))
		markerArray.markers.append(createLine(myRobot.curPos, myRobot.viaPt, 1))
		viz_publisher.publish(markerArray)
		if not gotviaPt:
			rospy.loginfo("CURRENT ROBOT POSITION IS:")
			rospy.loginfo(myRobot.curPos)
			myRobot.viaPt = copy.deepcopy(myRobot.farthestObsInFront)
			rospy.loginfo("GOT POSITION!")
			rospy.loginfo(myRobot.viaPt)
			posTimer = rospy.get_time()
			recentPos = myRobot.curPos
			gotviaPt = True
			reverse = False
		myRobot.movToAPoint(vel_msg, 0.2, 0.05, 0.5)

		if np.linalg.norm(myRobot.curPos-recentPos) > 0.1:
			recentPos = myRobot.curPos

		if myRobot.closestDistToObs <= 0.4 and myRobot.obsDir == 'left':
			vel_msg.angular.z = 1.0

		if myRobot.closestDistToObs <= 0.4 and myRobot.obsDir == 'right':
			vel_msg.angular.z = -1.0

		if ((rospy.get_time() - posTimer >= 10.0 and np.linalg.norm(myRobot.curPos-recentPos) <= 0.1) or \
			(myRobot.obsDir == 'both' and \
			myRobot.minAngleToClosestObs in range(61) and \
			myRobot.minAngleToClosestObs in range(360, 361))) and \
			not reverse: 
			reverse = True
			reverseTimer = rospy.get_time()

		if reverse and rospy.get_time() - reverseTimer > 5.0:
			rospy.loginfo("ROBOT SHOULD BE OUT OF STUCK PLACE NOW...")
			vel_msg.angular.z = 0
			vel_msg.linear.x = 0
			reverse = False
			gotviaPt = False
			posTimer = rospy.get_time()
			rospy.loginfo("TRYING AGAIN")

		if reverse:
			rospy.loginfo("ROBOT IS STUCK SOMEWHERE! REVERSING!")
			vel_msg.angular.z = 0
			vel_msg.linear.x = -0.2

		myRobot.checkIfPointBehind(vel_msg, 0.5)
		if dontMove: vel_msg.linear.x = 0
		if dontRot: vel_msg.angular.z = 0
		vel_publisher.publish(vel_msg)
		
		rospy.sleep(0.01)
		if np.linalg.norm(myRobot.curPos-myRobot.viaPt) <= 1.5:
			myRobot.visitedPts.append(myRobot.viaPt)
			if len(myRobot.visitedPts) >= 20: myRobot.visitedPts[:] = []
			gotviaPt = False

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

			
			
























