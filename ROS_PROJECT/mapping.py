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
	pt_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

	rospy.loginfo("WAITING FOR ROBOT POSITION...")
	while not myRobot.gotRobotPos:
		rospy.sleep(0.01)
	rospy.loginfo("GOT ROBOT POSITION!")
	
	#dummy loop to wait for laser scanner to give a valid first point
	time = rospy.get_time()
	while rospy.get_time() - time <= 3:
		markerID = 0

	tryAgain = False
	vel_msg = Twist()
	markerID = 0
	obsMarkerList = []

	while not rospy.is_shutdown():
		success = False
		markerArray = MarkerArray()

		#erase previous line markers to draw new ones
		if markerArray.markers:
			for marker in markerArray.markers:
				marker.action = marker.DELETE	
			viz_publisher.publish(markerArray)
			markerArray.markers[:] = []

		#draw a marker for every point you decide to move to
		myRobot.getNewPt = True
		while myRobot.getNewPt:
			x = 2
		rospy.loginfo("CURRENT ROBOT POSITION IS:")
		rospy.loginfo(myRobot.curPos)
		rospy.loginfo("GOT OBSTACLE POSITION TO FOLLOW:")
		rospy.loginfo(myRobot.farthestObsAround)
		obsMarker = createPoint(myRobot.farthestObsAround, markerID)
		obsMarkerList.append(obsMarker)
		pt_publisher.publish(obsMarker)
		markerID += 1		
		
		#get intermediate points from robot's current position till goal position
		myRobot.viaPts.append([myRobot.curPos[0], myRobot.curPos[1]])
		if np.linalg.norm(myRobot.curPos-myRobot.farthestObsAround) > 0.1:
			t = 0.1/(np.linalg.norm(myRobot.curPos-myRobot.farthestObsAround))
			while t <= 1:
				xt = (1-t)*myRobot.curPos[0] + t*myRobot.farthestObsAround[0]
				yt = (1-t)*myRobot.curPos[1] + t*myRobot.farthestObsAround[1]
				myRobot.viaPts.append([xt, yt])
				t += t
		myRobot.viaPts.append(copy.deepcopy([myRobot.farthestObsAround[0], myRobot.farthestObsAround[1]]))

		#draw line markers
		ID = 0
		for i in range(len(myRobot.viaPts) - 1):
			markerArray.markers.append(createLine(myRobot.viaPts[i], myRobot.viaPts[i+1], ID))
			ID += 1
		viz_publisher.publish(markerArray)

		#follow points to goal
		myRobot.followTrajectory(viz_publisher, vel_msg, vel_publisher, markerArray, tryAgain, success)
		rospy.loginfo("ARRIVED AT POSITION, NOW GOING TO NEW ONE")
		myRobot.visitedPts.append(myRobot.viaPts[-1])
		myRobot.viaPts[:] = []
		if len(myRobot.visitedPts) >= 60:
			rospy.loginfo("MORE THAN 60 POINTS VISITED!")
			myRobot.visitedPts[:] = []
			for obsMark in obsMarkerList:
				obsMark.action = obsMark.DELETE
				pt_publisher.publish(obsMark)
			markerID = 0
			obsMarkerList[:] = []

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

			
			
























