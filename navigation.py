#!/usr/bin/env python
import rospy
import tf
import copy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np

from Map import *
from Robot import *
from utils import *

def main():
	rospy.init_node('navigation', anonymous=True)
	myRobot = Robot()
	myMap = Map()
	rospy.Subscriber('/scan', LaserScan, myRobot.laserScannerCallback)
	rospy.Subscriber('/map', OccupancyGrid, myMap.getMap)
	rospy.Subscriber("/cmd_vel", Twist, myRobot.getRobotSpeed)
	rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, myRobot.updateRobotPos)
	vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	viz_publisher = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)		
	
	goalFile = open('goals.txt', 'r+')
	goals = []
	while True:
		line = goalFile.readline()
		if line == '':
			break
		else:
			coords = list(line.split(" "))
			goals.append(np.array([float(coords[0]), float(coords[1])]))
	
	rospy.loginfo("WAITING FOR MAP DATA...")
	while not myMap.gotMapData:
		rospy.sleep(0.01)
	rospy.loginfo("GOT MAP DATA!")
	
	tryAgain = False
	vel_msg = Twist()

	#localize robot in the beginning
	time = rospy.get_time()
	while rospy.get_time() - time <= 45.0:
		vel_msg.angular.z = 5.0
		vel_publisher.publish(vel_msg)
	print("TIME TAKEN TO LOCALIZE: " + str(rospy.get_time() - time))

	#make one or both of these true if you are trying to observe other robot behaviors
	#useful if you want to see whether the robot successfully turns towards a goal or not 
	dontMove = False
	dontRot = False
	all_points_reached = False
	while not rospy.is_shutdown() and not all_points_reached:
		markerArray = MarkerArray()
		for goal in goals:
			success = False
			while not success:
				#erase previous markers to draw new ones
				if markerArray.markers:
					for marker in markerArray.markers:
						marker.action = marker.DELETE	
					viz_publisher.publish(markerArray)
					markerArray.markers[:] = []		
				
				#get all points from goal position to robot's position
				viaPts = getPoints(myMap, myRobot.curPos, goal)
				if not viaPts:
					rospy.loginfo("UNABLE TO FIND A PATH TO FOLLOWING POINT: ")
					rospy.loginfo(goal)
					rospy.loginfo("NOW I WILL TRY NEXT POINT...")
					break
				myRobot.viaPts = copy.deepcopy(viaPts)
					
				#draw points
				ID = 0
				for i in range(len(viaPts) - 1):
					markerArray.markers.append(createLine(viaPts[i], viaPts[i+1], ID))
					ID += 1
				viz_publisher.publish(markerArray)
				rospy.sleep(1)		

				#follow points to goal
				tryAgain = False
				for pt in viaPts:
					myRobot.viaPt = pt
					reverseTimer = 0.0
					posTimer = rospy.get_time()
					time = rospy.get_time()
					reverse = False
					while np.linalg.norm(myRobot.curPos-myRobot.viaPt) > 0.6:
						viz_publisher.publish(markerArray)
						dt = rospy.get_time() - time
						time = rospy.get_time()
						myRobot.movToAPoint(vel_msg, 0.2, 0.05, 0.5)

						if myRobot.closestDistToObs <= 0.4 and myRobot.obsDir == 'left':
							vel_msg.angular.z = 1.0

						if myRobot.closestDistToObs <= 0.4 and myRobot.obsDir == 'right':
							vel_msg.angular.z = -1.0

						if (rospy.get_time() - posTimer >= 20.0 or \
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
							posTimer = rospy.get_time()
							rospy.loginfo("TRYING AGAIN")
							tryAgain = True
							success = False
							break

						if reverse:
							rospy.loginfo("ROBOT IS STUCK SOMEWHERE! REVERSING!")
							vel_msg.angular.z = 0
							vel_msg.linear.x = -0.2

						myRobot.checkIfPointBehind(vel_msg, 0.5)
						
						#useful when troubleshooting
						if dontMove: vel_msg.linear.x = 0
						if dontRot: vel_msg.angular.z = 0
						vel_publisher.publish(vel_msg)
						rospy.sleep(0.01)
					if tryAgain: break
					if np.linalg.norm(myRobot.curPos-myRobot.viaPt) <= 0.5: success = True
		rospy.sleep(0.01)
		all_points_reached = True

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
