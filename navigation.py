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
	localizationTime = 60.0
	while rospy.get_time() - time <= localizationTime:
		vel_msg.angular.z = 5.0
		vel_publisher.publish(vel_msg)
		if rospy.get_time() - time >= localizationTime % 10: rospy.log_info("TIME SPENT LOCALIZING: " + str(rospy.get_time() - time) + " SECONDS OUT OF " + str(localizationTime))
	vel_msg.angular.z = 0.0
	vel_publisher.publish(vel_msg)

	all_points_reached = False
	while not rospy.is_shutdown() and not all_points_reached:
		markerArray = MarkerArray()
		for goal in goals:
			success = False
			while not success:
				#erase previous line markers to draw new ones
				if markerArray.markers:
					for marker in markerArray.markers:
						marker.action = marker.DELETE	
					viz_publisher.publish(markerArray)
					markerArray.markers[:] = []		
				
				#get all points from goal position to robot's position
				myRobot.viaPts = getPoints(myMap, myRobot.curPos, goal)
				if not myRobot.viaPts:
					rospy.loginfo("UNABLE TO FIND A PATH TO FOLLOWING POINT: ")
					rospy.loginfo(goal)
					rospy.loginfo("NOW I WILL TRY NEXT POINT")
					break
					
				#draw line markers
				ID = 0
				for i in range(len(myRobot.viaPts) - 1):
					markerArray.markers.append(createLine(myRobot.viaPts[i], myRobot.viaPts[i+1], ID))
					ID += 1
				viz_publisher.publish(markerArray)	

				#follow points to goal
				success = myRobot.followTrajectory(viz_publisher, vel_msg, vel_publisher, markerArray, tryAgain, success)
				myRobot.viaPts[:] = []
		rospy.sleep(0.01)
		all_points_reached = True

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
