import rospy
import random

from probabilistic_road_map import *
#from astar import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from scipy.interpolate import Rbf
from math import isnan, ceil
import numpy as np
import matplotlib.pyplot as plt

def getPoints(myMap, initialPos, goalPos):
	initialPosXInMap = int((initialPos[0] - myMap.originX) * 1/myMap.resolution)
	initialPosYInMap = int((initialPos[1] - myMap.originY) * 1/myMap.resolution)
	initialPosInMap = [initialPosXInMap, initialPosYInMap]
	goalPosXInMap = int((goalPos[0] - myMap.originX) * 1/myMap.resolution)
	goalPosYInMap = int((goalPos[1] - myMap.originY) * 1/myMap.resolution)
	goalPosInMap = [goalPosXInMap, goalPosYInMap]
	
	#Note: Map x-axis increases from bottom to top and y-axis increases from left to right
	#Need to align map to coordinates by rotating it 90 degrees clockwise and then flipping it
	#upside down
	myGrid = np.flipud(np.rot90(np.array(myMap.grid)))
	myGridOrg = np.copy(myGrid)
	myGridOrg[initialPosXInMap,initialPosYInMap] = 150
	myGridOrg[goalPosXInMap,goalPosYInMap] = 50
	sx = initialPosXInMap
	sy = initialPosYInMap
	gx = goalPosXInMap
	gy = goalPosYInMap
	robot_size = 1

	ox = list(map(int, np.where(myGrid == 100.0)[0].tolist()))
	oy = list(map(int, np.where(myGrid == 100.0)[1].tolist()))
	if sx in ox and sy in oy:
		rospy.loginfo("REMOVING ROBOT POSITION FROM OBSTACLE LIST")
		ox.remove(sx)
		oy.remove(sy)

	attempts = 0
	while attempts < 10:
		rx, ry = PRM_planning(sx, sy, gx, gy, ox, oy, robot_size)
		candidates = [list(pos) for pos in zip(rx, ry)]
		if not candidates:
			attempts += 1
			rospy.loginfo("COULDN'T FIND ANY PATH AFTER " + str(attempts) + " ATTEMPTS! TRYING AGAIN")
		else: break
		rospy.sleep(0.01)
	if not candidates:
		rospy.loginfo("COULDN'T FIND ANY PATH!")
		return []
	else:
		#rospy.loginfo("ASTAR COMPLETE!")
		rospy.loginfo("PRM COMPLETE!")
		for cand in candidates:
			myGridOrg[int(cand[0]),int(cand[1])] = 200
		myGridOrg = np.rot90(np.flipud(myGridOrg), axes=(1,0))

		fig = plt.figure(figsize=(6, 3.2))
		ax = fig.add_subplot(111)
		ax.set_title('colorMap')
		plt.imshow(myGridOrg)
		ax.set_aspect('equal')

		cax = fig.add_axes([0.12, 0.1, 0.78, 0.8])
		cax.get_xaxis().set_visible(False)
		cax.get_yaxis().set_visible(False)
		cax.patch.set_alpha(0)
		cax.set_frame_on(False)
		plt.colorbar(orientation='vertical')
		plt.show()

		xVals = []
		yVals = []
		#convert back to simulation coordinates
		for candidate in candidates:
			xVals.append(int(candidate[0]) * myMap.resolution + myMap.originX)
			yVals.append(int(candidate[1]) * myMap.resolution + myMap.originY)
		realPts = [list(pos) for pos in zip(xVals, yVals)]
		return realPts[::-1]
		z = np.polyfit(np.array(xVals), np.array(yVals), 5)
		p = np.poly1d(z)
		if xVals[0] < xVals[-1]:
			xNewVals = np.arange(xVals[0], xVals[-1], 0.3)
		else:
			xNewVals = np.arange(xVals[0], xVals[-1], -0.3)
		np.append(xNewVals, xVals[-1])
		yNewVals = []
		for xNewVal in xNewVals:
			yNewVals.append(p(xNewVal))
		realPts = [list(pos) for pos in zip(xNewVals, yNewVals)]
		rospy.sleep(0.01)
		return realPts[::-1]

def createPoint(pt, markerID):
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	marker.ns = "robot"
	marker.header.stamp = rospy.get_rostime()
	marker.id = markerID

	# marker scale
	marker.scale.x = 0.1
	marker.scale.y = 0.1
	marker.scale.z = 0.1

	# marker color
	marker.color.a = 1.0
	marker.color.r = 0.0
	marker.color.g = 1.0
	marker.color.b = 1.0

	# marker orientaiton
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0

	# marker position
	marker.pose.position.x = pt[0]
	marker.pose.position.y = pt[1]
	marker.pose.position.z = 0.0

	return marker

def createLine(pt1, pt2, markerID):
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.type = marker.LINE_STRIP
	marker.action = marker.ADD
	marker.id = markerID

	# marker scale
	marker.scale.x = 0.03
	marker.scale.y = 0.03
	marker.scale.z = 0.03

	# marker color
	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.color.g = 1.0
	marker.color.b = 0.0

	# marker orientaiton
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0

	# marker position
	marker.pose.position.x = 0.0
	marker.pose.position.y = 0.0
	marker.pose.position.z = 0.0

	# marker line points
	marker.points = []
	# first point
	first_line_point = Point()
	first_line_point.x = pt1[0]
	first_line_point.y = pt1[1]
	first_line_point.z = 0.0
	marker.points.append(first_line_point)
	# second point
	second_line_point = Point()
	second_line_point.x = pt2[0]
	second_line_point.y = pt2[1]
	second_line_point.z = 0.0
	marker.points.append(second_line_point)

	return marker
