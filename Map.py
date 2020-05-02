import rospy
import numpy as np
import sys
from math import sin, cos, radians, isnan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class Map:
	def __init__(self):
		self.rows = 0
		self.cols = 0
		self.resolution = 0
		self.originX = 0
		self.originY = 0
		self.grid = np.zeros((1000, 1000))
		self.gotMapData = False
	
	def getMap(self, mapMsg):
		mapInfo = mapMsg.info
		mapDataTuple = mapMsg.data
		self.rows = mapInfo.height
		self.cols = mapInfo.width
		self.resolution = mapInfo.resolution
		self.originX = mapInfo.origin.position.x
		self.originY = mapInfo.origin.position.y
		mapData = list(mapDataTuple)
		self.grid = (np.zeros((self.rows, self.cols)))
		currCell = 0
		for i in range(self.rows):
			for j in range(self.cols):
				if mapData[currCell] > 0 or mapData[currCell] == -1:
					self.grid[i,j] = 100
				currCell += 1
		self.gotMapData = True
		rospy.loginfo("GOT MAP OF SIZE: " + str(self.rows) + " x " + str(self.cols))
		rospy.loginfo("MAP RESOLUTION: " + str(self.resolution))

	def updateGrid(self, angle, dist, position):
		realPointX = position[0] + dist*cos(radians(angle))
		realPointY = position[1] + dist*sin(radians(angle))
		gridX = int((realPointX - self.originX) * 1/self.resolution)
		gridY = int((realPointY - self.originY) * 1/self.resolution)
		self.grid[gridX][gridY] = 100
