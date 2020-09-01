import numpy as np
import copy
import rospy
import random
from math import sin, cos, degrees, radians, atan2
from scipy.optimize import minimize
import tf

class Robot:
	def __init__(self):
		self.closestDistToObs = 999
		self.minAngleToClosestObs = -1
		self.farthestObsAround = np.array([0.0, 0.0])
		self.visitedPts = []
		self.obsDir = 'None'
		self.gotRobotPos = False
		self.obsList = []
		self.curYaw = 0.0
		self.curPos = np.array([0.0, 0.0])
		self.curSpeed = 0.0
		self.curOmega = 0.0
		self.viaPt = np.array([0.0, 0.0])
		self.viaPts = []
		self.robotFacingGoal = False
		self.getNewPt = False
		self.randomNess = 0.0

	def getRobotSpeed(self, twistMsg):
		self.curSpeed = twistMsg.linear.x
		self.curOmega = twistMsg.angular.z

	#2d inverse homogeneous transform to get coordinates (x, y) from reference frame to robot frame
	#theta is angle of rotation of robot frame wrt map frame
	#(x0, y0) are coordinates of origin of robot frame in map frame
	#[ cos(theta)	sin(theta)		-x0*cos(theta)-y0*sin(theta)]	   [x]
	#[-sin(theta)	cos(theta)		 x0*sin(theta)-y0*cos(theta)]	*  [y]
	#[     0            0                        1              ]      [1]
	def getCoordsInRobotFrame(self, coords):
		coordsAsNumpyArray = np.array([[coords[0]], [coords[1]], [1]])
		robotCoords = np.array([[self.curPos[0]], [self.curPos[1]]])
		theta = radians(self.curYaw)
		rotationMat = np.array([[cos(theta), sin(theta)], [-sin(theta), cos(theta)]])
		translationVector = np.matmul(-1*rotationMat, robotCoords)	
		homogeneousMat = np.hstack((rotationMat, translationVector))
		return np.matmul(homogeneousMat, coordsAsNumpyArray)

	#2d homogeneous transform to get coordinates (x, y) from robot frame to reference frame
	#theta is angle of rotation of robot frame wrt map frame
	#(x0, y0) are coordinates of origin of robot frame in map frame
	#[ cos(theta)	-sin(theta)		x0]		[x]
	#[ sin(theta)	cos(theta)		y0]	 *  [y]
	#[     0            0            1]     [1]
	def getCoordsInRefFrame(self, coords):
		coordsAsNumpyArray = np.array([[coords[0]], [coords[1]], [1]])
		translationVector = np.array([[self.curPos[0]], [self.curPos[1]]])
		theta = radians(self.curYaw)
		rotationMat = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
		homogeneousMat = np.hstack((rotationMat, translationVector))
		return np.matmul(homogeneousMat, coordsAsNumpyArray)

	def updateRobotPos(self, pose):
		self.curPos[0] = pose.pose.pose.position.x
		self.curPos[1] = pose.pose.pose.position.y
		thePose = pose.pose.pose.orientation
		quaternion = (thePose.x, thePose.y, thePose.z, thePose.w)
		rpy = tf.transformations.euler_from_quaternion(quaternion)
		self.curYaw = degrees(rpy[2])
		self.gotRobotPos = True

	def inRangeOfPrevVisitedPts(self, candidate):
		for pt in self.visitedPts:
			if np.linalg.norm(np.array([pt[0], pt[1]])-np.array([candidate[0,0], candidate[1,0]])) <= 1.0:
				return True
		return False
				
	def laserScannerCallback(self, laserScanner):
		minVal = 999
		min_range = laserScanner.range_min
		max_range = laserScanner.range_max
		left_and_right = 0
		self.obsList[:] = []
		for i in range(270, 360):
			laser_range = laserScanner.ranges[i]
			if laser_range <= 0.5 and min_range <= laser_range <= max_range:
				obsCoords = self.getCoordsInRefFrame(np.array([laser_range*cos(radians(i)), laser_range*sin(radians(i))]))
				self.obsList.append([obsCoords[0], obsCoords[1]])

		for i in range(90):
			laser_range = laserScanner.ranges[i]
			if laser_range <= 0.5 and min_range <= laser_range <= max_range:
				obsCoords = self.getCoordsInRefFrame(np.array([laser_range*cos(radians(i)), laser_range*sin(radians(i))]))
				self.obsList.append([obsCoords[0], obsCoords[1]])
		
		for i in range(320, 360):
			laser_range = laserScanner.ranges[i]
			if laser_range <= minVal and (min_range <= laser_range <= max_range):
				minVal = laser_range
				self.minAngleToClosestObs = i
				self.obsDir = 'left'
			if laser_range <= 0.3:
				left_and_right += 1
		for i in range(50, 90):
			laser_range = laserScanner.ranges[i]
			if laser_range <= minVal and (min_range <= laser_range <= max_range):
				minVal = laser_range
				self.minAngleToClosestObs = i
				self.obsDir = 'right'
			if laser_range <= 0.3:
				left_and_right += 1
		self.closestDistToObs = minVal
		if left_and_right == 2: self.obsDir = 'both'

		if self.getNewPt:
			minVal = 999
			self.farthestObsAround = np.array([9999.0, 9999.0])
			for i in range(270, 360):
				laser_range = laserScanner.ranges[i]
				if laser_range <= minVal and (min_range <= laser_range <= max_range) and self.getNewPt:
					farthestObsCoords = copy.deepcopy(self.getCoordsInRefFrame(np.array([laser_range*cos(radians(i)), laser_range*sin(radians(i))])))
					if not self.inRangeOfPrevVisitedPts(farthestObsCoords):
						minVal = laser_range
						self.farthestObsAround[0] = copy.deepcopy(farthestObsCoords[0])
						self.farthestObsAround[1] = copy.deepcopy(farthestObsCoords[1])
			for i in range(90):
				laser_range = laserScanner.ranges[i]
				if laser_range <= minVal and (min_range <= laser_range <= max_range) and self.getNewPt:
					farthestObsCoords = copy.deepcopy(self.getCoordsInRefFrame(np.array([laser_range*cos(radians(i)), laser_range*sin(radians(i))])))
					if not self.inRangeOfPrevVisitedPts(farthestObsCoords):
						minVal = laser_range
						self.farthestObsAround[0] = copy.deepcopy(farthestObsCoords[0])
						self.farthestObsAround[1] = copy.deepcopy(farthestObsCoords[1])
			if np.array_equal(np.array([9999, 9999]), self.farthestObsAround) and self.visitedPts:
				rospy.loginfo("UNABLE TO FIND VALID CLOSEST POINT, NOW LOOKING FOR VALID FARTHEST POINT")
				maxVal = 0
				self.farthestObsAround = np.array([9999.0, 9999.0])
				for i in range(270, 360):
					laser_range = laserScanner.ranges[i]
					if laser_range >= maxVal and (min_range <= laser_range <= max_range) and self.getNewPt:
						farthestObsCoords = copy.deepcopy(self.getCoordsInRefFrame(np.array([laser_range*cos(radians(i)), laser_range*sin(radians(i))])))
						if not self.inRangeOfPrevVisitedPts(farthestObsCoords):
							maxVal = laser_range
							self.farthestObsAround[0] = copy.deepcopy(farthestObsCoords[0])
							self.farthestObsAround[1] = copy.deepcopy(farthestObsCoords[1])
				for i in range(90):
					laser_range = laserScanner.ranges[i]
					if laser_range >= maxVal and (min_range <= laser_range <= max_range) and self.getNewPt:
						farthestObsCoords = copy.deepcopy(self.getCoordsInRefFrame(np.array([laser_range*cos(radians(i)), laser_range*sin(radians(i))])))
						if not self.inRangeOfPrevVisitedPts(farthestObsCoords):
							maxVal = laser_range
							self.farthestObsAround[0] = copy.deepcopy(farthestObsCoords[0])
							self.farthestObsAround[1] = copy.deepcopy(farthestObsCoords[1])
				if np.array_equal(np.array([9999, 9999]), self.farthestObsAround) and self.visitedPts:
					rospy.loginfo("UNABLE TO FIND ANY VALID POINT, TIME TO RANDOMLY CHOOSE A POINT FROM THE PREVIOUS 5 VISITED POINTS")
					self.farthestObsAround = np.array(random.choice(self.visitedPts[-5:]))
		self.getNewPt = False

	def checkIfPointBehind(self, twistMsg, Kh):
		#get coordinates of viaPt in robot frame
		p_in_robot = self.getCoordsInRobotFrame(self.viaPt)
		angleDiff = atan2(p_in_robot[1][0], p_in_robot[0][0])
		if abs(degrees(angleDiff)) > 90:
			twistMsg.angular.z = Kh * angleDiff
			twistMsg.linear.x = 0
			self.robotFacingGoal = False
		else:
			self.robotFacingGoal = True

	def movToAPoint(self, twistMsg, Ka, Kr, qstar):
		U_att_grad = Ka * (self.curPos-self.viaPt)
		U_rep_grad = np.array([0.0, 0.0])
		for obs in self.obsList:
			d = np.linalg.norm(self.curPos-np.array(obs))
			if d <= qstar:
				x = self.curPos[0]
				y = self.curPos[1]
				xstar = self.viaPt[0]
				ystar = self.viaPt[1]
				U_rep_grad += (Kr * (1/qstar - 1/d)**2) * (1/d) * np.array([x-xstar,y-ystar])
		qdot = -(U_att_grad + U_rep_grad)
		xdot = qdot[0]
		ydot = qdot[1]
		theta = radians(self.curYaw)
		twistMsg.linear.x = xdot*cos(theta) + ydot*sin(theta)
		twistMsg.angular.z = (1/0.1)*(ydot*cos(theta) - xdot*sin(theta))
		twistMsg.linear.x = np.clip(twistMsg.linear.x, -0.3, 0.3)
		twistMsg.angular.z = np.clip(twistMsg.angular.z, -0.3, 0.3)

	def followTrajectory(self, viz_publisher, vel_msg, vel_publisher, markerArray, tryAgain, success):
		tryAgain = False
		for pt in self.viaPts:
			self.viaPt = pt
			reverseTimer = 0.0
			posTimer = rospy.get_time()
			time = rospy.get_time()
			reverse = False
			while np.linalg.norm(self.curPos-self.viaPt) > 0.6:
				viz_publisher.publish(markerArray)
				dt = rospy.get_time() - time
				time = rospy.get_time()
				self.movToAPoint(vel_msg, 0.2, 0.05, 0.5)

				if self.closestDistToObs <= 0.4 and self.obsDir == 'left':
					vel_msg.angular.z = 1.0

				if self.closestDistToObs <= 0.4 and self.obsDir == 'right':
					vel_msg.angular.z = -1.0

				if (rospy.get_time() - posTimer >= 20.0 or \
					(self.obsDir == 'both' and \
					self.minAngleToClosestObs in range(61) and \
					self.minAngleToClosestObs in range(360, 361))) and \
					not reverse: 
					reverse = True
					reverseTimer = rospy.get_time()

				if reverse and rospy.get_time() - reverseTimer > 5.0:
					rospy.loginfo("ROBOT SHOULD BE OUT OF STUCK PLACE NOW")
					vel_msg.angular.z = 0
					vel_msg.linear.x = 0
					reverse = False
					posTimer = rospy.get_time()
					rospy.loginfo("TRYING AGAIN")
					tryAgain = True
					success = False
					break

				if reverse:
					vel_msg.angular.z = 0
					vel_msg.linear.x = -0.2

				self.checkIfPointBehind(vel_msg, 0.5)
				vel_publisher.publish(vel_msg)
				rospy.sleep(0.01)
			if tryAgain: break
			if np.linalg.norm(self.curPos-self.viaPt) <= 0.5: success = True
		return success
