import Leap
import time
import math
import sys
import numpy
from math import(atan2, degrees, radians, cos, asin, sqrt)



class LeapController(object):
	def __init__(self, scale_factor = 1, z_offset = 0):
		self.controller = Leap.Controller()
		self.sf = scale_factor 
		self.z_off = z_offset
		leap_wait = 0
		while self.controller.is_connected != True:
			time.sleep(0.2)
			leap_wait = leap_wait + 1
			if leap_wait > 10:
				sys.exit('Could not connect to Leap')
		if self.controller.is_connected == True:
			print "Connected to LeapMotion"
	

	#Obtain XYZRPY information for right hand (mm and radins)
	def right(self):
		leapFrame = self.controller.frame()
		#create right hand object
		if leapFrame.hands.rightmost.is_right and len(leapFrame.hands):
			rightHand = leapFrame.hands.rightmost
		
	
			# Extract X,Y,Z roll pitch yaw from Leap
			x = (-rightHand.palm_position.z + 50)
			y = (-rightHand.palm_position.x) 
			z = (rightHand.palm_position.y)

			pitch = rightHand.direction.pitch - 0.1745
			yaw = -rightHand.direction.yaw
			roll = rightHand.palm_normal.roll

			grip = rightHand.grab_strength

			P = [(self.sf*x), (self.sf*y), self.sf*(z-self.z_off)]

			R = self._get_R_matrix(rightHand)
			Q = self._R_to_quat(R)
		
			return (P, Q, grip)

	#Obtain XYZRPY information for left hand (mm and radins)
	def left(self):
		leapFrame = self.controller.frame()
		#create right hand object
		if leapFrame.hands.leftmost.is_left and len(leapFrame.hands):
			leftHand = leapFrame.hands.leftmost
	
			# Extract X,Y,Z roll pitch yaw from Leap
			x = (-leftHand.palm_position.z + 50)
			y = (-leftHand.palm_position.x) 
			z = (leftHand.palm_position.y)

			pitch = leftHand.direction.pitch - 0.1745
			yaw = -leftHand.direction.yaw
			roll = leftHand.palm_normal.roll

			grip = leftHand.grab_strength

			P = numpy.array([(self.sf*x), (self.sf*y), self.sf*(z-self.z_off)])

			R = self._get_R_matrix(leftHand)
			Q = self._R_to_quat(R)
		
			return P, Q, grip


    #Create rotation matrix from hand direction (z-axis) and palm normal (x-axis)
	def _get_R_matrix(self, hand_object):
		#Extract x_axis data from direction (corrected for base Z-axis up)
		x_axis = [-(hand_object.direction[2]), 
				  -(hand_object.direction[0]), 
				    hand_object.direction[1]] 

		#Extract z_axis data from palm normal (corrected for base X-axis forward)
		z_axis = [(hand_object.palm_normal[2]), 
				  (hand_object.palm_normal[0]), 
				    -(hand_object.palm_normal[1])]

		y_axis = numpy.cross(z_axis, x_axis)

		return numpy.array([x_axis,y_axis,z_axis])

	def _R_to_quat(self, R):
		
		#Invert rotation matrix from R in EE to R in Base
		R = numpy.linalg.inv(R)

		qx = sqrt(R[0][0]-R[1][1]-R[2][2]+1)/2
		qy = sqrt(-R[0][0]+R[1][1]-R[2][2]+1)/2
		qz = sqrt(-R[0][0]-R[1][1]+R[2][2]+1)/2
		qw = sqrt(R[0][0]+R[1][1]+R[2][2]+1)/2

		quaternion = [qx, qy, qz, qw]

		if qx == max(quaternion):
			quaternion[1] = (R[1][0]+R[0][1])/(4*quaternion[0])
			quaternion[2] =	(R[2][0]+R[0][2])/(4*quaternion[0])
			quaternion[3] = (R[2][1]-R[1][2])/(4*quaternion[0])

			return quaternion

		elif qy == max(quaternion):
			quaternion[0] = (R[1][0]+R[0][1])/(4*quaternion[1])
			quaternion[2] = (R[2][1]+R[1][2])/(4*quaternion[1])
			quaternion[3] = (R[0][2]-R[2][0])/(4*quaternion[1])

			return quaternion

		elif qz == max(quaternion):
			quaternion[0] = (R[0][2]+R[2][0])/(4*quaternion[2])
			quaternion[1] = (R[2][1]+R[1][2])/(4*quaternion[2])
			quaternion[3] = (R[1][0]-R[0][1])/(4*quaternion[2])

			return quaternion

		elif qw == max(quaternion):
			quaternion[0] = (R[2][1]-R[1][2])/(4*quaternion[3])
			quaternion[1] = (R[0][2]-R[2][0])/(4*quaternion[3])
			quaternion[2] = (R[1][0]-R[0][1])/(4*quaternion[3])

			return quaternion

		else:
			return 'error'

	#Extract Roll, Pitch, Yaw from rotation matrix (ZYX Euler or XYZ )
	def _R_to_rpy(self, R):
		pass

