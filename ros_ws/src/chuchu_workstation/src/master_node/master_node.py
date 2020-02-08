#!/usr/bin/env python

import sys
import rospy
from chuchu_workstation.msg import Int16ArrayHeader
from std_srvs.srv import SetBool


class master_node(object):
	def __init__(self):
		# Initialize node
		rospy.init_node('ws_master_node', anonymous = True)

		# Create subscriber for CHUCHU state
		self.stateSub = rospy.Subscriber('chuchu_state', Int16ArrayHeader, self._state_callback)

		# Create subscriber for trainer
		self.trainer = rospy.Subscriber('trainer_pos', Int16ArrayHeader, self._training_pos)

		# Create subscriber for controller
		self.controller = rospy.Subscriber('controller_pos', Int16ArrayHeader, self._controller_pos)

		# Create publisher for CHUCHU state
		self.statePub = rospy.Publisher('chuchu_state_ws', Int16ArrayHeader, queue_size=10)

		# Create publisher for commanded positions
		self.commander = rospy.Publisher('commanded_positions', Int16ArrayHeader, queue_size=10)

		# Boolean to check if hardware is initialized
		self.hardwareInit = False

	# Send initialization signal to onboard hardware
	def init_hardware(self):
		# Wait for service to become available
		rospy.wait_for_service('init_hardware')

		try:
			print "Initializing Hand...."
			# Try to initialize hardware
			set_bool = rospy.ServiceProxy('init_hardware', SetBool)
			ret = set_bool(True)
			# Set state to bool returned by hardware
			self.hardwareInit = ret.success
			print ret

		# If there is an exception, assume not initialized
		except rospy.ServiceException:
			self.hardwareInit = False


	def _state_callback(self, data):
		# Publish data if hand is initialized
		if self.hardwareInit == True:
			# Normalize tactile data
			data.data = self._normalize_tactile(data.data)

			self.statePub.publish(data)

	def _training_pos(self, data):
		# Get state from rosparams
		flag = rospy.get_param('train_flag')

		# If training:
		if flag == True & self.hardwareInit:
			# Publish to commander
			self.commander.publish(data)

	def _controller_pos(self, data):
		# Get state from rosparams
		flag = rospy.get_param('train_flag')

		# If not training:
		if flag == False & self.hardwareInit:
			# Publish to commander
			self.commander.publish(data)

	def _normalize_tactile(self, data):
		# Parse motor and tactile data
		motorData = data[0:16]
		tactileData = data[16:]
		maxTactile = max(tactileData)

		# If maximum value is zero, set to 1
		if maxTactile ==0:
			maxTactile = 1

		normData = []
		
		# Normalize and bin
		for i in range(0, len(tactileData)):
			# Normalize
			normData.append(float(tactileData[i])/float(maxTactile))

			# Bin
			if normData[i] < 0.3:
				normData[i] = 0
			elif (normData[i] >= 0.3 and normData[i] < 0.5):
				normData[i] = 1
			elif (normData[i] >= 0.5 and normData[i] < 0.7):
				normData[i] = 2
			else:
				normData[i] = 3

		# Append max value to the end of vector
		normData.append(maxTactile)

		return (list(motorData) + normData)
			


if __name__ == '__main__':
	mn = master_node()
	mn.init_hardware()

	rospy.spin()