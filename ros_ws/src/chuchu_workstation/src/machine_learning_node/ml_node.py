#!/usr/bin/env python

import rospy, sys
from chuchu_workstation.msg import Int16ArrayHeader
from std_msgs.msg import Float32
from chuchu_workstation.srv import SetString, SetStringResponse
import NeuralNetwork
import numpy as np

class ml_node(object):
	def __init__(self):
		# Initialize node
		rospy.init_node('ml_node', anonymous = True)

		# Class variables
		self.state = []
		self.trainingSet = ''

		# Instantiate Neural Network
		self.nn = NeuralNetwork.NeuralNetwork(250,250,250,100)

		# Flags
		self.trained_flag = False
		self.init_train_flag = False
		self.load_flag = False
		self.prediction_flag = False

		# Subscribe to CHUCU state topic
		self.stateSub = rospy.Subscriber('chuchu_state_ws', Int16ArrayHeader, self._state_callback)

		# Publish confidence level
		self.clPub = rospy.Publisher('grasp_quality', Float32, queue_size = 10)

		# Services
		self.train_srv = rospy.Service('train_nn', SetString, self._train_callback)
		self.load_srv = rospy.Service('load_nn', SetString, self._load_callback)

	def load_nn_parameters(self, trainingSetName):
		# Try to load paramters
		if not(self.nn.load_parameters(trainingSetName)):
			# If failed, load training data
			self.nn.load_training_set(trainingSetName)

			# Then train
			self.nn.train_neural_network()

		self.trained_flag = True

	def _state_callback(self, msg):
		# Tensorflow graph does not work w/ROS callbacks. 
		# Workaround by updating a state variable 
		# and publishing grasp quality in a while loop
		self.state = np.array(msg.data, dtype = np.float32)
		# Set flag
		self.prediction_flag = True

	def _train_callback(self, req):
		# Set training file to request name
		self.trainingSet = req.data

		# Set training flag
		self.init_train_flag = True

		# Enter loop, timeout after 20s
		count = 0
		while self.init_train_flag:
			if count >= 20:
				self.init_train_flag = False
				return SetStringResponse(False, "training timeout")
			count += 1
			rospy.sleep(1)

		return SetStringResponse(True, "training success")

	def _load_callback(self, req):
		# Set training file to request name
		self.trainingSet = req.data

		# Set load flag
		self.load_flag = True

		# Enter loop, timeout after 5s
		count = 0
		while self.load_flag:
			if count >= 5:
				self.load_flag = False
				return SetStringResponse(False, "load timeout")
			count += 1
			rospy.sleep(1)

		return SetStringResponse(True, "load success")

if __name__ == '__main__':
	mln = ml_node()

	#mln.nn.load_training_set('redball')
	#mln.nn.train_neural_network()

	#mln.load_nn_parameters('redball')

	r = rospy.Rate(20)

	# State machine to train, load parameters and get predictions
	while not rospy.is_shutdown():
		try:
			# If training service has been called
			if mln.init_train_flag:
				# Load training data
				try:
					mln.nn.load_training_set(mln.trainingSet)
					# Train neural network
					mln.nn.train_neural_network()
					# Mark as trained
					mln.trained_flag = True
					# Reset flag
					mln.init_train_flag = False
				except: 
					pass
				
			# If load parameters has been called
			if mln.load_flag:
				try:
					# Load parameters
					mln.load_nn_parameters(mln.trainingSet)
					# Reset flag
					mln.load_flag = False
				except:
					pass

			# If parameters are loaded and NN is trained, publish prediction 
			elif mln.state != [] and mln.trained_flag and mln.prediction_flag:
				# Publish prediction
				mln.clPub.publish(mln.nn.get_prediction(mln.state))
				# Reset flag
				mln.prediction_flag = False

		except rospy.ROSInterruptException:
			pass

		r.sleep()

		
