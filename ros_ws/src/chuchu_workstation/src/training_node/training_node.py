#!/usr/bin/env python

import sys, time
# add UI resource path to sys path
sys.path.append('/home/grant/ros_ws/src/chuchu_workstation/resource')
from training_ui import Ui_Form
import rospy
from chuchu_workstation.msg import Int16ArrayHeader 
from chuchu_workstation.srv import SetString
from PyQt4 import QtGui
from std_srvs.srv import SetBool

class training_node(QtGui.QDialog):
	def __init__(self):
		super(training_node, self).__init__()

		# Initialize node
		rospy.init_node('training_node', anonymous = True)

		# Set up variables for slider positions
		self.sliders = Int16ArrayHeader()
		self.sliders.data = [90, 10, 10, 10, 10, 10, 10, 10]
		rospy.Timer(rospy.Duration(0.5), self._pub_slider)

		# Set up variable for robot state
		# For publishing, so extract just data from message
		self.robotState = ()

		# Set up slider publisher using timer
		self.sliderPub = rospy.Publisher('trainer_pos', Int16ArrayHeader, queue_size=10)

		# Subscribe to CHUCHU state topic
		self.stateSub = rospy.Subscriber('chuchu_state_ws', Int16ArrayHeader, self._state_callback)

		# Setup UI from compiled .ui file
		self._ui = Ui_Form()
		self._ui.setupUi(self)

		# Connect slider signals to callback
		self._ui.T1.valueChanged.connect(self._slider_callback)
		self._ui.T2.valueChanged.connect(self._slider_callback)
		self._ui.F1_1.valueChanged.connect(self._slider_callback)
		self._ui.F1_2.valueChanged.connect(self._slider_callback)
		self._ui.F2_1.valueChanged.connect(self._slider_callback)
		self._ui.F2_2.valueChanged.connect(self._slider_callback)
		self._ui.F3_1.valueChanged.connect(self._slider_callback)
		self._ui.F3_2.valueChanged.connect(self._slider_callback)

		# Connect button signals to callbacks
		self._ui.save_button.clicked.connect(self._save_callback)
		self._ui.train_button.clicked.connect(self._train_callback)

		# Show UI window
		self.show()

	def _pub_slider(self, event):
		# Publish slider message at 2 HZ
		self.sliderPub.publish(self.sliders)

	def _slider_callback(self, value):
		# Get value of slider
		val = int(value % 100)

		# Get ID of slider (0-7)
		sliderID = int(value / 100)

		# Set message data to updated slider value
		self.sliders.data[sliderID] = val

		#print "Value: " + str(val) + " Slider: " + str(sliderID) 

	def _save_callback(self):
		# Get quality from combo box
		quality = self._ui.grasp_list.currentText()

		# Get filename from text input
		fn = self._ui.fn_input.text()

		if fn != '':
			# Open file
			f = open("/home/grant/ros_ws/src/chuchu_workstation/src/machine_learning_node/training_sets/" + fn + '.txt', 'a')

			# Check grasp quality
			if quality == "Bad Grasp":
				# Write robot state and grasp quality to new lines
				stateString = ','.join([str(x) for x in self.robotState])
				f.write(stateString + '\n')
				# Write 0,1 for bad grasp
				f.write('1,0\n')

				# Reset combo box to "Not Selected"
				self._ui.grasp_list.setCurrentIndex(0)

				# Set sliders to 0 (opens hand)
				time.sleep(0.25)
				#self._ui.T1.setValue(90)
				#self._ui.T2.setValue(10)
				#self._ui.F1_1.setValue(10)
				#self._ui.F1_2.setValue(10)
				#self._ui.F2_1.setValue(10)
				#self._ui.F2_2.setValue(10)
				#self._ui.F3_1.setValue(10)
				#self._ui.F3_2.setValue(10)

				# Calibrate sensors
				#set_bool = rospy.ServiceProxy('calibrate_sensors', SetBool)
				#set_bool(True)

			elif quality == "Good Grasp":
				# Write robot state and grasp quality to new lines
				stateString = ','.join([str(x) for x in self.robotState])
				f.write(stateString + '\n')
				# Write 1,0 for good grasp
				f.write('0,1\n')

				# Reset combo box to "Not Selected"
				self._ui.grasp_list.setCurrentIndex(0)

				# Set sliders to 0 (opens hand)
				time.sleep(0.25)
				#self._ui.T1.setValue(90)
				#self._ui.T2.setValue(10)
				#self._ui.F1_1.setValue(10)
				#self._ui.F1_2.setValue(10)
				#self._ui.F2_1.setValue(10)
				#self._ui.F2_2.setValue(10)
				#self._ui.F3_1.setValue(10)
				#self._ui.F3_2.setValue(10)

				# Calibrate sensors
				#set_bool = rospy.ServiceProxy('calibrate_sensors', SetBool)
				#set_bool(True)

			else:
			    pass

			# Close file
			f.close()

	def _train_callback(self):
		print "Training ..."

		try:
			# Wait for service to become available
			rospy.wait_for_service("train_nn")
			trainSrv = rospy.ServiceProxy("train_nn", SetString)

			print trainSrv(str(self._ui.fn_input.text()))
		except:
			print "service call failed"



	def _state_callback(self, data):
		self.robotState = data.data


if __name__ == '__main__':
	# Create Qt app
	app = QtGui.QApplication(sys.argv)

	# Initialize node object
	tn = training_node()

	# Exit on app
	if app.exec_() == 0:
		# Set train_flag back to false
		rospy.set_param('train_flag', False)

		# Exit cleanly (shuts down node)
		sys.exit()

	# Spin... weeeee!
	rospy.spin()