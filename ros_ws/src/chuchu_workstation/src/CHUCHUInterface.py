import rospy
from chuchu_workstation.srv import SetString
from std_srvs.srv import SetBool


class CHUCHUInterface(object):
	# Nothing to initialize, just open hand
	def __init__(self):
		self.open()

	# Open hand. Returns True on success, false otherwise
	def open(self):
		rospy.wait_for_service('init_hardware')

		openSrv = rospy.ServiceProxy('init_hardware', SetBool)

		return openSrv(True).success

	# Close hand. Returns True on success, false otherwise
	def close(self):
		rospy.wait_for_service('close_hand')

		closeSrv = rospy.ServiceProxy('close_hand', SetBool)

		return closeSrv(True).success


	# Load presaved NN parameters. Returns True on success, false otherwise
	def load_nn_parameters(self, filename):
		rospy.wait_for_service('load_nn')

		loadSrv = rospy.ServiceProxy('load_nn', SetString)

		return loadSrv(filename).success