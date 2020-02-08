#!/usr/bin/env python
import rospy
import random, math
from chuchu_workstation.msg import Int16ArrayHeader
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float32

BASE_POSITION = [30,10,10,10,10,10,10,10]

class controller_node(object):
	def __init__(self):
		# Initialize node
		rospy.init_node('controller_node')

		# Variables
		self.graspQuality = 0

		# Subscribe to grasp_quality
		self.graspSub = rospy.Subscriber('grasp_quality', Float32, self._gq_callback)

		# Publisher to controller_pos 
		self.pub = rospy.Publisher('controller_pos', Int16ArrayHeader, queue_size = 10)

		# Close service
		self.closeSrv = rospy.Service('close_hand', SetBool, self._close_hand)



	def _gq_callback(self, data):
		self.graspQuality = data


	def _close_hand(self, req):
		# Move to initial position before beginning SA search

		msg = Int16ArrayHeader()
		
		#msg.data = BASE_POSITION
		#self.pub.publish(msg)
	 	#rospy.sleep(5)

	 	# Begin simulated annealing process
	 	prevSol = BASE_POSITION
	 	prevCost = self.graspQuality.data
	 	T = 1
	 	alpha = 0.9
	 	minT = 0.005

	 	lastGood = False
	 	goodGrasps = 0

	 	while (T > minT):
	 		# Create deltas
	 		"""
	 		T1 = random.randint(-3,1)
	 		T2 = random.randint(-1,3)
	 		J1 = random.randint(-1,4)
	 		J2 = random.randint(-1,4)
	 		deltaPos = [T1, T2, J1 + random.randint(-1,1), J2 + random.randint(-1,2), J1 + random.randint(-1,1), 
	 					J2 + random.randint(-1,2), J1 + random.randint(-1,1), J2 + random.randint(-1,2)]
			"""

	 		deltaPos = [int(random.uniform(2-3*prevCost,5-4*prevCost)) for i in range(0,8)]
	 		# Flip sign of thumb
	 		deltaPos[0] = -deltaPos[0]
	 		
			# Create new solution
	 		newSol = [prevSol[i] + deltaPos[i] for i in range(0,8)]

	 		# Move to new position
	 		msg.data = newSol
	 		self.pub.publish(msg)
	 		rospy.sleep(0.3)

	 		# Get cost of new position
	 		newCost = self.graspQuality.data

	 		# Calculate acceptance probability
	 		ap = math.exp((newCost-prevCost)/T)

	 		# If accepted, move update solution, if not, move back to old position
	 		if ap > random.random():
	 			prevSol = newSol
	 			prevCost = newCost
	 		else:
	 			msg.data = prevSol
	 			self.pub.publish(msg)
	 			rospy.sleep(0.3)

	 		print self.graspQuality
	 		if self.graspQuality.data > 0.8:
	 			lastGood = True
	 			goodGrasps += 1

	 			if goodGrasps > 5:
	 				return SetBoolResponse(True, 'hand closed successfully')
	 		else:
	 			lastGood = False
	 			goodGrasps = 0

	 		# Reduce temperature
	 		T = T*alpha

	 	return SetBoolResponse(False, 'hand close failed')
		





if __name__ == '__main__':

	controller = controller_node()
	rospy.spin()
