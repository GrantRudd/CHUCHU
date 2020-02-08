import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import signal, sys
import baxter_interface


class BaxterLeap (object):
	def __init__(self):
		self.en = baxter_interface.RobotEnable()
		self.en.enable()
		print 'Robot Enabled...'
		rightArm = baxter_interface.Limb('right')

		self.sub = rospy.Subscriber("LeapPoseIKReq", PoseStamped, self.Move_Arm)
		rospy.spin()

	def clean_exit(self, sig, frame):
		print '\nExiting...'
		sys.exit()

	def IK_solver(self, pose):
		ns = "ExternalTools/" + 'right' + "/PositionKinematicsNode/IKService"
		iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		ikreq = SolvePositionIKRequest()
		ikreq.pose_stamp.append(pose)

		try:
			rospy.wait_for_service(ns, 5.0)
			resp = iksvc(ikreq)
			
			if resp.isValid == True:
				return zip(resp.name, resp.position)

		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			return 1

	def Move_Arm(self, pose):
		self.IK_solver(pose)


def clean_exit(sig, frame):
	print '\nExiting...'
	sys.exit()

def main():
	signal.signal(signal.SIGINT, clean_exit)
	#initialize ROS node
	rospy.init_node("Baxter", anonymous = True)

	bax = BaxterLeap()






if __name__ == "__main__":
	main()
