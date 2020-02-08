import rospy
import RoboticsLeapModule as RML
import signal, sys


from geometry_msgs.msg import(
	Pose,
	PoseStamped,
	Point,
	Quaternion
	)

from std_msgs.msg import Header



#clean exit when ctrl-c
def clean_exit(sig, frame):
	print '\nExiting...'
	sys.exit()


def main():
	signal.signal(signal.SIGINT, clean_exit)
	#initialize ROS node
	rospy.init_node("LeapHandler", anonymous = True)
	#initialize ROS publisher
	pub = rospy.Publisher("LeapPoseIKReq", PoseStamped, queue_size = 10)

	leap = RML.LeapController(scale_factor = 0.01)


	while True:

		ret = leap.right()

		if ret != None:
			hdr = Header(stamp=rospy.Time.now(), frame_id='base')

			P, Q, G = ret
			
			pose = PoseStamped(
				header = hdr,
				pose = Pose(
					position = Point(
						x = P[0],
						y = P[1],
						z = P[2]),
					orientation = Quaternion(
						x = Q[0],
						y = Q[1],
						z = Q[2],
						w = Q[3])
					)
				)
			
			rospy.loginfo(pose)
			pub.publish(pose)
		



		rospy.sleep(0.1)



if __name__ == '__main__':
	main()