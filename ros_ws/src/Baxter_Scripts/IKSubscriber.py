#!/usr/bin/env python
import rospy, sys
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header, String
from baxter_core_msgs.srv import *
import baxter_interface




#takes in a limb string, position and orientation vectors
#and returns a dictionary of joint angles of the format {'joint_name': joint_value}

def pose_subscriber():
	
	rospy.Subscriber('pose_topic_right', PoseStamped, IK, 'right')


	rospy.spin()


def IK(pose,limb):	
	pose = create_pose_stamp(pose)

	srv_path = '/ExternalTools/' + limb + '/PositionKinematicsNode/IKService'
	srv = rospy.ServiceProxy(srv_path, SolvePositionIK)
	
	IKRequest = SolvePositionIKRequest()
	IKRequest.pose_stamp.append(pose)
	IKRequest.seed_mode = 2
	result = srv(IKRequest)

	angle_dict = {}
	
	for i in range(0,len(result.joints[0].position)):
		angle_dict[result.joints[0].name[i]] = result.joints[0].position[i]

	print angle_dict
	right_hand.move_to_joint_positions(angle_dict)
		
	return angle_dict


def create_pose_stamp(time, position, orientation):
	pose = PoseStamped(
			header = Header(stamp = time, frame_id = 'base'),
			pose = Pose(
				position = Point(
					x = position['x'],
					y = position['y'],
					z = position['z'],
					),
				orientation = Quaternion(
                    x = orientation['x'],
					y = orientation['y'],
					z = orientation['z'],
                    w = orientation['w'],
                ),
			)
		)
	return pose




if __name__ == '__main__':	
	try:
		rospy.init_node('IK_Subscriber', anonymous = True)
		en = baxter_interface.RobotEnable()
		en.enable()
		right_hand = baxter_interface.Limb('right')
		right_hand.set_joint_position_speed(0.9)
		right_hand.move_to_neutral()

		print 'ENABLED AND NEUTRAL'
		rospy.sleep(3)
		
		pose_subscriber()
	except KeyboardInterrupt:

		en.disable()
		rospy.sleep(3)
		sys.exit()
	