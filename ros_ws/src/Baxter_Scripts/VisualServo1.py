import cv2 
import numpy as np
import rospy
import sys 
from baxter_pykdl import baxter_kinematics

import baxter_interface
from baxter_core_msgs.srv import *
from cv_bridge import CvBridge, CvBridgeError



from std_msgs.msg import (
	Float32MultiArray, 
	MultiArrayLayout, 
	MultiArrayDimension, 
	String,
	Header)
from sensor_msgs.msg import Image, Range
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

startPosition = Point(x = 0.7, y = -0.3, z = 0.3)
startOrientation = Quaternion(x = 0.0 , y = 1.0, z = 0.0, w = 0.0)

startPos = [0.7,-0.3,0.3]
startOri = [0.0, 1.0, 0.0, 0.0]
jointNames = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

#simple PD controller class
class PD(object):
	def __init__(self, Kp, Kd, dT, sf = 1):
		self.lastError = 0
		self.Kp = Kp
		self.Kd = Kd
		self.dT = dT
		self.sf = sf

	def update_value(self, setPoint, current):
		error = setPoint - current 
		newVal = (self.Kp*error + self.Kd*(error - self.lastError)/self.dT)*self.sf

		self.lastError = error

		return newVal, error

#class for baxter range sensors (from irrange.py)
class BaxterRangeSensor():
	def __init__(self):
		root_name = '/robot/range/'
		self.range = {'left':0, 'right':0}
		self._left_sensor = rospy.Subscriber(root_name + 'left_hand_range/state',
								Range, self._get_range, 'left')
		self._right_sensor = rospy.Subscriber(root_name + 'right_hand_range/state',
								Range, self._get_range, 'right')

	def _get_range(self, msg, side):
		if msg.range < 0.4:
			self.range[side] = msg.range
		else:
			self.range[side] = 0.4

	def cleanup(self):
		self._rs.disable()

class objectCenter():
	def __init__(self):
		self.centerFromCv = rospy.Subscriber('/center', Point, self._get_center)	
		self.center = (0,0)
	def _get_center(self, point):
		self.center = (point.x, point.y)
	

def obj_center(msg):
	msg = str(msg).split()
	x,y = msg[1].split(',')
	x = int(x)
	y = int(y)
	
	return x



def pose_subscriber():
	rospy.Subscriber('pose_topic_right', PoseStamped, IK, 'right')
	
	return None



def IK(pose ,limb):
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
	return angle_dict

def create_pose_stamp(time, pos, ori):
	pose = PoseStamped(
			header = Header(stamp = time, frame_id = 'base'),
			pose = Pose(
				position = pos,
				orientation = ori
			)
		)
	return pose



def main():
	#initialize rospy node
	rospy.init_node('Viz1')
	#enable baxter robot
	rospy.sleep(5)
	en = baxter_interface.RobotEnable()
	en.enable()
	print 'robot enabled'
	#initialize right arm and right arm range sensor
	rightArm = baxter_interface.Limb('right')
	rightArm.set_joint_position_speed(0.5)
	rightArm.set_command_timeout(0.1)
	rightGripper = baxter_interface.Gripper('right')
	
	rs = BaxterRangeSensor()
	center = objectCenter()

	#instantiate KDL 
	kin = baxter_kinematics('right')
	print '' #unknown error pops up upon instantiation

	#move robot to start position
	pose = create_pose_stamp(rospy.Time.now(), startPosition, startOrientation)
	startJointVals = IK(pose, 'right')
	rightArm.move_to_joint_positions(startJointVals, timeout = 2.0)
	rightGripper.calibrate()

	
	#Create z-height PD control
	dT = 0.003 #PD update period
	zPD = PD(0.25, 0.1, dT,0.75)
	#create x PD control
	xPD = PD(0.09, 0.025, dT, 0.005)
	#create y PD control
	yPD = PD(0.09, 0.025, dT, 0.005)

	print "Press any key to start."
	raw_input()
	
	
	loopTime = rospy.get_time()


	while True:
		if abs(loopTime - rospy.get_time()) > dT: 
			#get rangefinder value
			rng = rs.range['right']
			
			(x,y) = center.center
			#update velocities
			xvel, xError = xPD.update_value(200,y)
			yvel, yError = yPD.update_value(320,x)
			zvel, zError = zPD.update_value(0.1, rng)

			vels = np.transpose(np.array([[xvel, yvel, zvel, 0, 0, 0]], np.float64))
			print vels
				
			jac = kin.jacobian_pseudo_inverse()
			jointVels = np.transpose(np.dot(jac, vels)).tolist()
			jointVels = dict(zip(jointNames,jointVels[0]))

			rightArm.set_joint_velocities(jointVels)
			
			loopTime = rospy.get_time()
			
			if abs(zError) <= 0.01:
				rightArm.exit_control_mode()
				#Move arm back so that ball is centered
				endPose = rightArm.endpoint_pose()
				endPose = create_pose_stamp(rospy.Time.now(),
											Point(endPose['position'].x - 0.035, 
							   					  endPose['position'].y, 
							   					  endPose['position'].z), 
											endPose['orientation'])
				endVals = IK(endPose, 'right')
				rightArm.move_to_joint_positions(endVals, timeout = 2.0)
				#Close gripper and move to start position, then open gripper
				print 'position reached'
				rightGripper.close()
				rospy.sleep(2)
				rightArm.move_to_joint_positions(startJointVals, timeout = 2.0)
				rightGripper.open()
				rospy.sleep(5)

				break
		
	print 'Shutting Down'
	en.disable()
	sys.exit()


if __name__ == '__main__':
	main()
	
