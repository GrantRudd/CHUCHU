import cv2 
import numpy as np
import rospy
import sys 
from baxter_pykdl import baxter_kinematics

import PID

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
    Point,
    PoseStamped,
    Pose
)

#right arm workspace observation pose
neutralPose = {'right_s0': 0.46799250845056845, 
			   'right_s1': -1.1774500450946894, 
			   'right_w0': -0.14893887267791586, 
			   'right_w1': 1.556499596997215, 
			   'right_w2': 0.050884535906072774, 
			   'right_e0': 0.38726367826766334, 
			   'right_e1': 1.2203835637095095}
#list containing all joint names
jointNames = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']


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
	

	#print angle_dict
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

#Baxter IR range sensor class
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


class Baxter_PP(object):
	
	def __init__(self):
		self.goalPose = {}

		#Initialize ROS node
		rospy.init_node('Pick_N_Place', anonymous = True)
		#Enable Robot
		self.en = baxter_interface.RobotEnable()
		self.en.enable()
		print 'Robot Enabled...'

		#Initialize right arm and right gripper
		self.rightArm = baxter_interface.Limb('right')
		self.rightArm.set_joint_position_speed(0.5)
		self.rightArm.set_command_timeout(0.1)
		self.rightGripper = baxter_interface.Gripper('right')

		#instantiate range sensor
		self.rs = BaxterRangeSensor()

		#initialize right arm wheel button
		self.rightNavigator = baxter_interface.Navigator('right')
		#Blink LEDs
		self.rightNavigator.inner_led = False
		self.rightNavigator.outer_led = False
		for i in range(0,6):
			self.rightNavigator.inner_led = not self.rightNavigator.inner_led
			self.rightNavigator.outer_led = not self.rightNavigator.outer_led
			rospy.sleep(0.5)

		#initialize baxter KDL
		self.KDL = baxter_kinematics('right')
		print ''

		#move arm to neutral position and calibrate gripper
		self.rightArm.move_to_joint_positions(neutralPose)
		self.rightGripper.calibrate()

		#Create z-height PID control
		self.zReg = PID.PID(Kp = 0.2, Kd =  0.1, Ki =  0, scaleFactor =  2)
		#create x PID control
		self.xReg = PID.PID(Kp = 0.1, Kd =  0.01, Ki =  0, scaleFactor =  0.005)
		#create y PID control
		self.yReg = PID.PID(Kp = 0.1, Kd =  0.01, Ki =  0, scaleFactor =  0.005)

		#subscribe to 'center' message (sent from CV module) and get center of object
		self.centerFromCv = rospy.Subscriber('/center', Point, self._get_center)	
		self.centerX = 0
		self.centerY = 0

	def _get_center(self,point):
		self.centerX = point.y
		self.centerY = point.x

	#move right arm back to home position
	def move_to_home(self):
		self.rightArm.move_to_joint_positions(neutralPose)

	def move_to_goal(self):
		self.rightArm.move_to_joint_positions(self.goalPose)

	#let user define the goal position
	def define_goal(self):
		print 'Move right arm to goal location and press navigator wheel button...'
		
		while self.goalPose == {}:
			#wait for navigator button press
			if self.rightNavigator.button0:
				self.goalPose = self.rightArm.joint_angles()

				print 'Goal Point Set... Moving to home pose'
				#Turn on LEDs
				self.rightNavigator.inner_led = True
				self.rightNavigator.outer_led = True

				rospy.sleep(3)
				#move to home position
				self.move_to_home()

	def pick_and_place(self):
		loopTime = rospy.get_time() #initialize loop time
		dT = 0.1 #PID update interval

		

		while True:
			#if the object is detected
			if (self.centerX != -1) and (self.centerY != -1):
				if rospy.get_time() - loopTime > dT:
					#Read sensor for current Z value
					current_z = self.rs.range['right']
					#update commanded velocities
					zVel, zError = self.zReg.update_value(0.1, current_z)
					yVel, yError = self.yReg.update_value(320, self.centerY)
					xVel, xError = self.xReg.update_value(200, self.centerX)
					#Create vector of velocities
					vels = np.transpose(np.array([[xVel, yVel, zVel, 0, 0, 0]]))
			
					#print xError, yError, zError
					#get inverse jacobian matrix from PyKDL
					jacobianInverse = self.KDL.jacobian_pseudo_inverse()
					#make a list of joint angular velocites 
					#Calculated using jacobian J_inv * X_dot = Q_dot
					jointVels = np.transpose(np.dot(jacobianInverse, vels)).tolist()
					jointVels = dict(zip(jointNames,jointVels[0]))
					#update joint velocitied on the robot
					self.rightArm.set_joint_velocities(jointVels)

					
					#Once the object is within range...
					if abs(zError) < 0.01:
						self.rightArm.exit_control_mode()
						#Move arm back so that ball is centered
						endPose = self.rightArm.endpoint_pose()
						endPose = create_pose_stamp(rospy.Time.now(),
													Point(endPose['position'].x - 0.035, 
									   					  endPose['position'].y, 
									   					  endPose['position'].z), 
														  endPose['orientation'])
						endVals = IK(endPose, 'right')
						self.rightArm.move_to_joint_positions(endVals, timeout = 2.0)
						#Close gripper
						print 'Object Reached. Grabbing...'
						self.rightGripper.close()
						rospy.sleep(1)
						#Move arm to home position
						self.move_to_home()
						rospy.sleep(2)
						#Move to goal position and drop object
						self.move_to_goal()
						rospy.sleep(1)
						self.rightGripper.open()
						rospy.sleep(2)
						#move back to home position and wait for object to settle
						self.move_to_home()
						rospy.sleep(5)
					#update loop time	
					loopTime = rospy.get_time()	
			#otherwise, move to home position and wait
			else:
				self.move_to_home()
				rospy.sleep(1)

def main():
	#instantiate and initialize Baxter
	baxter = Baxter_PP()
	#define goal point
	baxter.define_goal()
	#run pick and place routine until CTRL-C
	baxter.pick_and_place()

	
if __name__ == '__main__':
	main()