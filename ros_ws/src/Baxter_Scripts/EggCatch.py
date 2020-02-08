import numpy as np
import rospy
import sys 
from baxter_pykdl import baxter_kinematics
import baxter_interface


#left arm workspace observation pose
neutralPose = {'left_w0': -0.8118593319883552,
			   'left_w1': 1.5397332158399841, 
			   'left_w2': 1.7606264492954837, 
			   'left_e0': -1.080689465065274, 
			   'left_e1': 1.3667768820059036, 
			   'left_s0': 0.17564080021287987, 
			   'left_s1': -0.7232719414879726}
#list containing all joint names
jointNames = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']

trajectory = []

rospy.init_node('egg', anonymous = True)

en = baxter_interface.RobotEnable()
en.enable()
print 'Baxter Enabled'

KDL = baxter_kinematics('left')
print''


leftArm = baxter_interface.Limb('left')
leftArm.set_command_timeout(0.1)
leftArm.set_joint_position_speed(1.0)
#leftArm.move_to_joint_positions(neutralPose)

f = open('trajectory.txt', 'r')

while True:
	s = f.readline()
	if s != '':
		trajectory.append(float(s))
	else:
		break

f.close()

print len(trajectory)


loopTime = rospy.get_time()
dT = 0.01
i = 0

while True:
	if (rospy.get_time() - loopTime) > dT:

		print trajectory[i]
		vels = np.transpose(np.array([[0, 0, trajectory[i], 0, 0, 0]]))

		jacobianInverse = KDL.jacobian_pseudo_inverse()
		#make a list of joint angular velocites 
		#Calculated using jacobian J_inv * X_dot = Q_dot
		jointVels = np.transpose(np.dot(jacobianInverse, vels)).tolist()
		jointVels = dict(zip(jointNames,jointVels[0]))
		#update joint velocitied on the robot
		leftArm.set_joint_velocities(jointVels)

		

		if (i-1) == len(trajectory):
			break
		else:
			i += 1

		loopTime = rospy.get_time()