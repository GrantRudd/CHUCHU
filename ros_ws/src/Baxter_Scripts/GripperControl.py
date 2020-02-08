import baxter_interface
import sys
import rospy


rospy.init_node('grippy')
#enable baxter robot
en = baxter_interface.RobotEnable()
en.enable()
print 'robot enabled'
#initialize right arm and right arm range sensor

rightGripper = baxter_interface.Gripper('right')
rightGripper.calibrate()

while True:
	k = raw_input()

	if k == 'c':
		rightGripper.close()
	elif k == 'o':
		rightGripper.open()
	elif k == 'q':
		en.disable()
		sys.exit()
