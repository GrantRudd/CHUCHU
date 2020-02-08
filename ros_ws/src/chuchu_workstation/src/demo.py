#!/usr/bin/env python

import sys
import rospy
import CHUCHUInterface
import baxter_interface
import numpy as np



# Baxter positions
pos1 = [-0.4966262800779027, 0.45405831321408247, 1.0722525707319042, -0.04678641403050512, 1.4841264122791378, 1.6820099339163561, -2.3082575905705762]
pos2 = [-0.3792767498046685, 0.7190534943212877, 1.1086846144441829, -0.5073641455931006, 1.8265876231745564, 1.8526652965686083, -1.817383738447244]

# Baxter joint name dict
jointNames = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']

# Zip positions into appropriate dictionaries
pos1 = dict(zip(jointNames, pos1))
pos2 = dict(zip(jointNames, pos2))

# Initialize node
rospy.init_node('CHUCHU_demo', anonymous = True)

# Initialize right arm and right arm range sensor
rightArm = baxter_interface.Limb('right')
rightArm.set_joint_position_speed(0.5)
rightArm.set_command_timeout(0.1)

# Initialize Baxter Head
head = baxter_interface.Head()
head.set_pan(0)

# Initialize CHUCHU
chuchu = CHUCHUInterface.CHUCHUInterface()
ret = chuchu.load_nn_parameters('redball')

if ret == False:
	print "Failed to load NN parameters ..."
	sys.exit()

print "Baxter and CHUCHU Initialized ..."

# *************************************
# ******** BEGIN DEMO ROUTINE *********
# *************************************

# Move Baxter to start position and wait
rightArm.move_to_joint_positions(pos2, timeout = 2.0)
rospy.sleep(2)

# Make sure CHUCHU is open
ret = chuchu.open()

# Check if success
if ret == False:
	print "Failed to open CHUCHU ..."
	sys.exit()

# Move Baxter to pick up position and wait
rightArm.move_to_joint_positions(pos1, timeout = 2.0)
rospy.sleep(2)

# Close CHUCHU
ret = chuchu.close()

# Check if success
if ret == False:
	print "Failed to close CHUCHU ..."
	# Shake head "NO" if failed
	head.set_pan(np.pi/6.0)
	head.set_pan(-np.pi/6.0)
	head.set_pan(np.pi/6.0)
	head.set_pan(-np.pi/6.0)
	rospy.sleep(1)
	head.set_pan(0)
	rospy.sleep(1)

else:
	# Nod head "YES" if gripped
	head.command_nod()
	rospy.sleep(1)
	head.command_nod()
	rospy.sleep(1)

# Move Baxter to start position
rightArm.move_to_joint_positions(pos2, timeout = 2.0)
rospy.sleep(2)

# Open CHUCHU
ret = chuchu.open()

# Check if success
if ret == False:
	print "Failed to open CHUCHU ..."
	sys.exit()

# ***********************************
# ******** END DEMO ROUTINE *********
# ***********************************
print "Finished successfully ..."
sys.exit()