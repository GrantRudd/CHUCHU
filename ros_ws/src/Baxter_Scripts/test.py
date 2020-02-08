import baxter_interface
import numpy as np
import rospy

pos1 = [-0.3712233506682701, 0.8866408953977691, 1.1911360817930239, -0.5407282277296084, 1.3364807614451668, 1.6540147845374473, -2.0298400775693737]
pos2 = [-0.3792767498046685, 0.7190534943212877, 1.1086846144441829, -0.5073641455931006, 1.8265876231745564, 1.8526652965686083, -1.817383738447244]

jointNames = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']

pos1 = dict(zip(jointNames, pos1))
pos2 = dict(zip(jointNames, pos2))

rospy.init_node('BaxterFuck')

#initialize right arm and right arm range sensor
rightArm = baxter_interface.Limb('right')
rightArm.set_joint_position_speed(0.5)
rightArm.set_command_timeout(0.1)


rightArm.move_to_joint_positions(pos1, timeout = 2.0)

rospy.sleep(2)

rightArm.move_to_joint_positions(pos2, timeout = 2.0)

rospy.sleep(2)

rightArm.move_to_joint_positions(pos1, timeout = 2.0)