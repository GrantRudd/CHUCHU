import rospy
import sys
import argparse
from baxter_interface import (
	Limb,
	Gripper,
	DigitalIO,
	Navigator,
	RobotEnable
	)


class Baxter_Interactive(object):

	def __init__(self, arm):
		#Vector to record poses
		self.recorded = []
		self.arm = arm
		#enable Baxter
		self._en = RobotEnable() 
		self._en.enable()
		#use DigitalIO to connect to gripper buttons
		self._close_button = DigitalIO(self.arm + '_upper_button')
		self._open_button = DigitalIO(self.arm + '_lower_button')
		#set up gripper
		self._gripper = Gripper(self.arm)
		self._gripper.calibrate()
		#set up navigator
		self._navigator = Navigator(self.arm)
		#set up limb
		self._limb = Limb(self.arm)
		self._limb.set_joint_position_speed(0.5)
		self._limb.move_to_neutral()

		print 'Ready to record...'


	def recorder(self):
		doneRecording = False

		while not doneRecording:
			if self._navigator.button0:
				self.recorded.append(self._limb.joint_angles())
				print 'Waypoint Added'
				rospy.sleep(1)

			if self._close_button.state:
				self.recorded.append('CLOSE')
				self._gripper.close()
				print 'Gripper Closed'
				rospy.sleep(1)

			if self._open_button.state:
				self.recorded.append('OPEN')
				self._gripper.open()
				print 'Gripper Opened'
				rospy.sleep(1)

			if self._navigator.button1:
				print 'END RECORDING'
				doneRecording = True
				rospy.sleep(3)

		while doneRecording:
			for waypoint in self.recorded:
				if waypoint == 'OPEN':
					self._gripper.open()
					rospy.sleep(1)
				elif waypoint == 'CLOSE':
					self._gripper.close()
					rospy.sleep(1)
				else:
					self._limb.move_to_joint_positions(waypoint)

			#print 'Would you like to replay (y/n)?'
			#resp = raw_input()

			#if resp == 'n':
				#print 'Exiting...'
				#return False
				
def main():
	parser = argparse.ArgumentParser()
	parser.add_argument('-l', '--limb', help = 'Specify limb [right, left]')
	args = parser.parse_args()

	rospy.init_node('Interactive_1')

	b = Baxter_Interactive(args.limb)
	
	while b.recorder():
		pass

	

if __name__ == '__main__':
	main()
	sys.exit()