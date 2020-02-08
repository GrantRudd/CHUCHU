#!/usr/bin/env python
import rospy, sys
import cv2
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from baxter_interface.camera import CameraController
from baxter_core_msgs.srv import ListCameras
from cv_bridge import CvBridge, CvBridgeError

topicPath = '/cameras/right_hand_camera/image'
displayPath = '/robot/xdisplay'

def publisher(objCenter):
	pub = rospy.Publisher('center', Point, queue_size = 1)
	pub.publish(objCenter)

#empty function to pass to trackbar callback
def nothing(blahhhh):
	pass


def set_cameras(res):
	ls = rospy.ServiceProxy('cameras/list', ListCameras)
	rospy.wait_for_service('cameras/list', timeout=10)

	cameraList = ls()

	
	if cameraList.cameras.count('right_hand_camera')== 0:
		c = CameraController(cameraList.cameras[0])
		c.close

	else:
		cRight = CameraController('right_hand_camera')
		cRight.open()
		cRight.resolution = res 
		cRight.gain = 20
		cRight.exposure = 30
		
		print "Right Camera Open: RES = " + str(cRight.resolution)

	return None

	


class img_processing (object):
	def __init__(self):
		self.baxterImage = rospy.Subscriber(topicPath, Image, self._convert_image)
		self.baxterDisplay = rospy.Publisher(displayPath, Image, latch=True, queue_size=1)

		self.bridge = CvBridge()

	def _convert_image(self,data):
		#Convert from image message to 
		img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		

		imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		
		#Make a numpy array with lower HSV color bounds
		lowerLimits = np.array([cv2.getTrackbarPos('H lower', 'Object'),
						  cv2.getTrackbarPos('S lower', 'Object'),
						  cv2.getTrackbarPos('V lower', 'Object')])

		#Make a numpy array with upper HSV color bounds
		upperLimits = np.array([cv2.getTrackbarPos('H upper', 'Object'),
						  cv2.getTrackbarPos('S upper', 'Object'),
						  cv2.getTrackbarPos('V upper', 'Object')])
		#Create mask image
		mask = cv2.inRange(imgHsv, lowerLimits, upperLimits)

		#apply mask to raw frame
		obj = cv2.bitwise_and(img, img, mask = mask)

		contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		
		if contours:
			cont = contours[0]
			for contour in contours:
				if cv2.contourArea(contour) > cv2.contourArea(cont):
					cont = contour
			cv2.drawContours(obj, cont, -1, (0,0,255),2)

			(x,y), r = cv2.minEnclosingCircle(cont)
			center = (int(x), int(y))
			radius = int(r)
			cv2.circle(obj, center, radius, (0,255,0), 2)
			pt = Point(
					x = center[0],
					y = center[1],
					z = 0,
					)
		
			publisher(pt)


		#Display filtered image to Baxter's Screen
		#objImg = cv2.resize(obj, (1024,600))
		#objImg = self.bridge.cv2_to_imgmsg(objImg, encoding="passthrough")
		#self.baxterDisplay.publish(objImg)

		#cv2.imshow('Raw Video', img)
		cv2.imshow('Object', obj)
		cv2.waitKey(1)

def main():
	if cv2.useOptimized == False:
		cv2.setUseOptimized(True)
		print 'CV optimized: ON'
	else:
		print 'CV optimized: ON'

	set_cameras((640, 400))
	#create named window to add sliders to
	cv2.namedWindow('Object')

	#create sliders for upper and lower bounds for HSV values (FOR RED BALL)
	cv2.createTrackbar('H upper', 'Object', 130, 179, nothing)
	cv2.createTrackbar('H lower', 'Object', 100, 179, nothing)
	cv2.createTrackbar('S upper', 'Object', 255, 255, nothing)
	cv2.createTrackbar('S lower', 'Object', 70, 255, nothing)
	cv2.createTrackbar('V upper', 'Object', 255, 255, nothing)
	cv2.createTrackbar('V lower', 'Object', 65, 255, nothing)

	rospy.init_node('CVTest', anonymous = True)
	

	conversion = img_processing()
	

	try:
		rospy.spin()	
	except KeyboardInterrupt:
		print 'Quitting...'
		baxterImage.unregister()
		sys.exit()
		cv2.destroyAllWindows



if __name__ == '__main__':
	main()
	

