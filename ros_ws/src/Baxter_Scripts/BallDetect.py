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

def nothing(blahhhh):
	pass

def publisher(objCenter):
	pub = rospy.Publisher('center', Point, queue_size = 1)
	pub.publish(objCenter)


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
		cRight.exposure = 25
		
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

		circle = np.zeros((400,640), np.uint8)
		cv2.circle(circle, (320,200), 50, (255,0,0), -1)
		circleCont, heirarchy = cv2.findContours(circle, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		
		
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray, (7,7), 0)

		edges = cv2.Canny(gray, cv2.getTrackbarPos('param1', 'Object'), cv2.getTrackbarPos('param2', 'Object'))

		kernel = np.ones((4,4), np.uint8)
		edges = cv2.dilate(edges, kernel, iterations = 1)
		edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, kernel)
		
		contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		#cv2.drawContours(img, contours, -1, (0,0,255),3)
	
		biggest = [(0,0),0]
		for contour in contours:
			if cv2.contourArea(contour) > 500:
				if cv2.matchShapes(contour, circleCont[0], 1, 0.0) < (float(cv2.getTrackbarPos('error', 'Object'))/1000):
					(x,y),r = cv2.minEnclosingCircle(contour)
					center = (int(x), int(y))
					r = int(r)
					cv2.circle(img, center, r, (0,0,255), 3)
					if r > biggest[1]:
						biggest = [center, r]
					
		pt = Point(
					x = biggest[0][0],
					y = biggest[0][1],
					z = 0,
					)
		
		publisher(pt)	
			
		cv2.imshow('Object', img)
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
	cv2.createTrackbar('error', 'Object', 15, 1000, nothing)
	cv2.createTrackbar('param1', 'Object', 25, 1000, nothing)
	cv2.createTrackbar('param2', 'Object', 60, 1000, nothing)
	cv2.createTrackbar('minRad', 'Object', 110, 255, nothing)
	#initialize node to send contour data
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