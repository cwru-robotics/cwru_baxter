#!/usr/bin/env python
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
if __name__ == '__main__':
	rospy.init_node('my_image_display', anonymous=True)
	pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
	images = ["1.jpg","2.jpg","3.jpg","4.jpg","5.jpg","6.jpg","7.jpg","8.jpg","9.jpg","10.jpg",
	"11.jpg","12.jpg","13.jpg","14.jpg","15.jpg","16.jpg","17.jpg","18.jpg"] #e.g: put your list of images here 
	while not rospy.is_shutdown():
		for path in images:
			img = cv2.imread(path)
			msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
			pub.publish(msg)
			rospy.sleep(0.07)
