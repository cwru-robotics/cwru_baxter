#!/usr/bin/env python
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
if __name__ == '__main__':
	rospy.init_node('my_image_display', anonymous=True)
	pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
	images = ["confusion0001.png","confusion0002.png","confusion0003.png","confusion0004.png","confusion0005.png",
"confusion0006.png","confusion0007.png","confusion0008.png","confusion0009.png","confusion0010.png",
"confusion0011.png","confusion0012.png","confusion0013.png","confusion0014.png","confusion0015.png",
"confusion0016.png","confusion0017.png","confusion0018.png","confusion0019.png","confusion0020.png",
"confusion0021.png","confusion0022.png","confusion0023.png","confusion0024.png","confusion0025.png",
"confusion0026.png","confusion0027.png","confusion0028.png","confusion0029.png","confusion0030.png",
"confusion0031.png","confusion0032.png","confusion0033.png","confusion0034.png","confusion0035.png",
"confusion0036.png","confusion0037.png","confusion0038.png","confusion0039.png","confusion0040.png",
"confusion0041.png","confusion0042.png","confusion0043.png","confusion0044.png","confusion0045.png",
"confusion0046.png","confusion0047.png","confusion0048.png","confusion0049.png","confusion0050.png",
"confusion0051.png","confusion0052.png","confusion0053.png","confusion0054.png","confusion0055.png",
"confusion0056.png","confusion0057.png","confusion0058.png","confusion0059.png","confusion0060.png",
"approval0001.png","approval0002.png","approval0003.png","approval0004.png","approval0005.png",
"approval0006.png","approval0007.png","approval0008.png","approval0009.png","approval0010.png",
"approval0011.png","approval0012.png","approval0013.png","approval0014.png","approval0015.png",
"approval0016.png","approval0017.png","approval0018.png","approval0019.png","approval0020.png",
"approval0021.png","approval0022.png","approval0023.png","approval0024.png","approval0025.png",
"approval0026.png","approval0027.png","approval0028.png","approval0029.png","approval0030.png",
"approval0031.png","approval0032.png","approval0033.png","approval0034.png","approval0035.png",
"approval0036.png","approval0037.png","approval0038.png","approval0039.png","approval0040.png",
"approval0041.png","approval0042.png","approval0043.png","approval0044.png","approval0045.png",
"approval0046.png","approval0047.png","approval0048.png","approval0049.png","approval0050.png",
"approval0051.png","approval0052.png","approval0053.png","approval0054.png","approval0055.png",
"disapproval0001.png","disapproval0002.png","disapproval0003.png","disapproval0004.png","disapproval0005.png",
"disapproval0006.png","disapproval0007.png","disapproval0008.png","disapproval0009.png","disapproval0010.png",
"disapproval0011.png","disapproval0012.png","disapproval0013.png","disapproval0014.png","disapproval0015.png",
"disapproval0016.png","disapproval0017.png","disapproval0018.png","disapproval0019.png","disapproval0020.png",
"disapproval0021.png","disapproval0022.png","disapproval0023.png","disapproval0024.png","disapproval0025.png",
"disapproval0026.png","disapproval0027.png","disapproval0028.png","disapproval0029.png","disapproval0030.png",
"disapproval0031.png","disapproval0032.png","disapproval0033.png","disapproval0034.png","disapproval0035.png",
"disapproval0036.png","disapproval0037.png","disapproval0038.png","disapproval0039.png","disapproval0040.png",
"disapproval0041.png","disapproval0042.png","disapproval0043.png","disapproval0044.png","disapproval0045.png",
"disapproval0046.png","disapproval0047.png","disapproval0048.png","disapproval0049.png","disapproval0050.png",
"disapproval0051.png","disapproval0052.png","disapproval0053.png","disapproval0054.png","disapproval0055.png",
"disapproval0056.png","disapproval0057.png","disapproval0058.png","disapproval0059.png","disapproval0060.png",
"disapproval0061.png","disapproval0062.png","disapproval0063.png","disapproval0064.png","disapproval0065.png",
"happiness0001.png","happiness0002.png","happiness0003.png","happiness0004.png","happiness0005.png",
"happiness0006.png","happiness0007.png","happiness0008.png","happiness0009.png","happiness0010.png",
"happiness0011.png","happiness0012.png","happiness0013.png","happiness0014.png","happiness0015.png",
"happiness0016.png","happiness0017.png","happiness0018.png","happiness0019.png","happiness0020.png",
"happiness0021.png","happiness0022.png","happiness0023.png","happiness0024.png","happiness0025.png",
"happiness0026.png","happiness0027.png","happiness0028.png","happiness0029.png","happiness0030.png",
"happiness0031.png","happiness0032.png","happiness0033.png","happiness0034.png","happiness0035.png",
"happiness0036.png","happiness0037.png","happiness0038.png","happiness0039.png","happiness0040.png",
"happiness0041.png","happiness0042.png","happiness0043.png","happiness0044.png","happiness0045.png",
"happiness0046.png","happiness0047.png","happiness0048.png","happiness0049.png","happiness0050.png",
"happiness0051.png","happiness0052.png","happiness0053.png","happiness0054.png","happiness0055.png",
"idle0001.png","idle0002.png","idle0003.png","idle0004.png","idle0005.png",
"idle0006.png","idle0007.png","idle0008.png","idle0009.png","idle0010.png",
"idle0011.png","idle0012.png","idle0013.png","idle0014.png","idle0015.png",
"idle0016.png","idle0017.png","idle0018.png","idle0019.png","idle0020.png",
"idle0021.png","idle0022.png","idle0023.png","idle0024.png","idle0025.png",
"idle0026.png","idle0027.png","idle0028.png","idle0029.png","idle0030.png",
"idle0031.png","idle0032.png","idle0033.png","idle0034.png","idle0035.png",
"idle0036.png","idle0037.png","idle0038.png","idle0039.png","idle0040.png",
"idle0041.png","idle0042.png","idle0043.png","idle0044.png","idle0045.png",
"idle0046.png","idle0047.png","idle0048.png","idle0049.png","idle0050.png",
"idle0051.png","idle0052.png","idle0053.png","idle0054.png","idle0055.png",
"idle0056.png","idle0057.png","idle0058.png","idle0059.png","idle0060.png",
"idle0061.png","idle0062.png","idle0063.png","idle0064.png","idle0065.png",
"idle0066.png","idle0067.png","idle0068.png","idle0069.png","idle0070.png","idle0071.png","idle0072.png",
"thinking0001.png","thinking0002.png","thinking0003.png","thinking0004.png","thinking0005.png",
"thinking0006.png","thinking0007.png","thinking0008.png","thinking0009.png","thinking0010.png",
"thinking0011.png","thinking0012.png","thinking0013.png","thinking0014.png","thinking0015.png",
"thinking0016.png","thinking0017.png","thinking0018.png","thinking0019.png","thinking0020.png",
"thinking0021.png","thinking0022.png","thinking0023.png","thinking0024.png","thinking0025.png",
"thinking0026.png","thinking0027.png","thinking0028.png","thinking0029.png","thinking0030.png",
"thinking0031.png","thinking0032.png","thinking0033.png","thinking0034.png","thinking0035.png",
"thinking0036.png","thinking0037.png","thinking0038.png","thinking0039.png","thinking0040.png",
"thinking0041.png","thinking0042.png","thinking0043.png","thinking0044.png","thinking0045.png",
"thinking0046.png","thinking0047.png","thinking0048.png","thinking0049.png","thinking0050.png",
"thinking0051.png","thinking0052.png","thinking0053.png","thinking0054.png","thinking0055.png",
"thinking0056.png","thinking0057.png","thinking0058.png","thinking0059.png","thinking0060.png",
"thinking0061.png","thinking0062.png","thinking0063.png","thinking0064.png","thinking0065.png"] #e.g: put your list of images here 
	count=0
	#while (count<2):
	while not rospy.is_shutdown():
		for path in images:
			img = cv2.imread(path)
			msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
			pub.publish(msg)
			rospy.sleep(0)
			#count=count+1
