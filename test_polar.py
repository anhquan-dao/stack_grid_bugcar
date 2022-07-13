import cv2
from cv2 import WARP_INVERSE_MAP
import time

import numpy as np
import rospy
import numpy_indexed as npi
from nav_msgs.msg import OccupancyGrid
a = cv2.imread("/home/thang/Downloads/test_polar1.png",cv2.IMREAD_GRAYSCALE)
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import MapMetaData
# a = cv2.circle(a, (a.shape[0]/2 -1, a.shape[1]/2-1), 3, 100, -1)

print(a.shape)

start_timer = time.time()
print(time.time() - start_timer)



def cb(data):
	cropped_polar = data.data
	info = data.info
	height = info.height
	width = info.width
	# print(len(a),info.width,info.height)
	map = np.asarray(cropped_polar).reshape((height,width)).astype(np.uint8)
	map = cv2.flip(map,0)
	cv2.imshow("Original Map", map)
	shape = (map.shape[1],map.shape[0])

	polar_transformed = cv2.warpPolar(map, shape, (0,map.shape[0]/2-1), 300,cv2.WARP_POLAR_LINEAR)
	# find all occupied point
	empty_array = np.zeros(polar_transformed.shape)
	vc = np.where(polar_transformed==100)
	xxx = np.transpose(np.stack((vc[0],vc[1])))
	min_index_xy = npi.group_by(xxx[:,0]).min(xxx[:,1])
	for i,min_index_x in enumerate(min_index_xy[1]):
		empty_array = cv2.circle(empty_array,(min_index_x,min_index_xy[0][i]),2,100,-1)

	
	re_original_with_obstacle = cv2.warpPolar(empty_array, shape, (0,polar_transformed.shape[0]/2-1), 300,cv2.WARP_INVERSE_MAP)
	re_original_with_obstacle = re_original_with_obstacle.astype(np.int8)
	re_original_with_obstacle[map==255] = -1
	re_original_with_obstacle = cv2.flip(re_original_with_obstacle,0)


	# cropped_polar = np.zeros((100,300)).astype(np.uint8)
	# # from 0 deg to 90 deg here clockwise
	# cropped_polar[50:,:] = polar_transformed[:50,:]
	# # from 270 deg to 360 deg here clockwise
	# cropped_polar[:50,:] = polar_transformed[150:,:]
	# cropped_polar=cv2.resize(cropped_polar,(cropped_polar.shape[1],cropped_polar.shape[0]*2))
	data.data = re_original_with_obstacle.flatten().tolist()

	data.header.frame_id = "base_link"

	b.publish(data)
	# cv2.imshow("x", cropped_polar)
	cv2.imshow("polar_transformed", polar_transformed)
	cv2.imshow("Re-Original with Sobel", re_original_with_obstacle)
	cv2.waitKey(1)

rospy.init_node("haha",disable_signals=True)
a = rospy.Subscriber("/front",OccupancyGrid,cb)
b = rospy.Publisher("/polar_processed_front",OccupancyGrid,queue_size=2)
rospy.spin()