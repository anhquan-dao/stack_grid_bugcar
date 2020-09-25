#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import tf as ros_tf

from std_msgs.msg import Header

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Bool

import costmap_2d
import numpy as np
import imutils
import cv2
import time
import threading

from stacked_grid.srv import initStackService
from stacked_grid.srv import killStackService

def PoseStamped_to_mat(p = PoseStamped()):
    q = p.pose.orientation
    pos = p.pose.position
    T = ros_tf.transformations.quaternion_matrix([q.x,q.y,q.z,q.w])
    T[:3,3] = np.array([pos.x,pos.y,pos.z])
    return T

def tf_matrix_by_pose(tf_listener, main_grid = OccupancyGrid(), overlay_grid = OccupancyGrid()):

    overlay_origin_stamped = PoseStamped()
    overlay_origin_stamped.pose = overlay_grid.info.origin
    overlay_origin_stamped.header = overlay_grid.header

    if main_grid.header.frame_id != overlay_grid.header.frame_id:
        while True:
            try:
                overlay_origin_stamped = tf_listener.transformPose(main_grid.header.frame_id, overlay_origin_stamped)
                break
            except ros_tf.LookupException:
                pass
            except ros_tf.ExtrapolationException, e:
                print(e)
                overlay_origin_stamped.header.stamp = tf_listener.getLatestCommonTime(overlay_grid.header.frame_id, main_grid.header.frame_id)


    main_origin_stamped = PoseStamped()
    main_origin_stamped.pose = main_grid.info.origin
    main_origin_stamped.header = main_grid.header

    mat_main = PoseStamped_to_mat(main_origin_stamped)
    mat_overlay = PoseStamped_to_mat(overlay_origin_stamped)

    mat_overlay_in_main = np.matmul(np.linalg.inv(mat_main),mat_overlay)

    return mat_overlay_in_main

def stack_by_tf_matrix(main, overlay, tf_matrix, \
                       main_resolution = float(), overlay_resolution = float(),\
                       main_origin = Pose()):

    overlay_height, overlay_width = overlay.shape[0], overlay.shape[1]
    main_height, main_width = main.shape[0], main.shape[1]

    # Match resolution between main and overlay
    scale_factor = main_resolution/overlay_resolution
    overlay = cv2.resize(overlay, (0,0), fx = scale_factor, fy = scale_factor)
    
    tf_matrix_2d = np.zeros(shape= (2,3), dtype = np.float32)
    tf_matrix_2d[:2, :2] = tf_matrix[:2,:2]
    tf_matrix_2d[0, 2] = tf_matrix[0,3] / main_resolution
    tf_matrix_2d[1, 2] = tf_matrix[1,3] / main_resolution

    return cv2.warpAffine(overlay, tf_matrix_2d, (main_height,main_width))

class processGrid:
    def __init__(self, request):
        self.tf_listener = ros_tf.TransformListener()

        self.main_grid = OccupancyGrid()
        self.main_map_img = np.ndarray(shape = (1,1,1),dtype=np.uint8)

        self.secondary_grid  = list()
        self.layer_topics = list()
        self.layer_subscriber = list()

        self.pub = rospy.Publisher
        self.pub_thread = threading.Thread

        self.origin_sub = rospy.Subscriber
        self.origin_sub_thread = threading.Thread

        self.costmap_info = {
            'name'      : request.costmap_name.data,
            'frame_id_' : str(),
            'size_x_'   : 0,
            'size_y_'   : 0,
            'resolution_' : 0,
            'origin_x_' : 0.0,
            'origin_y_' : 0.0
        }
        self.origin_updated = list
        self.main_origin_stamped = PoseStamped()
        self.initialized = False
        self.running = True

    def __del__(self):
        self.running = False 
        
    def occupancy_callback(self, data, args):
        index = args
        if self.origin_updated[index]:
            self.layer_subscriber[index].info.map_load_time = data.info.map_load_time
            self.layer_subscriber[index].header.stamp = data.header.stamp
            self.layer_subscriber[index].header.frame_id = data.header.frame_id
           
            self.layer_subscriber[index].info.width = data.info.width
            self.layer_subscriber[index].info.height = data.info.height
            self.layer_subscriber[index].info.resolution = data.info.resolution

            self.layer_subscriber[index].info.origin = data.info.origin
            

            self.layer_subscriber[index].data = data.data

            if(self.costmap_info['resolution_'] == 0):
                rospy.loginfo("yo")
                self.costmap_info['resolution_'] = rospy.get_param(self.costmap_info['name'] + "/resolution")
            else:
                self.stackToMainGrid(args)

            self.origin_updated[index] = False

    def init_process(self, request):
        success = Bool()
        node_name = String()
    
        if self.initialized:
            rospy.logfatal("Service process for " + self.costmap_info['name'] + ") has been initalized")
            success.data = True
            return {'init_success': success}
        self.initialized = True

        # Get basic information of costmap
        self.costmap_info['frame_id_']   = rospy.get_param(self.costmap_info['name'] + "/frame_id")
        self.costmap_info['size_x_']     = rospy.get_param(self.costmap_info['name'] + "/size_x")
        self.costmap_info['size_y_']     = rospy.get_param(self.costmap_info['name'] + "/size_y")
        self.costmap_info['resolution_'] = rospy.get_param(self.costmap_info['name'] + "/resolution")

        # Set basic params for the main_grid
        self.main_grid.header.frame_id = self.costmap_info['frame_id_']
        self.main_grid.info.width = self.costmap_info['size_x_']
        self.main_grid.info.height = self.costmap_info['size_y_']
        self.main_grid.info.resolution = self.costmap_info['resolution_']
        
        self.main_grid.data = [0] * self.costmap_info['size_x_'] *self. costmap_info['size_y_']
        
        # Subscribe to specified topics in request
        self.layer_topics.extend(request.layer_topic)
        for i in self.layer_topics:
            self.layer_subscriber.append(OccupancyGrid())
            self.secondary_grid.append(np.ndarray)
            
            index = len(self.layer_subscriber) - 1
            rospy.Subscriber(i.data, OccupancyGrid, self.occupancy_callback,(index))
        self.origin_updated = [False] * len(self.layer_subscriber)
        for i in self.secondary_grid:
            i = np.zeros(shape = (self.costmap_info['size_x_'],self.costmap_info['size_y_'],1),dtype = np.float32)
        
        self.main_map_img = np.zeros(shape = (self.costmap_info['size_x_'],self.costmap_info['size_y_'],len(self.secondary_grid)),dtype=np.float32)
        
        self.pub = rospy.Publisher(rospy.get_name() + '/python_layer', OccupancyGrid, queue_size=1)
        self.pub_thread = threading.Thread(target = self.pubPythonLayer)
        self.pub_thread.setDaemon(True)
        self.pub_thread.start()

        rospy.Subscriber(self.costmap_info['name'] + "/origin", PoseStamped, self.GetOriginCallback)

        success.data = True
        node_name.data = rospy.get_name() + '/python_layer'
        return {'init_success': success, 'python_layer_topic' : node_name}

    def pubPythonLayer(self):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
        while self.running or not rospy.is_shutdown():          
            self.main_grid.header.stamp = rospy.Time.now()
            self.main_grid.info.map_load_time = rospy.Time.now()
            
            main_map = self.main_map_img
            
            main_map = np.max(main_map, axis = 2)
            self.main_grid.data = main_map.astype(int).ravel().tolist()

            self.pub.publish(self.main_grid)

    def GetOriginCallback(self, data):
        self.origin_updated[:] = [True] * len(self.origin_updated)
        dx = data.pose.position.x - self.main_grid.info.origin.position.x
        dy = data.pose.position.y - self.main_grid.info.origin.position.y
        if dx == 0 and dy == 0:
            pass
        else:
            self.main_grid.info.origin.position.x = data.pose.position.x
            self.main_grid.info.origin.position.y = data.pose.position.y
            self.main_grid.header = data.header
            translation_mat = np.float32([[1,0,dx],[0,1,dy]])
            for r in self.main_map_img.swapaxes(2,0).swapaxes(1,2):
                r = cv2.warpAffine(r, translation_mat, r.shape)     

    def stackToMainGrid(self, index):   
        tf_matrix = tf_matrix_by_pose(self.tf_listener, self.main_grid, self.layer_subscriber[index])
        #rospy.loginfo("get tf_matrix: " + str(1/(time.time() - dt)))
        
        grid_width = self.layer_subscriber[index].info.width
        grid_height = self.layer_subscriber[index].info.height
        grid_resolution = self.layer_subscriber[index].info.resolution
        layer = np.reshape(np.fromiter(self.layer_subscriber[index].data,np.float32),(grid_height,grid_width))
        
        #dt = time.time()
        
        self.main_map_img[:,:,index]= stack_by_tf_matrix(self.main_map_img, layer, tf_matrix, \
                                            self.main_grid.info.resolution, grid_resolution, \
                                            self.main_grid.info.origin)
