#!/usr/bin/env python2
# -*- coding: utf-8 -*-

'''  Import libraries  '''
from stackedGridPython import *

service_process = dict()
costmap_name = list()

def init_srv(request):
    global service_process
    global costmap_name

    if request.costmap_name.data in service_process.keys():
        rospy.logfatal("Service process for " + request.costmap_name.data + ") has been initalized")
        return False
        
    new_process = processGrid(request)
    service_process[request.costmap_name.data] = new_process
    
    return new_process.init_process(request)

def end_srv(request):
    del service_process[request.costamp_name.data]

if __name__ == "__main__":

    rospy.init_node('pythonStackServer')

    rospy.loginfo("Init " + rospy.get_name().strip('/') + " to process layer-stacking")
    rospy.loginfo("Waiting for client...")

    start_process_service = rospy.Service('initStackService', initStackService, init_srv)
    end_process_call = rospy.Service('killStackService', killStackService, end_srv)
    rospy.spin()
