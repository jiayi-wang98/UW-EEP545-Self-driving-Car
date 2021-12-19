#!/usr/bin/env python

import rospy
import numpy as np
from lab5.srv import *
import Utils
from nav_msgs.srv import GetMap

PLANNER_SERVICE_TOPIC = '/planner_node/get_car_plan'

# Testing pose sets:
SOURCE1 = [156, 1080, 0.0]
TARGET1 = [519, 828, 0.0]

SOURCE2 = [765, 504, 0.0]
TARGET2 = [1608, 729, 0.0]

SOURCE3 = [2328, 462, 0.0]
TARGET3 = [456, 732, 0.0]

POINT1 = [2500, 640, 0.0]
POINT2 = [2600, 660, 0.0]
POINT3 = [2600, 541, 0.0]
POINT4 = [1880, 440, 0.0]
POINT5 = [1435, 545, 0.0]
POINT6 = [1250, 460, 0.0]
POINT7 = [540, 835, 0.0]

POINT=[[2500, 640, 0.0],[2600, 660, 0.0],[2600, 541, 0.0],[2014,368,0.0],[1880, 440, 0.0],[1435, 545, 0.0],[1260, 460, 0.0],[1012,458,0.0],[787,576,0.0],[540, 835, 0.0]]

B_POINT1 = [1910, 340, 0.0]
B_POINT2 = [1500, 210, 0.0]
B_POINT3 = [1520, 435, 0.0]
B_POINT4 = [1130, 400, 0.0]
B_POINT5 = [670, 840, 0.0]

if __name__ == '__main__':

    rospy.init_node('planner_test', anonymous=True)

    map_service_name = rospy.get_param("~static_map", "static_map")
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    map_info = rospy.ServiceProxy(map_service_name, GetMap)().map.info

    rospy.wait_for_service(PLANNER_SERVICE_TOPIC)
    get_plan = rospy.ServiceProxy(PLANNER_SERVICE_TOPIC, GetPlan)

    try:
        #resp = get_plan(Utils.map_to_world(POINT1, map_info), Utils.map_to_world(POINT2, map_info),Utils.map_to_world(POINT3, map_info),Utils.map_to_world(POINT4, map_info),Utils.map_to_world(POINT5, map_info),Utils.map_to_world(POINT6, map_info),Utils.map_to_world(POINT7, map_info))
        for i in range(len(POINT)):
             POINT[i]=Utils.map_to_world(POINT[i], map_info)
        resp=get_plan(np.array(POINT).flatten())
        print np.array(resp.plan).reshape(-1, 3)
        print resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e

