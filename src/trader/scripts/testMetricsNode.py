#! /usr/bin/python2
# coding:utf-8

import rospy
import math

from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

from trader.msg import Task, actionToDo
from trader.srv import metrics

def getMetrics():
    myTestingTask = Task()
    myTestingTask.id = "1";
    myTestingTask.goalPosition_p.position = Point(*(0.2,0.5,0))
    myTestingTask.goalPosition_p.orientation = Quaternion(*(quaternion_from_euler(0, 0, 180*math.pi/180, axes='sxyz')))
    myTestingTask.toDo.x = 1


    rospy.wait_for_service("metricsNode")
    try:
        getMetricsTester = rospy.ServiceProxy("metricsNode", metrics)
        metrics_1 = getMetricsTester(myTestingTask)
        print "Cost: " + str(metrics_1.cost)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



if __name__ == '__main__':
    try:
        getMetrics()
    except rospy.ROSInterruptException:
        pass
