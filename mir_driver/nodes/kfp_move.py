#!/usr/bin/env python

import rospy
import sys
import argparse
import numpy
import time

import geometry_msgs.msg as gm
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseFeedback, MoveBaseResult
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from tf.transformations import quaternion_from_euler
from actionlib_msgs.msg import GoalStatusArray
from scipy.spatial import distance


mir1_position = [[17.5, 17.6, 6.6, 6.9],
                 [8.0, 5.0, 5.1, 10.7],
                 [-0.71, -1.0, 0.71, -0.15]]
                 # [0.71, 0.00, 0.70, 0.99]]
mir2_position = [[5.0, 11.0, 5.0, 11.0],
                 [1.2, 9.0, 5.0, 11.0],
                 [0.0, 1.0, 5.0, 11.0]]
                 # [1.06, 0.0, 5.0, 11.0]]
pos_m1 = [0.0, 0.0]
pos_m2 = [0.0, 0.0]
mir_status = [-1, -1]
f1 = 0
pub = rospy.Publisher('/mir_state', String, queue_size=10)


def mir1_feed(data):
    # global pos_m1_x, pos_m1_y
    global pos_m1, mir_status
    location = data.feedback.base_position.pose
    status = data.status.status
    print(1, status, location.position.x, location.position.y)
    # pos_m1_x = float(location.position.x)
    # pos_m1_y = float(location.position.y)
    pos_m1 = [float(location.position.x), float(location.position.y)]
    mir_status[0] = status
    # rospy.sleep(0.5)


def mir2_feed(data):
    # global pos_m2_x, pos_m2_y
    global pos_m2, mir_status
    location = data.feedback.base_position.pose
    status = data.status.status
    print(2, status, location.position.x, location.position.y)
    # pos_m2_x = float(location.position.x)
    # pos_m2_y = float(location.position.y)
    pos_m2 = [float(location.position.x), float(location.position.y)]
    mir_status[1] = status
    # rospy.sleep(0.5)


# def mir1_status(data):
#     print(data.status_list.goal_id.status)
#
#
# def mir2_status(data):
#     print(data.status_list.goal_id.status)
#
#
def mir1_move(p_x, p_y, o_z):
    mir1_pub = rospy.Publisher("mir/move_base_simple/goal", PoseStamped, queue_size=5)

    p = PoseStamped()

    p.header.seq = 1
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = "map"

    p.pose.position.x = p_x
    p.pose.position.y = p_y
    p.pose.position.z = 0.0

    p.pose.orientation.x = 0.0
    p.pose.orientation.y = 0.0
    p.pose.orientation.z = o_z
    p.pose.orientation.w = 1.0

    # rospy.sleep(1)
    mir1_pub.publish(p)

    print("done mir1")


def mir2_move(p_x, p_y, o_z):
    mir2_pub = rospy.Publisher("mir2/move_base_simple/goal", PoseStamped, queue_size=5)

    p = PoseStamped()

    p.header.seq = 1
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = "map"

    p.pose.position.x = p_x
    p.pose.position.y = p_y
    p.pose.position.z = 0.0

    p.pose.orientation.x = 0.0
    p.pose.orientation.y = 0.0
    p.pose.orientation.z = o_z
    p.pose.orientation.w = 1.0

    # rospy.sleep(1)
    mir2_pub.publish(p)

    print("done mir 2")


def timer_callback(event):
    global mir_status

    while not rospy.is_shutdown():
        pub.publish(mir_status)


def start():
    global f1
    mir1_move(mir1_position[0][0], mir1_position[1][0], mir1_position[2][0])
    mir2_move(mir2_position[0][0], mir2_position[1][0], mir2_position[2][0])
    f1 = 1


def make_it_happen():
    global f1
    rospy.init_node('kfp_mir_move')

    if f1 == 0:
        start()

    rospy.Subscriber("mir/move_base/feedback", MoveBaseActionFeedback, mir1_feed)
    rospy.Subscriber("mir2/move_base/feedback", MoveBaseActionFeedback, mir2_feed)
    timer = rospy.Timer(rospy.Duration(2.0), timer_callback)

    rospy.spin()
    timer.shutdown()


if __name__ == '__main__':
    try:
        make_it_happen()
    except rospy.ROSInterruptException:
        pass









