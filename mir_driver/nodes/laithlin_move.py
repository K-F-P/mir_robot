#!/usr/bin/env python3

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

#wspolrzedne punkyow (x, y, obrot z)
mir1_position = [[17.5, 17.6, 6.6, 6.9],
                 [8.0, 5.0, 5.1, 10.7],
                 [-0.71, -1.0, 0.71, -0.15]]
                 # [0.71, 0.00, 0.70, 0.99]]
mir2_position = [[5.0, 11.0, 5.0, 11.0],
                 [1.2, 9.0, 5.0, 11.0],
                 [0.0, 1.0, 5.0, 11.0]]
                 # [1.06, 0.0, 5.0, 11.0]]

#obecne wspolrzedne brane z feedback
pos_m1 = [0.0, 0.0]
pos_m2 = [0.0, 0.0]

#moj kod
m1_m = PoseStamped()
m2_m = PoseStamped()

stat_go1 = 0
stat_go2 = 0

#flagi do grafow
mir_status = [-1, -1]

#flaga startowa jak rowna zero to startuje
# f1 = 0

#moj kod

f_mir1 = 0
f_mir2 = 0

done1 = False
done2 = False
started = False

send1 = False
send2 = False

#to jest wlasny pub
pub = rospy.Publisher('/mir_state', String, queue_size=10)

#moj kod
mir1_pub = rospy.Publisher("mir/move_base_simple/goal", PoseStamped, queue_size=5)
mir2_pub = rospy.Publisher("mir2/move_base_simple/goal", PoseStamped, queue_size=5)

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

    #moj kod
    global done1
    if(not done1):
        done1 = True


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

    # moj kod
    global done2
    if (not done2):
        done2 = True

# def mir1_status(data):
#     print(data.status_list.goal_id.status)
#
#
# def mir2_status(data):
#     print(data.status_list.goal_id.status)
#
#
def mir1_move(p_x, p_y, o_z):
    # to ponizej bylo tylko odkomentowane
    #mir1_pub = rospy.Publisher("mir/move_base_simple/goal", PoseStamped, queue_size=5)

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

    # to ponizej bylo tylko odkomentowane
    # mir1_pub.publish(p)
    #
    # print("done mir1")

    #moj kod
    return p


def mir2_move(p_x, p_y, o_z):
    # to ponizej bylo tylko odkomentowane
    #mir2_pub = rospy.Publisher("mir2/move_base_simple/goal", PoseStamped, queue_size=5)

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

    #to ponizej bylo tylko odkomentowane
    #mir2_pub.publish(p)
    #print("done mir 2")

    #moj kod
    return p




def timer_callback(event):
    global mir_status, mir2_pub, m1_m, m2_m, send1, send2, stat_go2, stat_go1

    #to bylo odkomentowane
    # while not rospy.is_shutdown():
    #     pub.publish(mir_status)

    #moj kod

    if (done1 is True) and (done2 is True):
        pub.publish(mir_status)

    if (started):
        mir2_pub.publish(m2_m)
        print("m2 done")

        mir1_pub.publish(m1_m)
        print("m1 done")

        if stat_go1 == 2:
            send1 = True

        if stat_go2 == 2:
            send2 = True




def start():
    global f_mir1, f_mir2,  middle1, middle2
    # mir1_move(mir1_position[0][0], mir1_position[1][0], mir1_position[2][0])
    # mir2_move(mir2_position[0][0], mir2_position[1][0], mir2_position[2][0])

    #moj kod
    global m1_m, m2_m, started
    m1_m = mir1_move(mir1_position[0][0], mir1_position[1][0], mir1_position[2][0])
    m2_m = mir2_move(mir2_position[0][0], mir2_position[1][0], mir2_position[2][0])
    if(not started):
        started = True
    #moj kod

    f_mir1 = 1
    f_mir2 = 1

#moj kod

def mir1_reach(m_r):
    global m1_m, f_mir1, send1, stat_go1

    stat = m_r.status_list[0]
    #print(stat.status)

    stat_go1 = stat.status

    if stat_go1 == 3:
        if (f_mir1 == 1) and (send1 is True):
            m1_m = mir1_move(mir1_position[0][1], mir1_position[1][1], mir1_position[2][1])
            print("mir 1 krok 2")
            f_mir1 = 2
            send1 = False

        elif (f_mir1 == 2) and (send1 is True):
            m1_m = mir1_move(mir1_position[0][2], mir1_position[1][2], mir1_position[2][2])
            print("mir 1 krok 3")
            f_mir1 = 0
            send1 = False



def mir2_reach(m_r):
    global m2_m, f_mir2, send2, stat_go2

    stat = m_r.status_list[0]
    #print(stat.status)

    stat_go2 = stat.status

    if stat_go2 == 3:
        if (f_mir2 == 1) and (send2 is True):
            m2_m = mir2_move(mir2_position[0][1], mir2_position[1][1], mir2_position[2][1])
            print("mir 2 krok 2")
            f_mir2 = 2
            send2 = False

        elif (f_mir2 == 2) and (send2 is True):
            m2_m = mir2_move(mir2_position[0][2], mir2_position[1][2], mir2_position[2][2])
            print("mir 2 krok 3")
            f_mir2 = 0
            send2 = False




#moj kod

#TODO funkcja obliczajaca odleglosc razem z tym zeby sie zatrzymywaly jak ona jest za mala

def mir_distance():
    global pos_m1, pos_m2
    dist = distance.euclidean(pos_m1, pos_m2)
    return dist

def make_it_happen():
    global f_mir1, f_mir2
    rospy.init_node('kfp_mir_move')

    if (f_mir1 == 0) and (f_mir2 == 0):
        start()

    #TODO z /move_base/status wyciac numr statusu jak zrobie echo na tym topicu to jest cyfra ktora trzeba wyciagnac

    #moj kod
    rospy.Subscriber("mir/move_base/status", GoalStatusArray, mir1_reach)
    rospy.Subscriber("mir2/move_base/status", GoalStatusArray, mir2_reach)

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









