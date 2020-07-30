#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion

p2 = Point(0.935, -2.101, 0)
q2 = Quaternion(0, 0, -0.027, 1)

p3 = Point(2.682, -2.146, 0)
q3 = Quaternion(0, 0, 1, 0.003)

p4 = Point(0.878, -10901, 0)
q4 = Quaternion(0, 0, 0.572, 0.821)

p5 = Point(1.518, -0.980, 0)
q5 = Quaternion(0, 0, 0.007, 1)

p6 = Point(1.816, -1.047, 0)
q6 = Quaternion(0, 0, -0.035, 0.999)

p7 = Point(2.119, -1.060, 0)
q7 = Quaternion(0, 0, -0.035, 0.999)

path_points = [p2, p3, p2, p4, p5, p6, p7]
path_orientations = [q2, q3, q2, q4, q5, q6, q7]
def movebase_client():
    rospy.init_node("movebase_client")
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    for i in range(len(path_points)):
        print(str(i))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = path_points[i]
        goal.target_pose.pose.orientation = path_orientations[i]
        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        
if __name__ == '__main__':
    try:
        movebase_client()
    except rospy.ROSInterruptException:
        pass
