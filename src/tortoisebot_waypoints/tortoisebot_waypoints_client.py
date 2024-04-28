#! /usr/bin/env python3

import rospy
import actionlib
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionFeedback,WaypointActionResult,WaypointActionGoal

def feedback_cb(msg):
    pass

def call_server():

    client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)

    client.wait_for_server()

    goal = WaypointActionGoal()

    #set pose
    goal.position.x = 0.0
    goal.position.y = 0.5
    goal.position.z = 0.0

    client.send_goal(goal, feedback_cb=feedback_cb)

    client.wait_for_result()

    result = client.get_result()

    return result

if __name__ == '__main__':

    try:
        rospy.init_node('action_client')
        result = call_server()
        print('Succefully')
    except rospy.ROSInterruptException as e:
        print('Something wrong')