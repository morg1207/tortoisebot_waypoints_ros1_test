#! /usr/bin/env python3

from tortoisebot_waypoints.msg import WaypointActionGoal, WaypointActionResult, WaypointActionFeedback
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
import actionlib
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionFeedback,WaypointActionResult,WaypointActionGoal
import rospy
import rosunit
import unittest
import rostest
import time
import sys
import math

PKG = 'tortoisebot_waypoints'
NAME = 'waypoints_test'


class TestRobotControl(unittest.TestCase):#
#class TestRobotControl():

    _dist_precision = 0.05
    _yaw_precision = math.pi / 90

    
    def setUp(self):
    #def __init__(self):
        rospy.init_node('test_action_node')
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        self.goal_yaw = 0.0
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.state_action = False


    def feedback_cb(self, msg):
        pass

    def odom_callback(self, msg):

        self.current_yaw = msg.pose.pose.orientation
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        #print("Odometria")

    def euler_to_quaternion(self, msg):

        orientation_list = [msg.x, msg.y, msg.z, msg.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw

    def call_server(self):
        
        self.client.wait_for_server()
        goal = WaypointActionGoal()
        x,y, theta = self.request_values()
        #set pose
        goal.position.x = x
        goal.position.y = y
        goal.position.z = theta

        self.goal_x = goal.position.x
        self.goal_y = goal.position.y
        self.goal_yaw = goal.position.z

        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        self.state_action = result
        print(f"El resultado e:  [result]")
        return result

    def request_values(self):
        # Solicitar al usuario que ingrese los parámetros x, y, y theta
        #x = float(input("Por favor, ingresa el valor de x: "))
        #y = float(input("Por favor, ingresa el valor de y: "))
        #theta = float(input("Por favor, ingresa el valor de theta: "))

        # Imprimir los valores ingresados para confirmación

        x= -0.2
        y= 0.2
        theta= 3.14
        #print("Valores ingresados:")
        #print("x =", x)
        #print("y =", y)
        #print("theta =", theta)
        return x,y, theta

    def test_correct_rotation(self):

        self.state_action = self.call_server()
        
        self.final_yaw = self.euler_to_quaternion(self.current_yaw)
        print("Final Yaw:")
        print(self.final_yaw)
        yaw_error = self.goal_yaw - self.final_yaw
        print("Yaw Diff:")
        print(yaw_error)
        self.assertTrue(( abs(yaw_error) <= self._yaw_precision), "Integration error. Rotation was not between the expected values.")
        print("Action state:")
        print(self.state_action)

    def test_correct_distance(self):

        
        error_distance = math.sqrt( math.pow(self.goal_x - self.current_x,2)+math.pow(self.goal_y - self.current_y,2) )
        print("Final error:")
        print(error_distance)

        self.assertTrue(( abs(error_distance) <= self._dist_precision ), "Integration error. Error distance was not between the expected values.")
        print("Action state:")
        print(self.state_action)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestRobotControl, sys.argv) 
    #test = TestRobotControl()
    #test.test_correct_rotation()