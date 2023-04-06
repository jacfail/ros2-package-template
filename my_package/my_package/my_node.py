#!/usr/bin/env python3
# my_node.py
# Jacob Faile


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import math
import os


class My_Node(Node):
    def __init__(self):
        super().__init__("My_Node")
        print('My_Node Starting Up...')

        # create subscriber
        self._example_subscription = self.create_subscription(
            NavigateToPose_FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self._example_callback,
            10)
        self._example_subscription  # prevent unused variable warning

        # create publisher
        self.example_publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)


    def _example_callback(self, data):

        # print out subscribed data
        print(data)

        # create goal Pose Stamped message and send to the publisher
        goal = PoseStamped()
        self.goal_position = Point()
        self.goal_orientation = Quaternion()

        self.goal_position.x = 1.0
        self.goal_position.y = 0.0
        self.goal_position.z = 0.0

        self.goal_orientation.x = 1.0
        self.goal_orientation.y = 0.0
        self.goal_orientation.z = 0.0
        self.goal_orientation.w = 0.0

        goal.pose.position = self.goal_position
        goal.pose.orientation = self.goal_orientation

        # publish new data
        self.goalpoint_publisher_.publish(goal)
    

def main(args=None):
    rclpy.init()
    run_my_node = My_Node()

    rclpy.spin(run_my_node)

    #Clean up and shutdown.
    run_my_node.destroy_node()  
    rclpy.shutdown()


if __name__ == "__main__":
    main()