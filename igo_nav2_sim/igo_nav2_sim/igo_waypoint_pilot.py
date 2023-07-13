#! /usr/bin/env python3
# Copyright 2018 Intel Corporation.
# Copyright 2020 Florian Gramss
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import math
import os
import sys
import time

from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

import zmq

import tf_transformations

class Waypoint():
    def __init__(self, x, y, theta):
        self.posX = x
        self.posY = y
        self.theta = theta

    def to_PoseStamped(self):
        pose = PoseStamped()
        pose.pose.position.x = self.posX
        pose.pose.position.y = self.posY
        pose.pose.position.z = 0.0
        [qX, qY, qZ, qW] = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        pose.pose.orientation.x = qX
        pose.pose.orientation.y = qY
        pose.pose.orientation.z = qZ
        pose.pose.orientation.w = qW
        return(pose)

    def to_string(self):
        return("({:0.3f}, {:0.3f})@{:0.3f}°".format(self.posX, self.posY, math.degrees(self.theta)))




class IgoWaypointPilot(Node):

    def __init__(self):
        super().__init__(node_name='igo_waypoint_pilot')
        self.initial_pose_received = False
        self.goal_pose = Pose()
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = []

    def info_msg(self, msg: str):
        self.get_logger().info('\033[1;37;44m' + msg + '\033[0m')

    def warn_msg(self, msg: str):
        self.get_logger().warn('\033[1;37;43m' + msg + '\033[0m')

    def error_msg(self, msg: str):
        self.get_logger().error('\033[1;37;41m' + msg + '\033[0m')

    def defineWaypoints(self):
        self.waypoints.clear()
        if (False):
            # Square
            self.waypoints.append(Waypoint(0.0, 0.0, math.pi))
            self.waypoints.append(Waypoint(4.0, 0.0, -math.pi/2))
            self.waypoints.append(Waypoint(4.0, 4.0, 0.0))
            self.waypoints.append(Waypoint(0.0, 4.0, math.pi/2))
        elif (False):
            # Parcours for simple_warehouse
            # 1st rectangle
            self.waypoints.append(Waypoint(-1.0, 0.0, math.pi))
            self.waypoints.append(Waypoint(4.0, 0.0, -math.pi/2))
            self.waypoints.append(Waypoint(4.0, 4.0, 0.0))
            self.waypoints.append(Waypoint(-1.0, 4.0, 0.0))
            # into box 2 (forward)
            self.waypoints.append(Waypoint(-2.0, 4.5, 0.0))
            self.waypoints.append(Waypoint(-6.5, 4.5, 0.0))
            self.waypoints.append(Waypoint(-2.0, 4.5, math.pi/2))
            # into box 1 (backward)
            self.waypoints.append(Waypoint(-2.0, 7.5, math.pi/2))
            self.waypoints.append(Waypoint(-2.0, 7.5, math.pi))
            self.waypoints.append(Waypoint(-8.0, 7.5, math.pi))
            self.waypoints.append(Waypoint(-2.0, 7.5, math.pi))
            self.waypoints.append(Waypoint(-2.0, 7.5, math.pi/2))
            # turn in the middle
            self.waypoints.append(Waypoint(-2.0, 3.0, math.pi/2))
            self.waypoints.append(Waypoint(-2.0, 3.0, 0.0))
            # into box 4 (backward)
            self.waypoints.append(Waypoint(-2.0, -2.0, -math.pi/2))
            self.waypoints.append(Waypoint(-2.0, -2.0, math.pi))
            self.waypoints.append(Waypoint(-8.0, -2.0, math.pi))
            self.waypoints.append(Waypoint(-2.0, -2.0, math.pi))
            self.waypoints.append(Waypoint(-2.0, -2.0, -math.pi/2))
        elif (False):
            # Parcours for workshop_warehouse
            # 1st rectangle
            self.waypoints.append(Waypoint(-3.5, -5.5, math.pi))
            self.waypoints.append(Waypoint(8.5, -5.5, math.pi))
            self.waypoints.append(Waypoint(8.5, -5.5, -math.pi/2))
            self.waypoints.append(Waypoint(8.5, 14.0, 0.0))
            self.waypoints.append(Waypoint(0.0, 14.0, 0.0))
            self.waypoints.append(Waypoint(0.0, 14.0, math.pi/2))
            self.waypoints.append(Waypoint(0.0, 18.0, math.pi/2))
            self.waypoints.append(Waypoint(0.0, 14.0, math.pi/2))
            self.waypoints.append(Waypoint(0.0, 14.0, math.pi))
            self.waypoints.append(Waypoint(8.5, 14.0, math.pi))
            self.waypoints.append(Waypoint(8.5, 14.0, math.pi/2))
            self.waypoints.append(Waypoint(8.5, -5.5, math.pi/2))
            self.waypoints.append(Waypoint(8.5, -5.5, 0.0))
            self.waypoints.append(Waypoint(-3.5, -5.5, 0.0))
        elif (True):
            # Parcours for real warehouse, robotic section
            self.waypoints.append(Waypoint(19.5, 7.0, 0.0))
            self.waypoints.append(Waypoint(7.5, 7.0, 0.0))
            self.waypoints.append(Waypoint(5.0, 5.25, math.pi/2))
            self.waypoints.append(Waypoint(7.5, 3.5, math.pi))
            self.waypoints.append(Waypoint(19.5, 3.5, math.pi))
            self.waypoints.append(Waypoint(22.0, 5.25, -math.pi/2))


    def runNavigateAction(self):
        # Sends a `NavToPose` action request and waits for completion
        self.info_msg("Waiting for 'NavigateToPose' action server")
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.info_msg("'NavigateToPose' action server not available, waiting...")

        self.goal_pose.position.x = 4.0
        self.goal_pose.position.y = 0.0
        self.goal_pose.position.z = 0.0
        self.goal_pose.orientation.x = 0.0
        self.goal_pose.orientation.y = 0.0
        self.goal_pose.orientation.z = 0.707
        self.goal_pose.orientation.w = 0.707
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.getStampedPoseMsg(self.goal_pose)

        self.info_msg('Sending goal request...')
        send_goal_future = self.action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.error_msg('Goal rejected')
            return False

        self.info_msg('Goal accepted')
        get_result_future = goal_handle.get_result_async()

        future_return = True
        self.info_msg("Waiting for 'NavigateToPose' action to complete")
        rclpy.spin_until_future_complete(self, get_result_future)
        status = get_result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.info_msg('Goal failed with status code: {0}'.format(status))
            return False

        if not future_return:
            return False

        self.info_msg('Goal succeeded!')
        return True


    def navigateUsingWaypoints(self):
        self.defineWaypoints()

        # Sends a `NavToPose` action request and waits for completion
        self.info_msg("Waiting for 'NavigateToPose' action server")
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.info_msg("'NavigateToPose' action server not available, waiting...")

        for idx in range(0, len(self.waypoints)):
            goal_msg = NavigateToPose.Goal()
            self.info_msg("Next waypoint is " + self.waypoints[idx].to_string())
            goal_msg.pose = self.waypoints[idx].to_PoseStamped()
            goal_msg.pose.header.frame_id = "map"

            self.info_msg('Sending goal request...')
            send_goal_future = self.action_client.send_goal_async(goal_msg)

            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.error_msg('Goal rejected')
                return False

            self.info_msg('Goal accepted')
            get_result_future = goal_handle.get_result_async()

            future_return = True
            self.info_msg("Waiting for 'NavigateToPose' action to complete")
            rclpy.spin_until_future_complete(self, get_result_future)
            status = get_result_future.result().status
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.info_msg('Goal failed with status code: {0}'.format(status))
                return False

            if not future_return:
                return False

            self.info_msg('Goal succeeded!')

        self.info_msg('All waypoints done!')
        return True

    def doParcours(self, cnt):
        for idx in range(0, cnt):
            self.warn_msg("Start parcours #{:d}/{:d}".format(idx, cnt))
            self.navigateUsingWaypoints()


    def distanceFromGoal(self):
        d_x = self.current_pose.position.x - self.goal_pose.position.x
        d_y = self.current_pose.position.y - self.goal_pose.position.y
        distance = math.sqrt(d_x * d_x + d_y * d_y)
        self.info_msg('Distance from goal is: ' + str(distance))
        return distance

    def getStampedPoseMsg(self, pose: Pose):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose = pose
        return msg


def main():
    # The robot(s) positions from the input arguments
    # parser = argparse.ArgumentParser(description='System-level navigation tester node')
    # group = parser.add_mutually_exclusive_group(required=True)
    # group.add_argument('-r', '--robot', action='append', nargs=4,
    #                    metavar=('init_x', 'init_y', 'final_x', 'final_y'),
    #                    help='The robot starting and final positions.')
    # group.add_argument('-rs', '--robots', action='append', nargs=5,
    #                    metavar=('name', 'init_x', 'init_y', 'final_x', 'final_y'),
    #                    help="The robot's namespace and starting and final positions. " +
    #                         'Repeating the argument for multiple robots is supported.')

    # args, unknown = parser.parse_known_args()

    rclpy.init()

    # Create testers for each robot
    pilot = IgoWaypointPilot()
    if False:
        pilot.runNavigateAction()
    elif False:
        pilot.navigateUsingWaypoints()
    else:
        pilot.doParcours(200)

if __name__ == '__main__':
    main()
