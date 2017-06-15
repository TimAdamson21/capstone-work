#!/usr/bin/env python

import actionlib
import trajectory_msgs.msg
import control_msgs.msg
import geometry_msgs.msg
import math
import rospy
from pprint import pprint


LOOK_AT_ACTION_NAME = 'head_controller/point_head'  # TODO: Get the name of the look-at action
PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory'  # TODO: Get the name of the pan/tilt action
PAN_JOINT = 'head_pan_joint'  # TODO: Get the name of the head pan joint
TILT_JOINT = 'head_tilt_joint'  # TODO: Get the name of the head tilt joint
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = fetch_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = -math.pi/2  # TODO: Minimum pan angle, in radians.
    MAX_PAN = math.pi/2  # TODO: Maximum pan angle, in radians.
    MIN_TILT = -math.pi/4  # TODO: Minimum tilt angle, in radians.
    MAX_TILT = math.pi/2  # TODO: Maximum tilt angle, in radians.


    def __init__(self):
        self._look_at_action_client = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, control_msgs.msg.PointHeadAction)
        self._pan_tilt_action_client = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        print("waiting for look_at server")
        self._look_at_action_client.wait_for_server()
        print("the look_at server is up!")
        print("waiting for pan_tilt server")
        self._pan_tilt_action_client.wait_for_server()
        print("the pan_tilt server is up!")
        pass

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        goal = control_msgs.msg.PointHeadGoal()
        pprint(goal)
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration.secs = 1
        goal.target.header.frame_id = frame_id
        pprint(goal)
        self._look_at_action_client.send_goal(goal)
        print("waiting for response")
        self._look_at_action_client.wait_for_result()
        print("Response received")
        print(self._look_at_action_client.get_state())
        return self._look_at_action_client.get_result()


        # TODO: Create goal
        # TODO: Fill out the goal (we recommend setting min_duration to 1 second)
        # TODO: Send the goal
        # TODO: Wait for result

        rospy.logerr('Not implemented.')

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """

        if pan <= Head.MAX_PAN and pan >= Head.MIN_PAN and tilt >= Head.MIN_TILT and tilt <= Head.MAX_TILT:
            goal = control_msgs.msg.FollowJointTrajectoryGoal()
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions.append(pan)
            point.positions.append(tilt)
            point.time_from_start.secs = PAN_TILT_TIME
            goal.trajectory.points.append(point)
            goal.trajectory.joint_names.append(PAN_JOINT)
            goal.trajectory.joint_names.append(TILT_JOINT)
            self._pan_tilt_action_client.send_goal(goal)
            print("Waiting for response")
            self._pan_tilt_action_client.wait_for_result()
            print("Response recieved")
            return self._pan_tilt_action_client.get_result()


            pprint(goal)
            print('\n\n')
            pprint(point)
        # TODO: Check that the pan/tilt angles are within joint limits
        # TODO: Create a trajectory point
        # TODO: Set positions of the two joints in the trajectory point
        # TODO: Set time of the trajectory point

        # TODO: Create goal
        # TODO: Add joint names to the list
        # TODO: Add trajectory point created above to trajectory

        # TODO: Send the goal
        # TODO: Wait for result
        else:
            rospy.logerr("please enter valid angles")
