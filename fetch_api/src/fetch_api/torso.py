#!/usr/bin/env python

import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import rospy
from pprint import pprint

ACTION_NAME = "torso_controller/follow_joint_trajectory"
JOINT_NAME = "torso_lift_joint"
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        self._ac = actionlib.SimpleActionClient(ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        print("now waiting for server")
        self._ac.wait_for_server()
        print("successfully connected to server")
        # TODO: Create actionlib client
        # TODO: Wait for server
        pass

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        if height >= Torso.MIN_HEIGHT and height <= Torso.MAX_HEIGHT:
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions.append(height)
            point.time_from_start.secs = TIME_FROM_START
            goal = control_msgs.msg.FollowJointTrajectoryGoal()
            pprint(goal)
            goal.trajectory.joint_names.append(JOINT_NAME)
            goal.trajectory.points.append(point)
            self._ac.send_goal(goal)
            print("waiting for result")
            self._ac.wait_for_result()
            print("successful result")
            return self._ac.get_result()
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        # TODO: Create a trajectory point
        # TODO: Set position of trajectory point
        # TODO: Set time of trajectory point

        # TODO: Create goal
        # TODO: Add joint name to list
        # TODO: Add the trajectory point created above to trajectory

        # TODO: Send goal
        # TODO: Wait for result
        
        else:
          rospy.logerr("Improper height set")
        
