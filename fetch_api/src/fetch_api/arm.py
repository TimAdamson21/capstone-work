import actionlib
import trajectory_msgs.msg
import control_msgs.msg
import rospy
import moveit_python
import moveit_commander
from pprint import pprint
from .moveit_goal_builder import MoveItGoalBuilder
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from pprint import pprint


from .arm_joints import ArmJoints

TIME_FROM_START = 5


class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """
    def __init__(self):
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self._ac = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory",
                                                control_msgs.msg.FollowJointTrajectoryAction)
        self._move_group_ac = actionlib.SimpleActionClient("move_group", MoveGroupAction)
        print("waiting for joint trajectory server")
        self._ac.wait_for_server()
        print("joint trajectory server is up!")
        print("waiting for move group server")
        print("launch the move group server with roslaunch fetch_api move_group.launch")
        self._move_group_ac.wait_for_server()
        print("move group server  is up!")

    def check_pose(self,
                   pose_stamped,
                   allowed_planning_time=10.0,
                   group_name='arm',
                   tolerance=0.01):
        return self.move_to_pose(
            pose_stamped,
            allowed_planning_time=allowed_planning_time,
            group_name=group_name,
            tolerance=tolerance,
            plan_only=True)

    def move_to_pose(self,
                     pose_stamped,
                     allowed_planning_time = 10.0,
                     execution_timeout = 15.0,
                     group_name = 'arm',
                     num_planning_attempts = 1,
                     plan_only = False,
                     replan=False,
                     replan_attempts=5,
                     tolerance=0.01):
        """Moves the end-effector to a pose, using motion planning.

        Args:
            pose: geometry_msgs/PoseStamped. The goal pose for the gripper.

        Returns:
            string describing the error if an error occurred, else None.
        """
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_pose_goal(pose_stamped)
        goal_builder.allowed_planning_time = allowed_planning_time
        goal_builder.num_planning_attempts = num_planning_attempts
        goal_builder.plan_only = plan_only
        goal_builder.replan = replan
        goal_builder.replan_attempts = replan_attempts
        goal_builder.tolerance = tolerance
        goal = goal_builder.build()
        self._move_group_ac.send_goal(goal)
        #print("Waiting for pose goal reply")
        self._move_group_ac.wait_for_result(rospy.Duration(execution_timeout))
        #print("Pose goal result recieved")
        result = self._move_group_ac.get_result()
        if result is not None:
            error_string = moveit_error_string(result.error_code.val)
        else:
            error_string = "BAD RESULT"

        if error_string == "SUCCESS":
            return None
        else:
            return error_string

    def compute_ik(self, pose_stamped, timeout=rospy.Duration(5)):
            """Computes inverse kinematics for the given pose.

            Note: if you are interested in returning the IK solutions, we have
                shown how to access them.

            Args:
                pose_stamped: geometry_msgs/PoseStamped.
                timeout: rospy.Duration. How long to wait before giving up on the
                    IK solution.

            Returns: True if the inverse kinematics were found, False otherwise.
            """
            request = GetPositionIKRequest()
            request.ik_request.pose_stamped = pose_stamped
            request.ik_request.group_name = 'arm'
            request.ik_request.timeout = timeout
            response = self._compute_ik(request)
            error_str = moveit_error_string(response.error_code.val)
            success = error_str == 'SUCCESS'
            if not success:
                return False
            joint_state = response.solution.joint_state
            for name, position in zip(joint_state.name, joint_state.position):
                if name in ArmJoints.names():
                    rospy.loginfo('{}: {}'.format(name, position))
            return True


    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = arm_joints.values()
        point.time_from_start.secs = TIME_FROM_START
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.points.append(point)
        goal.trajectory.joint_names = ArmJoints.names()

        self._ac.send_goal(goal)
        print("Waiting for reply")
        self._ac.wait_for_result()
        print("Result received")
        return self._ac.get_result()

    def cancel_all_goals(self):
        self._ac.cancel_all_goals()  # Your action client from Lab 7
        self._move_group_ac.cancel_all_goals()  # From this lab


def moveit_error_string(val):
    """Returns a string associated with a MoveItErrorCode.

    Args:
        val: The val field from moveit_msgs/MoveItErrorCodes.msg

    Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
        if the value is invalid.
    """
    if val == MoveItErrorCodes.SUCCESS:
        return 'SUCCESS'
    elif val == MoveItErrorCodes.FAILURE:
        return 'FAILURE'
    elif val == MoveItErrorCodes.PLANNING_FAILED:
        return 'PLANNING_FAILED'
    elif val == MoveItErrorCodes.INVALID_MOTION_PLAN:
        return 'INVALID_MOTION_PLAN'
    elif val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'
    elif val == MoveItErrorCodes.CONTROL_FAILED:
        return 'CONTROL_FAILED'
    elif val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
        return 'UNABLE_TO_AQUIRE_SENSOR_DATA'
    elif val == MoveItErrorCodes.TIMED_OUT:
        return 'TIMED_OUT'
    elif val == MoveItErrorCodes.PREEMPTED:
        return 'PREEMPTED'
    elif val == MoveItErrorCodes.START_STATE_IN_COLLISION:
        return 'START_STATE_IN_COLLISION'
    elif val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return 'START_STATE_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_IN_COLLISION:
        return 'GOAL_IN_COLLISION'
    elif val == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
        return 'GOAL_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
        return 'GOAL_CONSTRAINTS_VIOLATED'
    elif val == MoveItErrorCodes.INVALID_GROUP_NAME:
        return 'INVALID_GROUP_NAME'
    elif val == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
        return 'INVALID_GOAL_CONSTRAINTS'
    elif val == MoveItErrorCodes.INVALID_ROBOT_STATE:
        return 'INVALID_ROBOT_STATE'
    elif val == MoveItErrorCodes.INVALID_LINK_NAME:
        return 'INVALID_LINK_NAME'
    elif val == MoveItErrorCodes.INVALID_OBJECT_NAME:
        return 'INVALID_OBJECT_NAME'
    elif val == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
        return 'FRAME_TRANSFORM_FAILURE'
    elif val == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
        return 'COLLISION_CHECKING_UNAVAILABLE'
    elif val == MoveItErrorCodes.ROBOT_STATE_STALE:
        return 'ROBOT_STATE_STALE'
    elif val == MoveItErrorCodes.SENSOR_INFO_STALE:
        return 'SENSOR_INFO_STALE'
    elif val == MoveItErrorCodes.NO_IK_SOLUTION:
        return 'NO_IK_SOLUTION'
    else:
        return 'UNKNOWN_ERROR_CODE'