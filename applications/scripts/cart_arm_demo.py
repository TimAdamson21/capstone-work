#! /usr/bin/env python

import fetch_api
import rospy
from geometry_msgs.msg import *




def main():
    rospy.init_node('cart_arm_demo')
    arm = fetch_api.Arm()

    def shutdown():
        arm.cancel_all_goals()

    rospy.on_shutdown(shutdown)

    pose1 = Pose(Point(0.042, 0.384, 1.826), Quaternion(0.173, -0.693, -0.242, 0.657))
    pose2 = Pose(Point(0.047, 0.545, 1.822), Quaternion(-0.274, -0.701, 0.173, 0.635))
    ps1 = PoseStamped()
    ps1.header.frame_id = 'base_link'
    ps1.pose = pose1
    ps2 = PoseStamped()
    ps2.header.frame_id = 'base_link'
    ps2.pose = pose2
    gripper_poses = [ps1, ps2]
    while True:
        for x in gripper_poses:
            error = arm.move_to_pose(x)
            rospy.sleep(0.5)
            if error is not None:
                rospy.logerr(error)


            # Move the arm

if __name__ == '__main__':
    main()