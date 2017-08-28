#!/usr/bin/env python

import rospy
import fetch_api
from geometry_msgs.msg import *
from joint_state_reader import JointStateReader

import sys, select, termios, tty
from pprint import pprint
import math

msg = """
Control Your Fetch!
---------------------------
Moving around:
        w
   a    s    d

Space: force stop
i/k: increase/decrease only linear speed by 5 cm/s
u/j: increase/decrease only angular speed by 0.25 rads/s
anything else: stop smoothly

CTRL-C to quit
"""

moveBindings = {'w': (.1, 0, 0), 'a': (0, .1, 0), 'd': (0, -.1, 0), 's': (-.1, 0, 0), 't': (0, 0, 0.1), 'g': (0, 0, -0.1)}

speedBindings = {
    'i': (0.05, 0),
    'k': (-0.05, 0),
    'u': (0, 0.25),
    'j': (0, -0.25),
}

turnBindings = {
    'b': 0.25,
    'v': -0.25
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed = .2
turn = 1


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

def turn_arm(angle):
    reader = JointStateReader()
    rospy.sleep(0.5)
    names = fetch_api.ArmJoints.names()
    arm_vals = reader.get_joints(names)
    pprint(arm_vals)
    arm_vals[5] = arm_vals[5] + angle
    pprint(arm_vals)
    arm.move_to_joints(fetch_api.ArmJoints.from_list(arm_vals))

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_arm_teleop')
    base = fetch_api.Base()
    arm = fetch_api.Arm()
    x = 0.5
    y = 0.3
    z = 1.0

    x_fact = .1
    y_fact = .1
    z_fact = 0.5
    w_fact = 0.25
    status = 0
    count = 0
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    quat_x = 0
    quat_y = 1
    quat_z = 0
    quat_w = math.pi / 2


    print msg
    print vels(speed, turn)
    while True:
        try:
            init_x = x
            init_y = y
            init_z = z
            init_w = quat_w

            key = getKey()
            if key in moveBindings.keys():
                x += x_fact * moveBindings[key][0]
                y += y_fact * moveBindings[key][1]
                z += z_fact * moveBindings[key][2]
            elif key in speedBindings.keys():
                x_fact += speedBindings[key][0]
                y_fact += speedBindings[key][1]
            elif key in turnBindings.keys():
                quat_w += w_fact * turnBindings[key]
                continue
            pose = Pose(Point(x, y, z), Quaternion(quat_x, quat_y, quat_z, quat_w))
            ps = PoseStamped()
            ps.header.frame_id = 'base_link'
            ps.pose = pose
            error = arm.move_to_pose(ps, 3) #This is the all important line
            print(error)
            print("{\n x: %f, y: %f, z: %f, w: %f" % (x, y, z, quat_w))
            print(" x_step: %f, y_step: %f, z_step: %f, w_step: %f" % (x_fact * moveBindings['w'][0],
                                                                      y_fact * moveBindings['a'][1],
                                                                      z_fact * moveBindings['t'][2],
                                                                      w_fact * turnBindings['b']
                                                                      ))
            if error == "PLANNING_FAILED":
                x = init_x
                y = init_y
                z = init_z
                quat_w = init_w
            rospy.sleep(0.02)
        except Exception as e:
            print(type(e))
            pprint(e)
            print("Shutting down")
            break
       # except Exception as e:
        #    print(type(e))
         #   pprint(e)
          #  print("That didn't work")
           # continue
    #termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
